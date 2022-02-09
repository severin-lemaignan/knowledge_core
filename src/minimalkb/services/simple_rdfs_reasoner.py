import logging

logger = logging.getLogger("minimalKB." + __name__)
DEBUG_LEVEL = logging.DEBUG

import time
import datetime
import sqlite3

from minimalkb.backends.sqlite import sqlhash
from minimalkb.kb import DEFAULT_MODEL

REASONER_RATE = 5  # Hz


class OntoClass:
    def __init__(self, name):
        self.name = name
        self.parents = set()
        self.children = set()
        self.instances = set()
        self.equivalents = set()

    def __repr__(self):
        return (
            self.name
            + "\n\tParents: "
            + str(self.parents)
            + "\n\tChildren: "
            + str(self.children)
            + "\n\tInstances: "
            + str(self.instances)
        )


# recursive functions for ontologic inheritances :
# ------------------------------------------------


def addequivalent(equivalentclasses, cls, equivalent, memory=None):
    """
    propagation of equivalence properties :
    ---------------------------------------
        1) transitivity : (A = B) and (B = C) then (A = C)
        2) symetry : (A = B) then (B = A)
        3) the reflexive property results from symetry and transitivity.

    we need to update the classes to make the additions available for the other calls of inheritance functions

    the memory set is used to prevent the method to loop infinitly
    for instance :

        let B in the set of equivalents of A
        then the method do with (A and B) :

            1) equivalentclasses.add(A.name , B.name)
            2) A is added to the set of equivalents of B
            3) for every equivalents C of the set of B do the same with (A and C)
                but inside this set we find A (we just added it !)
                so :
                    1) equivalentclasses.add(A.name , A.name) --> ok, reflexivity is added
                    2) B is added to the set of equivalents of A --> (no effect because it was already here, but ok)
                    3) for every equivalents C of the set of A do the same with (A and C)
                        but inside we find B !!!
                        so :
                            without memory, it loops infinitly
                            but with memory, it finds that B have already been handled
    """
    if not memory:
        memory = set()

    if equivalent not in memory:
        memory.add(equivalent)

        equivalentclasses.add((cls.name, equivalent.name))
        cls.equivalents.add(equivalent)  # update classes

        # reflexive property :
        equivalent.equivalents.add(cls)  # update classes

        # transitive property :
        for e in frozenset(equivalent.equivalents):
            addequivalent(equivalentclasses, cls, e, memory)


def addsubclassof(subclassof, scls, cls):
    """
    propagation of inclusions :
    ---------------------------
        the inclusions are just transitives : (A in B) and (B in C) then (A in C)

    we need to update the classes to make the additions available for the other calls of inheritance functions
    """
    subclassof.add((scls.name, cls.name))
    # update classes
    cls.children.add(scls)
    scls.parents.add(cls)

    # transitivity :
    for p in frozenset(cls.parents):
        addsubclassof(subclassof, scls, p)


def addoverclassof(subclassof, cls, ocls):
    """
    back-track propagation of inclusion :
    -------------------------------------
        this backtracking seems to be unusful (it does the same thing than in addsubclassof)
        but is used after adding equivalences :
        indeed, the property " (A = B) and (C in A) then (C in B) " cannot be taken in account by the method addsubclassof

    we need to update the classes to make the additions available for the other calls of inheritance functions
    """

    subclassof.add((cls.name, ocls.name))
    # update classes :
    ocls.children.add(cls)
    cls.parents.add(ocls)

    # transitivity :
    for c in frozenset(cls.children):
        addoverclassof(subclassof, c, cls)


def addinstance(rdftype, instance, cls):
    """
    propagation of instances :
    ---------------------------
        the instances are just transitives : (A in B) and (B in C) then (A in C)

    don't need to update the classes because they are not used by the other functions
    """

    rdftype.add((instance, cls.name))

    for p in cls.parents:
        addinstance(rdftype, instance, p)


# ----------------------------------------


class SQLiteSimpleRDFSReasoner:

    SYMMETRIC_PREDICATES = {"owl:differentFrom", "owl:sameAs", "owl:disjointWith"}

    def __init__(self, database="kb.db"):
        self.db = sqlite3.connect(":memory:")  # create a memory database
        self.shareddb = sqlite3.connect(database)

        # create the tables
        # taken from http://stackoverflow.com/questions/4019081
        query = None
        for line in self.shareddb.iterdump():
            if "triples" in line:
                query = line
                break
        self.db.executescript(query)

        self.running = True
        logger.info(
            "Reasoner (simple RDFS) started. Classification running at %sHz"
            % REASONER_RATE
        )

    ####################################################################
    ####################################################################
    def classify(self):

        starttime = time.time()
        ok = self.copydb()
        if not ok:
            logger.info(
                "The reasoner couldn't copy the fact database. Probably"
                "due to the database being cleared. Skipping"
                "classification."
            )
            return

        models = self.get_models()
        newstmts = []

        for model in models:
            rdftype, subclassof, equivalentclasses = self.get_missing_taxonomy_stmts(
                model
            )

            newstmts += [(i, "rdf:type", c, model) for i, c in rdftype]
            newstmts += [(cc, "rdfs:subClassOf", cp, model) for cc, cp in subclassof]
            newstmts += [
                (eq1, "owl:equivalentClass", eq2, model)
                for eq1, eq2 in equivalentclasses
            ]

            newstmts += self.symmetric_statements(model)

        if newstmts:
            logger.debug(
                "Reasoner added new statements to the knowledge base:\n -"
                + "\n - ".join(["%s %s %s (in %s)" % stmt for stmt in newstmts])
            )

            self.update_shared_db(newstmts)

            logger.info("Classification took %fsec." % (time.time() - starttime))

    def get_models(self):
        with self.db:
            return [
                row[0] for row in self.db.execute("SELECT DISTINCT model FROM triples")
            ]

    def get_onto(self, db, model=DEFAULT_MODEL):

        onto = {}

        rdftype = None
        subclassof = None
        equivalentclasses = None
        with db:
            rdftype = {
                (row[0], row[1])
                for row in db.execute(
                    """SELECT subject, object FROM triples 
                       WHERE (predicate='rdf:type' AND model=?)
                    """,
                    [model],
                )
            }
            subclassof = {
                (row[0], row[1])
                for row in db.execute(
                    """SELECT subject, object FROM triples 
                       WHERE (predicate='rdfs:subClassOf' AND model=?)
                    """,
                    [model],
                )
            }
            equivalentclasses = {
                (row[0], row[1])
                for row in db.execute(
                    """SELECT subject, object FROM triples 
                       WHERE (predicate='owl:equivalentClass' AND model=?)
                    """,
                    [model],
                )
            }

        for cc, cp in subclassof:
            parent = onto.setdefault(cp, OntoClass(cp))
            child = onto.setdefault(cc, OntoClass(cc))
            child.parents.add(parent)
            parent.children.add(child)

        for i, c in rdftype:
            onto.setdefault(c, OntoClass(c)).instances.add(i)

        for ec1, ec2 in equivalentclasses:
            equi1 = onto.setdefault(ec1, OntoClass(ec1))
            equi2 = onto.setdefault(ec2, OntoClass(ec2))
            equi1.equivalents.add(equi2)
            equi2.equivalents.add(equi1)

        return onto, rdftype, subclassof, equivalentclasses

    def get_missing_taxonomy_stmts(self, model=DEFAULT_MODEL):

        onto, rdftype, subclassof, equivalentclasses = self.get_onto(self.db, model)

        newrdftype = set()
        newsubclassof = set()
        newequivalentclasses = set()

        for name, cls in onto.items():
            for p in frozenset(cls.parents):
                addsubclassof(newsubclassof, cls, p)
            for i in cls.instances:
                addinstance(newrdftype, i, cls)

            for equivalent in frozenset(cls.equivalents):
                addequivalent(newequivalentclasses, cls, equivalent)

            for equivalent in cls.equivalents:
                for p in frozenset(cls.parents):
                    addsubclassof(newsubclassof, equivalent, p)
                for c in frozenset(cls.children):
                    addoverclassof(newsubclassof, c, equivalent)
                for i in cls.instances:
                    addinstance(newrdftype, i, equivalent)

        newrdftype -= rdftype
        newsubclassof -= subclassof
        newequivalentclasses -= equivalentclasses
        return newrdftype, newsubclassof, newequivalentclasses

    def symmetric_statements(self, model):

        with self.db:
            stmts = {
                (row[0], row[1], row[2], model)
                for row in self.db.execute(
                    """SELECT subject, predicate, object FROM triples 
                        WHERE (predicate IN ('%s') AND model=?)
                        """
                    % "', '".join(self.SYMMETRIC_PREDICATES),
                    [model],
                )
            }

        return {
            (o, p, s, m) for s, p, o, m in stmts
        } - stmts  # so we keep only the new symmetrical statements

    ######################################################################
    ######################################################################
    def copydb(self):
        """Tried several other options (with ATTACH DATABASE -> that would likely lock the shared database as well, with iterdump, we miss the 'OR IGNORE')"""
        try:
            res = self.shareddb.execute("SELECT * FROM triples")
            with self.db:
                self.db.execute("DELETE FROM triples")
                self.db.executemany(
                    """INSERT INTO triples
                                VALUES (?, ?, ?, ?, ?, ?, ?, ?)""",
                    res,
                )
            return True
        except sqlite3.OperationalError:
            # can happen if the main application is in the middle of clearing the
            # database (ie, DROP triples)
            return False

    def update_shared_db(self, stmts):

        logger.debug("Reasoner added %s new statements: %s" % (len(stmts), stmts))

        timestamp = datetime.datetime.now().isoformat()
        stmts = [
            [sqlhash(s, p, o, model), s, p, o, model, timestamp]
            for s, p, o, model in stmts
        ]

        with self.shareddb:
            self.shareddb.executemany(
                """INSERT OR IGNORE INTO triples
                     (hash, subject, predicate, object, model, timestamp, inferred)
                     VALUES (?, ?, ?, ?, ?, ?, 1)""",
                stmts,
            )

    def __call__(self, *args):

        try:
            while self.running:
                time.sleep(1.0 / REASONER_RATE)
                self.classify()
        except KeyboardInterrupt:
            return


reasoner = None


def start_reasoner(db):
    global reasoner

    if not reasoner:
        reasoner = SQLiteSimpleRDFSReasoner()
    reasoner.running = True
    reasoner()


def stop_reasoner():

    if reasoner:
        reasoner.running = False
