import logging; logger = logging.getLogger("minimalKB."+__name__);
DEBUG_LEVEL=logging.DEBUG

import time
import datetime
import sqlite3

from minimalkb.backends.sqlite import sqlhash
from minimalkb.kb import DEFAULT_MODEL

REASONER_RATE = 5 #Hz

class OntoClass:
    def __init__(self, name):
        self.name = name
        self.parents = set()
        self.children = set()
        self.instances = set()
        self.equivalents = set()

    def __repr__(self):
        return self.name + \
               "\n\tParents: " + str(self.parents) + \
               "\n\tChildren: " + str(self.children) + \
               "\n\tInstances: " + str(self.instances)

class SQLiteSimpleRDFSReasoner:

    SYMMETRIC_PREDICATES = {"owl:differentFrom", "owl:sameAs", "owl:disjointWith"}

    def __init__(self, database = "kb.db"):
        self.db = sqlite3.connect(':memory:') # create a memory database
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
        logger.info("Reasoner (simple RDFS) started. Classification running at %sHz" % REASONER_RATE)

    ####################################################################
    ####################################################################
    def classify(self):


        starttime = time.time()
        ok = self.copydb()
        if not ok:
            logger.info("The reasoner couldn't copy the fact database. Probably"
                    "due to the database being cleared. Skipping"
                    "classification.")
            return

        models = self.get_models()
        newstmts = []

        for model in models:
            rdftype, subclassof = self.get_missing_taxonomy_stmts(model)

            newstmts += [(i, "rdf:type", c, model) for i,c in rdftype]
            newstmts += [(cc, "rdfs:subClassOf", cp, model) for cc,cp in subclassof]

            newstmts += self.symmetric_statements(model)


        if newstmts:
            logger.debug("Reasoner added new statements to the knowledge base:\n -" +\
                         "\n - ".join(["%s %s %s (in %s)" % stmt for stmt in newstmts]))

            self.update_shared_db(newstmts)

            logger.info("Classification took %fsec." % (time.time() - starttime))

    def get_models(self):
        with self.db:
            return [row[0] for row in self.db.execute("SELECT DISTINCT model FROM triples")]

    def get_onto(self, db, model = DEFAULT_MODEL):

        onto = {}

        rdftype = None
        subclassof = None
        equivalentclasses = None
        with db:
            rdftype = {(row[0], row[1]) for row in db.execute(
                    '''SELECT subject, object FROM triples 
                       WHERE (predicate='rdf:type' AND model=?)
                    ''', [model])}
            subclassof = {(row[0], row[1]) for row in db.execute(
                    '''SELECT subject, object FROM triples 
                       WHERE (predicate='rdfs:subClassOf' AND model=?)
                    ''', [model])}
            equivalentclasses = {(row[0], row[1]) for row in db.execute(
                    '''SELECT subject, object FROM triples 
                       WHERE (predicate='owl:equivalentClass' AND model=?)
                    ''', [model])}


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


        return onto, rdftype, subclassof

    def get_missing_taxonomy_stmts(self, model = DEFAULT_MODEL):

        onto, rdftype, subclassof = self.get_onto(self.db, model)

        newrdftype = set()
        newsubclassof = set()

        def addinstance(instance, cls):
            newrdftype.add((instance, cls.name))
            for p in cls.parents:
                addinstance(instance, p)

        def addsubclassof(scls, cls):
            newsubclassof.add((scls.name, cls.name))
            for p in cls.parents:
                addsubclassof(scls, p)

        for name, cls in onto.items():
            for i in cls.instances:
                addinstance(i, cls)
            for p in cls.parents:
                addsubclassof(cls, p)

            for equivalent in cls.equivalents:
                for i in cls.instances:
                    addinstance(i, equivalent)
                for p in cls.parents:
                    addsubclassof(equivalent, p)


        newrdftype -= rdftype
        newsubclassof -= subclassof
        return newrdftype, newsubclassof

    def symmetric_statements(self, model):


        with self.db:
            stmts = {(row[0], row[1], row[2], model) for row in self.db.execute(
                    '''SELECT subject, predicate, object FROM triples 
                        WHERE (predicate IN ('%s') AND model=?)
                        ''' % "', '".join(self.SYMMETRIC_PREDICATES), [model])}

        return {(o, p, s, m) for s, p, o, m in stmts} - stmts # so we keep only the new symmetrical statements




    ######################################################################
    ######################################################################
    def copydb(self):
        """ Tried several other options (with ATTACH DATABASE -> that would likely lock the shared database as well, with iterdump, we miss the 'OR IGNORE')
        """
        try:
            res = self.shareddb.execute("SELECT * FROM triples")
            with self.db:
                self.db.execute("DELETE FROM triples")
                self.db.executemany('''INSERT INTO triples
                                VALUES (?, ?, ?, ?, ?, ?, ?, ?)''',
                                res)
            return True
        except sqlite3.OperationalError: 
            # can happen if the main application is in the middle of clearing the
            # database (ie, DROP triples)
            return False

    def update_shared_db(self, stmts):

        logger.debug("Reasoner added %s new statements: %s" % (len(stmts), stmts))

        timestamp = datetime.datetime.now().isoformat()
        stmts = [[sqlhash(s,p,o,model), s, p, o, model, timestamp] for s,p,o,model in stmts]

        with self.shareddb:
            self.shareddb.executemany('''INSERT OR IGNORE INTO triples
                     (hash, subject, predicate, object, model, timestamp, inferred)
                     VALUES (?, ?, ?, ?, ?, ?, 1)''', stmts)


    def __call__(self, *args):

        try:
            while self.running:
                time.sleep(1./REASONER_RATE)
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

