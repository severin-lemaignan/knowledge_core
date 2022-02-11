import logging

logger = logging.getLogger("minimalKB." + __name__)
DEBUG_LEVEL = logging.DEBUG

import hashlib

import datetime
import sqlite3

from .sqlite_queries import query, simplequery, matchingstmt
from minimalkb.kb import DEFAULT_MODEL
from minimalkb.helpers import memoize

TRIPLETABLENAME = "triples"
TRIPLETABLE = """CREATE TABLE IF NOT EXISTS %s
                    ("hash" TEXT PRIMARY KEY NOT NULL  UNIQUE , 
                    "subject" TEXT NOT NULL , 
                    "predicate" TEXT NOT NULL , 
                    "object" TEXT NOT NULL , 
                    "model" TEXT NOT NULL ,
                    "timestamp" DATETIME DEFAULT CURRENT_TIMESTAMP NOT NULL ,
                    "expires" DATETIME ,
                    "inferred" BOOLEAN DEFAULT 0 NOT NULL)"""


def sqlhash(s, p, o, model):
    return hashlib.md5(("%s%s%s%s" % (s, p, o, model)).encode("utf-8")).hexdigest()


class SQLStore:
    def __init__(self):
        self.conn = sqlite3.connect("kb.db")
        self.create_kb()

        self._functionalproperties = frozenset()

    def create_kb(self):

        with self.conn:
            self.conn.execute(TRIPLETABLE % TRIPLETABLENAME)

    def clear(self):
        with self.conn:
            self.conn.execute("DROP TABLE %s" % TRIPLETABLENAME)

        self.create_kb()
        self.clear_cache()

        self.onupdate()

    def clear_cache(self):
        """Clears all the memoize results (ie, clears the cache of all methods
        using the decorator @memoize)
        """
        for obj in [getattr(self, name) for name in dir(self)]:
            if hasattr(obj, "cache"):
                obj.cache.clear()

    def add(self, stmts, model=DEFAULT_MODEL, lifespan=0, replace=False):

        timestamp = datetime.datetime.now()
        expires = None
        if lifespan > 0:
            expires = (timestamp + datetime.timedelta(seconds=lifespan)).isoformat()

        timestamp = timestamp.isoformat()

        if replace:
            with self.conn:
                self.conn.executemany(
                    "DELETE FROM %s WHERE subject=? AND predicate=? AND model=?"
                    % TRIPLETABLENAME,
                    [(s, p, model) for s, p, o in stmts],
                )

        if expires:
            stmts = [
                [sqlhash(s, p, o, model), s, p, o, model, timestamp, expires]
                for s, p, o in stmts
            ]
        else:
            stmts = [
                [sqlhash(s, p, o, model), s, p, o, model, timestamp]
                for s, p, o in stmts
            ]
        with self.conn:
            try:
                if expires:
                    self.conn.executemany(
                        """INSERT OR IGNORE INTO %s
                            (hash, subject, predicate, object, model, timestamp, expires)
                            VALUES (?, ?, ?, ?, ?, ?, ?)"""
                        % TRIPLETABLENAME,
                        stmts,
                    )
                else:
                    self.conn.executemany(
                        """INSERT OR IGNORE INTO %s
                            (hash, subject, predicate, object, model, timestamp)
                            VALUES (?, ?, ?, ?, ?, ?)"""
                        % TRIPLETABLENAME,
                        stmts,
                    )
            except sqlite3.InterfaceError as err:
                logger.error(
                    "Interface error while inserting statements in the SQLite backend.\nError: %s\nStatements: %s"
                    % (err, stmts)
                )
                for s in stmts:
                    logger.warn("Re-try to insert statement %s..." % s)
                    self.conn.executemany(
                        """INSERT OR IGNORE INTO %s
                            (hash, subject, predicate, object, model, timestamp)
                            VALUES (?, ?, ?, ?, ?, ?)"""
                        % TRIPLETABLENAME,
                        [s],
                    )
            except sqlite3.IntegrityError as err:
                logger.error(
                    "Integrity error while inserting statements in the SQLite backend.\nError: %s\nStatements: %s"
                    % (err, stmts)
                )

        self.onupdate()

    def delete(self, stmts, model=DEFAULT_MODEL):

        hashes = [[sqlhash(s, p, o, model)] for s, p, o in stmts]

        with self.conn:
            # removal is non-monotonic. Remove all inferred statements
            self.conn.execute("DELETE FROM %s WHERE inferred=1" % TRIPLETABLENAME)

            self.conn.executemany(
                """DELETE FROM %s 
                        WHERE (hash=?)"""
                % TRIPLETABLENAME,
                hashes,
            )

        self.onupdate()

    def update(self, stmts, model=DEFAULT_MODEL, lifespan=0):

        stmts_to_add = []
        stmts_to_replace = []
        for stmt in stmts:
            s, p, o = stmt
            if p in self._functionalproperties:
                stmts_to_replace.append(stmt)
            else:
                stmts_to_add.append(stmt)

        if stmts_to_add:
            self.add(stmts_to_add, model, lifespan)
        if stmts_to_replace:
            logger.debug("Updating functional values: %s" % stmts_to_replace)
            self.add(stmts_to_replace, model, lifespan, replace=True)

    def about(self, resource, models):

        params = {"res": resource}

        # workaround to feed a variable number of models
        models = list(models)
        for i in range(len(models)):
            params["m%s" % i] = models[i]

        query = """
                SELECT subject, predicate, object 
                FROM %s
                WHERE ((subject=:res OR predicate=:res OR object IN ('"%s"',:res))
                       AND model IN (%s))""" % (
            TRIPLETABLENAME,
            resource,
            ",".join([":m%s" % i for i in range(len(models))]),
        )

        with self.conn:
            res = self.conn.execute(query, params)
            return [[row[0], row[1], row[2]] for row in res]

    def has(self, stmts, models):

        candidates = set()
        for s in stmts:
            if not candidates:
                candidates = set(matchingstmt(self.conn, s, models))
            else:
                candidates &= set(matchingstmt(self.conn, s, models))

        return len(candidates) > 0

    def query(self, vars, patterns, models):
        return query(self.conn, vars, patterns, models)

    @memoize
    def label(self, concept, models=[]):
        labels = simplequery(self.conn, (concept, "rdfs:label", "?label"), models)
        if labels:
            return labels.pop()
        else:
            return concept

    @memoize
    def typeof(self, concept, models):
        classes = self.classesof(concept, False, models)
        if classes:
            if "owl:ObjectProperty" in classes:
                return "object_property"
            elif "owl:DatatypeProperty" in classes:
                return "datatype_property"
            elif "owl:Class" in classes:
                return "class"
            elif self.is_literal(concept):
                return "literal"
            else:
                return "instance"
        if (
            self.instancesof(concept, False, models)
            or self.subclassesof(concept, False, models)
            or self.superclassesof(concept, False, models)
        ):
            return "class"

        stmts_if_predicate = matchingstmt(self.conn, ("?s", concept, "?o"), models)
        if stmts_if_predicate:
            if self.is_literal(stmts_if_predicate[0][3]):
                return "datatype_property"
            else:
                return "object_property"

        logger.warn("Concept <%s> has undecidable type." % concept)
        return "undecided"

    def classesof(self, concept, direct, models=[]):
        if direct:
            logger.warn("Direct classes are assumed to be the asserted is-a relations")
            return list(
                simplequery(
                    self.conn,
                    (concept, "rdf:type", "?class"),
                    models,
                    assertedonly=True,
                )
            )
        return list(simplequery(self.conn, (concept, "rdf:type", "?class"), models))

    def instancesof(self, concept, direct, models=[]):
        if direct:
            logger.warn(
                "Direct instances are assumed to be the asserted is-a relations"
            )
            return list(
                simplequery(
                    self.conn,
                    ("?instances", "rdf:type", concept),
                    models,
                    assertedonly=True,
                )
            )
        return list(simplequery(self.conn, ("?instances", "rdf:type", concept), models))

    def superclassesof(self, concept, direct, models=[]):
        if direct:
            logger.warn(
                "Direct super-classes are assumed to be the asserted subClassOf relations"
            )
            return list(
                simplequery(
                    self.conn,
                    (concept, "rdfs:subClassOf", "?superclass"),
                    models,
                    assertedonly=True,
                )
            )
        return list(
            simplequery(self.conn, (concept, "rdfs:subClassOf", "?superclass"), models)
        )

    def subclassesof(self, concept, direct, models=[]):
        if direct:
            logger.warn(
                "Direct sub-classes are assumed to be the asserted subClassOf relations"
            )
            return list(
                simplequery(
                    self.conn,
                    ("?subclass", "rdfs:subClassOf", concept),
                    models,
                    assertedonly=True,
                )
            )
        return list(
            simplequery(self.conn, ("?subclass", "rdfs:subClassOf", concept), models)
        )

    ###################################################################################

    def onupdate(self):
        self._functionalproperties = frozenset(
            self.instancesof("owl:FunctionalProperty", False)
        )

    def has_stmt(self, pattern, models):
        """Returns True if the given statment exist in
        *any* of the provided models.
        """

        s, p, o = pattern
        query = "SELECT hash FROM %s WHERE hash=?" % TRIPLETABLENAME
        for m in models:
            if self.conn.execute(query, (sqlhash(s, p, o, m),)).fetchone():
                return True

        return False

    @memoize
    def is_literal(self, atom):
        """The definition of a literal follows the Turtle grammar:
        http://www.w3.org/TeamSubmission/turtle/#literal
        """
        if atom in ["true", "false"]:  # only lower-case!
            return True
        if atom[0] in ['"', "'"] and atom[-1] in ['"', "'"]:
            return True
        try:
            float(atom)  # test for integer, double, decimal
            return True
        except ValueError:
            pass

        if "@" in atom:  # langague tag
            return True

        if "^^" in atom:  # covers all XSD datatypes in Turtle syntax
            return True

        return False


def get_vars(s):
    return [x for x in s if x.startswith("?")]


def nb_variables(s):
    return len(get_vars(s))
