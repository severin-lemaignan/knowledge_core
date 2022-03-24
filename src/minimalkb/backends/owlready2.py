import logging

logger = logging.getLogger("minimalKB." + __name__)
DEBUG_LEVEL = logging.DEBUG

import datetime

import owlready2
from owlready2.sparql.main import Translator

DEFAULT_MODEL = "default"
from minimalkb.helpers import memoize

BASE_IRI = "http://www.pal-robotics.com/kb/"

PREFIXES = {
    "oro": "http://kb.openrobots.org#",
    "cyc": "http://sw.opencyc.org/concept/",
    "rdf": "http://www.w3.org/1999/02/22-rdf-syntax-ns#",
    "rdfs": "http://www.w3.org/2000/01/rdf-schema#",
    "owl": "http://www.w3.org/2002/07/owl#",
    "xsd": "http://www.w3.org/2001/XMLSchema#",
}


def model_iri(model):
    return BASE_IRI + model + "#"


class OwlReady2Store:
    def __init__(self):

        self.ontologies = {}
        self.initialize_model(DEFAULT_MODEL)

    def initialize_model(self, model):
        # world = owlready2.World(filename=model + ".sqlite3")
        world = owlready2.World()  # creates in-memory sqlite database
        self.ontologies[model] = world.get_ontology(model_iri(model))

        # self.add(
        #    ["owl:Thing rdf:type owl:Class", "owl:Nothing rdf:type owl:Class"], model
        # )

        self.clear(models=[model])

    def expand_prefix(self, entity):

        if not ":" in entity:
            return entity

        prefix, entity = entity.split(":")

        return (
            Translator(self.ontologies[DEFAULT_MODEL].world).expand_prefix(prefix)
            + entity
        )

    def abbr_resource(self, res):
        w = self.ontologies[DEFAULT_MODEL].world
        return w._abbreviate(res)

    def abbr_stmt(self, stmt):

        return (self.abbr_resource(r) for r in stmt)

    def format_stmt(self, s, p, o):

        w = self.ontologies[DEFAULT_MODEL].world

        return w._unabbreviate(s), w._unabbreviate(p), w._unabbreviate(o)

    def fix_prefixes(self, stmt):
        s, p, o = stmt

        return (
            self.fix_prefix(s),
            self.fix_prefix(p),
            self.fix_prefix(o) if not self.is_literal(o) else self.encode_literal(o),
        )

    def fix_prefix(self, res):

        # variable?
        if res.startswith("?"):
            return res

        # TODO: this code is *incorrect* for FQN IRI!! (it would add an incorrect ':' prefix)
        return ":" + res if ":" not in res else res

    def to_iri(self, r, expand_storid=True):
        if hasattr(r, "iri"):
            return r.iri
        if expand_storid and type(r) == int:  # OwlReady2 internal storid -> expand it
            return self.ontologies[DEFAULT_MODEL].world._unabbreviate(r)
        return r

    def replace_prefix(self, iri):

        if iri.startswith(BASE_IRI):
            return iri.split("#")[-1]

        for p, p_iri in PREFIXES.items():
            s = iri.split(p_iri)
            if len(s) == 2:
                return p + ":" + s[1]

        return iri

    def format_sparql_result(self, raw):

        # if the SPARQL request updates the model (eg 'INSERT' or 'DELETE'),
        # owlready2 returns the number of statements updated.
        if type(raw) == int:
            logger.debug("%s statements updated" % raw)
            return []

        res = []
        for row in raw:
            clean_row = []
            for r in row:
                iri = self.to_iri(r)

                if type(iri) == str:
                    clean_row.append(self.replace_prefix(iri))
                else:
                    clean_row.append(str(iri).lower())

            res.append(clean_row)

        return res

    def reset(self):

        self.clear_cache()

        for model in self.ontologies.keys():
            self.ontologies[model].destroy()

        self.ontologies = {}
        self.initialize_model(DEFAULT_MODEL)

    def clear(self, models=None):

        # clear everything
        if models is None:
            self.reset()

        else:
            for model in models:
                self._sparql(model, "DELETE { ?s ?p ?o . } WHERE { ?s ?p ?o . }")

        self.onupdate()

    def clear_cache(self):
        """Clears all the memoize results (ie, clears the cache of all methods
        using the decorator @memoize)
        """
        for obj in [getattr(self, name) for name in dir(self)]:
            if hasattr(obj, "cache"):
                obj.cache.clear()

    def _sparql(self, model, query, params=()):
        q = "BASE <%s>\n" % (model_iri(model))
        q += "PREFIX : <%s>\n" % (model_iri(model))
        for p, iri in PREFIXES.items():
            q += "PREFIX %s: <%s>\n" % (p, iri)

        # for model updates, OwlReady2 requires the target model to be
        # specified with 'WITH'
        if query.strip().startswith("INSERT") or query.strip().startswith("DELETE"):
            q += "WITH <%s>\n" % (model_iri(model))

        q += query

        logger.debug("Executing SPARQL query in model: %s\n%s" % (model, q))

        return self.ontologies[model].world.sparql(
            q, params=params, error_on_undefined_entities=False
        )

    def add(self, stmts, model=DEFAULT_MODEL, lifespan=0, replace=False):

        if not model in self.ontologies:
            self.initialize_model(model)

        q = "INSERT {\n"
        for stmt in stmts:
            s, p, o = self.fix_prefixes(stmt)
            q += "%s %s %s .\n" % (s, p, o)
        q += "} WHERE {}"

        self._sparql(model, q)

        #
        #        timestamp = datetime.datetime.now()
        #        expires = None
        #        if lifespan > 0:
        #            expires = (timestamp + datetime.timedelta(seconds=lifespan)).isoformat()
        #
        #        timestamp = timestamp.isoformat()
        #
        #        if replace:
        #            raise NotImplementedError("not implemented")
        #
        #        if expires:
        #            raise NotImplementedError("not implemented")
        #
        #        for stmt in stmts:
        #            s, p, o = self.abbr_stmt(stmt)
        #            self.ontologies[model]._add_obj_triple_spo(s, p, o)
        #
        self.onupdate()

    def delete(self, stmts, model=DEFAULT_MODEL):

        q = "DELETE {\n"
        for stmt in stmts:
            s, p, o = self.fix_prefixes(stmt)
            q += "%s %s %s .\n" % (s, p, o)
        q += "} WHERE {}"

        self._sparql(model, q)

        self.onupdate()

    def update(self, stmts, model=DEFAULT_MODEL, lifespan=0):

        return self.add(stmts, model, lifespan)

    def sparql(self, query, models):
        """Performs a SPARQL query on all given models and return the result.

        Standard prefixes and the default prefix of each model are added to the
        query, which is then executed 'as it'. As such, the namespaces of the
        resources used in the query need to be correct. In particular,
        resources belonging to the model's namespace (ie, all the custom
        entities created in this model) have to be prefixed by ':', as
        indicated in the SPARQL spec
        (https://www.w3.org/TR/sparql11-query/#sparqlSyntax).
        """

        result = []

        for model in models:
            if not model in self.ontologies:
                logger.warn("Trying to run a SPARQL query on a non-existent model!")
                return []

            result += self.format_sparql_result(self._sparql(model, query))

        return result

    def about(self, resource, models):
        """Returns all statements involving the resource."""

        is_literal = self.is_literal(resource)

        if not is_literal:
            res = self.fix_prefix(resource)

        result = []

        for model in models:
            if not model in self.ontologies:
                logger.warn("Trying to list statements from an non-existent model!")
                return []

            q = "SELECT ?s ?p WHERE { ?s ?p %s . }" % res

            for sp in self.format_sparql_result(self._sparql(model, q)):
                result.append([sp[0], sp[1], resource])

            if not is_literal:
                q = "SELECT ?s ?o WHERE { ?s %s ?o . }" % res

                for sp in self.format_sparql_result(self._sparql(model, q)):
                    result.append([sp[0], resource, sp[1]])

                q = "SELECT ?p ?o WHERE { %s ?p ?o . }" % res

                for sp in self.format_sparql_result(self._sparql(model, q)):
                    result.append( [resource, sp[0], sp[1]] )

        return result

    def has(self, stmts, models):

        for model in models:
            q = "SELECT ?s ?p ?o WHERE { ?s ?p ?o .\n"
            for p in stmts:
                s, p, o = self.fix_prefixes(p)
                q += "%s %s %s .\n" % (s, p, o)
            q += "}"

            if len(list(self._sparql(model, q))) > 0:
                return True

        return False

    def named_variables(self, vars):
        """
        Returns the list of variable that have been explicitely named (eg,
        not the 'stars' wildcards, that would have been replaced by anonymous
        variable starting with '__' by pykb)
        """

        return [v for v in vars if not v.startswith("?__")]

    def query(self, vars, patterns, models):

        res = []

        named_variables = self.named_variables(vars)
        # if we have named variable + extra 'anonymous' variable (coming from
        # eg '*'), ignore them
        if named_variables and len(named_variables) != len(vars):
            vars = named_variables

        for model in models:
            q = "SELECT %s WHERE {\n" % " ".join(vars)
            for p in patterns:
                s, p, o = self.fix_prefixes(p)
                q += "%s %s %s .\n" % (s, p, o)
            q += "}"

            if len(named_variables) > 1:
                res += [
                    dict(zip(vars, r))
                    for r in self.format_sparql_result(self._sparql(model, q))
                ]
            else:
                res += self.format_sparql_result(self._sparql(model, q))

        # if a single variable is requested, 'unpack' the result
        if len(vars) == 1:
            res = [r[0] for r in res]

        return res

    @memoize
    def label(self, concept, models=[]):
        raise NotImplementedError("not implemented")
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

        import pdb;pdb.set_trace()
        stmts_if_predicate = matchingstmt(self.conn, ("?s", concept, "?o"), models)
        if stmts_if_predicate:
            if self.is_literal(stmts_if_predicate[0][3]):
                return "datatype_property"
            else:
                return "object_property"

        logger.warn("Concept <%s> has undecidable type." % concept)
        return "undecided"

    def classesof(self, concept, direct, models=[]):
        res = []

        resource = self.fix_prefix(concept)

        for model in models:
            q = "SELECT ?c WHERE {\n"
            q += "%s rdf:type%s ?c .\n" % (resource, "" if direct else "*")
            q += "}"

            res += self.format_sparql_result(self._sparql(model, q))

        return res

    def instancesof(self, concept, direct, models=[]):
        res = []

        resource = self.fix_prefix(concept)

        for model in models:
            q = "SELECT ?i WHERE {\n"
            q += "?i rdf:type%s %s .\n" % (resource, "" if direct else "*")
            q += "}"

            res += self.format_sparql_result(self._sparql(model, q))

        return res

    def superclassesof(self, concept, direct, models=[]):
        res = []

        resource = self.fix_prefix(concept)

        for model in models:
            q = "SELECT ?c WHERE {\n"
            q += "%s rdfs:subClassOf%s ?c .\n" % (resource, "" if direct else "*")
            q += "}"

            res += self.format_sparql_result(self._sparql(model, q))

        return res

    def subclassesof(self, concept, direct, models=[]):
        res = []

        resource = self.fix_prefix(concept)

        for model in models:
            q = "SELECT ?c WHERE {\n"
            q += "?c rdfs:subClassOf%s %s .\n" % (resource, "" if direct else "*")
            q += "}"

            res += self.format_sparql_result(self._sparql(model, q))

        return res

    ###################################################################################

    def onupdate(self):
        pass
        # self._functionalproperties = frozenset(
        #    self.instancesof("owl:FunctionalProperty", False)
        # )

    @memoize
    def encode_literal(self, atom):
        """The definition of a literal follows the Turtle grammar:
        http://www.w3.org/TeamSubmission/turtle/#literal
        """
        if atom in ["true", "false"]:  # only lower-case!
            return '"%s"^^xsd:boolean' % atom
        if atom[0] in ['"', "'"] and atom[-1] in ['"', "'"]:
            return atom
        try:
            float(atom)  # test for integer, double, decimal
            return atom
        except ValueError:
            pass

        if "^^" in atom:  # covers all XSD datatypes in Turtle syntax
            return atom

        if "@" in atom:  # langague tag
            return atom

        return None

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

        if "@" in atom:  # language tag
            return True

        if "^^" in atom:  # covers all XSD datatypes in Turtle syntax
            return True

        return False

    def literal_to_python(self, literal):

        if not literal:
            return None

        if not self.is_literal(literal):
            return literal

        if literal == "true":
            return True
        if literal == "false":
            return False

        if "@" in literal:  # langague tag
            return literal

        # TODO: parsing of XSD datatype to python objects not done yet!!
        if "^^" in literal:  # covers all XSD datatypes in Turtle syntax
            return literal

        import ast

        return ast.literal_eval(literal)


def get_vars(s):
    return [x for x in s if x.startswith("?")]


def nb_variables(s):
    return len(get_vars(s))
