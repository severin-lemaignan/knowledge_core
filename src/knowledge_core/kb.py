from knowledge_core import __version__
from .exceptions import KbServerError
import hashlib
import traceback
from queue import Queue, Empty
import pathlib
from datetime import datetime
import time
from decimal import Decimal
import logging
import re

logger = logging.getLogger("KnowledgeCore." + __name__)

try:
    import rdflib
    from rdflib.util import date_time
    from rdflib.namespace import Namespace, RDF, RDFS, OWL, XSD
    from rdflib.term import Node, BNode, Literal, URIRef, Variable
    from rdflib import Graph, Dataset

    logger.info("Using RDFlib %s" % rdflib.__version__)
except ImportError:
    logger.error("RDFlib is required. Please install it.")
    import sys

    sys.exit(1)


has_reasoner = False
try:
    import reasonable

    has_reasoner = True
    logger.info(
        "reasonable OWL2 RL reasoner available. Running with reasoning enabled."
    )
except ImportError:
    logger.warning(
        "reasonable OWL2 RL reasoner not available. Install it with "
        "`pip install reasonable`. Running without reasoning."
    )


def stable_hash(s, p="", o="", model=""):
    return hashlib.md5(("%s%s%s%s" % (s, p, o, model)).encode("utf-8")).hexdigest()


DEFAULT_MODEL = "default"
REASONER_RATE = 5  # Hz
EXPIRED_STMTS_CHECK_RATE = 1  # Hz
ACTIVE_CONCEPT_LIFESPAN = 5  # sec

IRIS = {
    "oro": "http://kb.openrobots.org#",
    "cyc": "http://sw.opencyc.org/concept/",
    "rdf": "http://www.w3.org/1999/02/22-rdf-syntax-ns#",
    "rdfs": "http://www.w3.org/2000/01/rdf-schema#",
    "owl": "http://www.w3.org/2002/07/owl#",
    "xsd": "http://www.w3.org/2001/XMLSchema#",
    "dbr": "https://dbpedia.org/resource/",
    "dbo": "https://dbpedia.org/ontology/",
    "dbp": "https://dbpedia.org/property/",
    "prov": "http://www.w3.org/ns/prov#",
    "foaf": "http://xmlns.com/foaf/0.1/",
}

DEFAULT_PREFIX = "oro"

default_ns = Namespace(IRIS[DEFAULT_PREFIX])

SPARQL_PREFIXES = "".join(["PREFIX %s: <%s>\n" % (p, iri)
                          for p, iri in IRIS.items()])
SPARQL_PREFIXES += "PREFIX : <%s>\n" % IRIS[DEFAULT_PREFIX]
SPARQL_PREFIXES += "BASE <%s>\n" % IRIS[DEFAULT_PREFIX]


# reference: https://www.w3.org/TeamSubmission/n3/#syntax
N3_PROLOGUE = " ".join(["@prefix %s: <%s>." % (p, iri)
                       for p, iri in IRIS.items()])
N3_PROLOGUE += " @prefix : <%s>. " % IRIS[DEFAULT_PREFIX]
N3_PROLOGUE += "@base <%s>. " % IRIS[DEFAULT_PREFIX]
N3_PROLOGUE += (
    "@keywords a,true,false. "  # see https://www.w3.org/TeamSubmission/n3/#keywords
)

QUOTE_REGEX = re.compile(
    r'^(\".*\"|\'.*\'|\"\"\".*\"\"\"|\'\'\'.*\'\'\')$', re.DOTALL)


def ORO(term: str):
    return URIRef(IRIS[DEFAULT_PREFIX] + term)


EXPIRES_ON_TERM = ORO("expiresOn")


def api(fn):
    fn._api = True
    return fn


def compat(fn):
    fn._compat = True
    return fn


# @memoize
def parse_stmts_to_graph(stmts):

    data = N3_PROLOGUE

    for stmt in stmts:
        if " " not in stmt:
            raise KbServerError(
                "invalid syntax for statement %s: it should be formed of 3 terms."
                % stmt
            )
        stmt = " ".join([turtle_escape(t) for t in stmt.split()])
        data += " %s . " % stmt

    try:
        return Graph().parse(data=data, format="n3")
    except rdflib.plugins.parsers.notation3.BadSyntax:
        raise KbServerError("invalid syntax for statements %s" % stmts)


# @memoize
def parse_stmts(stmts):

    return list(parse_stmts_to_graph(stmts))


def turtle_escape(string):
    """
    Escapes special characters in a string to be used in a turtle statement.

    according to the turtle spec, the following characters needs to be escaped:
    ~.-!$&'()*+,;=/?#@%_

    https://www.w3.org/TR/turtle/#reserved

    """
    if bool(QUOTE_REGEX.fullmatch(string)):
        return string

    for c in "~.-!$&'()*+,;=/#%":  # exclude ? and @, as they might be legitimate in a term
        string = string.replace(c, "\\" + c)
    return string


# @memoize
def parse_stmt(stmt):

    logger.warning("Parsing statement: %s" % stmt)
    stmt = " ".join([turtle_escape(t) for t in stmt.split()])

    try:
        return list(Graph().parse(data=N3_PROLOGUE + "%s ." % stmt, format="n3"))[0]
    except rdflib.plugins.parsers.notation3.BadSyntax as bs:
        raise KbServerError(
            f"invalid syntax for statement <{stmt}>. Original error: {bs}")
    except IndexError as ie:
        raise KbServerError(
            f"invalid syntax for statement <{stmt}>. Original error: {ie}")
    except AttributeError as ae:
        raise KbServerError(
            f"invalid syntax for statement <{stmt}>. Original error: {ae}")


# @memoize
def parse_term(term):

    if type(term) in [bool, int, float]:
        return Literal(term)

    try:
        # TODO: correct, but not super effective!
        return list(
            Graph().parse(data=N3_PROLOGUE + " <s> <p> %s ." %
                          turtle_escape(term), format="n3")
        )[0][2]
    except rdflib.plugins.parsers.notation3.BadSyntax as bs:
        raise KbServerError(
            f"invalid syntax for term <{term}>. Original error: {bs}")


def shorten_term(graph, term):

    if isinstance(term, URIRef):
        return graph.qname(term)
    elif isinstance(term, Literal):
        return term.value()
    else:
        return term.n3()


def shorten(graph, stmt, double_quote_for_str=False):
    """
    Shorten a full-qualified URI reference to instead use prefixes.

    Optionally, ensure that strings are double-quoted, for further JSON serialization.
    """
    is_single_term = isinstance(stmt, str)

    if is_single_term:
        stmt = [stmt]

    res = []
    for t in stmt:
        if isinstance(t, URIRef):
            res.append(graph.qname(t))
        elif isinstance(t, Literal):
            val = t.toPython()

            # workaround: decimal.Decimal is not JSON-serializable -> convert it to a float
            if type(val) == Decimal:
                val = float(val)

            if double_quote_for_str and isinstance(val, str):
                val = f'"{val}"'
            res.append(val)
        else:
            res.append(t.n3())
    if is_single_term:
        return res[0]
    else:
        return res


def shortenN(graph, stmts):
    return [shorten(graph, s) for s in stmts]


def shorten_graph(graph, double_quote_for_str=False):
    """
    Return a list of s, p, o statements contained in a graph, with their URIs shorten.

    (eg, using prefixes when possible).
    """
    return [shorten(graph, s, double_quote_for_str) for s in graph.triples([None, None, None])]


def get_variables(stmt):
    return [v for v in stmt if isinstance(v, Variable)]


def get_all_variables(stmts):
    vars = []
    for p in stmts:
        vars += [v.n3() for v in get_variables(parse_stmt(p))]

    return vars


class dotdict(dict):
    """dot.notation access to dictionary attributes."""

    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__


class hashabledict(dict):
    def __hash__(self):
        return hash(tuple(sorted(self.items())))


class Event:
    def __init__(self, kb, patterns, one_shot, models):
        self.kb = kb

        self.patterns = patterns
        self.one_shot = one_shot

        self.models = models

        self.id = "evt_" + str(
            stable_hash(
                str(sorted(self.patterns)) +
                str(one_shot) + str(sorted(self.models))
            )
        )

        self.content = None

        self.vars = get_all_variables(self.patterns)

        self.valid = True

        self.previous_instances = set()

        logger.debug("Creating new event. Looking existing matchs...")
        instances = self.kb.find(
            self.patterns, self.vars, frozenset(self.models))
        logger.debug("Event created with initial instances %s" % instances)

        self.previous_instances = set([hashabledict(row) for row in instances])

    def __hash__(self):
        return hash(self.id)

    def __eq__(self, other):
        return hash(self) == hash(other)

    def __cmp__(self, other):
        return hash(self).__cmp__(hash(other))

    def evaluate(self):
        if not self.valid:
            return False

        if self.one_shot:
            self.valid = False

        instances = self.kb.find(
            self.patterns, self.vars, frozenset(self.models))

        instances = set([hashabledict(row) for row in instances])

        newinstances = instances - self.previous_instances

        # previous_instances must be set to the current set of matching instance
        # else we won't trigger events when an instance disappear and
        # re-appear later.
        self.previous_instances = instances

        if not newinstances:
            return False

        self.content = list(newinstances)
        return True


class KnowledgeCore:

    MEMORYPROFILE_DEFAULT = ""
    MEMORYPROFILE_SHORTTERM = "SHORTTERM"

    def __init__(self, filenames=None, enable_reasoner=True, auto_activeconcepts=False):
        """
        Create a new KnowledgeCore instance.

        :param filenames: a list of path to ontologies to pre-load in the
        knowledge base
        :param enable_reasoner: if True, enable automatic knowledge base
        materialisation using the 'reasonable' RDF/OWL reasoner
        :param auto_activeconcepts: if True, every subject of a statement added
        to the knowledge base is also marked as an instance of 'ActiveConcept'
        for ACTIVE_CONCEPT_LIFESPAN seconds (default to 5 sec)
        """
        self.reasoner_enabled = has_reasoner and enable_reasoner
        if not self.reasoner_enabled:
            logger.warn("Running without OWL2 RL reasoner.")

        self.auto_activeconcepts = auto_activeconcepts
        self.active_concepts = set()

        _api = [
            getattr(self, fn) for fn in dir(self) if hasattr(getattr(self, fn), "_api")
        ]
        import inspect

        self._api = {fn.__name__ +
                     str(inspect.signature(fn)): fn for fn in _api}

        apilist = [
            key + (" (compatibility)" if hasattr(val, "_compat") else "")
            for key, val in self._api.items()
        ]

        logger.debug(
            "Initializing the KnowledgeCore with the following API: \n\t- "
            + "\n\t- ".join(apilist)
        )

        self.incomingrequests = Queue()
        self.requestresults = {}

        self.active_evts = set()
        self.eventsubscriptions = {}

        self._last_expired_stmts_check = time.time()

        # create a RDFlib dataset
        self.ds = Dataset()
        self.models = {}
        self.create_model(DEFAULT_MODEL)

        self._functionalproperties = frozenset()

        if filenames:
            for filename in filenames:
                self.load(filename)

    @api
    def hello(self):
        return "KnowledgeCore, v.%s" % __version__

    @api
    def version(self):
        return str(__version__)

    @api
    def load(self, filename, models=None):

        models = self.normalize_models(models)
        for model in models:
            logger.info("Loading file <%s> in model <%s>" % (filename, model))
            self.models[model].graph.parse(
                filename, publicID=IRIS[DEFAULT_PREFIX])
            self.models[model].is_dirty = True

        self.onupdate()

    @api
    def save(self, path=None, basename="kb", models=None):

        path = pathlib.Path(path)
        models = self.normalize_models(models)
        for model in models:
            filename = path / (basename + "-" + model + ".rdf")
            logger.info("Saving knowledge base (model <%s>) to %s" %
                        (model, filename))
            self.models[model].graph.serialize(str(filename), format="xml")

    @api
    def clear(self):
        logger.warn("Clearing the knowledge base!")
        self.active_evts.clear()
        self.eventsubscriptions.clear()

        for m, g in self.models.items():
            self.ds.remove_graph(g.graph)

        self.create_model(DEFAULT_MODEL)

    @api
    def methods(self):
        return list(self._api.keys())

    @api
    def about(self, raw_term, models=None):
        """Return the list of triples where `term` is either subject, predicate or object."""
        result = []

        models = self.normalize_models(models)

        term = None
        try:
            term = parse_term(raw_term)
        except KbServerError:
            # error while parsing the raw term: probably a
            # string with spaces. In that case, only search in the labels.
            pass

        for model in models:
            g = self.models[model].materialized_graph

            if term:
                for triple in g.triples((term, None, None)):
                    result.append(shorten(g, triple))
                for triple in g.triples((None, term, None)):
                    result.append(shorten(g, triple))
                for triple in g.triples((None, None, term)):
                    result.append(shorten(g, triple))

            for s, p, o in g.triples((None, RDFS.label, None)):
                if raw_term in o.value:
                    result.append(shorten(g, (s, p, o)))

        return result

    @api
    def lookup(self, term, models=None):
        """
        Search the knowledge base for a term matching a string.

        The search is performed both on terms' names and on label.

        Returns the list of found terms, alongside with their type (one of
        instance, class, datatype_property, object_property, literal)
        """
        models = self.normalize_models(models)
        logger.info(
            "Lookup for "
            + str(term)
            + " in "
            + (str(models) if models else "default model.")
        )

        exact_match = set()
        approximate_match = set()

        for model in models:
            g = self.models[model].materialized_graph

            for s, p, o in g:
                ss, sp, so = [str(x) for x in shorten(g, [s, p, o])]

                if term == ss:
                    exact_match.add((ss, s))
                elif term.lower() in ss.lower():
                    approximate_match.add((ss, s))

                if term == sp:
                    exact_match.add((sp, p))
                elif term.lower() in sp.lower():
                    approximate_match.add((sp, p))

                if term == so:
                    # the term is the label of a node? add the node
                    if p == RDFS.label:
                        exact_match.add((ss, s))
                    else:  # otherwise, add the literal
                        exact_match.add((so, o))

                elif term.lower() in so.lower():
                    # the term is the label of a node? add the node
                    if p == RDFS.label:
                        approximate_match.add((ss, s))
                    else:  # otherwise, add the literal
                        approximate_match.add((so, o))

        exact_match = [(s, self.typeof(t, models)) for s, t in exact_match]
        approximate_match = [(s, self.typeof(t, models))
                             for s, t in approximate_match]
        if exact_match:
            logger.info("Found exact match: " + str(exact_match))
        if approximate_match:
            logger.info("Found approximate match: " + str(approximate_match))
        if not exact_match and not approximate_match:
            logger.info("No match found")

        return exact_match + approximate_match

    def _instancesof(self, term, direct=False, models=[]):

        models = self.normalize_models(models)
        result = []

        if not isinstance(term, Node):
            term = parse_term(term)

        for model in models:
            if direct:
                result += self.models[model].graph.subjects(RDF.type, term)
            else:
                result += self.models[model].materialized_graph.subjects(
                    RDF.type, term)

        return list(filter(lambda t: type(t) != BNode, result))

    @api
    def classesof(self, term, direct=False, models=[]):
        cls = self._classesof(term, direct, models)
        return [
            shorten_term(self.models[DEFAULT_MODEL].materialized_graph, c) for c in cls
        ]

    def _classesof(self, term, direct=False, models=[]):

        models = self.normalize_models(models)
        result = []

        if not isinstance(term, Node):
            term = parse_term(term)

        for model in models:
            if direct:
                result += self.models[model].graph.objects(term, RDF.type)
            else:
                result += self.models[model].materialized_graph.objects(
                    term, RDF.type)

        return list(filter(lambda t: type(t) != BNode, result))

    def _subclassesof(self, term, direct=False, models=[]):

        models = self.normalize_models(models)
        result = []

        if not isinstance(term, Node):
            term = parse_term(term)

        for model in models:
            if direct:
                result += self.models[model].graph.subjects(
                    RDFS.subClassOf, term)
            else:
                result += self.models[model].materialized_graph.subjects(
                    RDFS.subClassOf, term
                )

        return list(filter(lambda t: type(t) != BNode, result))

    def _superclassesof(self, term, direct=False, models=[]):

        models = self.normalize_models(models)
        result = []

        if not isinstance(term, Node):
            term = parse_term(term)

        for model in models:
            if direct:
                result += self.models[model].graph.objects(
                    term, RDFS.subClassOf)
            else:
                result += self.models[model].materialized_graph.objects(
                    term, RDFS.subClassOf
                )

        return list(filter(lambda t: type(t) != BNode, result))

    @api
    def label(self, term, models=[]):
        """
        Return the labels attached to a term, as a dictionary.

        Example return value:
         `{"default": "label1", "lang_code1": label1, "lang_code2": "label2", ...}`
        where the 'default' key returns either the English version of the
        label, or the name of the term, if no label is available, and the other
        keys provide localised version of the label, if available in the
        knowledge base.
        """
        models = self.normalize_models(models)
        result = {}

        if not isinstance(term, URIRef):
            term = parse_term(term)

        for model in models:

            labels = self.models[model].materialized_graph.objects(
                term, RDFS.label)
            for label in labels:
                if label.language:
                    result[label.language] = label
                else:
                    result["default"] = label

        if "default" not in result:
            if "en" in result:
                result["default"] = result["en"]
            else:
                result["default"] = shorten_term(
                    self.models[DEFAULT_MODEL].graph, term)

        return result

    def typeof(self, term, models):

        if not isinstance(term, Node):
            term = parse_term(term)

        if isinstance(term, Literal):
            return "literal"

        classes = self._classesof(term, False, models)
        if classes:
            if OWL.ObjectProperty in classes:
                return "object_property"
            elif OWL.DatatypeProperty in classes:
                return "datatype_property"
            elif OWL.Class in classes:
                return "class"
            else:
                return "instance"

        if (
            self._instancesof(term, False, models)
            or self._subclassesof(term, False, models)
            or self._superclassesof(term, False, models)
        ):
            return "class"

        for model in models:
            stmts_if_predicate = list(
                self.models[model].materialized_graph.triples(
                    [None, term, None])
            )

            if stmts_if_predicate:
                if isinstance(stmts_if_predicate[0][2], Literal):
                    return "datatype_property"
                else:
                    return "object_property"

        logger.warn("Concept <%s> has undecidable type." % term.n3())
        return "undecided"

    @api
    def details(self, raw_term, model=None):
        """
        Return a dictionary containing the details on a given resource.

        The following information is returned:
        - 'label': resource label, if any, literal value for literals, else resource ID.
        - 'id': resource ID or 'literal' for literals
        - 'type': one of ['instance', 'class', 'object_property', 'datatype_property', 'undecided']
        - 'sameAs': list of equivalent classes or instances, if any
        - 'attributes':
            - for classes, a list of three dictionaries:
                {"name": "Parents", "id": "superClasses", "values": [ids...]}
                {"name": "Children", "id": "subClasses", "values": [ids...]}
                {"name": "Instances", "id": "instances", "values": [ids...]}
                (only direct super/sub-classes and instances)
            - for instances, a list of one dictionary:
                {"name": "Classes", "id": "classes", "values": [ids...]}
                (only direct classes)
        - 'relations': a list of statements in which the term is one of
          subject, predicate, object (similar to about())
        """
        models = self.normalize_models(model)
        if len(models) != 1:
            raise KbServerError(
                "the details() method can only operate on a single model. Got: %s" % models)

        model = list(models)[0]

        try:
            term = parse_term(raw_term)
        except KbServerError:
            raise KbServerError("<%s> is an invalid n3 term." % raw_term)

        g = self.models[model].materialized_graph

        res = {}
        res["id"] = shorten_term(g, term)
        res["label"] = self.label(term, models)
        res["type"] = self.typeof(term, models)

        res["attributes"] = []
        if res["type"] == "class":
            res["attributes"].append(
                {
                    "name": "Parents",
                    "id": "superClasses",
                    "values": [
                        {"id": shorten_term(
                            g, r), "name": self.label(r, models)}
                        for r in self._superclassesof(term, True, models)
                    ],
                }
            )

            res["attributes"].append(
                {
                    "name": "Children",
                    "id": "subClasses",
                    "values": [
                        {"id": shorten_term(
                            g, r), "label": self.label(r, models)}
                        for r in self._subclassesof(term, True, models)
                    ],
                }
            )

            res["attributes"].append(
                {
                    "name": "Instances",
                    "id": "instances",
                    "values": [
                        {"id": shorten_term(
                            g, r), "label": self.label(r, models)}
                        for r in self._instancesof(term, True, models)
                    ],
                }
            )

        elif res["type"] == "instance":
            res["attributes"] = [
                {
                    "name": "Classes",
                    "id": "classes",
                    "values": [
                        {"id": shorten_term(
                            g, r), "label": self.label(r, models)}
                        for r in self._classesof(term, True, models)
                        if r not in [
                            ORO("ActiveConcept"),
                            OWL.NamedIndividual
                        ]
                    ],
                }
            ]

        stmts = []
        for triple in g.triples((term, None, None)):
            if self.is_fact(triple):
                stmts.append(shorten(g, triple))
        for triple in g.triples((None, term, None)):
            stmts.append(shorten(g, triple))
        for triple in g.triples((None, None, term)):
            if self.is_fact(triple):
                stmts.append(shorten(g, triple))
        res["relations"] = stmts

        return res

    def is_fact(self, triple):

        if not all(type(t) != BNode for t in triple):
            return False

        if triple[1] in [RDF.type,
                         RDFS.subClassOf,
                         RDFS.label,
                         ORO("openCycUri"),
                         OWL.disjointWith,
                         RDF.first,
                         RDFS.domain,
                         RDFS.range,
                         RDFS.comment,
                         ]:
            return False

        return True

    @api
    def exist(self, raw_stmts, models=None):
        """Check if all the statements exist -- eg, are materialised -- in all the given models."""
        models = self.normalize_models(models)

        logger.info(
            "Checking existence of "
            + str(raw_stmts)
            + " in "
            + (str(models) if models else "default model.")
        )
        stmts = [parse_stmt(s) for s in raw_stmts]

        vars = get_all_variables(raw_stmts)

        # no variable? simply check if all the statements are asserted
        if len(vars) == 0:
            for model in models:
                for stmt in stmts:
                    if stmt not in self.models[model].materialized_graph:
                        return False

            return True

        # else, run a query
        return bool(self.find(raw_stmts, vars, models))

    @api
    def revise(self, stmts, policy):
        """
        Revise (add, retract, update) statements in the knowledge base.

        Supported policy options:
        - method: string in [add, safe_add, retract, update, safe_update, revision]
        - models: list of strings
        - lifespan: duration before automatic removal of statements, in
          seconds, float
        """
        if type(policy) != dict:
            raise KbServerError("Expected a dictionary as policy")

        if isinstance(stmts, str):
            raise KbServerError("A list of statements is expected")

        if not policy["method"]:
            raise KbServerError(
                "No policy specified for method in revise. Expected one of update, add, retract")

        vars = get_all_variables(stmts)

        subgraph = parse_stmts_to_graph(stmts)
        parsed_stmts = "\n\t- ".join(
            [" ".join([str(t) for t in s])
             for s in shorten_graph(subgraph, double_quote_for_str=True)]
        )

        models = self.normalize_models(policy.get("models", []))

        if policy["method"] in ["update", "safe_update", "add", "safe_add", "revision"]:

            if len(vars) > 0:
                raise KbServerError(
                    "You can not add/update statements containing variables: %s" % stmts)

            if policy["method"].startswith("safe"):
                logger.warn(
                    "Warning: %s is not implemented. Performing %s instead."
                    % (policy["method"], policy["method"][5:])
                )
            if policy["method"].endswith("add"):
                logger.warn(
                    "Warning: %s is deprecated. Performing update instead."
                    % (policy["method"])
                )

            lifespan = policy.get("lifespan", 0)

            logger.info(
                "Updating "
                + str(list(models))
                + " with:\n\t- "
                + parsed_stmts
                + (" (lifespan: %ssec)" % lifespan if lifespan else "")
            )
            for model in models:
                for s, p, o in subgraph.triples([None, None, None]):

                    # special-case <subject rdf:type ActiveConcept>:
                    # when seeing this kind of statement, mark the subject as
                    # an active concept for ACTIVE_CONCEPT_LIFESPAN seconds.
                    #
                    # Note that if a lifespan was associated to this statement, and
                    # that lifespan is smaller than ACTIVE_CONCEPT_LIFESPAN,
                    # the given lifespan is used instead.
                    if p == RDF.type and o == ORO("ActiveConcept"):
                        self.mark_active_concept(s, model)

                    else:
                        if p in self._functionalproperties:
                            self.models[model].graph.set((s, p, o))
                        else:
                            self.models[model].graph.add((s, p, o))

                        if self.auto_activeconcepts:
                            self.mark_active_concept(s, model)

                if lifespan:

                    expiry_ts = time.time() + lifespan
                    expiry_date = datetime.fromtimestamp(
                        expiry_ts).strftime("%d/%m/%Y, %H:%M:%S")
                    expiry_date_xsd = date_time(expiry_ts)

                    # do we already have lifespan information for these facts?
                    # if so, we update
                    gr = [g for g in self.models[model].metadata.subjects()
                          if len(subgraph-g) == 0]
                    if gr:
                        gr = gr[0]
                        logger.info("Updating expiry date to %s" % expiry_date)
                    else:
                        gr = subgraph
                        logger.info(
                            "This statement will expire on %s" % expiry_date)

                    self.models[model].metadata.set(
                        (gr, EXPIRES_ON_TERM, Literal(expiry_date_xsd, datatype=XSD.dateTime)))

                self.models[model].is_dirty = True

        elif policy["method"] == "retract":

            if len(vars) > 0:

                if len(stmts) > 1:
                    raise KbServerError("Removing multiple statements that \
                            include variables or wildcards is not supported. \
                            Wildcards are only permitted with a single \
                            pattern.")

                # we've been provided with a pattern that contains at least one variable.
                # find() all matching values for the varaible(s), and reconstruct the final
                # list of statements to retract that match that pattern.

                res = self.find(stmts)

                for tok in stmts[0].split():
                    # if the token is *not* a variable, add it to the find() results 'as it'
                    if not tok.startswith("?"):
                        for e in res:
                            e[tok] = tok

                # tok_order is a copy of the pattern, without the leading '?'
                tok_order = [tok[1:] if tok.startswith(
                    "?") else tok for tok in stmts[0].split()]

                # final list of statments to remove
                new_stmts = []
                for e in res:
                    new_stmts.append(" ".join([str(e[tok_order[0]]), str(
                        e[tok_order[1]]), parse_term(e[tok_order[2]]).n3()]))

                subgraph = parse_stmts_to_graph(new_stmts)
                parsed_stmts = "\n\t- ".join(
                    [" ".join([str(t) for t in s])
                     for s in shorten_graph(subgraph)]
                )

            logger.info("Deleting from " + str(list(models)) +
                        ":\n\t- " + parsed_stmts)
            for model in models:
                self.models[model].graph -= subgraph
                self.models[model].is_dirty = True

        else:
            raise KbServerError(
                "Unknown method in revise: %s" % policy["method"])

        self.onupdate()

    def mark_active_concept(self, term: Node, model: str):
        expiry_date = date_time(time.time() + ACTIVE_CONCEPT_LIFESPAN)
        subgraph = Graph()
        for p, iri in IRIS.items():
            subgraph.bind(p, iri)
        subgraph.bind("", IRIS[DEFAULT_PREFIX])

        subgraph.add((term, RDF.type, ORO("ActiveConcept")))
        concept = shorten_term(subgraph, term)
        logger.info("Marking <%s> as ActiveConcept" % concept)
        self.active_concepts.add(concept)
        self.models[model].graph += subgraph
        self.models[model].metadata.add(
            (subgraph, EXPIRES_ON_TERM, Literal(expiry_date, datatype=XSD.dateTime)))

    @api
    @compat
    def add(self, stmts, models=None, lifespan=0):
        logger.warn(
            "Warning: <add> is deprecated. Performing <update> instead.")
        return self.revise(
            stmts, {"method": "update", "models": models, "lifespan": lifespan}
        )

    @api
    def remove(self, stmts, models=None):
        return self.revise(stmts, {"method": "retract", "models": models})

    @api
    def update(self, stmts, models=None, lifespan=0):
        return self.revise(
            stmts, {"method": "update", "models": models, "lifespan": lifespan}
        )

    @api
    def sparql(self, query, model=None):
        """
        Perform a raw SPARQL query on a given model ('default' by default).

        The SPARQL PREFIX and BASE are automatically added, no need to do it
        manually (even though you can if you want to use non-standard
        prefixes).

        Note that you are responsible for writing a syntactically corret SPARQL
        query. In particualar, all non-literal/non-variable terms must have a
        namespace (or a prefix).

        Results is returned as a JSON object that follow the standard JSON
        serialization of SPARQL Results
        (https://www.w3.org/TR/2013/REC-sparql11-results-json-20130321/)

        Example:
        -------
        ```
        >>> kb += ["myself age 40"]
        >>> kb.sparql("SELECT ?age WHERE { :myself :age ?age . }")
        {'results':
            {'bindings': [
                {'age':
                    {'type': 'typed-literal',
                     'value': '40',
                     'datatype': 'http://www.w3.org/2001/XMLSchema#integer'}
                }]
            },
          'head': {'vars': ['age']}
          'query': # original query
         }

        """
        models = self.normalize_models(model)
        if len(models) != 1:
            logger.error("sparql() can only be executed on a single model")
            return None

        model = list(models)[0]

        logger.info("Executing SPARQL query in model: %s\n%s" % (model, query))

        sparql_res, query = self._sparql(model, query, raw=True)

        import json

        res = json.loads(sparql_res.serialize(format="json"))
        res["query"] = query

        return res

    @api
    def find(self, patterns, vars=None, models=None):
        """
        Perform a query on one or several models.

        `patterns`: a list of triples containing unbound terms (eg, variables).
        As a convenience, if the patterns do not contain any variable, `find`
        will return whether the provided list of triples is present in the
        model(s).

        `vars`: the list of variables (prefixed with `?`) that you want to
        select. If None or an empty list, returns all the variables found in
        the patterns.

        `models`: the list of models you want to run the query against. The
        returned result will be the conjunction of results when running the
        query on each models separately.

        A list of dictionaries is returned with possible combination of values
        for the different variables. For instance, find(["?agent desires
        ?action", "?action rdf:type Jump"],["?agent","?action"]) would return
        something like: [{"agent": "james", "action": "jumpHigh"}, {"agent":
        "laurel", "action": "jumpHigher"}]

        If you were using anonymous variables (starting with ?__) in your query
        (eg generated by `pykb` when using a `*` wildcard), they will be
        renamed `var1`, `var2`, etc.

        Note that RDF blank nodes (BNode) are not returned.
        """
        if not vars or (len(vars) == 1 and vars[0] == ""):
            vars = get_all_variables(patterns)
            if len(vars) == 0:
                return patterns if self.exist(patterns, models) else []

        models = self.normalize_models(models)

        patterns = [parse_stmt(p) for p in patterns]

        parsed_patterns = "\n\t- ".join(
            [" ".join([str(t) for t in s]) for s in shortenN(
                self.models[DEFAULT_MODEL].graph, patterns)]
        )

        normalised_patterns = [
            " ".join([t.n3() for t in stmt]) for stmt in patterns]

        logger.info(
            "Searching "
            + str(vars)
            + " in models "
            + str(models)
            + " matching:\n\t- "
            + parsed_patterns
        )

        res = []

        named_variables = self.named_variables(vars)
        if named_variables:
            # if we have named variable + extra 'anonymous' variable (coming from
            # eg '*'), ignore the anonymous ones
            if len(named_variables) != len(vars):
                vars = named_variables

            # remove the '?' at the start of the variables
            vars_naked = [var[1:] for var in vars]

        # only anonymous variables? rename them var1,var2,var3,var4...
        else:
            vars_naked = ["var%s" % (i + 1) for i in range(len(vars))]

        for model in models:
            q = "SELECT %s WHERE {\n" % " ".join(vars)
            for p in normalised_patterns:
                q += "%s .\n" % p
            q += "}"

            sparql_res, _ = self._sparql(model, q)

            res += [
                dict(zip(vars_naked, r))
                for r in [shorten(self.models[model].graph, row) for row in sparql_res]
            ]

        logger.info("Found: " + str(res))
        return res

    @api
    def subscribe(self, patterns, one_shot=False, models=None):
        """
        Subscribe to a specified event in the ontology.

        Every time the model(s) is (are) updated, the provided `patterns` are
        evaluated against the set of asserted and inferred triples. If at least
        one triple is returned, the event is fired.

        The terms bound to the named variables in the patterns are attached to
        the fired event.

        For instance:
        ```
        import time
        from knowledge_core.api import KB

        def on_new_robot_instance(instances):
            print("New robots: " + ", ".join(instances))

        kb = KB()
        kb.subscribe(["?robot rdf:type Robot"], callback=on_new_robot_instance)

        kb += ["myself rdf:type Robot"]
        time.sleep(1)
        # should print "New robots: myself"
        ```

        If `one_shot` is set to true, the event is discarded once it has fired
        once.
        """
        models = self.normalize_models(models)

        event = Event(self, patterns, one_shot, models)

        nb_prev_evt = len(self.active_evts)
        self.active_evts.add(event)
        if len(self.active_evts) > nb_prev_evt:
            logger.info("Registered a new event: %s" %
                        patterns + " in " + str(models))
        else:
            logger.info(
                "Event handler already existing for %s. No need to add it." % patterns)

        return event.id

    @api
    def close(self):
        # This code should actually never be called: the 'close' request
        # is intercepted at the front-end level (eg, socket server) to close
        # the connection.
        raise NotImplementedError

    @api
    def debugger(self):
        import pdb

        pdb.set_trace()

    ############################################################################
    ############################################################################

    def _sparql(self, model, query, raw=False):
        """
        Private method to query the knowledge graph with SPARQL queries.

        :param raw: if True, returns the raw SPARQLResult object; if False,
        returns a list of triples fulfulling the query.

        :return: result of the query(see `raw` parameter) and original query,
        augmented with standard SPARQL prefixes.

        TODO: as a(potential?) optimization, pass initNs to graph.query,
              instead of adding the PREFIX strings to the query
        """
        q = SPARQL_PREFIXES
        q += query

        logger.debug("Executing SPARQL query in model: %s\n%s" % (model, q))

        import pyparsing

        try:
            res = self.models[model].materialized_graph.query(q)
            if raw:
                return res, q
            else:
                no_bnode = [stmt for stmt in filter(
                    lambda s: all(type(t) != BNode for t in s), res)]
                return no_bnode, q
        except pyparsing.ParseException:
            raise KbServerError(
                "Syntax error while parsing SPARQL query:\n%s" % q)
        except TypeError as te:
            raise KbServerError(
                "Syntax error while parsing SPARQL query:\n%s\nException was: %s" % (q, te))

    def named_variables(self, vars):
        """
        Return the list of variable that have been explicitely named.

        eg, not the 'stars' wildcards, that would have been replaced by anonymous
        variable starting with '__' by pykb.
        """
        return [v for v in vars if not v.startswith("?__")]

    def onupdate(self):

        # need to materialise as soon as possible after the model has been
        # changed so that events are triggered in a timely fashion
        self.materialise()

        self._functionalproperties = frozenset(
            self._instancesof("owl:FunctionalProperty", False)
        )

        to_remove = [
            e_id for e_id, clients in self.eventsubscriptions.items() if len(clients) == 0]
        for e_id in to_remove:
            logger.info(
                f"Removing handler for event {e_id} as no clients anymore")
            self.remove_event(e_id)
        if self.active_evts:
            logger.info(
                "Checking the %s active event handler(s) against new facts"
                % len(self.active_evts)
            )
        for e in list(self.active_evts):
            if e.evaluate():
                clients = self.eventsubscriptions[e.id]
                logger.info(
                    "Event %s triggered. Informing %s clients." % (
                        e.id, len(clients))
                )
                for client in clients:
                    msg = ("event", e)
                    self.requestresults.setdefault(client, Queue()).put(msg)
                if not e.valid:
                    self.active_evts.remove(e)
                    del self.eventsubscriptions[e.id]

    def remove_event(self, evt_id):
        del self.eventsubscriptions[evt_id]
        evts_to_remove = []
        for e in self.active_evts:
            if e.id == evt_id:
                evts_to_remove.append(e)
        for e in evts_to_remove:
            self.active_evts.remove(e)

    def materialise(self, models=None):

        start = time.time()

        if models is None:
            models = self.models.keys()

        for model in models:
            g = self.models[model]
            if not g.is_dirty:
                continue

            if not self.reasoner_enabled:
                g.materialized_graph = g.graph
            else:
                r = reasonable.PyReasoner()
                r.from_graph(g.graph)

                g.materialized_graph = Graph()
                g.materialized_graph.namespace_manager = g.graph.namespace_manager
                g.materialized_graph += r.reason()

            g.is_dirty = False

        end = time.time()

        if end - start > 0.0001:  # did we actually classify anything?
            logger.info(
                "Materialisation performed by reasoner in %.1fms"
                % ((end - start) * 1000)
            )

    def check_expired_stmts(self):
        logger.debug("Checking for expired statements...")

        # TODO: as a (potential?) optimization, pass initNs to graph.query,
        # instead of adding the PREFIX strings to the query
        q = SPARQL_PREFIXES
        q += """
        SELECT ?subgraph ?date
        WHERE {
           ?subgraph :expiresOn ?date .
           FILTER (?date <= ?now)
        }
        """

        now = Literal(date_time(time.time()), datatype=XSD.dateTime)

        for name, model in self.models.items():

            res = model.metadata.query(q, initBindings={"now": now})
            if res:
                for row in res:
                    graph = row[0]
                    date = row[1]
                    for s, p, o in graph:
                        ss, sp, so = shorten(graph, (s, p, o))
                        if p == RDF.type and o == ORO("ActiveConcept"):
                            if ss in self.active_concepts:
                                self.active_concepts.remove(ss)
                        logger.warn(f"Removing expired statement <{ss} {sp} {so}>"
                                    f" from <{name}> (expired on {date})")
                    model.metadata.remove((graph, None, None))
                    model.graph -= graph
                    model.is_dirty = True

                self.materialise(models=[name])

    def create_model(self, model):
        g = self.ds.graph(IRIS[DEFAULT_PREFIX] + model)
        metadata_g = self.ds.graph(IRIS[DEFAULT_PREFIX] + model + "_metadata")

        # configure our namespace prefixes
        for p, iri in IRIS.items():
            g.bind(p, iri)
            metadata_g.bind(p, iri)

        g.bind("", IRIS[DEFAULT_PREFIX])
        metadata_g.bind("", IRIS[DEFAULT_PREFIX])

        self.models[model] = dotdict(
            {
                "graph": g,
                "metadata": metadata_g,
                # stores whether models have changes that would require re-classification
                "is_dirty": True,
                "materialized_graph": Graph(),
            }
        )

    def normalize_models(self, models):
        """
        Normalise a list of models.

        If 'models' is None or [], returns the default model (ie, the
        base model of the robot). If 'models' is 'all', then
        returns the set of all models known to the KB.
        Else, add the models to the list of all models, and return
        only the models passed as argument.
        """
        if models:
            if "all" in models:
                return frozenset(self.models)
            else:
                if isinstance(models, str):
                    models = [models]

                models = [m if m != "" else DEFAULT_MODEL for m in models]

                # do we have a new model? initialise it.
                for model in models:
                    if model not in self.models:
                        self.create_model(model)

                return frozenset(models)
        else:
            return {DEFAULT_MODEL}

    def execute(self, client, name, *args, **kwargs):

        if name == "close":

            # Event-related housekepping
            evts_id_to_remove = []
            for evt_id, clients in self.eventsubscriptions.items():
                if client in clients:
                    logger.info(
                        "Client closing, removing event subscription <%s>" % evt_id
                    )
                    clients.remove(client)
                if len(clients) == 0:
                    evts_id_to_remove.append(evt_id)
            for evt_id in evts_id_to_remove:
                logger.info(
                    "No one interested in event %s anymore. Removing it" % evt_id
                )
                self.remove_event(evt_id)
            ###############

            logger.info("Closing connection to client.")
            return

        f = getattr(self, name)
        if hasattr(f, "_compat"):
            logger.warn(
                "Using non-standard method %s. This may be " % f.__name__
                + "removed in the future!"
            )

        msg = None
        try:
            res = None
            if args or kwargs:
                res = f(*args, **kwargs)
                if name in ["subscribe"]:
                    self.eventsubscriptions.setdefault(res, []).append(client)
            else:
                res = f()
            msg = ("ok", res)
        except Exception as e:
            logger.info(traceback.format_exc())
            logger.error("request failed: %s" % e)
            msg = ("error", e)

        self.requestresults.setdefault(client, Queue()).put(msg)

    def submitrequest(self, client, name, *args, **kwargs):
        self.incomingrequests.put((client, name, args, kwargs))

    def process(self):
        now = time.time()
        if 1./(now - self._last_expired_stmts_check) < EXPIRED_STMTS_CHECK_RATE:
            self.check_expired_stmts()
            self._last_expired_stmts_check = time.time()

        try:
            client, name, args, kwargs = self.incomingrequests.get(True, 0.05)
            logger.debug(
                "Processing <%s(%s,%s)>..."
                % (
                    name,
                    ", ".join([str(a) for a in args]),
                    ", ".join(str(k) + "=" + str(v)
                              for k, v in kwargs.items()),
                )
            )
            self.execute(client, name, *args, **kwargs)
        except Empty:
            pass

        for client, pendingmsg in self.requestresults.items():
            while not pendingmsg.empty():
                msg = pendingmsg.get()
                logger.debug("sending %s to %s" % (msg, client))
                client.sendmsg(msg)
