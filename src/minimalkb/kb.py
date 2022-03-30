import logging

logger = logging.getLogger("minimalKB." + __name__)

import time

from queue import Queue, Empty
import traceback

try:
    import rdflib

    if rdflib.__version__ < "6.0.0":
        logger.error("RDFlib >= 6.0.0 is required. Please upgrade.")
        import sys

        sys.exit(1)
except ImportError:
    logger.error("RDFlib is required. Please install it.")
    import sys

    sys.exit(1)

from rdflib import Graph, Dataset
from rdflib.term import Node, Literal, URIRef, Variable
from rdflib.namespace import Namespace, RDF, RDFS, OWL

has_reasoner = False
try:
    import reasonable

    has_reasoner = True
except ImportError:
    logger.warning(
        "reasonable OWL2 RL reasoner not available. Install it with `pip install reasonable`. Running without reasoning."
    )


import hashlib


def stable_hash(s, p="", o="", model=""):
    return hashlib.md5(("%s%s%s%s" % (s, p, o, model)).encode("utf-8")).hexdigest()


DEFAULT_MODEL = "default"
REASONER_RATE = 5  # Hz


IRIS = {
    "oro": "http://kb.openrobots.org#",
    "cyc": "http://sw.opencyc.org/concept/",
    "rdf": "http://www.w3.org/1999/02/22-rdf-syntax-ns#",
    "rdfs": "http://www.w3.org/2000/01/rdf-schema#",
    "owl": "http://www.w3.org/2002/07/owl#",
    "xsd": "http://www.w3.org/2001/XMLSchema#",
}

DEFAULT_PREFIX = "oro"

default_ns = Namespace(IRIS[DEFAULT_PREFIX])

SPARQL_PREFIXES = "".join(["PREFIX %s: <%s>\n" % (p, iri) for p, iri in IRIS.items()])
SPARQL_PREFIXES += "PREFIX : <%s>\n" % IRIS[DEFAULT_PREFIX]
SPARQL_PREFIXES += "BASE <%s>\n" % IRIS[DEFAULT_PREFIX]

N3_PROLOGUE = " ".join(["@prefix %s: <%s>." % (p, iri) for p, iri in IRIS.items()])
N3_PROLOGUE += "@prefix : <%s>. " % IRIS[DEFAULT_PREFIX]
N3_PROLOGUE += "@base <%s>. " % IRIS[DEFAULT_PREFIX]
N3_PROLOGUE += (
    "@keywords a,true,false. "  # see https://www.w3.org/TeamSubmission/n3/#keywords
)


from .exceptions import KbServerError
from minimalkb import __version__

#from .services import lifespan

from .helpers import memoize


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
        data += " %s . " % stmt

    try:
        return Graph().parse(data=data, format="n3")
    except rdflib.plugins.parsers.notation3.BadSyntax:
        raise KbServerError("invalid syntax for statements %s" % stmts)


# @memoize
def parse_stmts(stmts):

    return list(parse_stmts_to_graph(stmts))


# @memoize
def parse_stmt(stmt):

    try:
        return list(Graph().parse(data=N3_PROLOGUE + "%s ." % stmt, format="n3"))[0]
    except rdflib.plugins.parsers.notation3.BadSyntax:
        raise KbServerError("invalid syntax for statement <%s>" % stmt)


# @memoize
def parse_term(term):

    try:
        # TODO: correct, but not super effective!
        return list(
            Graph().parse(data=N3_PROLOGUE + " <s> <p> %s ." % term, format="n3")
        )[0][2]
    except rdflib.plugins.parsers.notation3.BadSyntax:
        raise KbServerError("invalid syntax for term <%s>" % term)


def shorten_term(graph, term):

    if isinstance(term, URIRef):
        return graph.qname(term)
    else:
        return term.n3()


def shorten(graph, stmt):

    res = []
    for t in stmt:
        if isinstance(t, URIRef):
            res.append(graph.qname(t))
        else:
            res.append(t.n3())
    return res


def shortenN(graph, stmts):
    return [shorten(graph, s) for s in stmts]


def shorten_graph(graph):
    """returns a list of s,p,o statements contained in a graph, with their URIs shorten (eg, using prefixes when possible)"""
    return [shorten(graph, s) for s in graph.triples([None, None, None])]


def get_variables(stmt):
    return [v for v in stmt if isinstance(v, Variable)]

def get_all_variables(stmts):
    vars = []
    for p in stmts:
       vars += [v.n3() for v in get_variables(parse_stmt(p))]

    return vars

class dotdict(dict):
    """dot.notation access to dictionary attributes"""

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
                str(sorted(self.patterns)) + str(one_shot) + str(sorted(self.models))
            )
        )

        self.content = None

        self.vars = get_all_variables(self.patterns)

        self.valid = True

        self.previous_instances = set()

        instances = self.kb.find( self.patterns, self.vars, frozenset(self.models))
        logger.debug("Creating a event with initial instances %s" % instances)

        self.previous_instances = set([hashabledict(row) for row in instances])

    def __hash__(self):
        return hash(self.id)

    def __cmp__(self, other):
        return hash(self).__cmp__(hash(other))

    def evaluate(self):
        if not self.valid:
            return False

        if self.one_shot:
            self.valid = False

        instances = self.kb.find(self.patterns, self.vars, frozenset(self.models))

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


class MinimalKB:

    MEMORYPROFILE_DEFAULT = ""
    MEMORYPROFILE_SHORTTERM = "SHORTTERM"

    def __init__(self, filenames=None, enable_reasoner=True):

        self.reasoner_enabled = enable_reasoner
        if not self.reasoner_enabled:
            logger.warn("Running without OWL2 RL reasoner.")

        _api = [
            getattr(self, fn) for fn in dir(self) if hasattr(getattr(self, fn), "_api")
        ]
        import inspect

        self._api = {fn.__name__ + str(inspect.signature(fn)): fn for fn in _api}

        apilist = [
            key + (" (compatibility)" if hasattr(val, "_compat") else "")
            for key, val in self._api.items()
        ]

        logger.debug(
            "Initializing the MinimalKB with the following API: \n\t- "
            + "\n\t- ".join(apilist)
        )

        self.incomingrequests = Queue()
        self.requestresults = {}

        self.active_evts = set()
        self.eventsubscriptions = {}

        # create a RDFlib dataset
        self.ds = Dataset()
        self.models = {}
        self.create_model(DEFAULT_MODEL)

        self._functionalproperties = frozenset()

        self.start_services()

        if filenames:
            for filename in filenames:
                self.load(filename)

    @api
    def hello(self):
        return "MinimalKB, v.%s" % __version__

    @api
    def load(self, filename, models=None):

        models = self.normalize_models(models)
        for model in models:
            logger.info("Loading file <%s> in model <%s>" % (filename, model))
            self.models[model].graph.parse(filename, publicID=IRIS[DEFAULT_PREFIX])
            self.models[model].is_dirty = True

        self.onupdate()

    @api
    def clear(self):
        logger.warn("Clearing the knowledge base!")
        self.active_evts.clear()

        for m, g in self.models.items():
            self.ds.remove_graph(g.graph)

        self.create_model(DEFAULT_MODEL)

    @api
    def methods(self):
        return list(self._api.keys())

    @api
    def about(self, raw_term, models=None):

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
    def lookup(self, resource, models=None):
        """Search the knowledge base for a term matching a string. The search is
        performed both on terms' names and on label.

        Returns the list of found terms, alongside with their type (one of instance, class,
        datatype_property, object_property, literal)
        """
        models = self.normalize_models(models)
        logger.info(
            "Lookup for "
            + str(resource)
            + " in "
            + (str(models) if models else "default model.")
        )
        about = self.about(resource, models)

        if not about:
            logger.info("'%s' not found." % str(resource))
            return []

        matching_concepts = set()

        for s, p, o in about:
            if s == resource or p == resource or o == resource:
                matching_concepts.add(resource)
            elif p == shorten_term(self.models[DEFAULT_MODEL].graph, RDFS.label):
                matching_concepts.add(s)

        res = [(concept, self.typeof(concept, models)) for concept in matching_concepts]
        logger.info("Found: " + str(res))
        return res

    def _instancesof(self, term, direct=False, models=[]):

        models = self.normalize_models(models)
        result = []

        if not isinstance(term, Node):
            term = parse_term(term)

        for model in models:
            if direct:
                result += self.models[model].graph.subjects(RDF.type, term)
            else:
                result += self.models[model].materialized_graph.subjects(RDF.type, term)

        return result

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
                result += self.models[model].materialized_graph.objects(term, RDF.type)

        return result

    def _subclassesof(self, term, direct=False, models=[]):

        models = self.normalize_models(models)
        result = []

        if not isinstance(term, Node):
            term = parse_term(term)

        for model in models:
            if direct:
                result += self.models[model].graph.subjects(RDFS.subClassOf, term)
            else:
                result += self.models[model].materialized_graph.subjects(
                    RDFS.subClassOf, term
                )

        return result

    def _superclassesof(self, term, direct=False, models=[]):

        models = self.normalize_models(models)
        result = []

        if not isinstance(term, Node):
            term = parse_term(term)

        for model in models:
            if direct:
                result += self.models[model].graph.objects(term, RDFS.subClassOf)
            else:
                result += self.models[model].materialized_graph.objects(
                    term, RDFS.subClassOf
                )

        return result

    @api
    def label(self, term, models=[]):
        """returns the labels attached to a term, as a dictionary `{"default":"label1", "lang_code1": label1,
        "lang_code2": "label2",...}` where the 'default' key returns either the
        English version of the label, or the name of the term, if no label is
        available, and the other keys provide localised version of the label, if
        available in the knowledge base.
        """
        models = self.normalize_models(models)
        result = {}

        if not isinstance(term, URIRef):
            term = parse_term(term)

        for model in models:

            labels = self.models[model].materialized_graph.objects(term, RDFS.label)
            for label in labels:
                if label.language:
                    result[label.language] = label
                else:
                    result["default"] = label

        if not "default" in result:
            if "en" in result:
                result["default"] = result["en"]
            else:
                result["default"] = shorten_term(self.models[DEFAULT_MODEL].graph, term)

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
                self.models[model].materialized_graph.triples([None, term, None])
            )

            if stmts_if_predicate:
                if isinstance(stmts_if_predicate[0][2], Literal):
                    return "datatype_property"
                else:
                    return "object_property"

        logger.warn("Concept <%s> has undecidable type." % term.n3())
        return "undecided"

    @api
    def details(self, resource, models=None):
        """
        Returns a dictionary containing the following details on a given resource:
        - 'label': resource label, if any, literal value for literals, else resource ID.
        - 'id': resource ID or 'literal' for literals
        - 'type': one of ['instance', 'class', 'object_property', 'datatype_property', 'undecided']
        - 'sameAs': list of equivalent classes or instances, if any
        - 'attributes':
            - for classes, a list of three dictionaries:
                {"name": "Parents","id": "superClasses", "values":[ids...]}
                {"name": "Children","id": "subClasses", "values":[ids...]}
                {"name": "Instances","id": "instances", "values":[ids...]}
                (only direct super/sub-classes and instances)
            - for instances, a list of one dictionary:
                {"name": "Classes","id": "classes", "values":[ids...]}
                (only direct classes)
        """
        models = self.normalize_models(models)
        res = {}
        res["label"] = self.label(resource, models)
        res["id"] = resource
        res["type"] = self.typeof(resource, models)

        if res["type"] == "class":
            res["attributes"] = []
            res["attributes"].append(
                {
                    "name": "Parents",
                    "id": "superClasses",
                    "values": [
                        {"id": r, "name": self.label(r, models)}
                        for r in self._superclassesof(resource, True, models)
                    ],
                }
            )

            res["attributes"].append(
                {
                    "name": "Children",
                    "id": "subClasses",
                    "values": [
                        {"id": r, "label": setermlabel(r, models)}
                        for r in self._subclassesof(resource, True, models)
                    ],
                }
            )

            res["attributes"].append(
                {
                    "name": "Instances",
                    "id": "instances",
                    "values": [
                        {"id": r, "label": self.label(r, models)}
                        for r in self._instancesof(resource, True, models)
                    ],
                }
            )

        elif res["type"] == "instance":
            res["attributes"] = [
                {
                    "name": "Classes",
                    "id": "classes",
                    "values": [
                        {"id": r, "label": self.label(r, models)}
                        for r in self._classesof(resource, True, models)
                    ],
                }
            ]
        return res

    @api
    def exist(self, raw_stmts, models=None):
        """Returns True if all the statements exist (eg, are materialised) in
        all the given models.
        """

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
                    if not stmt in self.models[model].materialized_graph:
                        return False

            return True

        # else, run a query
        return bool(self.find(raw_stmts, vars, models))

    @api
    def revise(self, stmts, policy):
        """

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

        subgraph = parse_stmts_to_graph(stmts)
        parsed_stmts = "\n\t- ".join([" ".join(s) for s in shorten_graph(subgraph)])

        models = self.normalize_models(policy.get("models", []))

        if policy["method"] in ["add", "safe_add"]:

            lifespan = policy.get("lifespan", 0)

            logger.info(
                "Adding to "
                + str(list(models))
                + ":\n\t- "
                + parsed_stmts
                + (" (lifespan: %ssec)" % lifespan if lifespan else "")
            )
            for model in models:
                self.models[model].graph += subgraph  # TODO: lifespan
                self.models[model].is_dirty = True

        if policy["method"] == "retract":
            logger.info("Deleting from " + str(list(models)) + ":\n\t- " + parsed_stmts)
            for model in models:
                self.models[model].graph -= subgraph
                self.models[model].is_dirty = True

        if policy["method"] in ["update", "safe_update", "revision"]:

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

                    if p in self._functionalproperties:
                        self.models[model].graph.set((s, p, o))  # TODO: lifespan
                    else:
                        self.models[model].graph.add((s, p, o))  # TODO: lifespan
                self.models[model].is_dirty = True

        self.onupdate()

    @api
    def add(self, stmts, models=None, lifespan=0):
        return self.revise(
            stmts, {"method": "add", "models": models, "lifespan": lifespan}
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
        performs a raw SPARQL query on a given model ('default' by default).
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
        ```
        >>> kb += ["myself age 40"]
        >>> kb.sparql("SELECT ?age WHERE { :myself :age ?age . }")
        {'results':
            {'bindings': [
                {'age':
                    {'type': 'literal',
                     'value': '40',
                     'datatype': 'http://www.w3.org/2001/XMLSchema#integer'}
                }]
            },
          'head': {'vars': ['age']}
         }

        """

        models = self.normalize_models(model)
        if len(models) != 1:
            logger.error("sparql() can only be executed on a single model")
            return None

        model = list(models)[0]

        sparql_res = self._sparql(model, query)

        import json

        return json.loads(sparql_res.serialize(format="json"))

    @api
    def find(self, patterns, vars=None, models=None):
        """
        Performs a query on one or several models.

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
        for the different variables. For instance, find(["?agent", "?action"],
        ["?agent desires ?action", "?action rdf:type Jump"]) would return
        something like: [{"agent":"james", "action": "jumpHigh"}, {"agent":
        "laurel", "action":"jumpHigher"}]

        If you were using anonymous variables (starting with ?__) in your query
        (eg generated by `pykb` when using a `*` wildcard), they will be
        renamed `var1`, `var2`, etc.
        """

        if not vars:
            vars = get_all_variables(patterns)
            if len(vars) == 0:
                return self.exist(patterns, models)

        models = self.normalize_models(models)

        patterns = [parse_stmt(p) for p in patterns]

        parsed_patterns = "\n\t- ".join(
            [" ".join(s) for s in shortenN(self.models[DEFAULT_MODEL].graph, patterns)]
        )

        normalised_patterns = [" ".join([t.n3() for t in stmt]) for stmt in patterns]

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

            sparql_res = self._sparql(model, q)

            res += [
                dict(zip(vars_naked, r))
                for r in [
                    shorten(self.models[model].graph, row) for row in sparql_res
                ]
            ]

        logger.info("Found: " + str(res))
        return res

    @api
    def subscribe(self, patterns, one_shot=False, models=None):
        """Subscribes to a specified event in the ontology.

        Every time the model(s) is(are) updated, the provided `patterns` are
        evaluated against the set of asserted and inferred triples. If at least
        one triple is returned, the event is fired.

        The terms bounded to the named variables in the patterns are attached to
        the fired event.

        For instance:
        ```
        from kb import KB

        def on_new_robot_instance(instances):
            print("New robots: " + ", ".join(instances))

        with KB() as kb:
            kb.subscribe(["?robot rdf:type Robot"])

            kb += ["myself rdf:type Robot"] # should print "New robots: myself"
            time.sleep()
        ```

        If `one_shot` is set to true, the event is discarded once it has fired
        once.

        """

        models = self.normalize_models(models)

        logger.info("Registering a new event: %s" % patterns + " in " + str(models))

        event = Event(self, patterns, one_shot, models)

        self.active_evts.add(event)

        return event.id

    @api
    def close(self):
        """This code should actually never be called: the 'close' request
        is intercepted at the front-end level (eg, socket server) to close
        the connection.
        """
        raise NotImplementedError

    @api
    def debugger(self):
        import pdb

        pdb.set_trace()

    ################################################################################
    ################################################################################

    def _sparql(self, model, query):
        q = SPARQL_PREFIXES  # TODO: as a (potential?) optimization, pass initNs to graph.query, instead of adding the PREFIX strings to the query
        q += query

        logger.debug("Executing SPARQL query in model: %s\n%s" % (model, q))

        return self.models[model].materialized_graph.query(q)

    def named_variables(self, vars):
        """
        Returns the list of variable that have been explicitely named (eg,
        not the 'stars' wildcards, that would have been replaced by anonymous
        variable starting with '__' by pykb)
        """

        return [v for v in vars if not v.startswith("?__")]

    def onupdate(self):

        # need to materialise as soon as possible after the model has been
        # changed so that events are triggered in a timely fashion
        self.materialise()

        self._functionalproperties = frozenset(
            self._instancesof("owl:FunctionalProperty", False)
        )

        for e in list(self.active_evts):
            if e.evaluate():
                clients = self.eventsubscriptions[e.id]
                logger.info(
                    "Event %s triggered. Informing %s clients." % (e.id, len(clients))
                )
                for client in clients:
                    msg = ("event", e)
                    self.requestresults.setdefault(client, Queue()).put(msg)
                if not e.valid:
                    self.active_evts.remove(e)
                    del self.eventsubscriptions[e.id]

    def materialise(self, models=None):

        start = time.time()

        if models is None:
            models = self.models.keys()

        for model in models:
            g = self.models[model]
            if not g.is_dirty:
                continue

            if not has_reasoner or not self.reasoner_enabled:
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
            logger.debug(
                "Materialisation performed by reasoner in %.1fms"
                % ((end - start) * 1000)
            )

    def start_services(self, *args):

        # self._lifespan_manager = Process(
        #    target=lifespan.start_service, args=("kb.db",)
        # )
        # self._lifespan_manager.start()
        pass

    def stop_services(self):

        # self._lifespan_manager.join()
        pass

    def create_model(self, model):
        g = self.ds.graph(IRIS[DEFAULT_PREFIX] + model)

        # configure our namespace prefixes
        for p, iri in IRIS.items():
            g.bind(p, iri)

        g.bind("", IRIS[DEFAULT_PREFIX])

        self.models[model] = dotdict(
            {
                "graph": g,
                "is_dirty": True,  # stores whether models have changes that would require re-classification
                "materialized_graph": Graph(),
            }
        )

    def normalize_models(self, models):
        """If 'models' is None or [], returns the default model (ie, the
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

            ### Event-related housekepping
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
                logger.debug(
                    "No one interested in event %s anymore. Removing it" % evt_id
                )
                del self.eventsubscriptions[evt_id]
                evts_to_remove = []
                for e in self.active_evts:
                    if e.id == evt_id:
                        evts_to_remove.append(e)
                for e in evts_to_remove:
                    self.active_evts.remove(e)
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
            logger.debug(traceback.format_exc())
            logger.error("request failed: %s" % e)
            msg = ("error", e)

        self.requestresults.setdefault(client, Queue()).put(msg)

    def submitrequest(self, client, name, *args, **kwargs):
        self.incomingrequests.put((client, name, args, kwargs))

    def process(self):
        try:
            client, name, args, kwargs = self.incomingrequests.get(True, 0.05)
            logger.debug(
                "Processing <%s(%s,%s)>..."
                % (
                    name,
                    ", ".join([str(a) for a in args]),
                    ", ".join(str(k) + "=" + str(v) for k, v in kwargs.items()),
                )
            )
            self.execute(client, name, *args, **kwargs)
        except Empty:
            pass

        for client, pendingmsg in self.requestresults.items():
            while not pendingmsg.empty():
                client.sendmsg(pendingmsg.get())
