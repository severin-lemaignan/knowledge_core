import logging

from rdflib.graph import Dataset

logger = logging.getLogger("minimalKB." + __name__)

from queue import Queue, Empty
import json
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
from rdflib.term import Literal, URIRef, Variable
from rdflib.namespace import Namespace, NamespaceManager, RDF, RDFS, OWL, XSD

import hashlib


def stable_hash(s, p="", o="", model=""):
    return hashlib.md5(("%s%s%s%s" % (s, p, o, model)).encode("utf-8")).hexdigest()


DEFAULT_MODEL = "default"


IRIS = {
    "pal": "http://www.pal-robotics.com/kb/",
    "oro": "http://kb.openrobots.org#",
    "cyc": "http://sw.opencyc.org/concept/",
    "rdf": "http://www.w3.org/1999/02/22-rdf-syntax-ns#",
    "rdfs": "http://www.w3.org/2000/01/rdf-schema#",
    "owl": "http://www.w3.org/2002/07/owl#",
    "xsd": "http://www.w3.org/2001/XMLSchema#",
}

DEFAULT_PREFIX = "pal"

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

from .services.simple_rdfs_reasoner import start_reasoner, stop_reasoner
from .services import lifespan

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

    return Graph().parse(data=data, format="n3")


# @memoize
def parse_stmts(stmts):

    return list(parse_stmts_to_graph(stmts))


# @memoize
def parse_stmt(stmt):

    return list(Graph().parse(data=N3_PROLOGUE + "%s ." % stmt, format="n3"))[0]


# @memoize
def parse_term(term):

    # TODO: correct, but not super effective!
    return list(Graph().parse(data=N3_PROLOGUE + " <s> <p> %s ." % term, format="n3"))[
        0
    ][2]


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


class Event:

    NEW_INSTANCE = "NEW_INSTANCE"
    NEW_CLASS_INSTANCE = "NEW_CLASS_INSTANCE"
    NEW_INSTANCE_ONE_SHOT = "NEW_INSTANCE_ONE_SHOT"
    NEW_CLASS_INSTANCE_ONE_SHOT = "NEW_CLASS_INSTANCE_ONE_SHOT"

    def __init__(self, kb, type, trigger, var, patterns, models):
        self.kb = kb

        if type == Event.NEW_CLASS_INSTANCE:
            self.type = Event.NEW_INSTANCE
            self.var = "?instance"
            self.patterns = ["?instance rdf:type %s" % klass[0] for klass in patterns]
        elif type == Event.NEW_CLASS_INSTANCE_ONE_SHOT:
            self.type = Event.NEW_INSTANCE_ONE_SHOT
            self.var = "?instance"
            self.patterns = ["?instance rdf:type %s" % klass[0] for klass in patterns]
        else:
            self.type = type
            self.var = var
            self.patterns = patterns

        self.trigger = trigger
        self.models = models

        self.id = "evt_" + str(
            stable_hash(
                self.type
                + self.trigger
                + (self.var if self.var else "")
                + str(sorted(self.patterns))
                + str(sorted(self.models))
            )
        )

        self.content = None

        self.valid = True

        self.previous_instances = set()
        if type in [Event.NEW_INSTANCE, Event.NEW_INSTANCE_ONE_SHOT]:

            instances = self.kb.find([self.var], self.patterns, frozenset(self.models))
            logger.debug(
                "Creating a NEW_INSTANCE event with initial instances %s" % instances
            )
            self.previous_instances = set(instances)

    def __hash__(self):
        return hash(self.id)

    def __cmp__(self, other):
        return hash(self).__cmp__(hash(other))

    def evaluate(self):
        if "ONE_SHOT" in self.trigger:
            self.valid = False

        if self.type in [Event.NEW_INSTANCE, Event.NEW_INSTANCE_ONE_SHOT]:
            instances = set(
                self.kb.find([self.var], self.patterns, frozenset(self.models))
            )
            newinstances = instances - self.previous_instances

            # previous_instances must be set to the current set of matching instance
            # else we won't trigger events when an instance disappear and
            # re-appear later.
            self.previous_instances = instances

            if not newinstances:
                return False

            self.content = [
                i for i in newinstances
            ]  # for some reason, calling list() does not work
            return True


class MinimalKB:

    MEMORYPROFILE_DEFAULT = ""
    MEMORYPROFILE_SHORTTERM = "SHORTTERM"

    def __init__(self, filenames=None):

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

        # self.start_services()

        if filenames:
            for filename in filenames:
                self.load(filename)

    @api
    def hello(self):
        return "MinimalKB, v.%s" % __version__

    @compat
    @api
    def stats(self):
        return {"version": __version__}

    @api
    def load(self, filename, models=None):

        models = self.normalize_models(models)
        self.store.load(filename, models)

    @compat
    @api
    def listAgents(self):
        return list(self.models)

    @api
    def clear(self):
        logger.warn("Clearing the knowledge base!")
        self.active_evts.clear()

        for m, g in self.models.items():
            self.ds.remove_graph(g)

        self.create_model(DEFAULT_MODEL)

    @compat
    @api
    def reset(self):
        self.clear()

    @compat
    @api
    def listSimpleMethods(self):
        return self.methods()

    @api
    def methods(self):
        return list(self._api.keys())

    @api
    def about(self, term, models=None):

        result = []

        models = self.normalize_models(models)

        term = parse_term(term)

        for model in models:
            g = self.models[model]
            for triple in g.triples((term, None, None)):
                result += shorten(g, triple)
            for triple in g.triples((None, term, None)):
                result += shorten(g, triple)
            for triple in g.triples((None, None, term)):
                result += shorten(g, triple)

        return result

    @compat
    @api
    def lookupForAgent(self, agent, resource):
        return self.lookup(resource, models=[agent])

    @api
    def lookup(self, resource, models=None):
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
            if (
                isinstance(o, str) and resource in o and p == "rdfs:label"
            ):  # resource is the label -> add s
                matching_concepts.add(s)

        res = [(concept, self.typeof(concept, models)) for concept in matching_concepts]
        logger.info("Found: " + str(res))
        return res

    @api
    def instancesof(self, term, direct, models=[]):

        if direct:
            logger.warn(
                "'direct=True' not implemented! Returing all asserted 'is-a' relations."
            )

        models = self.normalize_models(models)
        result = []

        res = parse_term(term)
        for model in models:
            result.append(self.models[model].subjects(RDF.type, res))

        return result

    @api
    def classesof(self, term, direct, models=[]):

        if direct:
            logger.warn(
                "'direct=True' not implemented! Returing all asserted 'is-a' relations."
            )

        models = self.normalize_models(models)
        result = []

        res = parse_term(term)
        for model in models:
            result.append(self.models[model].objects(res, RDF.type))

        return result

    @api
    def subclassesof(self, term, direct, models=[]):

        if direct:
            logger.warn(
                "'direct=True' not implemented! Returing all asserted 'is-a' relations."
            )

        models = self.normalize_models(models)
        result = []

        res = parse_term(term)
        for model in models:
            result.append(self.models[model].subjects(RDFS.subClassOf, res))

        return result

    @api
    def superclassesof(self, term, direct, models=[]):

        if direct:
            logger.warn(
                "'direct=True' not implemented! Returing all asserted 'is-a' relations."
            )

        models = self.normalize_models(models)
        result = []

        res = parse_term(term)
        for model in models:
            result.append(self.models[model].objects(res, RDFS.subClassOf))

        return result

    def typeof(self, raw_term, models):

        term = parse_term(raw_term)

        if isinstance(term, Literal):
            return "literal"

        classes = self.classesof(raw_term, False, models)
        if classes:
            if "owl:ObjectProperty" in classes:
                return "object_property"
            elif "owl:DatatypeProperty" in classes:
                return "datatype_property"
            elif "owl:Class" in classes:
                return "class"
            else:
                return "instance"

        if (
            self.instancesof(raw_term, False, models)
            or self.subclassesof(raw_term, False, models)
            or self.superclassesof(raw_term, False, models)
        ):
            return "class"

        raise NotImplemented("to be done")
        stmts_if_predicate = matchingstmt(self.conn, ("?s", concept, "?o"), models)
        if stmts_if_predicate:
            if self.is_literal(stmts_if_predicate[0][3]):
                return "datatype_property"
            else:
                return "object_property"

        logger.warn("Concept <%s> has undecidable type." % concept)
        return "undecided"

    @api
    def details(self, resource, models=None):
        """
        Returns a dictionary containing the following details on a given resource:
        - 'name': resource label, if any, literal value for literals, else resource ID.
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
        res["name"] = self.store.label(resource, models)
        res["id"] = resource
        res["type"] = self.store.typeof(resource, models)

        if res["type"] == "class":
            res["attributes"] = []
            res["attributes"].append(
                {
                    "name": "Parents",
                    "id": "superClasses",
                    "values": [
                        {"id": r, "name": self.store.label(r, models)}
                        for r in self.superclassesof(resource, True, models)
                    ],
                }
            )

            res["attributes"].append(
                {
                    "name": "Children",
                    "id": "subClasses",
                    "values": [
                        {"id": r, "name": self.store.label(r, models)}
                        for r in self.subclassesof(resource, True, models)
                    ],
                }
            )

            res["attributes"].append(
                {
                    "name": "Instances",
                    "id": "instances",
                    "values": [
                        {"id": r, "name": self.store.label(r, models)}
                        for r in self.instancesof(resource, True, models)
                    ],
                }
            )

        elif res["type"] == "instance":
            res["attributes"] = [
                {
                    "name": "Classes",
                    "id": "classes",
                    "values": [
                        {"id": r, "name": self.store.label(r, models)}
                        for r in self.store.classesof(resource, True, models)
                    ],
                }
            ]
        return res

    @compat
    @api
    def getResourceDetails(self, concept):
        return self.details(concept)

    @api
    def check(self, stmts, models=None):
        logger.warn(
            "'check' has been invoked, but no classification supported."
            "Returning true if the statements are already asserted/inferred,"
            "false otherwise (ie, calls 'exist' instead of 'check')."
        )
        return self.exist(stmts, models)

    @api
    def exist(self, stmts, models=None):
        """Returns True if all the statements exist (eg, are materialised) in
        all the given models.
        """

        models = self.normalize_models(models)

        logger.info(
            "Checking existence of "
            + str(stmts)
            + " in "
            + (str(models) if models else "default model.")
        )
        stmts = [parse_stmt(s) for s in stmts]

        for model in models:
            for stmt in stmts:
                if not stmt in self.models[model]:
                    return False

        return True

    @api
    def revise(self, stmts, policy):
        """

        Supported policy options:
        - method: string in [add, safe_add, retract, update, safe_update, revision]
        - models: list of strings
        - lifespan: duration before automatic removal of statements, in
          seconds, float
        """

        if isinstance(stmts, str):
            raise KbServerError("A list of statements is expected")

        subgraph = parse_stmts_to_graph(stmts)
        parsed_stmts = "\n\t- ".join([" ".join(s) for s in shorten_graph(subgraph)])

        if type(policy) != dict:
            raise KbServerError("Expected a dictionary as policy")
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
                self.models[model] += subgraph  # TODO: lifespan

        if policy["method"] == "retract":
            logger.info("Deleting from " + str(list(models)) + ":\n\t- " + parsed_stmts)
            for model in models:
                self.models[model] -= subgraph

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
                        self.models[model].set((s, p, o))  # TODO: lifespan
                    else:
                        self.models[model].add((s, p, o))  # TODO: lifespan

        self.onupdate()

    @api
    def add(self, stmts, models=None, lifespan=0):
        return self.revise(
            stmts, {"method": "add", "models": models, "lifespan": lifespan}
        )

    @compat
    @api
    def safeAdd(self, stmts, lifespan=0):
        return self.revise(stmts, {"method": "safe_add", "lifespan": lifespan})

    @compat
    @api
    def addForAgent(self, agent, stmts, lifespan=0):
        return self.add(stmts, [agent], lifespan)

    @api
    def retract(self, stmts, models=None):
        return self.revise(stmts, {"method": "retract", "models": models})

    @compat
    @api
    def remove(self, stmts, models=None):
        return self.retract(stmts, models)

    @compat
    @api
    def removeForAgent(self, agent, stmts):
        return self.retract(stmts, [agent])

    @api
    def update(self, stmts, models=None, lifespan=0):
        return self.revise(
            stmts, {"method": "update", "models": models, "lifespan": lifespan}
        )

    @compat
    @api
    def findForAgent(self, agent, var, stmts):
        return self.find([var], stmts, None, [agent])

    @api
    def find(self, vars, patterns, models=None):
        """
        Depending on the arguments, three differents
        behaviours are possible:

        - if len(vars) == 0, 'find' returns the statement/set of statements if they all exist in the model.
        - if len(vars) == 1, 'find' returns a set of resources matching the patterns.
        - if len(vars) > 1:
            - if len(patterns) == 1, a list of statements matching the pattern
              is returned.
            - else, a list of dictionaries is returned with
            possible combination of values for the different variables. For
            instance, find(["?agent", "?action"], ["?agent desires ?action", "?action rdf:type Jump"])
            would return something like: [{"agent":"james", "action": "jumpHigh"}, {"agent": "laurel", "action":"jumpHigher"}]
        """

        if not vars:
            if not self.exist(patterns, models):
                return []
            return patterns

        models = self.normalize_models(models)

        patterns = [parse_stmt(p) for p in patterns]
        parsed_patterns = "\n\t- ".join(
            [" ".join(s) for s in shortenN(self.models[DEFAULT_MODEL], patterns)]
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
        # if we have named variable + extra 'anonymous' variable (coming from
        # eg '*'), ignore them
        if named_variables and len(named_variables) != len(vars):
            vars = named_variables

        # remove the '?' at the start of the variables
        vars_naked = [var[1:] for var in vars]

        for model in models:
            q = "SELECT %s WHERE {\n" % " ".join(vars)
            for p in normalised_patterns:
                q += "%s .\n" % p
            q += "}"

            sparql_res = self._sparql(model, q)

            # to serialise to JSON:
            # https://www.w3.org/TR/2013/REC-sparql11-results-json-20130321/
            # sparql_res.serialize(format="json")

            if len(named_variables) > 1:
                res += [
                    dict(zip(vars_naked, r))
                    for r in [shorten(self.models[model], row) for row in sparql_res]
                ]
            else:
                res += [shorten(self.models[model], row) for row in sparql_res]

        # if a single variable is requested, 'unpack' the result
        if len(vars) == 1:
            res = [r[0] for r in res]

        logger.info("Found: " + str(res))
        return res

    @api
    def findmpe(self, vars, pattern, constraints=None, models=None):
        """Finds the most probable explanation. Strictly equivalent to
        'find' until we support probabilities.
        """
        return find(self, vars, pattern, constraints=None, models=None)

    @api
    def subscribe(self, type, trigger, var, patterns, models=None):

        models = self.normalize_models(models)

        logger.info(
            "Registering a new event: %s %s for %s on %s"
            % (type, trigger, var, patterns)
            + " in "
            + (str(models) if models else "default model.")
        )

        event = Event(self, type, trigger, var, patterns, models)

        self.active_evts.add(event)

        return event.id

    @compat
    @api
    def registerEvent(self, type, trigger, patterns):
        return self.subscribe(type, trigger, None, patterns)

    @compat
    @api
    def discriminateForAgent(self, *args):
        raise NotImplementedError("discriminateForAgent not implemented in MinimalKB")

    @compat
    @api
    def getLabel(self, concept):
        return self.store.label(concept)

    @compat
    @api
    def getDirectClassesOf(self, concept):
        return self.getClassesOf(concept, True)

    @compat
    @api
    def getClassesOf(self, concept, direct=False):
        classes = self.classesof(concept, direct)

        return {cls: self.getLabel(cls) for cls in classes}

    @api
    def close(self):
        """This code should actually never be called: the 'close' request
        is intercepted at the front-end level (eg, socket server) to close
        the connection.
        """
        raise NotImplementedError

    ################################################################################
    ################################################################################

    def _sparql(self, model, query):
        q = SPARQL_PREFIXES  # TODO: as a (potential?) optimization, pass initNs to graph.query, instead of adding the PREFIX strings to the query
        q += query

        logger.debug("Executing SPARQL query in model: %s\n%s" % (model, q))

        return self.models[model].query(q)

    def named_variables(self, vars):
        """
        Returns the list of variable that have been explicitely named (eg,
        not the 'stars' wildcards, that would have been replaced by anonymous
        variable starting with '__' by pykb)
        """

        return [v for v in vars if not v.startswith("?__")]

    def onupdate(self):

        self._functionalproperties = frozenset(
            self.instancesof("owl:FunctionalProperty", False)
        )

        for e in self.active_evts:
            if e.evaluate():
                clients = self.eventsubscriptions[e.id]
                logger.info(
                    "Event %s triggered. Informing %s clients." % (e.id, len(clients))
                )
                for client in clients:
                    msg = ("event", e)
                    self.requestresults.setdefault(client, Queue()).put(msg)
                if not e.valid:
                    self.active_evts.discard(e)

    def start_services(self, *args):

        if self.backend == SQLITE:
            self._reasoner = Process(target=start_reasoner, args=("kb.db",))
            self._reasoner.start()

            self._lifespan_manager = Process(
                target=lifespan.start_service, args=("kb.db",)
            )
            self._lifespan_manager.start()

    def stop_services(self):
        self._reasoner.terminate()
        self._lifespan_manager.terminate()

        self._reasoner.join()
        self._lifespan_manager.join()

    def create_model(self, model):
        g = self.ds.graph(IRIS[DEFAULT_PREFIX] + model)

        # configure our namespace prefixes
        for p, iri in IRIS.items():
            g.bind(p, iri)

        g.bind("", IRIS[DEFAULT_PREFIX])

        self.models[model] = g

        self.add(
            ["owl:Thing rdf:type owl:Class", "owl:Nothing rdf:type owl:Class"], model
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

                # do we have a new model? initialise it.
                for model in models:
                    if model not in self.models:
                        self.create_model(model)

                return frozenset(models)
        else:
            return {DEFAULT_MODEL}

    def execute(self, client, name, *args, **kwargs):

        if name == "close":
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
                if name in ["subscribe", "registerEvent"]:
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
