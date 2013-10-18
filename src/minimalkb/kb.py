import logging; logger = logging.getLogger("minimalKB."+__name__);
DEBUG_LEVEL=logging.DEBUG

import shlex
from multiprocessing import Process

hasRDFlib = False
try:
    import rdflib
    import rdflib.namespace
    hasRDFlib = True
except ImportError:
    logger.warn("RDFlib not available. You won't be able to load existing ontologies.")
    pass

from exceptions import KbServerError

from backends.sqlite import SQLStore
#from backends.rdflib_backend import RDFlibStore

from reasoning.simple_rdfs import start_reasoner, stop_reasoner

def api(fn):
    fn._api = True
    return fn

def compat(fn):
    fn._compat = True
    return fn

def parse_stmt(stmt):
    return tuple(shlex.split(stmt.encode('utf8')))


class Event:

    NEW_INSTANCE = "NEW_INSTANCE"
    NEW_INSTANCE_ONE_SHOT = "NEW_INSTANCE_ONE_SHOT"

    def __init__(self, kb, type, trigger, var, patterns, models):
        self.kb = kb
        self.type = type
        self.trigger = trigger
        self.var = var
        self.patterns = patterns
        self.models = models

        self.id =  "evt_" + str(hash(self.type + \
                    self.trigger + \
                    self.var + \
                    str(sorted(self.patterns)) + \
                    str(sorted(self.models))))

        self.content = None

        self.valid = True

        self.previous_instances = set()
        if type in [Event.NEW_INSTANCE, Event.NEW_INSTANCE_ONE_SHOT]:
            instances = self.kb.store.query([self.var], self.patterns, self.models)
            logger.debug("Creating a NEW_INSTANCE event with initial instances %s"%instances)
            self.previous_instances = set(instances)

    def __hash__(self):
        return hash(self.id)

    def __cmp__(self, other):
        return hash(self).__cmp__(hash(other))

    def evaluate(self):
        if "ONE_SHOT" in self.trigger:
            self.valid = False

        if self.type in [Event.NEW_INSTANCE, Event.NEW_INSTANCE_ONE_SHOT]:
            instances = set(self.kb.store.query([self.var], self.patterns, self.models))
            newinstances = instances - self.previous_instances
        
            if not newinstances:
                return False

            self.content = [i for i in newinstances] # for some reason, calling list() does not work
            self.previous_instances = self.previous_instances | instances
            return True



class MinimalKB:

    MEMORYPROFILE_DEFAULT = ""
    MEMORYPROFILE_SHORTTERM = "SHORTTERM"

    def __init__(self, filename = None):
        _api = [getattr(self, fn) for fn in dir(self) if hasattr(getattr(self, fn), "_api")]
        self._api = {fn.__name__:fn for fn in _api}

        self.store = SQLStore()
        #self.store = RDFlibStore()

        self.models = {"myself"}

        apilist = [key + (" (compatibility)" if hasattr(val, "_compat") else "") for key, val in self._api.items()]

        logger.debug("Initializing the MinimalKB with the following API: \n\t- " + \
                "\n\t- ".join(apilist))

        self.active_evts = set()
        self.triggered_evts = []

        self.start_reasoner()

        if filename:
            self.load(filename)

    @api
    def hello(self):
        return "MinimalKB, v.0.1"

    @api
    def load(self, filename):
        logger.info("Loading triples from %s" % filename)

        if hasRDFlib:
            g = rdflib.Graph()
            nsm = rdflib.namespace.NamespaceManager(g)
            #namespace_manager.bind(DEFAULT_NAMESPACE[0], self.default_ns)
            g.parse(filename)
            triples = []
            for s,p,o in g:

                #skip blank nodes
                if  isinstance(s, rdflib.term.BNode) or \
                    isinstance(p, rdflib.term.BNode) or \
                    isinstance(o, rdflib.term.BNode):
                        continue
                try:
                    s = nsm.qname(s)
                except:
                    pass
                try:
                    p = nsm.qname(p)
                except:
                    pass
                try:
                    if isinstance(o, rdflib.term.Literal):
                        o = o.toPython()
                    else:
                        o = nsm.qname(o)
                except:
                    pass
                triples += [(s,p,o)]

            logger.debug("Importing:\n%s" % triples)
            self.store.add(triples)
        else:
            with open(filename, 'r') as triples:
                self.store.add([shlex.split(s.strip()) for s in triples.readlines()])

    @compat
    @api
    def listAgents(self):
        return list(self.models)

    @api
    def clear(self):
        logger.warn("Clearing the knowledge base!")
        self.store.clear()
        self.active_evts.clear()

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
        return self._api.keys()

    @api
    def about(self, resource, models = None):
        return self.store.about(resource, self.normalize_models(models))

    @compat
    @api
    def lookupForAgent(self, agent, resource):
        return self.lookup(resource, models = [agent])

    @api
    def lookup(self, resource, models = None):
        logger.info("Lookup for " + str(resource) + \
                    " in " + (str(models) if models else "any model."))

        about =  self.store.about(resource, self.normalize_models(models))
        if not about:
            return []

        return [(resource, "unknown")]

    @api
    def check(self, *args):
        logger.warn("'check' has been invoked, but no classification supported. Returning always True.")
        return True

    @api
    def exist(self, stmts, models = None):
        logger.info("Checking existence of " + str(stmts) + \
                    " in " + (str(models) if models else "any model."))
        stmts = [parse_stmt(s) for s in stmts]

        return self.store.has(stmts,
                              self.normalize_models(models))

    @api
    def revise(self, stmts, policy):

        if isinstance(stmts, (str, unicode)):
            raise KbServerError("A list of statements is expected")
        stmts = [parse_stmt(s) for s in stmts]

        if type(policy) != dict:
            raise KbServerError("Expected a dictionary as policy")
        models = self.normalize_models(policy.get('models', []))

        if policy["method"] in ["add", "safe_add"]:
            logger.info("Adding to " + str(list(models)) + ":\n\t- " + "\n\t- ".join([str(s) for s in stmts]))
            for model in models:
                self.store.add(stmts, model)

        if policy["method"] == "retract":
            logger.info("Deleting from " + str(list(models)) +":\n\t- " + "\n\t- ".join([str(s) for s in stmts]))
            for model in models:
                self.store.delete(stmts, model)

        if policy["method"] in ["update", "safe_update", "revision"]:
            logger.info("Updating " + str(list(models)) + " with:\n\t- " + "\n\t- ".join([str(s) for s in stmts]))
            for model in models:
                self.store.update(stmts, model)


        self.onupdate()

    @api
    def add(self, stmts, models = None):
        return self.revise(stmts,
                           {"method": "add",
                            "models": models})

    @compat
    @api
    def safeAdd(self, stmts):
        return self.revise(stmts, {"method": "safe_add"})


    @compat
    @api
    def addForAgent(self, agent, stmts):
        return self.add(stmts, [agent])

    @api
    def retract(self, stmts, models = None):
        return self.revise(stmts,
                           {"method": "retract",
                            "models": models})

    @compat
    @api
    def remove(self, stmts, models = None):
        return self.retract(stmts, models)

    @compat
    @api
    def removeForAgent(self, agent, stmts):
        return self.retract(stmts, [agent])


    @api
    def update(self, stmts, models = None):
        return self.revise(stmts,
                           {"method": "update",
                            "models": models})


    @compat
    @api
    def findForAgent(self, agent, var, stmts):
        return self.find([var], stmts, None, [agent])

    @api
    def find(self, vars, patterns, constraints = None, models = None):
        '''
        Depending on the arguments, three differents
        behaviours are possible:

        - if len(vars) == 1, 'find' returns a set of resources matching the patterns.
        - if len(vars) > 1:
            - if len(patterns) == 1, a list of statements matching the pattern
              is returned.
            - else, a list of dictionaries is returned with
            possible combination of values for the different variables. For
            instance, find(["?agent", "?action"], ["?agent desires ?action", "?action rdf:type Jump"])
            would return something like: [{"agent":"james", "action": "jumpHigh"}, {"agent": "laurel", "action":"jumpHigher"}]

        Note that 'constraints' is currently not supported.
        '''

        if not models:
            models = self.models

        patterns = [parse_stmt(p) for p in patterns]

        logger.info("Searching " + str(vars) + \
                    " in models " + str(models) + \
                    " matching:\n\t- " + "\n\t- ".join([str(p) for p in patterns]))

        res = self.store.query(vars, patterns, models)
        
        logger.info("Found: " + str(res))
        return res

    @api
    def findmpe(self, vars, pattern, constraints = None, models = None):
        """ Finds the most probable explanation. Strictly equivalent to
        'find' until we support probabilities.
        """
        return find(self, vars, pattern, constraints = None, models = None)

    @api
    def subscribe(self, type, trigger, var, patterns, models = None):

        models = self.normalize_models(models)
        patterns = [parse_stmt(p) for p in patterns]

        logger.info("Registering a new event: %s %s for %s on %s" % (type, trigger, var, patterns) + \
                    " in " + (str(models) if models else "any model."))

        event = Event(self, type, trigger, var, patterns, models)

        self.active_evts.add(event)

        return event.id

    @compat
    @api
    def discriminateForAgent(self, *args):
        raise NotImplementedError('discriminateForAgent not implemented in MinimalKB')

    @compat
    @api
    def getLabel(self, concept):
        res = self.store.query(["?label"], ["%s rdfs:label ?label" % concept], self.models)
        if not res:
            logger.info("Found no label for %s" % concept)
            return concept
        else:
            logger.info("Found label %s for %s" % (res[0], concept))
            return res[0]

    
    @compat
    @api
    def getDirectClassesOf(self, concept):
        return self.getClassesOf(concept, True)

    @compat
    @api
    def getClassesOf(self, concept, direct = False):
        classes = self.classesof(concept, direct)

        return {cls : self.getLabel(cls) for cls in classes}

    @api
    def classesof(self, concept, direct = False, models = None):
        models = self.normalize_models(models)
        return self.store.classesof(concept, direct, models)

    ################################################################################
    ################################################################################
    def onupdate(self):
        for e in self.active_evts:
            if e.evaluate():
                logger.info("Event %s triggered." % e.id)
                self.triggered_evts.append(e)
                if not e.valid:
                    self.active_evts.discard(e)
        self.classify()

    def start_reasoner(self, *args):
        self._reasoner = Process(target = start_reasoner, args = ('kb.db',))
        self._reasoner.start()

    def classify(self, *args):
        #p = Process(target = classifyOnce, args = ('kb.db',))
        #p.join()
        pass

    def normalize_models(self, models):
        """ If 'models' is None, [] or contains 'all', then
        returns the set of all models known to the KB.
        Else, add the models to the list of all models, and return
        only the models passed as argument.
        """
        if models:
            if "all" in models:
                return self.models
            else:
                if isinstance(models, (str, unicode)):
                    models = [models]
                #add to the set of all models
                self.models = self.models | set(models)
                return set(models)
        else:
            return self.models

    def execute(self, name, *args):
        f = getattr(self, name)
        if hasattr(f, "_compat"):
                logger.warn("Using non-standard method %s. This may be " % f.__name__ + \
                        "removed in the future!")
        
        if args:
            return f(*args)
        else:
            return f()

