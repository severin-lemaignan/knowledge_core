import logging

logger = logging.getLogger("minimalKB." + __name__)
DEBUG_LEVEL = logging.DEBUG


from rdflib import Graph, URIRef
from rdflib.namespace import NamespaceManager

from minimalkb.kb import DEFAULT_MODEL

from minimalkb.exceptions import KbServerError
from minimalkb.helpers import memoize


class RDFlibStore:
    def __init__(self):

        self.models = {}

    def clear(self):
        """Empties all knowledge models."""
        self.models = {}

    def add(self, stmts, model=DEFAULT_MODEL, lifespan=0, replace=False):
        """Add the given statements to the given model."""

        # creates a model if necessary
        graph = self._create_or_get_graph(model)

        for stmt in stmts:
            graph.add(stmt)

    def delete(self, stmts, model=DEFAULT_MODEL):
        """remove the given statements from the given model."""
        if not model in self.models:
            logger.warn("Trying to remove statements from an unexisting model!")
            return

        for stmt in stmts:
            self.models[model].remove(stmt)

    def update(self, stmts, model=DEFAULT_MODEL):
        """Add the given statements to the given model, updating the statements
        with a functional predicate.
        """

        logger.warn(
            "Currently with RDFlib backend, update is strictly equivalent to "
            + "add (ie, no functional property check"
        )

        self.add(stmts, model)

    def about(self, resource, models):
        """Returns all statements involving the resource."""
        res = []
        resource = parse_term(resource)

        for model in models:
            if not model in self.models:
                logger.warn("Trying to list statments from an unexisting model!")
                return []

            graph = self.models[model]
            res += [
                self._format_stmt(graph, s, p, o)
                for s, p, o in graph.triples((resource, None, None))
            ]
            res += [
                self._format_stmt(graph, s, p, o)
                for s, p, o in graph.triples((None, resource, None))
            ]
            res += [
                self._format_stmt(graph, s, p, o)
                for s, p, o in graph.triples((None, None, resource))
            ]

        return res

    def has(self, stmts, models):
        """Returns true if the statements or partial statements
        are present in the knowledge models.
        """
        raise NotImplementedError()

    def query(self, vars, patterns, models):
        raise NotImplementedError()

    def classesof(self, concept, direct, models):
        """Returns the RDF classes of the concept."""

        raise NotImplementedError()

    #############################################################################

    def _create_or_get_graph(self, name):
        if name not in self.models:
            graph = Graph()
            namespace_manager = NamespaceManager(Graph())
            namespace_manager.bind(DEFAULT_NAMESPACE[0], default_ns)
            graph.ns_manager = namespace_manager
            self.models[name] = graph

        return self.models[name]

    def _format_stmt(self, graph, s, p, o):

        nsm = graph.ns_manager

        stmt = [nsm.qname(s), nsm.qname(p), nsm.qname(o)]
        # remove ns prefix for default namespace
        default_prefix = DEFAULT_NAMESPACE[0] + ":"
        return [
            s[len(default_prefix) :] if s.startswith(default_prefix) else s
            for s in stmt
        ]

    def __str__(self):
        res = ""
        for name, graph in self.models.items():
            res += name + "\n" + graph.serialize(format="n3") + "\n"
        return res
