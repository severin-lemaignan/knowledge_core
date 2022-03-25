import logging

logger = logging.getLogger("minimalKB." + __name__)
DEBUG_LEVEL = logging.DEBUG


import shlex
from rdflib import Graph, URIRef
from rdflib.namespace import Namespace, NamespaceManager, RDF, RDFS, OWL, XSD

from minimalkb.exceptions import KbServerError

DEFAULT_NAMESPACE = ("chili", "http://chili-research.epfl.ch#")


class RDFlibStore:
    def __init__(self):

        self.default_ns = Namespace(DEFAULT_NAMESPACE[1])
        self.models = {}

    def clear(self):
        """Empties all knowledge models."""
        self.models = {}

    def add(self, stmts, model="default"):
        """Add the given statements to the given model."""

        # creates a model if necessary
        graph = self._create_or_get_graph(model)

        for stmt in stmts:
            graph.add(self._parse_stmt(stmt))

    def delete(self, stmts, model="default"):
        """remove the given statements from the given model."""
        if not model in self.models:
            logger.warn("Trying to remove statements from an unexisting model!")
            return

        for stmt in stmts:
            self.models[model].remove(self._parse_stmt(stmt))

    def update(self, stmts, model="default"):
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
        resource = self._parse_resource(resource)

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
            namespace_manager.bind(DEFAULT_NAMESPACE[0], self.default_ns)
            graph.ns_manager = namespace_manager
            self.models[name] = graph

        return self.models[name]

    def _parse_stmt(self, stmt):
        s, p, o = shlex.split(stmt)
        return (
            self._parse_resource(s),
            self._parse_resource(p),
            self._parse_resource(o),
        )

    def _parse_resource(self, resource):
        tokens = resource.split(":")
        if len(tokens) == 1:
            return self.default_ns[resource]

        ns, resource = tokens

        if ns == "rdf":
            return RDF[resource]
        elif ns == "rdfs":
            return RDFS[resource]
        elif ns == "owl":
            return OWL[resource]

        raise KbServerError("Unknown namespace <%s> for resource %s" % (ns, resource))

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
