#!/usr/bin/env python
# -*- coding: utf-8 -*-

from kb_msgs.srv import Manage
from kb_msgs.srv import Revise
from kb_msgs.srv import Query
from kb_msgs.srv import About
from kb_msgs.srv import Lookup
from kb_msgs.srv import Sparql
from kb_msgs.srv import Event
from kb_msgs.msg import ActiveConcepts
from std_msgs.msg import String

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import asyncio
import json
import random
import shlex
import threading
from typing import Coroutine

MANAGE_SRV = Manage, "/kb/manage"
REVISE_SRV = Revise, "/kb/revise"
QUERY_SRV = Query, "/kb/query"
ABOUT_SRV = About, "/kb/about"
LABEL_SRV = About, "/kb/label"
DETAILS_SRV = About, "/kb/details"
LOOKUP_SRV = Lookup, "/kb/lookup"
SPARQL_SRV = Sparql, "/kb/sparql"
EVENTS_SRV = Event, "/kb/events"
ACTIVE_CONCEPTS_TOPIC = "/kb/active_concepts"
EVENTS_NS = EVENTS_SRV[1] + "/"


class KbError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class KB:
    """
    Pythonic wrapper around the ROS API of KnowledgeCore.

    To quickly try it, using a built-in ROS node and executor:

    ```
    from knowledge_core.api import KB

    kb = KB()
    kb += ["john rdf:type Human", "tiago rdf:type Robot", "john likes tiago"]
    kb["?human rdf:type Human", "?human likes ?robot", "?robot rdf:type Robot"]
    ```

    To use it in a custom ROS node, pass the node to the constructor and spin the node using a
    MultiThreadedExecutor, e.g.:

    ```
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from knowledge_core.api import KB

    MyNodeClass(Node):
        def __init__(self, name):
            super().__init__(name)
            self.kb = KB(self)

    def main():
        rclpy.init()
        node = MyNodeClass("my_node")
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    ```

    """

    def __init__(self, node: Node = None):
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = Node("kb")
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self.executor.spin)
            self.thread.start()
            self.self_running = True
            print("self running KB")
        else:
            self.node = node
            self.self_running = False

        self.cb_group = MutuallyExclusiveCallbackGroup()

        self._manage_srv = self.node.create_client(*MANAGE_SRV, callback_group=self.cb_group)
        self._revise_srv = self.node.create_client(*REVISE_SRV, callback_group=self.cb_group)
        self._query_srv = self.node.create_client(*QUERY_SRV, callback_group=self.cb_group)
        self._about_srv = self.node.create_client(*ABOUT_SRV, callback_group=self.cb_group)
        self._label_srv = self.node.create_client(*LABEL_SRV, callback_group=self.cb_group)
        self._details_srv = self.node.create_client(*DETAILS_SRV, callback_group=self.cb_group)
        self._lookup_srv = self.node.create_client(*LOOKUP_SRV, callback_group=self.cb_group)
        self._sparql_srv = self.node.create_client(*SPARQL_SRV, callback_group=self.cb_group)
        self._events_srv = self.node.create_client(*EVENTS_SRV, callback_group=self.cb_group)

        while not self._manage_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                f'service {MANAGE_SRV[1]} not available, waiting again...')
        while not self._revise_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                f'service {REVISE_SRV[1]} not available, waiting again...')
        while not self._query_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                f'service {QUERY_SRV[1]} not available, waiting again...')
        while not self._about_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                f'service {ABOUT_SRV[1]} not available, waiting again...')
        while not self._label_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                f'service {LABEL_SRV[1]} not available, waiting again...')
        while not self._details_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                f'service {DETAILS_SRV[1]} not available, waiting again...')
        while not self._lookup_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                f'service {LOOKUP_SRV[1]} not available, waiting again...')
        while not self._sparql_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                f'service {SPARQL_SRV[1]} not available, waiting again...')
        while not self._events_srv.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                f'service {EVENTS_SRV[1]} not available, waiting again...')

        self._evt_subscribers = {}

        self._active_concepts_sub = self.node.create_subscription(
            ActiveConcepts, ACTIVE_CONCEPTS_TOPIC, self._on_active_concept, 1
        )
        self._active_concepts_callbacks = []

    def __del__(self):
        if self.self_running:
            self.executor.shutdown()
            self.thread.join()

    def on_active_concept(self, callback):
        self._active_concepts_callbacks.append(callback)

    def _on_active_concept(self, msg):
        for cb in self._active_concepts_callbacks:
            cb(msg.concepts)

    """
    Helper function to run an async function as a standard blocking one.

    It attempts to execute the coroutine in a new event loop.
    If it fails due to a running event loop already existing,
    it uses the latter to execute the coroutine.
    """
    @staticmethod
    def async_run(coro: Coroutine, *args, **kwargs):
        try:
            return asyncio.run(coro(*args, **kwargs))
        except RuntimeError:
            return asyncio.get_running_loop().run_until_complete(coro(*args, **kwargs))

    def hello(self):
        return self.async_run(self._hello)

    async def _hello(self):
        res = await self._manage_srv.call_async(
            Manage.Request(action=Manage.Request.STATUS))
        return json.loads(res.json)["name"]

    def stats(self):
        return self.async_run(self._stats)

    async def _stats(self):
        res = await self._manage_srv.call_async(
            Manage.Request(action=Manage.Request.STATUS))
        return json.loads(res.json)

    def clear(self):
        return self.async_run(self._clear)

    async def _clear(self):
        await self._manage_srv.call_async(
            Manage.Request(action=Manage.Request.CLEAR))

    def subscribe(self, pattern, callback, one_shot=False, models=[]):
        return self.async_run(self._subscribe, pattern, callback, one_shot, models)

    async def _subscribe(self, pattern, callback, one_shot=False, models=[]):
        """
        Allow to subscribe to an event, and get notified when the event is triggered.

        >>> def onevent(evt):
        >>>     print("In callback. Got evt %s" % evt)
        >>>
        >>> self.kb.subscribe(["?o isIn room"], onevent)
        >>> self.kb += ["alfred isIn room"]
        >>> # 'onevent' get called
        In callback. Got evt [u'alfred']


        The 'var' parameter can be used with the 'NEW_INSTANCE' type of event to
        tell which variable must be returned.

        The 'models' parameter allows for registering an event in a specific list
        of models. By default, the pattern is monitored on every models.

        Returns the event id of the newly created event.
        """

        class EvtRelay:
            """Small callback wrapper to load the json event into a python object."""

            def __init__(self, cb):
                self.cb = cb

            def callback(self, evt):
                self.cb(json.loads(evt.data))

        if isinstance(pattern, str):
            pattern = [pattern]

        evt = await self._events_srv.call_async(Event.Request(
            patterns=pattern, one_shot=one_shot, models=models))

        # check if we already have a registered an identical event pattern,
        # with an identifical callback

        if evt.id in self._evt_subscribers and [
            x
            for x in filter(
                lambda cb, callback=callback: cb[0] == callback,
                self._evt_subscribers[evt.id],
            )
        ]:
            self.node.get_logger().warn(
                "Same event already subscribed to with same callback. Skipping."
            )
            return evt.id

        evt_relay = EvtRelay(callback)
        self._evt_subscribers.setdefault(evt.id, []).append(
            (
                callback,
                self.node.create_subscription(
                    String, evt.topic, evt_relay.callback, 10, callback_group=self.cb_group)
            )
        )

        self.node.get_logger().debug("New event successfully registered with ID " + evt.id)

        return evt.id

    def find(self, patterns, vars=[], models=[]):
        return self.async_run(self._find, patterns, vars, models)

    async def _find(self, patterns, vars=[], models=[]):
        res = await self._query_srv.call_async(Query.Request(
            patterns=patterns, vars=vars, models=models))

        if not res.success:
            raise KbError(res.error_msg)

        return json.loads(res.json)

    def about(self, term, models=[]):
        return self.async_run(self._about, term, models)

    async def _about(self, term, models=[]):
        res = await self._about_srv.call_async(
            About.Request(term=term, models=models))

        if not res.success:
            raise KbError(res.error_msg)

        return json.loads(res.json)

    def label(self, term, lang=None, models=[]):
        return self.async_run(self._label, term, lang, models)

    async def _label(self, term, lang=None, models=[]):
        """
        Return the label associated to the term.

        If no label, returns the term itself.
        If lang is specified (2 letter code, eg `en`, `fr`), returns the label
        translated in that language, if available.
        If not specified, or if the desired language is not available, fallback
        to English.

        """
        res = await self._label_srv.call_async(
            About.Request(term=term, models=models))

        if not res.success:
            raise KbError(res.error_msg)

        labels = json.loads(res.json)

        if lang is None:
            return labels["default"]
        else:
            return labels.get(lang, labels["default"])

    def details(self, term, models=[]):
        return self.async_run(self._details, term, models)

    async def _details(self, term, models=[]):
        res = await self._details_srv.call_async(
            About.Request(term=term, models=models))

        if not res.success:
            raise KbError(res.error_msg)

        return json.loads(res.json)

    def lookup(self, query, models=[]):
        return self.async_run(self._lookup, query, models)

    async def _lookup(self, query, models=[]):
        res = await self._lookup_srv.call_async(
            Lookup.Request(query=query, models=models))

        if not res.success:
            raise KbError(res.error_msg)

        return json.loads(res.json)

    def __getitem__(self, *args):
        """
        Offer idiomatic Python syntax (accessors) to querying the ontology server.

        It uses the args (be it a string or a set of strings) to find concepts
        that match the pattern.
        An optional 'models' parameter can be given to specify the list of models the
        query is executed on.

        Depending on the argument, 4 differents behaviours are possible:

        - with a string that can not be lexically split into 3 tokens (ie, a string
          that do not look like a ``s p o`` tuple), a lookup is performed, and matching
          resource are returned
        - with a single ``s p o`` pattern:
            - if only one of s, p, o is an unbound variable, returns the list of resources
              matching this pattern.
            - if 2 or 3 of the tokens are unbound variables (like ``kb["* * *"]``
              or ``kb["* rdf:type *"]``), a list of statements matching the pattern
              is returned.
        - with a list of patterns, a list of dictionaries is returned with
          possible combination of values for the different variables. For
          instance, ``kb[["?agent desires ?action", "?action rdf:type Jump"]]``
          would return something like: ``[{"agent":"james", "action":
          "jumpHigh"}, {"agent": "laurel", "action":"jumpHigher"}]``

        Attention: if more than one argument is passed, and if the last
        argument is a list, this list is used as the set of models to execute
        the query on. If not such list is provided, the query is executed on
        all models.

        Use example:

        .. code:: python

            import kb

            kb = KB()

            for agent in kb["* rdf:type Agent"]:
                #...

            if kb["* livesIn ?house", "?house isIn toulouse", ['GERALD']]:
                #...

            #Assuming 'toulouse' has label "ville rose":
            city_id = kb["ville rose"]

        """
        args = args[0]

        # First, take care of models
        models = []
        if len(args) > 1 and isinstance(args[-1], list):
            models = args[-1]
            args = args[:-1]

        def get_vars(s):
            return [v for v in s if v.startswith("?")]

        # Single argument
        if isinstance(args, str) or len(args) == 1:
            pattern = args if isinstance(args, str) else args[0]
            toks = shlex.split(pattern)
            if len(toks) == 3:
                pattern = self._replacestar(toks)
                vars = get_vars(pattern)
                return self.find(["%s %s %s" % pattern], vars, models)
            else:
                lookup = self.lookup(pattern, models)
                return [concept[0] for concept in lookup]

        # List of patterns
        else:
            patterns = [self._replacestar(shlex.split(p)) for p in args]
            allvars = set()
            for p in patterns:
                allvars |= set(get_vars(p))

            return self.find(["%s %s %s" % p for p in patterns], list(allvars), models)

    def exist(self, pattern, models=[]):
        return len(self.find(pattern, models)) != 0

    def __contains__(self, pattern):
        """
        Check if a concept or statement(s) is asserted/infered in the knowledge base.

        The concept can be referenced by its URI or label; the statement or a
        set of statement is present are passed as a list of strings ("s p o")

        This allows syntax like:

        .. code:: python

            if 'Toto' in kb:
                #...
            if 'toto sees tata' in kb:
                #...

        """
        toks = shlex.split(pattern)
        if len(toks) == 3:
            pattern = self._replacestar(toks)
            return self.exist(["%s %s %s" % pattern])
        else:
            return True if self.lookup(pattern) else False

    def revise(self, stmts, policy):

        if policy["method"] in ["add", "update"]:
            return self.update(
                stmts,
                models=policy.setdefault("models", []),
                lifespan=policy.setdefault("lifespan", 0),
            )
        elif policy["method"] in ["remove", "retract"]:
            return self.remove(stmts, models=policy.setdefault("models", []))
        else:
            raise KbError("unknown revise policy %s" % policy["method"])

    def update(self, stmts, models=[], lifespan=0):
        return self.async_run(self._update, stmts, models, lifespan)

    async def _update(self, stmts, models=[], lifespan=0):

        if not (type(stmts) == list):
            stmts = [stmts]

        res = await self._revise_srv.call_async(
            Revise.Request(
                method=Revise.Request.UPDATE,
                statements=stmts,
                models=models,
                lifespan=Duration(seconds=lifespan).to_msg(),
            ))

        if not res.success:
            raise KbError(res.error_msg)

    def add(self, stmts, models=[], lifespan=0):
        self.update(stmts, models, lifespan)

    def remove(self, stmts, models=[]):
        return self.async_run(self._remove, stmts, models)

    async def _remove(self, stmts, models=[]):

        res = await self._revise_srv.call_async(
            Revise.Request(
                method=Revise.Request.REMOVE, statements=stmts, models=models
            ))

        if not res.success:
            raise KbError(res.error_msg)

    def __iadd__(self, stmts):
        """
        Allow to easily add new statements to the ontology with the ``+=`` operator.

        It can only add statement to the default robot's model (other agents' model are
        not accessible).

        .. code:: python

            kb = KB(<host>, <port>)
            kb += "toto likes icecream"
            kb += ["toto loves tata", "tata rdf:type Robot"]

        """
        if not (type(stmts) == list):
            stmts = [stmts]

        self.update(stmts)

        return self

    def __isub__(self, stmts):
        """
        Allow to easily retract statements from the ontology with the ``-=`` operator.

        It can only retract statements from the robot's model (other agents'
        model are not accessible).  If a statement doesn't exist, it is
        silently skipped.

        .. code:: python

            kb = KB(<host>, <port>)
            kb -= "toto likes icecream"
            kb -= ["toto loves tata", "tata rdf:type Robot"]

        """
        if not (type(stmts) == list):
            stmts = [stmts]

        self.remove(stmts)

        return self

    def _replacestar(self, pattern):
        res = []
        for tok in pattern:
            if tok == "*":
                res.append(
                    "?__"
                    + "".join(
                        random.sample(
                            "abcdefghijklmopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ", 5
                        )
                    )
                )
            else:
                res.append(tok)
        return tuple(res)
