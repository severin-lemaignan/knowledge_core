#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from knowledge_core.srv import Manage, ManageRequest
from knowledge_core.srv import Revise, ReviseRequest
from knowledge_core.srv import Query
from knowledge_core.srv import About
from knowledge_core.srv import Lookup
from knowledge_core.srv import Sparql
from knowledge_core.srv import Event
from std_msgs.msg import String

import json
import random
import shlex

MANAGE_SRV = "/kb/manage"
REVISE_SRV = "/kb/revise"
QUERY_SRV = "/kb/query"
ABOUT_SRV = "/kb/about"
LOOKUP_SRV = "/kb/lookup"
SPARQL_SRV = "/kb/sparql"
EVENTS_SRV = "/kb/events"
EVENTS_NS = EVENTS_SRV + "/"


class KbError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)


class KB:
    def __init__(self):
        rospy.wait_for_service(MANAGE_SRV)
        rospy.wait_for_service(REVISE_SRV)
        rospy.wait_for_service(QUERY_SRV)
        rospy.wait_for_service(ABOUT_SRV)
        rospy.wait_for_service(LOOKUP_SRV)
        rospy.wait_for_service(SPARQL_SRV)
        rospy.wait_for_service(EVENTS_SRV)

        self._manage_srv = rospy.ServiceProxy(MANAGE_SRV, Manage)
        self._revise_srv = rospy.ServiceProxy(REVISE_SRV, Revise)
        self._query_srv = rospy.ServiceProxy(QUERY_SRV, Query)
        self._about_srv = rospy.ServiceProxy(ABOUT_SRV, About)
        self._lookup_srv = rospy.ServiceProxy(LOOKUP_SRV, Lookup)
        self._sparql_srv = rospy.ServiceProxy(SPARQL_SRV, Sparql)
        self._events_srv = rospy.ServiceProxy(EVENTS_SRV, Event)

    def hello(self):
        res = self._manage_srv(action=ManageRequest.STATUS)
        return json.loads(res.json)["name"]

    def stats(self):
        res = self._manage_srv(action=ManageRequest.STATUS)
        return json.loads(res.json)

    def clear(self):
        self._manage_srv(action=ManageRequest.CLEAR)

    def subscribe(
        self,
        pattern,
        callback,
        one_shot=False,
        models=None,
    ):
        """Allows to subscribe to an event, and get notified when the event is
        triggered.

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
            """small callback wrapper to load the json event into a python object"""

            def __init__(self, cb):
                self.cb = cb

            def callback(self, evt):
                self.cb(json.loads(evt.data))

        if isinstance(pattern, str):
            pattern = [pattern]

        evt = self._events_srv(patterns=pattern, one_shot=one_shot, models=models)

        evt_relay = EvtRelay(callback)
        rospy.Subscriber(evt.topic, String, evt_relay.callback, queue_size=10)

        rospy.logdebug("New event successfully registered with ID " + evt.id)

        return evt.id

    def find(self, patterns, vars=[], models=[]):
        res = self._query_srv(patterns=patterns, vars=vars, models=models)

        if not res.success:
            raise KbError(res.error_msg)

        return json.loads(res.json)

    def about(self, term, models=[]):
        res = self._about_srv(term=term, models=models)

        if not res.success:
            raise KbError(res.error_msg)

        return json.loads(res.json)

    def lookup(self, query, models=[]):
        res = self._lookup_srv(query=query, models=models)

        if not res.success:
            raise KbError(res.error_msg)

        return json.loads(res.json)

    def __getitem__(self, *args):
        """This method introduces a different way of querying the ontology server.
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
        models = None
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
        """This will return 'True' is either a concept - described by its ID or
        label- or a statement or a set of statement is present (or can be infered)
        in the ontology.

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

        res = self._revise_srv(
            method=ReviseRequest.UPDATE, statements=stmts, models=models
        )

        if not res.success:
            raise KbError(res.error_msg)

    def add(self, stmts, models=[], lifespan=0):
        self.update(stmts, models, lifespan)

    def remove(self, stmts, models=[]):

        res = self._revise_srv(
            method=ReviseRequest.REMOVE, statements=stmts, models=models
        )

        if not res.success:
            raise KbError(res.error_msg)

    def __iadd__(self, stmts):
        """This method allows to easily add new statements to the ontology
        with the ``+=`` operator.
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
        """This method allows to easily retract statements from the ontology
        with the ``-=`` operator.
        It can only add statement to the robot's model (other agents' model are
        not accessible).
        If a statement doesn't exist, it is silently skipped.

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
