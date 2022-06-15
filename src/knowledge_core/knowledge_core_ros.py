import sys

try:
    import rospy
except ImportError:
    print(
        "Unable to load the ROS support. You might want to run with --no-ros"
        " to use KnowledgeCore without ROS support enabled."
    )
    sys.exit(1)

import json

from knowledge_core.exceptions import KbServerError

from knowledge_core.srv import Manage, Revise, Query, About, Lookup, Sparql, Event
from std_msgs.msg import String

EVENTS_TOPIC_NS = "/kb/events/"


class KnowledgeCoreROS:
    def __init__(self, kb):
        self.kb = kb

        rospy.init_node("knowledge_core", disable_signals=True)

        self.update_sub = rospy.Subscriber("/kb/add_fact", String, self.on_update_fact)
        self.retract_sub = rospy.Subscriber(
            "/kb/remove_fact", String, self.on_retract_fact
        )

        self.services = {
            "manage": rospy.Service("kb/manage", Manage, self.handle_manage),
            "revise": rospy.Service("kb/revise", Revise, self.handle_revise),
            "query": rospy.Service("kb/query", Query, self.handle_query),
            "about": rospy.Service("kb/about", About, self.handle_about),
            "lookup": rospy.Service("kb/lookup", Lookup, self.handle_lookup),
            "event": rospy.Service("kb/events", Event, self.handle_new_event),
            "sparql": rospy.Service("kb/sparql", Sparql, self.handle_sparql),
        }

        rospy.loginfo(
            """
KnowledgeCore
=============

Knowledge base started.

Available topics:
- /kb/add_fact [std_msgs/String]
- /kb/remove_fact [std_msgs/String]
- /kb/events/<id> [std_msgs/String] for 
  each subscribed event

Available services:
- /kb/manage [knowledge_core/Manage]
- /kb/revise [knowledge_core/Revise]
- /kb/query [knowledge_core/Query]
- /kb/about [knowledge_core/About]
- /kb/lookup [knowledge_core/Lookup]
- /kb/sparql [knowledge_core/Sparql]
- /kb/events [knowledge_core/Event]

"""
        )

    def on_update_fact(self, msg):
        self.kb.update([msg.data])

    def on_retract_fact(self, msg):
        self.kb.remove([msg.data])

    def handle_manage(self, req):

        from knowledge_core.srv import ManageResponse, ManageRequest

        try:
            if req.action == ManageRequest.CLEAR:
                self.kb.clear()
                return ManageResponse(success=True, error_msg="")

            elif req.action == ManageRequest.LOAD:
                if len(req.parameters) != 1:
                    return ManageResponse(
                        success=False,
                        error_msg="'load' expects 'parameters' to contain "
                        "exactly one URI pointing to the ontology to load",
                    )

                self.kb.load(req.parameters[0], models=req.models)
                return ManageResponse(success=True, error_msg="")

            elif req.action == ManageRequest.SAVE:
                if len(req.parameters) != 2:
                    return ManageResponse(
                        success=False,
                        error_msg="'save' expects 'parameters' to contain "
                        "exactly two values: the path and the basename. The "
                        "knowledge base will be saved as "
                        "`path/basename-<model>.xml (one file per model)",
                    )

                self.kb.save(req.parameters[0], req.parameters[1], models=req.models)
                return ManageResponse(success=True, error_msg="")

            elif req.action == ManageRequest.STATUS:
                status = {
                    "name": self.kb.hello(),
                    "version": self.kb.version(),
                    "reasoning_enabled": self.kb.reasoner_enabled,
                }
                return ManageResponse(
                    success=True, json=json.dumps(status), error_msg=""
                )
            else:
                return ManageResponse(
                    success=False, error_msg="Unknown 'manage' action: %s" % req.action
                )
        except KbServerError as kbe:
            return ManageResponse(success=False, error_msg=str(kbe))

    def handle_revise(self, req):

        from knowledge_core.srv import ReviseResponse

        policy = {"method": req.method, "models": req.models}
        try:
            self.kb.revise(req.statements, policy)

            return ReviseResponse(success=True, error_msg="")
        except KbServerError as kbe:
            return ReviseResponse(success=False, error_msg=str(kbe))

    def handle_query(self, req):

        from knowledge_core.srv import QueryResponse

        try:
            res = self.kb.find(req.patterns, req.vars, req.models)
            return QueryResponse(success=True, error_msg="", json=json.dumps(res))
        except KbServerError as kbe:
            return QueryResponse(success=False, error_msg=str(kbe))

    def handle_about(self, req):

        from knowledge_core.srv import AboutResponse

        try:
            res = self.kb.about(req.term, req.models)
            return AboutResponse(success=True, error_msg="", json=json.dumps(res))
        except KbServerError as kbe:
            return AboutResponse(success=False, error_msg=str(kbe))

    def handle_lookup(self, req):

        from knowledge_core.srv import LookupResponse

        try:
            res = self.kb.lookup(req.query, req.models)
            return LookupResponse(success=True, error_msg="", json=json.dumps(res))
        except KbServerError as kbe:
            return LookupResponse(success=False, error_msg=str(kbe))

    def handle_sparql(self, req):

        from knowledge_core.srv import SparqlResponse

        try:
            res = self.kb.sparql(req.query, req.models)
            return SparqlResponse(success=True, error_msg="", json=json.dumps(res))
        except KbServerError as kbe:
            return SparqlResponse(success=False, error_msg=str(kbe))

    def handle_new_event(self, req):

        from knowledge_core.srv import EventResponse

        class EvtRelay:
            """This inner class is used to 'pretend' the ROS backend is a regular 'client' to the knowledge base
            server. This is required for events as they are asynchronous -> the knowledge base server needs to keep
            a handle to all the 'clients' that have subscribed to an event for later notification.

            See eg KnowledgeCore.onupdate()
            """

            def __init__(self, evt_id, kb):
                self.evt = evt_id
                self.kb = kb
                self.has_been_subscribed = False
                self.pub = rospy.Publisher(
                    EVENTS_TOPIC_NS + evt_id, String, queue_size=10, latch=False
                )

            def sendmsg(self, msg):

                if self.pub.impl is None:
                    # publisher already unregistered from a different thread -- just ignore that message
                    return

                if self.pub.get_num_connections() == 0:
                    if self.has_been_subscribed:
                        rospy.logwarn(
                            "No-one listing to event <%s> anymore; removing it."
                            % self.evt
                        )
                        self.kb.remove_event(self.evt)
                        self.pub.unregister()
                else:
                    self.has_been_subscribed = True

                    evt = msg[1]
                    rospy.loginfo(
                        "Event <" + evt.id + "> triggered. Notifying ROS clients"
                    )
                    evt_msg = String()
                    evt_msg.data = json.dumps(evt.content)
                    self.pub.publish(evt_msg)

        evt = self.kb.subscribe(req.patterns, req.one_shot, req.models)

        evt_relay = EvtRelay(evt, self.kb)

        rospy.logwarn("Subscribed to event " + evt)

        self.kb.eventsubscriptions.setdefault(evt, []).append(evt_relay)

        return EventResponse(id=evt, topic=EVENTS_TOPIC_NS + evt)

    def shutdown(self):

        rospy.signal_shutdown("KnowledgeCore closing")
