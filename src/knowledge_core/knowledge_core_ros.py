import sys

try:
    import rclpy
    from rclpy.node import Node
except ImportError:
    print(
        "Unable to load the ROS 2 support. You might want to run with --no-ros"
        " to use KnowledgeCore without ROS 2 support enabled (or check an older"
        " version for ROS 1 support)."
    )
    sys.exit(1)

import json

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from knowledge_core.exceptions import KbServerError

from kb_msgs.srv import Manage, Revise, Query, About, Lookup, Sparql, Event
from std_msgs.msg import String

EVENTS_TOPIC_NS = "/kb/events/"

DIAGNOSTICS_FREQUENCY = 1  # Hz


def to_sec(duration_msg):
    return duration_msg.sec + duration_msg.nanosec / 1e9


class KnowledgeCoreROS(Node):
    """
    ROS wrapper around the KnowledgeCore knowledge base.

    To try it:
    ```
    import sys
    import rclpy

    from knowledge_core.kb import KnowledgeCore
    from knowledge_core.knowledge_core_ros import KnowledgeCoreROS

    kb = KnowledgeCore()

    rclpy.init(args=sys.argv)

    ros = KnowledgeCoreROS(kb)

    rclpy.spin()
    ```

    You can then interact with the knowledge base via its topics and services.
    """

    def __init__(self, kb):
        super().__init__('knowledge_core')

        self.kb = kb

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 1
        )
        self.last_diagnostics_ts = self.get_clock().now()

        self.update_sub = self.create_subscription(
            String, "/kb/add_fact", self.on_update_fact, 10)
        self.retract_sub = self.create_subscription(
            String, "/kb/remove_fact", self.on_retract_fact, 10)

        self.create_service(Manage, "kb/manage",  self.handle_manage)
        self.create_service(Revise, "kb/revise",  self.handle_revise)
        self.create_service(Query, "kb/query",  self.handle_query)
        self.create_service(About, "kb/about",  self.handle_about)
        self.create_service(Lookup, "kb/lookup",  self.handle_lookup)
        self.create_service(Event, "kb/events",  self.handle_new_event)
        self.create_service(Sparql, "kb/sparql",  self.handle_sparql)

        self.get_logger().info(
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
- /kb/manage [kb_msgs/Manage]
- /kb/revise [kb_msgs/Revise]
- /kb/query [kb_msgs/Query]
- /kb/about [kb_msgs/About]
- /kb/lookup [kb_msgs/Lookup]
- /kb/sparql [kb_msgs/Sparql]
- /kb/events [kb_msgs/Event]

"""
        )

    def on_update_fact(self, msg):
        try:
            self.kb.update([msg.data])
        except KbServerError as kse:
            self.get_logger().error(str(kse))

    def on_retract_fact(self, msg):
        try:
            self.kb.remove([msg.data])
        except KbServerError as kse:
            self.get_logger().error(str(kse))

    def handle_manage(self, req, response):

        from kb_msgs.srv import Manage

        try:
            if req.action == Manage.Request.CLEAR:
                self.kb.clear()
                response.success = True
                response.error_msg = ""
                return response

            elif req.action == Manage.Request.LOAD:
                if len(req.parameters) != 1:
                    response.success = False
                    response.error_msg = "'load' expects 'parameters' to contain exactly " \
                                         "one URI pointing to the ontology to load"
                    return response

                self.kb.load(req.parameters[0], models=req.models)
                response.success = True
                response.error_msg = ""
                return response

            elif req.action == Manage.Request.SAVE:
                if len(req.parameters) != 2:
                    response.success = False
                    response.error_msg = "'save' expects 'parameters' to contain "
                    "exactly two values: the path and the basename. The "
                    "knowledge base will be saved as "
                    "`path/basename-<model>.xml (one file per model)"
                    return response

                self.kb.save(
                    req.parameters[0], req.parameters[1], models=req.models)
                response.success = True
                response.error_msg = ""
                return response

            elif req.action == Manage.Request.STATUS:
                status = {
                    "name": self.kb.hello(),
                    "version": self.kb.version(),
                    "reasoning_enabled": self.kb.reasoner_enabled,
                }
                response.success = True
                response.json = json.dumps(status)
                response.error_msg = ""
                return response
            else:
                response.success = False
                response.error_msg = "Unknown 'manage' action: %s" % req.action
                return response

        except KbServerError as kbe:
            response.success = False
            response.error_msg = str(kbe)
            return response

    def handle_revise(self, req, response):

        policy = {
            "method": req.method,
            "models": req.models,
            "lifespan": to_sec(req.lifespan),
        }
        try:
            self.kb.revise(req.statements, policy)

            response.success = True
            response.error_msg = ""
            return response

        except KbServerError as kbe:
            response.success = False
            response.error_msg = str(kbe)
            return response

    def handle_query(self, req, response):

        try:
            res = self.kb.find(req.patterns, req.vars, req.models)
            response.success = True
            response.json = json.dumps(res)
            response.error_msg = ""
            return response

        except KbServerError as kbe:
            response.success = False
            response.error_msg = str(kbe)
            return response

    def handle_about(self, req, response):

        try:
            res = self.kb.about(req.term, req.models)
            response.success = True
            response.json = json.dumps(res)
            response.error_msg = ""
            return response

        except KbServerError as kbe:
            response.success = False
            response.error_msg = str(kbe)
            return response

    def handle_lookup(self, req, response):

        try:
            res = self.kb.lookup(req.query, req.models)
            response.success = True
            response.json = json.dumps(res)
            response.error_msg = ""
            return response

        except KbServerError as kbe:
            response.success = False
            response.error_msg = str(kbe)
            return response

    def handle_sparql(self, req, response):

        try:
            res = self.kb.sparql(req.query, req.models)
            response.success = True
            response.json = json.dumps(res)
            response.error_msg = ""
            return response

        except KbServerError as kbe:
            response.success = False
            response.error_msg = str(kbe)
            return response

    def handle_new_event(self, req, response):

        class EvtRelay:
            """
            Inner class to 'pretend' the ROS backend is a regular 'client' to the KB server.

            This is required for events as they are asynchronous -> the
            knowledge base server needs to keep a handle to all the 'clients'
            that have subscribed to an event for later notification.

            See eg KnowledgeCore.onupdate()
            """

            def __init__(self, node, evt_id, kb):
                self.evt = evt_id
                self.node = node
                self.kb = kb
                self.has_been_subscribed = False
                self.pub = self.node.create_publisher(
                    String, EVENTS_TOPIC_NS + evt_id, 1)

            def __eq__(self, other):
                return self.evt == other.evt

            def __hash__(self):
                return hash(self.evt)

            def sendmsg(self, msg):

                if self.pub.get_subscription_count() == 0:
                    if self.has_been_subscribed:
                        self.node.get_logger().warn(
                            "No one listing to event <%s> anymore; removing it."
                            % self.evt
                        )
                        self.node.destroy_publisher(self.pub)
                        self.kb.eventsubscriptions[self.evt].remove(self)
                else:
                    self.has_been_subscribed = True

                    evt = msg[1]
                    self.node.get_logger().info(
                        "Event <" + evt.id + "> triggered. Notifying ROS clients."
                    )
                    evt_msg = String()
                    evt_msg.data = json.dumps(evt.content)
                    self.pub.publish(evt_msg)

        try:
            evt = self.kb.subscribe(req.patterns, req.one_shot, req.models)

            evt_relay = EvtRelay(self, evt, self.kb)

            prev_nb_cb = len(self.kb.eventsubscriptions.setdefault(evt, set()))

            self.kb.eventsubscriptions[evt].add(evt_relay)

            if len(self.kb.eventsubscriptions[evt]) > prev_nb_cb:
                self.get_logger().warn("Subscribed to event " + evt)
            else:
                self.get_logger().info(
                    "Event %s already subscribed to. Nothing to do." % evt)

            response.id = evt
            response.topic = EVENTS_TOPIC_NS + evt
            return response

        except KbServerError as kbe:
            self.get_logger().error(
                f"Unable to create event! Original error was: {str(kbe)}")
            response.id = ""
            return response

    def step(self):
        now = self.get_clock().now()

        if (now - self.last_diagnostics_ts).nanoseconds > 1e9 / DIAGNOSTICS_FREQUENCY:
            if self.kb.reasoner_enabled:
                msg = DiagnosticStatus(
                    level=DiagnosticStatus.OK,
                    name="Reasoning: Knowledge base",
                    message="Knowledge base running, with OWL/RDF reasoner enabled",
                )
            else:
                msg = DiagnosticStatus(
                    level=DiagnosticStatus.WARN,
                    name="Reasoning: Knowledge base",
                    message="Knowledge base running, but OWL/RDF reasoner not enabled",
                )
            arr = DiagnosticArray()
            arr.header.stamp = self.get_clock().now().to_msg()
            arr.status = [msg]
            self.diagnostics_pub.publish(arr)

            self.last_diagnostics_ts = now

        rclpy.spin_once(self, timeout_sec=0)

    def shutdown(self):

        self.get_logger().info("KnowledgeCore closing")
        self.destroy_node()
        rclpy.shutdown()
