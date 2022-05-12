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

from knowledge_core.srv import Manage, Revise, Query, Sparql
from std_msgs.msg import String


class KnowledgeCoreROS:
    def __init__(self, kb):
        self.kb = kb

        rospy.init_node("knowledge_core", disable_signals=True)
        self.services = {
            "manage": rospy.Service("kb/manage", Manage, self.handle_manage),
            "revise": rospy.Service("kb/revise", Revise, self.handle_revise),
            "query": rospy.Service("kb/query", Query, self.handle_query),
            "sparql": rospy.Service("kb/sparql", Sparql, self.handle_sparql),
        }

        self.update_sub = rospy.Subscriber("/kb/add_fact", String, self.on_update_fact)
        self.retract_sub = rospy.Subscriber(
            "/kb/remove_fact", String, self.on_retract_fact
        )

        rospy.loginfo(
            """
KnowledgeCore
=============

Knowledge base started.

Available topics:
- /kb/add_fact [std_msgs/String]
- /kb/remove_fact [std_msgs/String]

Available services:
- /kb/manage [knowledge_core/Manage]
- /kb/revise [knowledge_core/Revise]
- /kb/query [knowledge_core/Query]
- /kb/sparql [knowledge_core/Sparql]


Available action servers:
- /kb/events [knowledge_core/Events]

"""
        )

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
                        error_msg="'load' expects 'parameters' to contain exactly one URI pointing to the ontology to load",
                    )
                self.kb.load(req.parameters[0], models=req.models)
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

    def handle_sparql(self, req):

        from knowledge_core.srv import SparqlResponse

        try:
            res = self.kb.sparql(req.query, req.models)
            return SparqlResponse(success=True, error_msg="", json=json.dumps(res))
        except KbServerError as kbe:
            return SparqlResponse(success=False, error_msg=str(kbe))

    def on_update_fact(self, msg):
        self.kb.update([msg.data])

    def on_retract_fact(self, msg):
        self.kb.remove([msg.data])

    def shutdown(self):

        rospy.signal_shutdown("KnowledgeCore closing")
