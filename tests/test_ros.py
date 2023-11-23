# -*- coding: utf-8 -*-

import json
import unittest
import pytest
import rclpy
from rclpy.node import Node
from kb_msgs.srv import Sparql
from kb_msgs.srv import Query
from kb_msgs.srv import Revise
from kb_msgs.srv import Manage

import launch_ros
import launch_testing
from launch import LaunchDescription

MANAGE_SRV = Manage, "/kb/manage"
REVISE_SRV = Revise, "/kb/revise"
QUERY_SRV = Query, "/kb/query"
SPARQL_SRV = Sparql, "/kb/sparql"


@pytest.mark.rostest
def generate_test_description():
    kb_node = launch_ros.actions.Node(
        package='knowledge_core',
        executable='knowledge_core',
        output='both',
        emulate_tty=True,
        arguments=["--debug", "--no-reasoner"])

    ld = LaunchDescription()
    ld.add_action(kb_node)
    ld.add_action(launch_testing.actions.ReadyToTest())
    return ld, {'kb_node': kb_node}


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, kb_node, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=kb_node)


class TestKB(unittest.TestCase):
    @classmethod
    def setUpClass(cls):

        if not rclpy.ok():
            rclpy.init()

        cls.node = Node("kb_unittests")
        cls.logger = cls.node.get_logger()

        cls.manage_srv = cls.node.create_client(*MANAGE_SRV)
        cls.revise_srv = cls.node.create_client(*REVISE_SRV)
        cls.query_srv = cls.node.create_client(*QUERY_SRV)
        cls.sparql_srv = cls.node.create_client(*SPARQL_SRV)

        if not cls.manage_srv.wait_for_service(timeout_sec=1.0):
            raise Exception(
                f'service {MANAGE_SRV[1]} not available.')
        if not cls.revise_srv.wait_for_service(timeout_sec=1.0):
            raise Exception(
                f'service {REVISE_SRV[1]} not available')
        if not cls.query_srv.wait_for_service(timeout_sec=1.0):
            raise Exception(
                f'service {QUERY_SRV[1]} not available')
        if not cls.sparql_srv.wait_for_service(timeout_sec=1.0):
            raise Exception(
                f'service {SPARQL_SRV[1]} not available')

    @classmethod
    def TearDownClass(cls):
        cls.manage_srv.destroy()
        cls.revise_srv.destroy()
        cls.query_srv.destroy()
        cls.sparql_srv.destroy()

        rclpy.shutdown()

    def manage(self, *args, **kwargs):
        future = self.manage_srv.call_async(Manage.Request(*args, **kwargs))
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def revise(self, *args, **kwargs):
        future = self.revise_srv.call_async(Revise.Request(*args, **kwargs))
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def query(self, *args, **kwargs):
        if len(args) >= 1:
            kwargs["patterns"] = args[0]
        if len(args) >= 2:
            kwargs["vars"] = args[1]
        if len(args) >= 2:
            kwargs["models"] = args[2]
        args = []

        future = self.query_srv.call_async(Query.Request(
            *args, **kwargs))
        rclpy.spin_until_future_complete(self.node, future)

        return future.result()

    def sparql(self, *args, **kwargs):
        future = self.sparql_srv.call_async(Sparql.Request(*args, **kwargs))
        rclpy.spin_until_future_complete(self.node, future)

        return future.result()

    def setUp(self):

        # clean-up the knowledge base before starting each test
        self.manage(action=Manage.Request.CLEAR)

    def test_basics(self):

        self.assertTrue(
            self.revise(statements=["ari rdf:type Robot"],
                        method=Revise.Request.ADD)
        )

        res = self.query(["?s rdf:type Robot"])
        self.assertCountEqual(json.loads(res.json), [{"s": "ari"}])

        self.manage(action=Manage.Request.CLEAR)

        res = self.query(["?s ?p ?o"])
        self.assertCountEqual(json.loads(res.json), [])

    def test_base_revise(self):

        self.assertTrue(
            self.revise(statements=["ari rdf:type Robot"],
                        method=Revise.Request.ADD)
        )

        res = self.query(["?s rdf:type Robot"])
        self.assertCountEqual(json.loads(res.json), [{"s": "ari"}])

        self.assertTrue(
            self.revise(
                statements=["tiago a Robot", "stockbot a Robot"],
                method=Revise.Request.ADD,
            )
        )

        res = self.query(["?s rdf:type Robot"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"s": "ari"},
                {"s": "tiago"},
                {"s": "stockbot"},
            ],
        )

        self.assertTrue(
            self.revise(
                statements=["tiago a Robot", "ari rdf:type Robot"],
                method=Revise.Request.DELETE,
            )
        )

        res = self.query(["?s rdf:type Robot"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"s": "stockbot"},
            ],
        )

    def test_retract_wildcards(self):

        self.assertTrue(
            self.revise(
                statements=[
                    "ari rdf:type Robot",
                    "ari isIn kitchen",
                    "tiago rdf:type Robot",
                    "tiago isIn living_room",
                ],
                method=Revise.Request.ADD,
            )
        )

        self.assertTrue(
            self.revise(
                statements=["ari ?p ?o"],
                method=Revise.Request.DELETE,
            )
        )

        res = self.query(["ari ?p ?o"])
        self.assertFalse(json.loads(res.json))

        self.assertTrue(
            self.revise(
                statements=["tiago isIn ?loc"],
                method=Revise.Request.DELETE,
            )
        )

        res = self.query(["tiago ?p ?o"])
        self.assertCountEqual(
            json.loads(res.json),
            [{"p": "rdf:type", "o": "Robot"}],
        )

        self.assertTrue(
            self.revise(
                statements=["?robot rdf:type Robot"],
                method=Revise.Request.DELETE,
            )
        )

        res = self.query(["?s rdf:type Robot"])
        self.assertFalse(json.loads(res.json))

        # TODO:
        # as of knowledge_core 2.8.9, retracting with wildcard is only support
        # for a single statement.
        # self.assertTrue(
        #    self.revise(
        #        statements=["stockbot rdf:type Robot", "talos rdf:type Robot"],
        #        method=Revise.Request.ADD,
        #    )
        # )

        # self.assertTrue(
        #    self.revise(
        #        statements=["stockbot ?p ?o", "talos ?a ?b"],
        #        method=Revise.Request.DELETE,
        #    )
        # )

        # res = self.query(["?s rdf:type Robot"])
        # self.assertFalse(json.loads(res.json))

    def test_errors(self):

        self.assertFalse(
            self.revise(statements=["term"], method=Revise.Request.ADD).success
        )
        self.assertFalse(
            self.revise(statements=["term term"],
                        method=Revise.Request.ADD).success
        )
        self.assertFalse(
            self.revise(
                statements=["term term term term"], method=Revise.Request.ADD
            ).success
        )

        self.assertFalse(
            self.revise(
                statements=["subject predicate object", "term term term term"],
                method=Revise.Request.ADD,
            ).success
        )

        # one of the previous statement is invalid -> no triples should have
        # been added at all
        res = self.query(["subject predicate object"])
        self.assertFalse(json.loads(res.json))

        # invalid query
        res = self.query(["subject"])
        self.assertFalse(res.success)

    def test_update(self):

        self.revise(
            statements=[
                "hasGender rdf:type owl:FunctionalProperty",
                "joe hasGender male",
            ],
            method=Revise.Request.ADD,
        )

        res = self.query(["joe hasGender ?gender"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"gender": "male"},
            ],
        )

        self.revise(
            statements=[
                "joe hasGender female",
            ],
            method=Revise.Request.UPDATE,
        )

        res = self.query(["joe hasGender ?gender"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"gender": "female"},
            ],
        )

    def test_queries(self):

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "ari rdf:type Robot",
                "Robot rdfs:subClassOf Machine",
                "Robot rdfs:subClassOf Agent",
                "joe rdf:type Human",
                "Human rdfs:subClassOf Agent",
                "joe eats carrot",
                "ari eats electricity",
                "eats rdfs:range Food",
            ],
        )

        res = self.query(patterns=["?agent rdf:type Robot"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"agent": "ari"},
            ],
        )

        res = self.query(["Robot rdfs:subClassOf ?cls"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"cls": "Machine"},
                {"cls": "Agent"},
            ],
        )

        res = self.query(["?subcls rdfs:subClassOf ?cls"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"cls": "Machine", "subcls": "Robot"},
                {"cls": "Agent", "subcls": "Robot"},
                {"cls": "Agent", "subcls": "Human"},
            ],
        )

        res = self.query(
            patterns=["?subcls rdfs:subClassOf ?cls"], vars=["?cls"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"cls": "Machine"},
                {"cls": "Agent"},
                {"cls": "Agent"},
            ],
        )

        res = self.query(["?__ rdfs:subClassOf Agent"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"var1": "Robot"},
                {"var1": "Human"},
            ],
        )

        res = self.query(
            patterns=["?agent rdf:type Robot", "?agent eats ?food"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"agent": "ari", "food": "electricity"},
            ],
        )

    def test_sparql(self):

        res = self.sparql(
            query="SELECT ?a WHERE { ?a :eats ?b . }")

        # self.assertCountEqual(json.loads(res.json)["results"]["bindings"], [])

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "joe eats carrot",
                "ari eats electricity",
            ],
        )

        res = self.sparql(
            query="SELECT ?a WHERE { ?a :eats ?b . }")

        self.assertEquals(len(json.loads(res.json)["results"]["bindings"]), 2)

        # invalid SPARQL! 'eats' has no namespace
        res = self.sparql(
            query="SELECT ?a WHERE { ?a eats ?b . }")
        self.assertFalse(res.success)

    def test_reasoning(self):
        # This test require a RDFS reasoner!

        status = json.loads(self.manage(action=Manage.Request.STATUS).json)

        if not status["reasoning_enabled"]:
            self.logger.warn(
                "RDFS Reasoner not available/enable. Skipping this test.")
            return

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "ari rdf:type Robot",
                "Robot rdfs:subClassOf Machine",
                "Robot rdfs:subClassOf Agent",
                "joe rdf:type Human",
                "Human rdfs:subClassOf Agent",
                "joe eats carrot",
                "ari eats electricity",
                "eats rdfs:range Food",
            ],
        )

        res = self.query(["?agent rdf:type Agent"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"agent": "joe"},
                {"agent": "ari"},
            ],
        )

        res = self.query(["?food rdf:type Food"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"food": "carrot"},
                {"food": "electricity"},
            ],
        )

        res = self.query(["?agent rdf:type Human", "?agent eats ?food"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"agent": "joe", "food": "carrot"},
            ],
        )
