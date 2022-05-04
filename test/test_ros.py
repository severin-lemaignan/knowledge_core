#!/usr/bin/env python
PKG = "test_knowledge_core"

import rospy
import unittest
import json

from knowledge_core.srv import Manage, ManageRequest
from knowledge_core.srv import Revise, ReviseRequest
from knowledge_core.srv import Query
from knowledge_core.srv import Sparql

MANAGE_SRV = "/kb/manage"
REVISE_SRV = "/kb/revise"
QUERY_SRV = "/kb/query"
SPARQL_SRV = "/kb/sparql"


class TestKB(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.wait_for_service(MANAGE_SRV)
        rospy.wait_for_service(REVISE_SRV)
        rospy.wait_for_service(QUERY_SRV)
        rospy.wait_for_service(SPARQL_SRV)

        cls.manage = rospy.ServiceProxy(MANAGE_SRV, Manage)
        cls.revise = rospy.ServiceProxy(REVISE_SRV, Revise)
        cls.query = rospy.ServiceProxy(QUERY_SRV, Query)
        cls.sparql = rospy.ServiceProxy(SPARQL_SRV, Sparql)

    def setUp(self):

        # clean-up the knowledge base before starting each test
        self.manage(action=ManageRequest.CLEAR)

    def test_basics(self):

        self.assertTrue(
            self.revise(statements=["ari rdf:type Robot"], method=ReviseRequest.ADD)
        )

        res = self.query(["?s rdf:type Robot"], None, None)
        self.assertCountEqual(json.loads(res.json), [{"s": "ari"}])

        self.manage(action=ManageRequest.CLEAR)

        res = self.query(["?s ?p ?o"], None, None)
        self.assertCountEqual(json.loads(res.json), [])

    def test_base_revise(self):

        self.assertTrue(
            self.revise(statements=["ari rdf:type Robot"], method=ReviseRequest.ADD)
        )

        res = self.query(["?s rdf:type Robot"], None, None)
        self.assertCountEqual(json.loads(res.json), [{"s": "ari"}])

        self.assertTrue(
            self.revise(
                statements=["tiago a Robot", "stockbot a Robot"],
                method=ReviseRequest.ADD,
            )
        )

        res = self.query(["?s rdf:type Robot"], None, None)
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
                method=ReviseRequest.DELETE,
            )
        )

        res = self.query(["?s rdf:type Robot"], None, None)
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"s": "stockbot"},
            ],
        )

    def test_errors(self):

        self.assertFalse(
            self.revise(statements=["term"], method=ReviseRequest.ADD).success
        )
        self.assertFalse(
            self.revise(statements=["term term"], method=ReviseRequest.ADD).success
        )
        self.assertFalse(
            self.revise(
                statements=["term term term term"], method=ReviseRequest.ADD
            ).success
        )

        self.assertFalse(
            self.revise(
                statements=["subject predicate object", "term term term term"],
                method=ReviseRequest.ADD,
            ).success
        )

        # one of the previous statement is invalid -> no triples should have
        # been added at all
        res = self.query(["subject predicate object"], None, None)
        self.assertFalse(json.loads(res.json))

        # invalid query
        res = self.query(["subject"], None, None)
        self.assertFalse(res.success)

    def test_update(self):

        self.revise(
            statements=[
                "hasGender rdf:type owl:FunctionalProperty",
                "joe hasGender male",
            ],
            method=ReviseRequest.ADD,
        )

        res = self.query(["joe hasGender ?gender"], None, None)
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
            method=ReviseRequest.UPDATE,
        )

        res = self.query(["joe hasGender ?gender"], None, None)
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"gender": "female"},
            ],
        )

    def test_queries(self):

        self.revise(
            method=ReviseRequest.ADD,
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

        res = self.query(
            patterns=["Robot rdfs:subClassOf ?cls"], vars=None, models=None
        )
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"cls": "Machine"},
                {"cls": "Agent"},
            ],
        )

        res = self.query(patterns=["?subcls rdfs:subClassOf ?cls"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"cls": "Machine", "subcls": "Robot"},
                {"cls": "Agent", "subcls": "Robot"},
                {"cls": "Agent", "subcls": "Human"},
            ],
        )

        res = self.query(
            patterns=["?subcls rdfs:subClassOf ?cls"], vars=["?cls"], models=None
        )
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"cls": "Machine"},
                {"cls": "Agent"},
                {"cls": "Agent"},
            ],
        )

        res = self.query(patterns=["?__ rdfs:subClassOf Agent"])
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"var1": "Robot"},
                {"var1": "Human"},
            ],
        )

        res = self.query(
            patterns=["?agent rdf:type Robot", "?agent eats ?food"],
            vars=None,
            models=None,
        )
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"agent": "ari", "food": "electricity"},
            ],
        )

    def test_sparql(self):

        res = self.sparql(query="SELECT ?a WHERE { ?a :eats ?b . }", models=None)

        self.assertCountEqual(json.loads(res.json)["results"]["bindings"], [])

        self.revise(
            method=ReviseRequest.ADD,
            statements=[
                "joe eats carrot",
                "ari eats electricity",
            ],
        )

        res = self.sparql(query="SELECT ?a WHERE { ?a :eats ?b . }", models=None)

        self.assertEquals(len(json.loads(res.json)["results"]["bindings"]), 2)

        # invalid SPARQL! 'eats' has no namespace
        res = self.sparql(query="SELECT ?a WHERE { ?a eats ?b . }", models=None)
        self.assertFalse(res.success)

    def test_reasoning(self):
        """This test require a RDFS reasoner!"""

        status = json.loads(self.manage(action=ManageRequest.STATUS).json)

        if not status["reasoning_enabled"]:
            rospy.logwarn("RDFS Reasoner not available/enable. Skipping this test.")
            return

        self.revise(
            method=ReviseRequest.ADD,
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

        res = self.query(patterns=["?agent rdf:type Agent"], vars=None, models=None)
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"agent": "joe"},
                {"agent": "ari"},
            ],
        )

        res = self.query(patterns=["?food rdf:type Food"], vars=None, models=None)
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"food": "carrot"},
                {"food": "electricity"},
            ],
        )

        res = self.query(
            patterns=["?agent rdf:type Human", "?agent eats ?food"],
            vars=None,
            models=None,
        )
        self.assertCountEqual(
            json.loads(res.json),
            [
                {"agent": "joe", "food": "carrot"},
            ],
        )


if __name__ == "__main__":
    import rostest

    rostest.rosrun(PKG, "test_kb", TestKB)
