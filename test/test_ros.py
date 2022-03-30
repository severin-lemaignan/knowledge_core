#!/usr/bin/env python
PKG = "test_minimalkb"

import rospy
import unittest
import json

from minimalkb.srv import Manage, ManageRequest
from minimalkb.srv import Revise, ReviseRequest
from minimalkb.srv import Query

MANAGE_SRV = "/kb/manage"
REVISE_SRV = "/kb/revise"
QUERY_SRV = "/kb/query"


class TestKB(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.wait_for_service(MANAGE_SRV)
        rospy.wait_for_service(REVISE_SRV)
        rospy.wait_for_service(QUERY_SRV)

        cls.manage = rospy.ServiceProxy(MANAGE_SRV, Manage)
        cls.revise = rospy.ServiceProxy(REVISE_SRV, Revise)
        cls.query = rospy.ServiceProxy(QUERY_SRV, Query)

    def setUp(self):

        # clean-up the knowledge base before starting each test
        self.manage(action=ManageRequest.CLEAR)

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


if __name__ == "__main__":
    import rostest

    rostest.rosrun(PKG, "test_kb", TestKB)
