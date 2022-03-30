#!/usr/bin/env python
PKG = "test_minimalkb"

import rospy
import unittest
import json

from minimalkb.srv import Revise, ReviseRequest, Query

REVISE_SRV = "/kb/revise"
QUERY_SRV = "/kb/query"


class TestKB(unittest.TestCase):
    def test_revise(self):
        rospy.wait_for_service(REVISE_SRV)
        rospy.wait_for_service(QUERY_SRV)

        revise = rospy.ServiceProxy(REVISE_SRV, Revise)
        query = rospy.ServiceProxy(QUERY_SRV, Query)

        self.assertTrue(
            revise(statements=["ari rdf:type Robot"], method=ReviseRequest.ADD)
        )

        self.assertCountEqual(
            json.loads(query(["?s rdf:type Robot"], None, None).json), [{"s": "ari"}]
        )

        self.assertTrue(
            revise(
                statements=["tiago a Robot", "stockbot a Robot"],
                method=ReviseRequest.ADD,
            )
        )

        self.assertCountEqual(
            json.loads(query(["?s rdf:type Robot"], None, None).json),
            [
                {"s": "ari"},
                {"s": "tiago"},
                {"s": "stockbot"},
            ],
        )

        self.assertTrue(
            revise(
                statements=["tiago a Robot", "ari rdf:type Robot"],
                method=ReviseRequest.DELETE,
            )
        )

        self.assertCountEqual(
            json.loads(query(["?s rdf:type Robot"], None, None).json),
            [
                {"s": "stockbot"},
            ],
        )


if __name__ == "__main__":
    import rostest

    rostest.rosrun(PKG, "test_kb", TestKB)
