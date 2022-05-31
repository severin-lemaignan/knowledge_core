#!/usr/bin/env python
PKG = "test_knowledge_core"

import rospy
import unittest
import json
from knowledge_core.srv import Manage, ManageRequest
from knowledge_core.srv import Revise, ReviseRequest
from knowledge_core.srv import Event
from std_msgs.msg import String

MANAGE_SRV = "/kb/manage"
REVISE_SRV = "/kb/revise"
QUERY_SRV = "/kb/query"
SPARQL_SRV = "/kb/sparql"
EVENTS_SRV = "/kb/events"
EVENTS_NS = EVENTS_SRV + "/"


class TestKBEvents(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.wait_for_service(MANAGE_SRV)
        rospy.wait_for_service(REVISE_SRV)
        rospy.wait_for_service(EVENTS_SRV)

        cls.manage = rospy.ServiceProxy(MANAGE_SRV, Manage)
        cls.revise = rospy.ServiceProxy(REVISE_SRV, Revise)
        cls.events = rospy.ServiceProxy(EVENTS_SRV, Event)

    def setUp(self):

        # clean-up the knowledge base before starting each test
        self.manage(action=ManageRequest.CLEAR)

        self.evt_active = False
        self.evt_semaphore = False
        self.evt2_semaphore = False

        self.last_evt = None

    def on_evt(self, evt):
        self.last_evt = json.loads(evt.data)
        self.evt_semaphore = True

    def on_other_evt(self, evt):
        self.last_evt = json.loads(evt.data)
        self.evt2_semaphore = True

    def check_event(self, should_trigger, other_evt=False):
        MAX_WAIT = 200  # ms

        elapsed = 0

        rospy.logwarn("Starting to wait for event...")
        if not other_evt:
            while elapsed < MAX_WAIT:
                if self.evt_semaphore:
                    break
                rospy.sleep(0.01)
                elapsed += 10

            if self.evt_semaphore ^ should_trigger:
                self.assertTrue(
                    False,
                    "The event did not trigger after %sms" % MAX_WAIT
                    if should_trigger
                    else "The event should *not* have triggered!",
                )
            if should_trigger:
                rospy.logwarn("Event triggered after %sms" % elapsed)
            else:
                rospy.logwarn("No trigger, as expected")

            self.evt_semaphore = False
        else:
            while elapsed < MAX_WAIT:
                if self.evt2_semaphore:
                    break
                rospy.sleep(0.01)
                elapsed += 10

            self.assertTrue(
                not (self.evt2_semaphore ^ should_trigger),
                "The other event did not trigger after %sms" % MAX_WAIT,
            )
            if should_trigger:
                rospy.logwarn("Event triggered after %sms" % elapsed)
            else:
                rospy.logwarn("No trigger, as expected")
            self.evt2_semaphore = False

    def on_active(self):
        self.evt_active = True

    def test_simple_event(self):

        self.evt_semaphore = False
        self.evt_active = False

        evt = self.events(patterns=["?human rdf:type Human"], one_shot=False)
        rospy.Subscriber(evt.topic, String, self.on_evt, queue_size=10)

        self.check_event(should_trigger=False)

        self.revise(
            method=ReviseRequest.ADD,
            statements=[
                "joe rdf:type Human",
            ],
        )

        self.check_event(should_trigger=True)

        self.assertCountEqual(self.last_evt, [{"human": "joe"}])

        self.revise(
            method=ReviseRequest.ADD,
            statements=[
                "joe rdf:type Human",
            ],
        )

        self.check_event(should_trigger=False)

        self.revise(
            method=ReviseRequest.ADD,
            statements=[
                "ari rdf:type Robot",
            ],
        )

        self.check_event(should_trigger=False)

        self.revise(
            method=ReviseRequest.ADD,
            statements=[
                "bill rdf:type Human",
            ],
        )

        self.check_event(should_trigger=True)
        self.assertCountEqual(self.last_evt, [{"human": "bill"}])

        self.revise(
            method=ReviseRequest.REMOVE,
            statements=[
                "bill rdf:type Human",
            ],
        )

        self.revise(
            method=ReviseRequest.ADD,
            statements=[
                "bill rdf:type Human",
            ],
        )

        self.check_event(should_trigger=True)
        self.assertCountEqual(self.last_evt, [{"human": "bill"}])

    def test_oneshot_event(self):

        self.evt_semaphore = False
        self.evt_active = False

        evt = self.events(patterns=["?human rdf:type Human"], one_shot=True)
        rospy.Subscriber(evt.topic, String, self.on_evt, queue_size=10)

        self.revise(
            method=ReviseRequest.ADD,
            statements=[
                "joe rdf:type Human",
            ],
        )

        self.check_event(should_trigger=True)

        self.revise(
            method=ReviseRequest.ADD,
            statements=[
                "john rdf:type Human",
            ],
        )

        self.check_event(should_trigger=False)

    def test_event_unsubscription(self):

        self.evt_semaphore = False
        self.evt_active = False

        evt = self.events(patterns=["?human rdf:type Human"], one_shot=False)
        sub = rospy.Subscriber(evt.topic, String, self.on_evt, queue_size=10)

        self.revise(
            method=ReviseRequest.ADD,
            statements=[
                "joe rdf:type Human",
            ],
        )

        self.check_event(should_trigger=True)

        sub.unregister()
        rospy.sleep(0.3)

        self.revise(
            method=ReviseRequest.ADD,
            statements=[
                "john rdf:type Human",
            ],
        )

        self.check_event(should_trigger=False)

        sub = rospy.Subscriber(EVENTS_NS + evt.id, String, self.on_evt, queue_size=10)

        self.revise(
            method=ReviseRequest.ADD,
            statements=[
                "bill rdf:type Human",
            ],
        )

        # as we have previously un-registered from the event topic, the
        # knowledge base should have seen that no-one is listening to the topic
        # anymore, and should disable the event.
        self.check_event(should_trigger=False)

    def test_two_events(self):

        self.evt_semaphore = False
        self.evt_active = False

        evt = self.events(patterns=["?s rdf:type Human"], one_shot=False)
        rospy.Subscriber(evt.topic, String, self.on_evt, queue_size=10)

        evt2 = self.events(patterns=["?s rdf:type Robot"], one_shot=False)
        rospy.Subscriber(evt2.topic, String, self.on_other_evt, queue_size=10)

        self.check_event(should_trigger=False)
        self.check_event(should_trigger=False, other_evt=True)

        self.revise(
            method=ReviseRequest.ADD,
            statements=[
                "joe rdf:type Human",
            ],
        )

        self.check_event(should_trigger=True)
        self.check_event(should_trigger=False, other_evt=True)
        self.assertCountEqual(self.last_evt, [{"s": "joe"}])

        self.revise(
            method=ReviseRequest.ADD,
            statements=[
                "nono rdf:type Robot",
            ],
        )

        self.check_event(should_trigger=False)
        self.check_event(should_trigger=True, other_evt=True)
        self.assertCountEqual(self.last_evt, [{"s": "nono"}])

        self.revise(
            method=ReviseRequest.ADD,
            statements=[
                "ari rdf:type Robot",
                "john rdf:type Human",
            ],
        )

        self.check_event(should_trigger=True)
        self.check_event(should_trigger=True, other_evt=True)


if __name__ == "__main__":
    import rostest

    rospy.init_node("knowledge_core_ros_test")

    rostest.rosrun(PKG, "test_kb_ros_events", TestKBEvents)
