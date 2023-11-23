# -*- coding: utf-8 -*-

import json
import unittest
import pytest
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String
from kb_msgs.srv import Query
from kb_msgs.srv import Revise
from kb_msgs.srv import Manage
from kb_msgs.srv import Event


import launch_ros
import launch_testing
from launch import LaunchDescription

MANAGE_SRV = Manage, "/kb/manage"
REVISE_SRV = Revise, "/kb/revise"
QUERY_SRV = Query, "/kb/query"
EVENTS_SRV = Event, "/kb/events"
EVENTS_NS = EVENTS_SRV[1] + "/"


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


class TestKBEvents(unittest.TestCase):
    @classmethod
    def setUpClass(cls):

        if not rclpy.ok():
            rclpy.init()

        cls.node = Node("kb_unittests")
        cls.logger = cls.node.get_logger()

        cls.manage_srv = cls.node.create_client(*MANAGE_SRV)
        cls.revise_srv = cls.node.create_client(*REVISE_SRV)
        cls.query_srv = cls.node.create_client(*QUERY_SRV)
        cls.events_srv = cls.node.create_client(*EVENTS_SRV)

        if not cls.manage_srv.wait_for_service(timeout_sec=1.0):
            raise Exception(
                f'service {MANAGE_SRV[1]} not available.')
        if not cls.revise_srv.wait_for_service(timeout_sec=1.0):
            raise Exception(
                f'service {REVISE_SRV[1]} not available')
        if not cls.query_srv.wait_for_service(timeout_sec=1.0):
            raise Exception(
                f'service {QUERY_SRV[1]} not available')
        if not cls.events_srv.wait_for_service(timeout_sec=1.0):
            raise Exception(
                f'service {EVENTS_SRV[1]} not available')

    @classmethod
    def TearDownClass(cls):
        cls.manage_srv.destroy()
        cls.revise_srv.destroy()
        cls.query_srv.destroy()
        cls.events_srv.destroy()

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

    def events(self, *args, **kwargs):
        future = self.events_srv.call_async(Event.Request(*args, **kwargs))
        rclpy.spin_until_future_complete(self.node, future)

        return future.result()

    def setUp(self):

        # clean-up the knowledge base before starting each test
        self.manage(action=Manage.Request.CLEAR)

        self.evt_active = False
        self.evt_semaphore = False
        self.evt2_semaphore = False

        self.last_evt = []

    def on_evt(self, evt):
        self.last_evt = json.loads(evt.data)
        self.evt_semaphore = True

    def on_other_evt(self, evt):
        self.last_evt = json.loads(evt.data)
        self.evt2_semaphore = True

    def sleep(self, d):

        target = Duration(seconds=d)
        elapsed = Duration()
        delta = Duration(seconds=0.01)

        while True:
            self.node.get_clock().sleep_for(delta)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            elapsed = Duration(
                nanoseconds=elapsed.nanoseconds + delta.nanoseconds)
            if elapsed > target:
                break

    def check_event(self, should_trigger, other_evt=False):
        MAX_WAIT = 200  # ms

        elapsed = 0

        self.node.get_logger().warn(
            "Starting to wait for event (max %sms)..." % MAX_WAIT)
        if not other_evt:
            while elapsed < MAX_WAIT:
                if self.evt_semaphore:
                    break
                rclpy.spin_once(self.node, timeout_sec=0.01)
                elapsed += 10

            if self.evt_semaphore ^ should_trigger:
                msg = "The event did not trigger after %sms" % MAX_WAIT \
                    if should_trigger \
                    else "The event should *not* have triggered!"
                self.node.get_logger().error(msg)
                self.assertTrue(
                    False,
                    msg)

            if should_trigger:
                self.node.get_logger().warn(
                    "Event triggered after %sms" % elapsed)
            else:
                self.node.get_logger().warn("No trigger, as expected")

            self.evt_semaphore = False
        else:
            while elapsed < MAX_WAIT:
                if self.evt2_semaphore:
                    break
                rclpy.spin_once(self.node, timeout_sec=0.01)
                elapsed += 10

            self.assertTrue(
                not (self.evt2_semaphore ^ should_trigger),
                "The other event did not trigger after %sms" % MAX_WAIT,
            )
            if should_trigger:
                self.node.get_logger().warn(
                    "Event triggered after %sms" % elapsed)
            else:
                self.node.get_logger().warn("No trigger, as expected")
            self.evt2_semaphore = False

    def on_active(self):
        self.evt_active = True

    def test_simple_event(self):

        self.evt_semaphore = False
        self.evt_active = False

        evt = self.events(patterns=["?robot rdf:type Robot"], one_shot=False)
        self.node.create_subscription(
            String, evt.topic, self.on_evt, 10)

        self.check_event(should_trigger=False)

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "joe rdf:type Robot",
            ],
        )

        self.check_event(should_trigger=True)

        self.assertCountEqual(self.last_evt, [{"robot": "joe"}])

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "joe rdf:type Robot",
            ],
        )

        self.check_event(should_trigger=False)

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "bill rdf:type Human",
            ],
        )

        self.check_event(should_trigger=False)

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "ari rdf:type Robot",
            ],
        )

        self.check_event(should_trigger=True)
        self.assertCountEqual(self.last_evt, [{"robot": "ari"}])

        self.revise(
            method=Revise.Request.REMOVE,
            statements=[
                "ari rdf:type Robot",
            ],
        )

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "ari rdf:type Robot",
            ],
        )

        self.check_event(should_trigger=True)
        self.assertCountEqual(self.last_evt, [{"robot": "ari"}])

    def test_oneshot_event(self):

        self.evt_semaphore = False
        self.evt_active = False

        evt = self.events(patterns=["?human rdf:type Human"], one_shot=True)
        self.node.create_subscription(
            String, evt.topic, self.on_evt, 10)

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "joe rdf:type Human",
            ],
        )

        self.check_event(should_trigger=True)

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "john rdf:type Human",
            ],
        )

        self.check_event(should_trigger=False)

    def test_event_unsubscription(self):

        self.evt_semaphore = False
        self.evt_active = False

        evt = self.events(patterns=["?human rdf:type Human"], one_shot=False)
        sub = self.node.create_subscription(
            String, evt.topic, self.on_evt, 1)

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "joe rdf:type Human",
            ],
        )

        self.check_event(should_trigger=True)

        self.node.destroy_subscription(sub)

        # time it takes for the topic to be unsubscribe, and for that
        # information to be propagated to the knowledge base's topic's
        # 'num_subscription'
        self.sleep(0.5)

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "john rdf:type Human",
            ],
        )

        self.check_event(should_trigger=False)

        sub = self.node.create_subscription(
            String, evt.topic, self.on_evt, 1)

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "bill rdf:type Human",
            ],
        )

        # as we have previously un-registered from the event topic, the
        # knowledge base should have seen that no-one is listening to the topic
        # anymore, and should have disabled the event.
        # -> nothing should be published on the event topic
        self.check_event(should_trigger=False)

    def test_two_events(self):

        self.evt_semaphore = False
        self.evt_active = False

        evt = self.events(patterns=["?s rdf:type Human"], one_shot=False)
        self.node.create_subscription(
            String, evt.topic, self.on_evt, 10)

        evt2 = self.events(patterns=["?s rdf:type Robot"], one_shot=False)
        self.node.create_subscription(
            String, evt2.topic, self.on_other_evt, 10)

        self.check_event(should_trigger=False)
        self.check_event(should_trigger=False, other_evt=True)

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "joe rdf:type Human",
            ],
        )

        self.check_event(should_trigger=True)
        self.check_event(should_trigger=False, other_evt=True)
        self.assertCountEqual(self.last_evt, [{"s": "joe"}])

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "nono rdf:type Robot",
            ],
        )

        self.check_event(should_trigger=False)
        self.check_event(should_trigger=True, other_evt=True)
        self.assertCountEqual(self.last_evt, [{"s": "nono"}])

        self.revise(
            method=Revise.Request.ADD,
            statements=[
                "ari rdf:type Robot",
                "john rdf:type Human",
            ],
        )

        self.check_event(should_trigger=True)
        self.check_event(should_trigger=True, other_evt=True)
