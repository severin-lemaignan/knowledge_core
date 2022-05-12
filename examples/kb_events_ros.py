#! python3

import rospy
from std_msgs.msg import String

from knowledge_core.msg import EventAction, EventGoal
from knowledge_core.srv import Manage
import actionlib
import json


def on_new_human(evt):
    evt = json.loads(evt.json)
    print("Event triggered!")
    print(evt)


rospy.init_node("knowledge_core_events_demo")

# connects to the knowledge base
on_humans_evt = actionlib.SimpleActionClient("/kb/events", EventAction)

on_humans_evt.wait_for_server()

# register the event pattern and a callback
on_humans_evt.send_goal(
    EventGoal(patterns=["?human rdf:type Human"], one_shot=False),
    feedback_cb=on_new_human,
)

rospy.wait_for_service("/kb/manage")
kb_manage_srv = rospy.ServiceProxy("/kb/manage", Manage)

kb_manage_srv(action="clear")

# for testing purposes, we create a simple knowledge facts publisher
facts_pub = rospy.Publisher("/kb/add_fact", String, queue_size=1)

rospy.sleep(0.5)

## this should trigger the event!
facts_pub.publish("john rdf:type Human")

rospy.sleep(0.5)

# this should not trigger anything
facts_pub.publish("ari rdf:type Robot")

rospy.sleep(0.5)

## this should trigger the event!
facts_pub.publish("bill rdf:type Human")

rospy.sleep(0.5)

# unsubscribe from the event
on_humans_evt.cancel_goal()
