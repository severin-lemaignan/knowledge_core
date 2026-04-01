#! python3

# Copyright 2026 IIIA-CSIC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json

import actionlib
from knowledge_core.msg import EventAction, EventGoal
from knowledge_core.srv import Manage
import rospy
from std_msgs.msg import String


def on_new_human(evt):
    evt = json.loads(evt.json)
    print('Event triggered!')
    print(evt)


rospy.init_node('knowledge_core_events_demo')

# connects to the knowledge base
on_humans_evt = actionlib.SimpleActionClient('/kb/events', EventAction)

on_humans_evt.wait_for_server()

# register the event pattern and a callback
on_humans_evt.send_goal(
    EventGoal(patterns=['?human rdf:type Human'], one_shot=False),
    feedback_cb=on_new_human,
)

rospy.wait_for_service('/kb/manage')
kb_manage_srv = rospy.ServiceProxy('/kb/manage', Manage)

kb_manage_srv(action='clear')

# for testing purposes, we create a simple knowledge facts publisher
facts_pub = rospy.Publisher('/kb/add_fact', String, queue_size=1)

rospy.sleep(0.5)

# this should trigger the event!
facts_pub.publish('john rdf:type Human')

rospy.sleep(0.5)

# this should not trigger anything
facts_pub.publish('ari rdf:type Robot')

rospy.sleep(0.5)

# this should trigger the event!
facts_pub.publish('bill rdf:type Human')

rospy.sleep(0.5)

# unsubscribe from the event
on_humans_evt.cancel_goal()
