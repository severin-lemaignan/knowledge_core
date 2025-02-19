# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_pal import get_pal_configuration


def generate_launch_description():

    ld = LaunchDescription()

    config = get_pal_configuration(
        pkg='knowledge_core',
        node='knowledge_viewer',
        ld=ld)

    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'])
        ]))
    ld.add_action(rosbridge_launch)

    knowledge_viewer_node = Node(
        package='knowledge_core',
        executable='knowledge_viewer',
        namespace='kb',
        name='knowledge_viewer',
        parameters=config["parameters"],
        remappings=config["remappings"],
        arguments=config["arguments"],
        output='both',
        emulate_tty=True,
    )

    ld.add_action(knowledge_viewer_node)

    return ld
