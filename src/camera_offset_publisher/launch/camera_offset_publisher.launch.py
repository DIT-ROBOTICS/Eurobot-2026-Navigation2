#!/usr/bin/env python3

# Copyright (c) 2026
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='1.5',
        description='Goal X position (meters)'
    )
    
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='1.0',
        description='Goal Y position (meters)'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='25.0',
        description='Publishing rate in Hz'
    )

    dock_side_arg = DeclareLaunchArgument(
        'dock_side',
        default_value='0',
        description='Use which side of camera to dock'
    )
    
    # Create the node
    camera_offset_publisher_node = Node(
        package='camera_offset_publisher',
        executable='camera_offset_publisher_node',
        name='camera_offset_publisher',
        output='screen',
        parameters=[{
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'dock_side': LaunchConfiguration('dock_side')
        }]
    )
    
    return LaunchDescription([
        goal_x_arg,
        goal_y_arg,
        publish_rate_arg,
        dock_side_arg,
        camera_offset_publisher_node,
    ])
