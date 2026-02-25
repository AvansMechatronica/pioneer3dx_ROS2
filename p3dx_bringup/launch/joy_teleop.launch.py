# Copyright (c) 2021 Juan Miguel Jimeno/Adapted (2025) by Gerard Harkema for Avans usage
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    left_handed_arg = DeclareLaunchArgument(
        'left_handed',
        default_value='false',
        description='Use left-handed joystick configuration (default: false for right-handed)'
    )

    joy_config_file = PythonExpression([
        "'joy_left_handed.yaml' if '",
        LaunchConfiguration('left_handed'),
        "' == 'true' else 'joy_right_handed.yaml'"
    ])

    joy_config_path = PathJoinSubstitution([
        FindPackageShare("p3dx_bringup"), 
        "config", 
        joy_config_file
    ])

    return LaunchDescription([
        left_handed_arg,
        
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            output='screen',
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_config_path]
        )
    ])