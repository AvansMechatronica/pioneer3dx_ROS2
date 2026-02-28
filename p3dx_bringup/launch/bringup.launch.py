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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    """Executed at runtime when the launch context exists."""
    
    # Evaluate launch arguments
    use_sim_time = False
    odom_topic = LaunchConfiguration('odom_topic')
    base_serial_port = LaunchConfiguration('base_serial_port')
    micro_ros_baudrate = LaunchConfiguration('micro_ros_baudrate')
    micro_ros_transport = LaunchConfiguration('micro_ros_transport')
    micro_ros_port = LaunchConfiguration('micro_ros_port')
    micro_ros_transport_value = micro_ros_transport.perform(context)


    # Package paths
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("p3dx_base"), "config", "ekf.yaml"]
    )

    description_launch_path = os.path.join(
        get_package_share_directory('p3dx_description'), 'launch', 'description.launch.py'
    )

    if micro_ros_transport_value == 'serial':
        micro_ros_agent_node = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', base_serial_port, '-b', micro_ros_baudrate]
        )
    else:
        micro_ros_agent_node = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=[micro_ros_transport_value, '--port', micro_ros_port]
        )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            ekf_config_path
        ],
        remappings=[("odometry/filtered", odom_topic)]
    )

        # Status monitor
    status_monitor_node = Node(
            package='p3dx_utils',
            executable='p3dx_status_monitor',
            name='p3dx_status_monitor',
            output='screen',
        )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'publish_joints': 'false',
            'urdf': LaunchConfiguration('urdf')
        }.items()
    )

    return [
        micro_ros_agent_node,
        description_launch,
        status_monitor_node,
        ekf_node,
    ]


def generate_launch_description():
    # Package paths for defaults
    desc_pkg = get_package_share_directory('p3dx_description')
    desc_pkg_gazebo = os.path.dirname(desc_pkg)
    
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("p3dx_description"), "urdf", "p3dx.urdf.xacro"]
    )


    return LaunchDescription([

        DeclareLaunchArgument(
            name='base_serial_port', 
            default_value='/dev/ttyACM0',
            description='Linorobot Base Serial Port'
        ),

        DeclareLaunchArgument(
            name='micro_ros_baudrate', 
            default_value='115200',
            description='micro-ROS baudrate'
        ),

        DeclareLaunchArgument(
            name='micro_ros_transport',
            default_value='udp4',
            description='micro-ROS transport'
        ),

        DeclareLaunchArgument(
            name='micro_ros_port',
            default_value='8888',
            description='micro-ROS udp/tcp port number'
        ),

        DeclareLaunchArgument(
            name='urdf',
            default_value=urdf_path,
            description='Absolute path to robot urdf file'
        ),

        DeclareLaunchArgument(
            name='odom_topic',
            default_value='/odom',
            description='EKF out odometry topic'
        ),
                
        OpaqueFunction(function=launch_setup)
    ])


#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940