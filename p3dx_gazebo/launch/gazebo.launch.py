# Copyright (c) 2021 Juan Miguel Jimeno
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    use_sim_time = True

    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("p3dx_base"), "config", "ekf.yaml"]
    )

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("p3dx_robots_description"), "urdf", "p3dx.urdf.xacro"]
    )
    
    world_path = PathJoinSubstitution(
        [FindPackageShare("p3dx_gazebo"), "worlds", "maze.world"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('p3dx_robots_description'), 'launch', 'description.launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('p3dx_gazebo'), 'rviz', 'p3dx_default.rviz']
    )

    desc_pkg = get_package_share_directory('p3dx_robots_description')
    desc_pkg_gazebo = desc_pkg + "/.."
    
    # === Gazebo resource path ===
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{desc_pkg_gazebo}:{os.environ.get('GZ_SIM_RESOURCE_PATH','')}"
    )


    return LaunchDescription([
        gz_resource_path,

        DeclareLaunchArgument(
            name='gui', 
            default_value='true',
            description='Enable Gazebo Client'
        ),
        
        DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
        ),

        DeclareLaunchArgument(
            name='odom_topic', 
            default_value='/odom',
            description='EKF out odometry topic'
        ),
        
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Gazebo world'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='true',
            description='Run rviz'
        ),

       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'gz_args': [' -r -s ', LaunchConfiguration('world')]
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            condition=IfCondition(LaunchConfiguration('gui')),
            launch_arguments={
                'gz_args': [' -g']
            }.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
                }
            ]
        ),


        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic', 'robot_description', 
                '-entity', 'pioneer3dx', 
                '-allow_renaming', 'true',
            ]
        ),


        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/odom/unfiltered@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                #"/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                #"/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                #"/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                #"/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            ],
            remappings=[
                #('/camera/camera_info', '/camera/color/camera_info'),
                #('/camera/image', '/camera/color/image_raw'),
                #('/camera/depth_image', '/camera/depth/image_rect_raw'),
                #('/camera/points', '/camera/depth/color/points'),
            ]
        ),



        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}, 
                ekf_config_path
            ],
            remappings=[("odometry/filtered", LaunchConfiguration("odom_topic"))]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': use_sim_time},
                        ]
        ),


    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940
