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
    use_sim_time = True
    gui = LaunchConfiguration('gui').perform(context)
    urdf_path = LaunchConfiguration('urdf').perform(context)
    world_path = LaunchConfiguration('world').perform(context)
    odom_topic = LaunchConfiguration('odom_topic')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    # Package paths
    gazebo_launch_path = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    )
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("p3dx_base"), "config", "ekf.yaml"]
    )
    description_launch_path = os.path.join(
        get_package_share_directory('p3dx_description'), 'launch', 'description.launch.py'
    )

    # Nodes and actions
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'gz_args': f'-r -s {world_path}'
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        condition=IfCondition(LaunchConfiguration('gui')),
        launch_arguments={
            'gz_args': '-g'
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'pioneer3dx',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ]
    )

    ros_gz_bridge = Node(
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
            ('/world/turtlebot3_world/model/pioneer3dx/joint_state', '/joint_states'),
            #('/camera/camera_info', '/camera/color/camera_info'),
            #('/camera/image', '/camera/color/image_raw'),
            #('/camera/depth_image', '/camera/depth/image_rect_raw'),
            #('/camera/points', '/camera/depth/color/points'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
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

    command_timeout = Node(
            package='p3dx_gazebo',
            executable='command_timeout',
            name='command_timeout'
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
        description_launch,
        gazebo_server,
        gazebo_client,
        spawn_robot,
        ros_gz_bridge,
        ekf_node,
        command_timeout,
    ]


def generate_launch_description():
    # Package paths for defaults
    desc_pkg = get_package_share_directory('p3dx_description')
    desc_pkg_gazebo = os.path.dirname(desc_pkg)
    
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("p3dx_description"), "urdf", "p3dx.urdf.xacro"]
    )
    
    world_path = PathJoinSubstitution(
        [FindPackageShare("p3dx_gazebo"), "worlds", "warehouse.sdf"]
    )

    # Gazebo resource path
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
            name='spawn_x',
            default_value='0.5',
            description='Robot spawn position in X axis'
        ),

        DeclareLaunchArgument(
            name='spawn_y',
            default_value='0.0',
            description='Robot spawn position in Y axis'
        ),

        DeclareLaunchArgument(
            name='spawn_z',
            default_value='0.0',
            description='Robot spawn position in Z axis'
        ),
        
        DeclareLaunchArgument(
            name='spawn_yaw',
            default_value='0.0',
            description='Robot spawn heading'
        ),
        
        OpaqueFunction(function=launch_setup)
    ])


#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940