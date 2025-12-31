import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Load configuration file
    config_path = os.path.join(
        get_package_share_directory('p3dx_bringup'),
        'config',
        'bringup.yaml'
    )
    
    # EKF configuration file path
    ekf_config_path = os.path.join(
        get_package_share_directory('p3dx_base'),
        'config',
        'ekf.yaml'
    )
    
    # Description launch file path
    description_launch_path = os.path.join(
        get_package_share_directory('p3dx_description'),
        'launch',
        'description.launch.py'
    )
    
    # Parameters
    use_sim_time = False
    odom_topic = 'odometry/filtered'
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Extract micro-ROS agent config
    agent_config = config.get('micro_ros_agent', {})
    transport = agent_config.get('transport', 'udp4')
    port = agent_config.get('port', '8888')
    verbosity = agent_config.get('verbosity', '5')
    respawn = agent_config.get('respawn', False)
    respawn_delay = agent_config.get('respawn_delay', 2.0)
    
    # Extract status monitor config
    monitor_config = config.get('status_monitor', {})
    monitor_enabled = monitor_config.get('enabled', True)
    
    # Include description launch
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'publish_joints': 'false'
        }.items()
    )
    
    nodes = [
        # Micro-ROS agent for WiFi connection
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=[transport, 
                       '--port', port,
                       '-v', verbosity
                       ],
            respawn=respawn,
            respawn_delay=respawn_delay,
            on_exit=None,
            emulate_tty=True,
            namespace='',
        ),
        # EKF node for sensor fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                ekf_config_path
            ],
            remappings=[("odometry/filtered", odom_topic)]
        ),
    ]
    
    # Add status monitor if enabled
    if monitor_enabled:
        nodes.append(
            Node(
                package='p3dx_utils',
                executable='p3dx_status_monitor',
                name='p3dx_status_monitor',
                output='screen',
                emulate_tty=True,
            )
        )
    
    return LaunchDescription([
        description_launch,
        *nodes
    ])
