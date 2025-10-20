from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value='worlds/empty.world',
        description='Path to the world file'
    )

    urdf_arg = DeclareLaunchArgument(
        'urdf',
        description='Absolute path to robot URDF file'
    )

    name_arg = DeclareLaunchArgument(
        'name',
        default_value='pioneer_robot',
        description='Robot name in Gazebo'
    )

    # Launch configurations
    world = LaunchConfiguration('world_name')
    urdf = LaunchConfiguration('urdf')
    name = LaunchConfiguration('name')

    # Include Gazebo (empty world) launch
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_urdf',
        arguments=[
            '-entity', name,
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Publish robot state from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf}]
    )

    return LaunchDescription([
        world_arg,
        urdf_arg,
        name_arg,
        gazebo_launch,
        robot_state_publisher,
        spawn_entity
    ])
