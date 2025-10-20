from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

import os

def launch_setup(context, *args, **kwargs):
    # Get URDF file path from argument
    urdf_path = LaunchConfiguration('urdf').perform(context)

    # Read URDF content
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # Get URDF/xacro file path from argument (use substitution with xacro)
    urdf_substitution = LaunchConfiguration('urdf')

    # Convert xacro -> URDF at launch time (works for both .xacro and .urdf: xacro will pass through .urdf)
    robot_description_cmd = Command([FindExecutable(name='xacro'), ' ', urdf_substitution])



    # Get RViz config path
    rviz_config_path = os.path.join(
        get_package_share_directory('amr_robots_description'),
        'rviz',
        'display_urdf.rviz'
    )

    # Define nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description_cmd, value_type=str)}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Make launch shut down when RViz closes
    shutdown_on_rviz_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=[
                # Emit a Shutdown event (actions are expected here, not raw functions)
                EmitEvent(event=Shutdown())
            ]
        )
    )

    return [robot_state_publisher, rviz_node, shutdown_on_rviz_exit]


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_robots_description')
    default_urdf_path = os.path.join(pkg_share, 'urdf', 'pioneer3dx_gazebo.urdf.xacro')

    urdf_arg = DeclareLaunchArgument(
        'urdf',
        default_value=default_urdf_path,
        description='Absolute path to the robot URDF file'
    )

    return LaunchDescription([
        urdf_arg,
        OpaqueFunction(function=launch_setup)
    ])
