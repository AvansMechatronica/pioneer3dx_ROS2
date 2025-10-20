from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    # Get URDF path from argument
    urdf_path = LaunchConfiguration('urdf').perform(context)

    # Read URDF contents
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    # RViz configuration file
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
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',  # ROS 2 equivalent of 'rviz'
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Automatically shut down when RViz closes (like required="true")
    shutdown_on_rviz_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=[
                lambda event, context: context.emit_event_sync('shutdown')
            ]
        )
    )

    return [robot_state_publisher, rviz_node, shutdown_on_rviz_exit]


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_robots_description')
    default_urdf = os.path.join(pkg_share, 'urdf', 'pioneer.urdf')

    urdf_arg = DeclareLaunchArgument(
        'urdf',
        default_value=default_urdf,
        description='Absolute path to the robot URDF file'
    )

    return LaunchDescription([
        urdf_arg,
        OpaqueFunction(function=launch_setup)
    ])
