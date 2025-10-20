from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    # === Get arguments ===
    urdf_path = LaunchConfiguration('urdf').perform(context)
    use_gui = LaunchConfiguration('joint_state_gui').perform(context).lower() == 'true'

    # === Load URDF file ===
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    # === Locate RViz configuration ===
    rviz_config_path = os.path.join(
        get_package_share_directory('amr_robots_description'),
        'urdf',
        'display_urdf.rviz'
    )

    # === Nodes ===
    # Joint State Publisher (GUI or headless)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui' if use_gui else 'joint_state_publisher',
        executable='joint_state_publisher_gui' if use_gui else 'joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',          # ROS 2 version of RViz
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # === Shutdown handler (when RViz closes) ===
    shutdown_on_rviz_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=[lambda event, context: context.emit_event_sync('shutdown')]
        )
    )

    return [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        shutdown_on_rviz_exit
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_robots_description')
    default_urdf = os.path.join(pkg_share, 'urdf', 'pioneer.urdf')

    urdf_arg = DeclareLaunchArgument(
        'urdf',
        default_value=default_urdf,
        description='Absolute path to the robot URDF file'
    )

    joint_state_gui_arg = DeclareLaunchArgument(
        'joint_state_gui',
        default_value='true',
        description='Use joint_state_publisher_gui if true'
    )

    return LaunchDescription([
        urdf_arg,
        joint_state_gui_arg,
        OpaqueFunction(function=launch_setup)
    ])
