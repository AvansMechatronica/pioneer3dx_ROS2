from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def launch_setup(context, *args, **kwargs):
    # Get arguments
    xacro_file = LaunchConfiguration('xacro').perform(context)
    use_gui = LaunchConfiguration('joint_state_gui').perform(context).lower() == 'true'

    # Process xacro -> URDF
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Get RViz config path
    rviz_config_path = os.path.join(
        get_package_share_directory('amr_robots_description'),
        'rviz',
        'display_urdf.rviz'
    )

    # Joint State Publisher (GUI or headless)
    if use_gui:
        jsp_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen'
        )
    else:
        jsp_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Automatically shut down when RViz closes
    shutdown_on_rviz_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=[
                lambda event, context: context.emit_event_sync('shutdown')
            ]
        )
    )

    return [jsp_node, rsp_node, rviz_node, shutdown_on_rviz_exit]


def generate_launch_description():
    # Default xacro file path
    pkg_share = get_package_share_directory('amr_robots_description')
    default_xacro = os.path.join(pkg_share, 'urdf', 'pioneer.xacro')

    # Declare launch arguments
    xacro_arg = DeclareLaunchArgument(
        'xacro',
        default_value=default_xacro,
        description='Absolute path to robot xacro file'
    )

    joint_state_gui_arg = DeclareLaunchArgument(
        'joint_state_gui',
        default_value='true',
        description='Launch joint_state_publisher_gui if true'
    )

    return LaunchDescription([
        xacro_arg,
        joint_state_gui_arg,
        OpaqueFunction(function=launch_setup)
    ])
