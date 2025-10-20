from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    # Get URDF model path from argument
    model_path = LaunchConfiguration('model').perform(context)

    # Read URDF content
    with open(model_path, 'r') as infp:
        robot_description = infp.read()

    # Declare nodes
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': False}]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    return [joint_state_publisher, robot_state_publisher]


def generate_launch_description():
    # Default model path inside package (optional)
    pkg_share = get_package_share_directory('amr_robots_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'pioneer.urdf')

    # Declare the model argument
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Absolute path to robot URDF model file'
    )

    return LaunchDescription([
        model_arg,
        OpaqueFunction(function=launch_setup)
    ])
