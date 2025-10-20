from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Locate the RViz configuration file
    rviz_config_path = os.path.join(
        get_package_share_directory('amr_robots_description'),
        'rviz',
        'display_urdf.rviz'
    )

    # Define the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Return the launch description
    return LaunchDescription([
        rviz_node
    ])
