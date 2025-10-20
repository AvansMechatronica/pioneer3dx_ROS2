from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Optional: allow namespace or params via launch arguments
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='ros2aria_node',
        description='ros2aria_node'
    )

    return LaunchDescription([
        node_name_arg,

        Node(
            package='ros2aria',
            executable='ros2aria_node',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[  # optional parameters if needed(see Ros2Aria.cpp)
                {"port": "/dev/ttyUSB0"},
                #{"baud": 115200},
                {"baud": 0},
                {"debug_aria": False},
                {"aria_log_filename": "Aria.log"},
            ],
        )
    ])
