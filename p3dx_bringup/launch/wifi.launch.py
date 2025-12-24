from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['udp4', 
                       '--port', '8888',
                       '-v', '5'   # Verbosity level (1–6)
                       ],
            respawn=False,
            respawn_delay=2.0,
            on_exit=None,
            emulate_tty=True,
            namespace='',
            # Use unique node name to prevent multiple instances
            # ROS 2 will prevent duplicate node names in same namespace
        )
    ])
