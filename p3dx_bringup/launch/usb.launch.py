from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_usb',
            output='screen',
            arguments=['serial', 
                       '--dev', '/dev/ttyUSB1', 
                       '-b', '115200',
                       '-v', '4'   # Verbosity level (1–6)
                       ],
        )
    ])

