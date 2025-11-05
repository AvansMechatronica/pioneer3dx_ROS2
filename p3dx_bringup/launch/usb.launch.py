from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Argument om het aantal agents op te geven
    num_agents_arg = DeclareLaunchArgument(
        'num_agents',
        default_value='1',
        description='Aantal micro-ROS agents om te starten'
    )

    num_agents = LaunchConfiguration('num_agents')

    # Deze functie maakt de nodes aan op basis van het argument
    def launch_setup(context, *args, **kwargs):
        count = int(LaunchConfiguration('num_agents').perform(context))
        nodes = []
        if 0:
            for i in range(count):
                nodes.append(
                Node(
                    package='micro_ros_agent',
                    executable='micro_ros_agent',
                    name=f'micro_ros_agent_usb_{i}',
                    output='screen',
                    arguments=[
                        'serial',
                        '--dev', f'/dev/ttyUSB{i}',
                        '-b', '115200',
                        '-v', '4'
                    ],
                )
                )

        for i in range(count):
            nodes.append(
                Node(
                    package='micro_ros_agent',
                    executable='micro_ros_agent',
                    name=f'micro_ros_agent_usb_{i}',
                    output='screen',
                    arguments=[
                        'serial',
                        '--dev', f'/dev/ttyACM{i}',
                        '-b', '115200',
                        '-v', '4'
                    ],
                )
            )
        return nodes
    return LaunchDescription([
        num_agents_arg,
        OpaqueFunction(function=launch_setup)
    ])

