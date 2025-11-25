from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import random
import math

def spawn_object(object_name, x, y , z, p, html_link=''):

    if len(html_link):
        file = html_link
    else:
        pkg_path = get_package_share_directory('ros_industrial_gazebo')
        file = pkg_path+'/models/' + object_name + '/model.sdf'

    entity = object_name + '_' + str(random.randint(0, 1000))

    print(file)
    print(entity)
    
    # ignition gazebo spawn entity node
    return Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        arguments=[
                '-x', str(x), '-y', str(y), '-z', str(z), '-P', str(p),
                '-name', entity,
                '-file', file
        ],
    )


def generate_launch_description():
    
    box1 = spawn_object('box', -4.0, -2.5, 0.0, 0, 'https://fuel.gazebosim.org/1.0/paserv/models/rigid%20cubic%20cardbox%20medium%20size')
    box2 = spawn_object('box', -4.0, -3.0, 0.0, 0, 'https://fuel.gazebosim.org/1.0/paserv/models/rigid%20cubic%20cardbox%20medium%20size')
    box3 = spawn_object('box', -4.0, -3.5, 0.0, 0, 'https://fuel.gazebosim.org/1.0/paserv/models/rigid%20cubic%20cardbox%20medium%20size')

    return LaunchDescription([
        box1,
        box2,
        box3,
    ])
