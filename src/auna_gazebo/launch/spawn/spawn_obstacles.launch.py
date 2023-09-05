"""Obstacle spawn launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from geometry_msgs.msg import Pose, Point

def generate_launch_description():
    gazebo_models_path = os.path.join(get_package_share_directory('auna_gazebo'), 'models')
    
    obstacles = [
        {'name': 'obstacle1', 'position': Pose(position=Point(x=6.0, y=0.0, z=0.35))},
        #{'name': 'obstacle1', 'position': Pose(position=Point(x=2.4, y=0.0, z=0.35))},
        {'name': 'obstacle2', 'position': Pose(position=Point(x=11.3, y=5.0, z=0.35))},
        {'name': 'obstacle3', 'position': Pose(position=Point(x=0.6, y=4.0, z=0.35))}
    ]

    spawner_nodes = []
    for obstacle in obstacles:
        spawner_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', obstacle['name'],
                '-file', os.path.join(gazebo_models_path, 'obstacle', 'obstacle.sdf'),
                '-x', str(obstacle['position'].position.x),
                '-y', str(obstacle['position'].position.y),
                '-z', str(obstacle['position'].position.z)
            ],
            output='screen'
        )
        spawner_nodes.append(spawner_node)

    ld = LaunchDescription()
    for node in spawner_nodes:
        ld.add_action(node)

    return ld



