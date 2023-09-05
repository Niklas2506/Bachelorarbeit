import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Package Directories
    gazebo_pkg_dir = get_package_share_directory('auna_gazebo')
    navigation_pkg_dir = get_package_share_directory('auna_nav2')

    # Paths to folders and files
    gazebo_launch_file_dir = os.path.join(gazebo_pkg_dir, 'launch', 'gazebo')
    spawn_launch_file_dir = os.path.join(gazebo_pkg_dir, 'launch', 'spawn')
    nav_launch_file_dir = os.path.join(navigation_pkg_dir, 'launch')
    obstacles_launch_file = os.path.join(gazebo_pkg_dir, 'launch', 'spawn')

    # Launch Argument Configurations
    world_name = LaunchConfiguration('world_name', default='racetrack_decorated')

    # Launch Arguments
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='racetrack_decorated',
        description='Gazebo world file name'
    )
    
    # Nodes and other launch files
    world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch_file_dir, 'gazebo_world.launch.py')),
        launch_arguments={
            'world_name': world_name
        }.items(),
    )
    spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(spawn_launch_file_dir, 'spawn_single_robot.launch.py')),
        launch_arguments={
            'world_name': world_name
        }.items(),
    )
    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_file_dir, 'navigation_single_robot.launch.py')),
        launch_arguments={
            'world_name': world_name
        }.items(),
    )
    
    # Include obstacle spawn launch file
    obstacles_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(obstacles_launch_file, 'spawn_obstacles.launch.py'))
    )
    
    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(world_name_arg)

    launch_description.add_action(world_cmd)
    launch_description.add_action(spawn_cmd)
    launch_description.add_action(nav_cmd)
    launch_description.add_action(obstacles_launch)

    return launch_description
