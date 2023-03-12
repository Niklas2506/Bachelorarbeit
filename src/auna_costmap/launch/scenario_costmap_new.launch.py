import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_dir = get_package_share_directory('auna_costmap')

    # Parameter files
    default_params_file = os.path.join(pkg_dir, 'config', 'costmap.yaml')

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    # Launch Arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Robot namespace for ROS nodes and topics'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameter file for navigation nodes'
    )

    remapping_tf = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    # Nodes and other launch files
    costmap_to_polygon = Node(
    	package='costmap_converter',
    	executable='standalone_converter',
    	name='standalone_converter',
    	namespace=namespace,
    	remappings=remapping_tf,
        parameters=[params_file],
        output='screen'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(params_file_arg)

    launch_description.add_action(costmap_to_polygon)

    return launch_description
