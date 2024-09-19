import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from os.path import join

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_pkg',              # Name of the package containing the node
            executable='traj_publisher',   # Name of the node executable
            name='traj_publisher',                    # Name of the node (optional)
            output='screen',                   # Output to screen (optional)
        ),

        Node(
            package='controller_pkg',              # Name of the package containing the node
            executable='controller_node',   # Name of the node executable
            name='controller_node',                    # Name of the node (optional)
            output='screen',                   # Output to screen (optional)
            parameters=[join(get_package_share_directory('controller_pkg'), 'config/params.yaml')],
        ),
    ])