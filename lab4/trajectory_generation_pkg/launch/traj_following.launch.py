import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os
from os.path import join

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='controller_pkg',              # Name of the package containing the node
            executable='controller_node',          # Name of the node executable
            name='controller_node',                # Name of the node (optional)
            output='screen',                       # Output to screen (optional)
            parameters=[join(get_package_share_directory('controller_pkg'), 'config', 'params.yaml')],
        ),

        Node(
            package='trajectory_generation',              # Name of the package containing the node
            executable='trajectory_generation_node',   # Name of the node executable
            name='trajectory_generation_node',                    # Name of the node (optional)
            output='screen',                   # Output to screen (optional)
        ),
        
    ])