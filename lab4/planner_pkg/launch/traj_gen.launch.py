import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os
from os.path import join

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='planner_pkg',                  # Name of the package containing the node
            executable='traj_vertices_publisher',   # Name of the node executable
            name='traj_vertices_publisher',         # Name of the node (optional)
            output='screen',                        # Output to screen (optional)
            parameters=[{
                'simulator_data_directory': os.environ['HOME'] + '/vnav/tesse/lab4/lab4_Data'
            }]
        ),
        
    ])
