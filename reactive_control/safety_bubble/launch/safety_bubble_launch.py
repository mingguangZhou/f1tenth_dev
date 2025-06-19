import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import logging

def generate_launch_description():
    package_share_dir = get_package_share_directory('safety_bubble')
    config_file = os.path.join(package_share_dir, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='safety_bubble',
            executable='safety_bubble_node',
            name='safety_bubble_node',
            output='screen',
            parameters=[config_file]  # Use the constructed absolute path
        ),
        Node(
            package='safety_bubble',
            executable='visualization_node',
            name='safety_bubble_visualizer',
            output='screen'
        )
    ])
