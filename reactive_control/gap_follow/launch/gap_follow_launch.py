import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_share_dir = get_package_share_directory('gap_follow')
    config_file = os.path.join(package_share_dir, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='gap_follow',
            executable='reactive_node',
            name='reactive_node',
            output='screen',
            parameters=[config_file]  # Use the constructed absolute path
        )
    ])