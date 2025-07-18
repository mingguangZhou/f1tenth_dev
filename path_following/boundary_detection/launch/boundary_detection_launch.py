from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='boundary_detection',
            executable='boundary_detection_node',
            name='boundary_detection_node',
            output='screen',
        ),
    ])
