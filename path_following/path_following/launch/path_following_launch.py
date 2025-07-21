from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_following',
            executable='planner_node',
            name='planner',
            output='screen'
        ),
        Node(
            package='path_following',
            executable='controller_node',
            name='controller',
            output='screen',
            parameters=[{
                'lookahead_distance': 0.3,
                'velocity': 1.5
            }]
        ),
        Node(
            package='path_following',
            executable='visualization_node',
            name='trajectory_visualizer',
            output='screen'
        ),

        # Boundary detection node
        Node(
            package='boundary_detection',
            executable='boundary_detection_node',
            name='boundary_detector',
            output='screen'
        )
    ])
