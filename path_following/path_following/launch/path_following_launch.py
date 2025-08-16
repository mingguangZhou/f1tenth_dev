from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_following',
            executable='planner_node',
            name='planner',
            output='screen',
            parameters=[{
                'min_start_x': 0.75
            }]
        ),
        Node(
            package='path_following',
            executable='controller_node',
            name='controller',
            output='screen',
            parameters=[{
                'lookahead_distance': 0.5,
                'velocity_max': 2.0,
                'velocity_min': 1.0,
                'steering_max_deg': 20.6
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
