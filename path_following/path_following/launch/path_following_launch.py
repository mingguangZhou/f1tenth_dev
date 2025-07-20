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
            output='screen'
        ),
        Node(
            package='path_following',
            executable='visualization_node',
            name='trajectory_visualizer',
            output='screen'
        )
    ])
