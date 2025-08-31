from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('path_following')
    
    # Define the smooth config file path
    config_file = os.path.join(pkg_share, 'config', 'planner_params_smooth.yaml')
    
    return LaunchDescription([
        Node(
            package='path_following',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[config_file]
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
