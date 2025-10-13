from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    # pkg_share = get_package_share_directory('path_following')
    
    # Config file paths
    config_file = os.path.join(
        get_package_share_directory('path_following'), 
        'config', 
        'planner_params.yaml'
    )

    drive_arbitration_config = os.path.join(
        get_package_share_directory('drive_arbitration'),
        'config',
        'drive_arbitration_params.yaml'
    )
    safety_bubble_config = os.path.join(
        get_package_share_directory('safety_bubble'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        # --- Path following
        Node(
            package='path_following',
            executable='planner_node',
            name='planner',
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

        # --- Boundary detection
        Node(
            package='boundary_detection',
            executable='boundary_detection_node',
            name='boundary_detection_node',
            output='screen'
        ),

        # --- Safety bubble (with YAML config)
        Node(
            package='safety_bubble',
            executable='safety_bubble_node',
            name='safety_bubble_node',
            output='screen',
            parameters=[safety_bubble_config]
        ),

        # --- Drive arbitration (with YAML config)
        Node(
            package='drive_arbitration',
            executable='drive_arbitrator_node',
            name='drive_arbitrator_node',
            output='screen',
            parameters=[drive_arbitration_config]
        ),
    ])
