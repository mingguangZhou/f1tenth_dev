from launch import LaunchDescription
from launch_ros.actions import Node
<<<<<<< HEAD
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
=======
>>>>>>> 4dd6227 (Sync file structure as on vehicle and update master launch file)
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
<<<<<<< HEAD
    # Get the package directory
    pkg_share = get_package_share_directory('path_following')
    
    # Define the config file path
    config_file = os.path.join(pkg_share, 'config', 'planner_params.yaml')
    
=======
    # Config file paths
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

>>>>>>> 4dd6227 (Sync file structure as on vehicle and update master launch file)
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

        # --- Boundary detection
        Node(
            package='boundary_detection',
            executable='boundary_detection_node',
            name='boundary_detector',
            output='screen'
        ),

        # --- Safety bubble (with yaml config)
        Node(
            package='safety_bubble',
            executable='safety_bubble_node',
            name='safety_bubble',
            output='screen',
            parameters=[safety_bubble_config]
        ),

        # --- Drive arbitration (with yaml config)
        Node(
            package='drive_arbitration',
            executable='drive_arbitrator_node',
            name='drive_arbitrator',
            output='screen',
            parameters=[drive_arbitration_config]
        )
    ])
