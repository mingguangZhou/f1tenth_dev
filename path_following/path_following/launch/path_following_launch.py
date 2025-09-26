from launch import LaunchDescription
from launch_ros.actions import Node
<<<<<<< HEAD
<<<<<<< HEAD
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
=======
>>>>>>> 4dd6227 (Sync file structure as on vehicle and update master launch file)
=======
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
>>>>>>> c7cb883 (fix path following master launch)
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
<<<<<<< HEAD
<<<<<<< HEAD
    # Get the package directory
    pkg_share = get_package_share_directory('path_following')
    
    # Define the config file path
    config_file = os.path.join(pkg_share, 'config', 'planner_params.yaml')
    
=======
    # Config file paths
=======
    # --- Config file paths
>>>>>>> c7cb883 (fix path following master launch)
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

        # --- Path planner
        Node(
            package='path_following',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[config_file]
        ),

        # --- Path follower controller
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

        # --- Visualization
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
