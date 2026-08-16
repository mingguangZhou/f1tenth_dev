from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('path_following_v2')
    centerline_share = get_package_share_directory('centerline_tools')
    params_file = os.path.join(pkg_share, 'config', 'path_following_v2_sim.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'centerline_csv_path',
            default_value=os.path.join(
                centerline_share,
                'centerline_output',
                'centerline_points_smooth.csv',
            ),
        ),
        DeclareLaunchArgument('centerline_direction', default_value='auto'),
        Node(
            package='path_following_v2',
            executable='path_generator_node',
            name='path_generator',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='path_following_v2',
            executable='local_trajectory_planner_node',
            name='local_trajectory_planner',
            output='screen',
            parameters=[
                params_file,
                {
                    'centerline_csv_path': LaunchConfiguration(
                        'centerline_csv_path'
                    ),
                    'centerline_direction': LaunchConfiguration(
                        'centerline_direction'
                    ),
                },
            ],
        ),
        Node(
            package='path_following_v2',
            executable='path_following_v2_node',
            name='path_following_v2',
            output='screen',
            parameters=[params_file],
        ),
    ])
