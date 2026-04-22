#!/usr/bin/env python3
"""Launch file for centerline publisher with explicit argument-to-parameter wiring."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('centerline_tools')
    default_config = os.path.join(pkg_share, 'config', 'centerline_publisher.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to parameter yaml file',
        ),
        DeclareLaunchArgument(
            'csv_path',
            default_value='centerline_output/centerline_points_smooth.csv',
            description='Relative or absolute CSV path',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='Frame id for published centerline',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='1.0',
            description='Republish rate in Hz',
        ),
        DeclareLaunchArgument(
            'publish_start_marker',
            default_value='true',
            description='Whether to publish a start-point sphere marker',
        ),
        DeclareLaunchArgument(
            'publish_point_markers',
            default_value='false',
            description='Whether to publish sampled point markers',
        ),
        DeclareLaunchArgument(
            'point_marker_stride',
            default_value='10',
            description='Stride for sampled point markers',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulated ROS time for message stamps',
        ),
        Node(
            package='centerline_tools',
            executable='centerline_publisher',
            name='centerline_publisher',
            output='screen',
            parameters=[
                {
                    'csv_path': ParameterValue(LaunchConfiguration('csv_path'), value_type=str),
                    'frame_id': ParameterValue(LaunchConfiguration('frame_id'), value_type=str),
                    'publish_rate_hz': ParameterValue(LaunchConfiguration('publish_rate_hz'), value_type=float),
                    'publish_start_marker': ParameterValue(LaunchConfiguration('publish_start_marker'), value_type=bool),
                    'publish_point_markers': ParameterValue(LaunchConfiguration('publish_point_markers'), value_type=bool),
                    'point_marker_stride': ParameterValue(LaunchConfiguration('point_marker_stride'), value_type=int),
                    'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                }
            ],
        ),
    ])
