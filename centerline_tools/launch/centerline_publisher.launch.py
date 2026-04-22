#!/usr/bin/env python3
"""Foxy-compatible launch file for centerline publisher."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


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
                LaunchConfiguration('config'),
                {
                    'csv_path': LaunchConfiguration('csv_path'),
                    'frame_id': LaunchConfiguration('frame_id'),
                    'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                    'publish_start_marker': LaunchConfiguration('publish_start_marker'),
                    'publish_point_markers': LaunchConfiguration('publish_point_markers'),
                    'point_marker_stride': LaunchConfiguration('point_marker_stride'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),
    ])
