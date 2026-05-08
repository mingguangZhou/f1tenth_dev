#!/usr/bin/env python3
"""Foxy-compatible launch file for centerline publisher.

This version intentionally merges the YAML file and launch-argument overrides
into one final parameter dictionary before creating the node.

Reason:
    In ROS 2 Foxy, using parameters=[yaml_file, override_dict] can be fragile
    depending on launch/substitution handling and parameter-file precedence.
    By resolving the LaunchConfigurations inside an OpaqueFunction and passing
    one dictionary to Node(parameters=[final_params]), command-line launch
    arguments such as direction:=reverse reliably override the YAML defaults.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _parse_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ['true', '1', 'yes', 'y', 'on']


def _load_yaml_parameters(config_path, node_name='centerline_publisher'):
    """Load a ROS 2 parameter YAML file and return the ros__parameters dict.

    Supports the normal structure:

        centerline_publisher:
          ros__parameters:
            key: value

    Also tolerates a direct ros__parameters/root dictionary fallback.
    """
    if not config_path:
        return {}

    if not os.path.exists(config_path):
        raise RuntimeError(f'Parameter YAML file does not exist: {config_path}')

    with open(config_path, 'r') as f:
        data = yaml.safe_load(f) or {}

    if node_name in data and isinstance(data[node_name], dict):
        return dict(data[node_name].get('ros__parameters', {}))

    if '/**' in data and isinstance(data['/**'], dict):
        return dict(data['/**'].get('ros__parameters', {}))

    if 'ros__parameters' in data:
        return dict(data.get('ros__parameters', {}))

    # Last fallback: allow a plain key/value YAML file.
    return dict(data)


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config').perform(context)

    params = _load_yaml_parameters(config_path)

    # Command-line launch arguments intentionally override YAML defaults.
    params.update({
        'csv_path': LaunchConfiguration('csv_path').perform(context),
        'frame_id': LaunchConfiguration('frame_id').perform(context),
        'path_topic': LaunchConfiguration('path_topic').perform(context),
        'marker_topic': LaunchConfiguration('marker_topic').perform(context),
        'waypoints_topic': LaunchConfiguration('waypoints_topic').perform(context),
        'direction': LaunchConfiguration('direction').perform(context),
        'publish_rate_hz': float(LaunchConfiguration('publish_rate_hz').perform(context)),
        'publish_start_marker': _parse_bool(LaunchConfiguration('publish_start_marker').perform(context)),
        'publish_point_markers': _parse_bool(LaunchConfiguration('publish_point_markers').perform(context)),
        'point_marker_stride': int(LaunchConfiguration('point_marker_stride').perform(context)),
        'publish_direction_arrows': _parse_bool(LaunchConfiguration('publish_direction_arrows').perform(context)),
        'direction_arrow_stride': int(LaunchConfiguration('direction_arrow_stride').perform(context)),
        'direction_arrow_length': float(LaunchConfiguration('direction_arrow_length').perform(context)),
        'use_sim_time': _parse_bool(LaunchConfiguration('use_sim_time').perform(context)),
    })

    return [
        Node(
            package='centerline_tools',
            executable='centerline_publisher',
            name='centerline_publisher',
            output='screen',
            parameters=[params],
        )
    ]


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
            'path_topic',
            default_value='/centerline_path',
            description='Topic for nav_msgs/Path centerline',
        ),
        DeclareLaunchArgument(
            'marker_topic',
            default_value='/centerline_markers',
            description='Topic for RViz MarkerArray centerline visualization',
        ),
        DeclareLaunchArgument(
            'waypoints_topic',
            default_value='/centerline_waypoints',
            description='Topic for rich Float64MultiArray waypoint rows',
        ),
        DeclareLaunchArgument(
            'direction',
            default_value='csv',
            description='Centerline direction: csv/normal or reverse',
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
            'publish_direction_arrows',
            default_value='true',
            description='Whether to publish sampled arrow markers showing centerline direction',
        ),
        DeclareLaunchArgument(
            'direction_arrow_stride',
            default_value='40',
            description='Stride for sampled direction arrow markers',
        ),
        DeclareLaunchArgument(
            'direction_arrow_length',
            default_value='0.35',
            description='Direction arrow length in meters',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulated ROS time for message stamps',
        ),
        OpaqueFunction(function=launch_setup),
    ])
