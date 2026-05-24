from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('path_following_v2')
    params_file = os.path.join(pkg_share, 'config', 'path_following_v2.yaml')

    return LaunchDescription([
        Node(
            package='path_following_v2',
            executable='path_generator_node',
            name='path_generator',
            output='screen',
            parameters=[
                params_file,
                {
                    # When true, path_generator subscribes to /rl_target_speed
                    # and uses the PPO model speed if the message is fresh.
                    # If the RL speed node is stopped/stale, it falls back to
                    # the original curvature-based speed rule.
                    'use_rl_speed': True,
                    'rl_speed_topic': '/rl_target_speed',
                    'rl_speed_timeout_sec': 0.5,
                }
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
