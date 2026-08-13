from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rl_speed_inference')
    default_params_file = os.path.join(pkg_share, 'config', 'rl_speed_inference.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        Node(
            package='rl_speed_inference',
            executable='ppo_speed_node',
            name='ppo_speed_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
