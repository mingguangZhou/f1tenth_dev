import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("drive_arbitration_v2")
    default_config = os.path.join(
        package_share, "config", "drive_arbitration_v2_sim.yaml"
    )
    config = LaunchConfiguration("config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Simulator drive_arbitration_v2 parameter YAML.",
        ),
        Node(
            package="drive_arbitration_v2",
            executable="raceline_guard_node",
            name="raceline_guard",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="drive_arbitration_v2",
            executable="drive_arbitrator_node",
            name="drive_arbitrator",
            output="screen",
            parameters=[config],
        ),
    ])
