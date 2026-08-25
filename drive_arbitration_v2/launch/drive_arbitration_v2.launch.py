import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    parameters = [LaunchConfiguration("config").perform(context)]
    platform_config = LaunchConfiguration("platform_config").perform(context)
    if platform_config:
        parameters.append(platform_config)

    return [
        Node(
            package="drive_arbitration_v2",
            executable="raceline_guard_node",
            name="raceline_guard",
            output="screen",
            parameters=parameters,
            arguments=[
                "--ros-args",
                "--log-level",
                LaunchConfiguration("raceline_guard_log_level").perform(context),
            ],
        ),
        Node(
            package="drive_arbitration_v2",
            executable="drive_arbitrator_node",
            name="drive_arbitrator",
            output="screen",
            parameters=parameters,
            arguments=[
                "--ros-args",
                "--log-level",
                LaunchConfiguration("drive_arbitrator_log_level").perform(context),
            ],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("drive_arbitration_v2")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=os.path.join(
                    package_share, "config", "drive_arbitration_v2.yaml"
                ),
                description="Canonical arbitration behavior profile.",
            ),
            DeclareLaunchArgument(
                "platform_config",
                default_value="",
                description="Optional sparse runtime adapter loaded after config.",
            ),
            DeclareLaunchArgument("raceline_guard_log_level", default_value="warn"),
            DeclareLaunchArgument(
                "drive_arbitrator_log_level", default_value="info"
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
