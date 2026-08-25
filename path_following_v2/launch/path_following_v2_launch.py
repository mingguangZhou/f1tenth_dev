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
            package="path_following_v2",
            executable="path_generator_node",
            name="path_generator",
            output="screen",
            parameters=parameters,
            arguments=[
                "--ros-args",
                "--log-level",
                LaunchConfiguration("path_generator_log_level").perform(context),
            ],
        ),
        Node(
            package="path_following_v2",
            executable="local_trajectory_planner_node",
            name="local_trajectory_planner",
            output="screen",
            parameters=parameters,
            arguments=[
                "--ros-args",
                "--log-level",
                LaunchConfiguration(
                    "local_trajectory_planner_log_level"
                ).perform(context),
            ],
        ),
        Node(
            package="path_following_v2",
            executable="path_following_v2_node",
            name="path_following_v2",
            output="screen",
            parameters=parameters,
            arguments=[
                "--ros-args",
                "--log-level",
                LaunchConfiguration("path_follower_log_level").perform(context),
            ],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("path_following_v2")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=os.path.join(
                    package_share, "config", "path_following_v2.yaml"
                ),
                description="Canonical path-following behavior profile.",
            ),
            DeclareLaunchArgument(
                "platform_config",
                default_value="",
                description="Optional sparse runtime adapter loaded after config.",
            ),
            DeclareLaunchArgument(
                "path_generator_log_level", default_value="warn"
            ),
            DeclareLaunchArgument(
                "local_trajectory_planner_log_level", default_value="info"
            ),
            DeclareLaunchArgument("path_follower_log_level", default_value="warn"),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
