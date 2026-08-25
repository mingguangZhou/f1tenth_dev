import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("path_following_v2")
    arguments = [
        DeclareLaunchArgument(
            "config",
            default_value=os.path.join(
                package_share, "config", "path_following_v2.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "platform_config",
            default_value=os.path.join(
                package_share, "config", "path_following_v2_sim.yaml"
            ),
        ),
        DeclareLaunchArgument("path_generator_log_level", default_value="warn"),
        DeclareLaunchArgument(
            "local_trajectory_planner_log_level", default_value="info"
        ),
        DeclareLaunchArgument("path_follower_log_level", default_value="warn"),
    ]

    forwarded = {
        name: LaunchConfiguration(name)
        for name in (
            "config",
            "platform_config",
            "path_generator_log_level",
            "local_trajectory_planner_log_level",
            "path_follower_log_level",
        )
    }
    arguments.append(
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            package_share,
                            "launch",
                            "path_following_v2_launch.py",
                        )
                    ),
                    launch_arguments=forwarded.items(),
                )
            ],
        )
    )
    return LaunchDescription(arguments)
