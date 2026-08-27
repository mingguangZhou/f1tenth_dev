import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("drive_arbitration_v2")
    arguments = [
        DeclareLaunchArgument(
            "config",
            default_value=os.path.join(
                package_share, "config", "drive_arbitration_v2.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "platform_config",
            default_value=os.path.join(
                package_share, "config", "drive_arbitration_v2_sim.yaml"
            ),
        ),
        DeclareLaunchArgument("raceline_guard_log_level", default_value="warn"),
        DeclareLaunchArgument("drive_arbitrator_log_level", default_value="info"),
    ]
    arguments.append(
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            package_share,
                            "launch",
                            "drive_arbitration_v2.launch.py",
                        )
                    ),
                    launch_arguments={
                        name: LaunchConfiguration(name)
                        for name in (
                            "config",
                            "platform_config",
                            "raceline_guard_log_level",
                            "drive_arbitrator_log_level",
                        )
                    }.items(),
                )
            ],
        )
    )
    return LaunchDescription(arguments)
