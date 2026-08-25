import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("reactive_control_v2")
    arguments = [
        DeclareLaunchArgument(
            "config",
            default_value=os.path.join(
                package_share, "config", "reactive_control_v2.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "platform_config",
            default_value=os.path.join(
                package_share, "config", "reactive_control_v2_sim.yaml"
            ),
        ),
        DeclareLaunchArgument("integration_config", default_value=""),
        DeclareLaunchArgument("integration_platform_config", default_value=""),
        DeclareLaunchArgument("drive_topic", default_value="/drive"),
        DeclareLaunchArgument(
            "nominal_cmd_topic",
            default_value="/reactive_control_v2/nominal_cmd",
        ),
        DeclareLaunchArgument("drive_command_source", default_value="stack"),
        DeclareLaunchArgument("upper_log_level", default_value="warn"),
        DeclareLaunchArgument("lower_log_level", default_value="info"),
    ]
    forwarded_names = (
        "config",
        "platform_config",
        "integration_config",
        "integration_platform_config",
        "drive_topic",
        "nominal_cmd_topic",
        "drive_command_source",
        "upper_log_level",
        "lower_log_level",
    )
    arguments.append(
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            package_share,
                            "launch",
                            "reactive_control_v2_launch.py",
                        )
                    ),
                    launch_arguments={
                        name: LaunchConfiguration(name) for name in forwarded_names
                    }.items(),
                )
            ],
        )
    )
    return LaunchDescription(arguments)
