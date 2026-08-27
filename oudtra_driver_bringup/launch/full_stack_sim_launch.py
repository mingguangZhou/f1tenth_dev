import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    path_share = get_package_share_directory("path_following_v2")
    reactive_share = get_package_share_directory("reactive_control_v2")
    arbitration_share = get_package_share_directory("drive_arbitration_v2")
    bringup_share = get_package_share_directory("oudtra_driver_bringup")
    defaults = {
        "raceline_csv_path": (
            "/sim_ws/src/centerline_tools/output_backup/ifac_roboracer/"
            "raceline_points_optimized.csv"
        ),
        "raceline_direction": "csv",
        "path_config": os.path.join(
            path_share, "config", "path_following_v2.yaml"
        ),
        "path_platform_config": "",
        "reactive_config": os.path.join(
            reactive_share, "config", "reactive_control_v2.yaml"
        ),
        "reactive_platform_config": "",
        "arbitration_config": os.path.join(
            arbitration_share, "config", "drive_arbitration_v2.yaml"
        ),
        "arbitration_platform_config": "",
        "integration_config": os.path.join(
            bringup_share, "config", "full_stack.yaml"
        ),
        "integration_platform_config": "",
        "localize_config": "",
        "start_particle_filter": "false",
        "final_drive_topic": "/drive",
        "path_generator_log_level": "warn",
        "local_trajectory_planner_log_level": "info",
        "path_follower_log_level": "warn",
        "reactive_upper_log_level": "warn",
        "raceline_guard_log_level": "warn",
        "drive_arbitrator_log_level": "info",
        "lower_safety_log_level": "info",
    }
    actions = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]
    forwarded = {name: LaunchConfiguration(name) for name in defaults}
    forwarded["platform"] = "sim"
    actions.append(
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            bringup_share, "launch", "full_stack_launch.py"
                        )
                    ),
                    launch_arguments=forwarded.items(),
                )
            ],
        )
    )
    return LaunchDescription(actions)
