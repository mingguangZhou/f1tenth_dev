import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration


LOGGER = get_logger("full_stack")


def _include(launch_file, launch_arguments):
    return GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments=launch_arguments.items(),
            )
        ],
    )


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _configured_or_default(context, name, default):
    configured = _value(context, name)
    return configured if configured else default


def _launch_stack(context):
    platform = _value(context, "platform")
    if platform not in ("sim", "onboard"):
        raise RuntimeError("platform must be 'sim' or 'onboard'")
    use_sim_time = "true" if platform == "sim" else "false"

    centerline_share = get_package_share_directory("centerline_tools")
    path_share = get_package_share_directory("path_following_v2")
    reactive_share = get_package_share_directory("reactive_control_v2")
    arbitration_share = get_package_share_directory("drive_arbitration_v2")
    bringup_share = get_package_share_directory("oudtra_driver_bringup")

    raceline_path = _value(context, "raceline_csv_path")
    if not raceline_path:
        raise RuntimeError("raceline_csv_path must not be empty")
    if os.path.isabs(raceline_path) and not os.path.isfile(raceline_path):
        raise RuntimeError(f"Raceline CSV does not exist: {raceline_path}")
    raceline_direction = _value(context, "raceline_direction")
    if raceline_direction not in ("csv", "normal", "reverse"):
        raise RuntimeError(
            "raceline_direction must be csv, normal, or reverse"
        )

    path_platform_default = ""
    reactive_platform_default = ""
    arbitration_platform_default = ""
    integration_platform_default = ""
    if platform == "sim":
        path_platform_default = os.path.join(
            path_share, "config", "path_following_v2_sim.yaml"
        )
        reactive_platform_default = os.path.join(
            reactive_share, "config", "reactive_control_v2_sim.yaml"
        )
        arbitration_platform_default = os.path.join(
            arbitration_share, "config", "drive_arbitration_v2_sim.yaml"
        )
        integration_platform_default = os.path.join(
            bringup_share, "config", "full_stack_sim.yaml"
        )

    LOGGER.info(
        "Resolved "
        f"platform={platform} "
        f"raceline_csv_path={raceline_path} "
        f"raceline_direction={raceline_direction}"
    )

    actions = [
        _include(
            os.path.join(
                centerline_share, "launch", "raceline_publisher.launch.py"
            ),
            {
                "csv_path": raceline_path,
                "direction": raceline_direction,
                "use_sim_time": use_sim_time,
            },
        ),
        _include(
            os.path.join(path_share, "launch", "path_following_v2_launch.py"),
            {
                "config": _value(context, "path_config"),
                "platform_config": _configured_or_default(
                    context, "path_platform_config", path_platform_default
                ),
                "path_generator_log_level": _value(
                    context, "path_generator_log_level"
                ),
                "local_trajectory_planner_log_level": _value(
                    context, "local_trajectory_planner_log_level"
                ),
                "path_follower_log_level": _value(
                    context, "path_follower_log_level"
                ),
            },
        ),
        _include(
            os.path.join(
                arbitration_share, "launch", "drive_arbitration_v2.launch.py"
            ),
            {
                "config": _value(context, "arbitration_config"),
                "platform_config": _configured_or_default(
                    context,
                    "arbitration_platform_config",
                    arbitration_platform_default,
                ),
                "raceline_guard_log_level": _value(
                    context, "raceline_guard_log_level"
                ),
                "drive_arbitrator_log_level": _value(
                    context, "drive_arbitrator_log_level"
                ),
            },
        ),
        _include(
            os.path.join(
                reactive_share, "launch", "reactive_control_v2_launch.py"
            ),
            {
                "config": _value(context, "reactive_config"),
                "platform_config": _configured_or_default(
                    context, "reactive_platform_config", reactive_platform_default
                ),
                "integration_config": _value(context, "integration_config"),
                "integration_platform_config": _configured_or_default(
                    context,
                    "integration_platform_config",
                    integration_platform_default,
                ),
                "drive_topic": _value(context, "final_drive_topic"),
                "nominal_cmd_topic": "/drive_arbitration_v2/selected_cmd",
                "drive_command_source": "stack",
                "upper_log_level": _value(context, "reactive_upper_log_level"),
                "lower_log_level": _value(context, "lower_safety_log_level"),
            },
        ),
    ]

    if _value(context, "start_particle_filter").lower() in (
        "true",
        "1",
        "yes",
        "on",
    ):
        particle_filter_share = get_package_share_directory("particle_filter")
        localize_config = _value(context, "localize_config")
        if not localize_config:
            localize_config = os.path.join(
                particle_filter_share, "config", "localize.yaml"
            )
        actions.insert(
            1,
            _include(
                os.path.join(
                    particle_filter_share, "launch", "localize_launch.py"
                ),
                {"localize_config": localize_config},
            ),
        )
    return actions


def generate_launch_description():
    path_share = get_package_share_directory("path_following_v2")
    reactive_share = get_package_share_directory("reactive_control_v2")
    arbitration_share = get_package_share_directory("drive_arbitration_v2")
    bringup_share = get_package_share_directory("oudtra_driver_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument("platform", default_value="onboard"),
            DeclareLaunchArgument("raceline_csv_path", default_value=""),
            DeclareLaunchArgument("raceline_direction", default_value="csv"),
            DeclareLaunchArgument(
                "path_config",
                default_value=os.path.join(
                    path_share, "config", "path_following_v2.yaml"
                ),
            ),
            DeclareLaunchArgument("path_platform_config", default_value=""),
            DeclareLaunchArgument(
                "reactive_config",
                default_value=os.path.join(
                    reactive_share, "config", "reactive_control_v2.yaml"
                ),
            ),
            DeclareLaunchArgument("reactive_platform_config", default_value=""),
            DeclareLaunchArgument(
                "arbitration_config",
                default_value=os.path.join(
                    arbitration_share, "config", "drive_arbitration_v2.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "arbitration_platform_config", default_value=""
            ),
            DeclareLaunchArgument(
                "integration_config",
                default_value=os.path.join(
                    bringup_share, "config", "full_stack.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "integration_platform_config", default_value=""
            ),
            DeclareLaunchArgument(
                "localize_config",
                default_value="",
            ),
            DeclareLaunchArgument("start_particle_filter", default_value="false"),
            DeclareLaunchArgument("final_drive_topic", default_value="/drive"),
            DeclareLaunchArgument(
                "path_generator_log_level", default_value="warn"
            ),
            DeclareLaunchArgument(
                "local_trajectory_planner_log_level", default_value="info"
            ),
            DeclareLaunchArgument("path_follower_log_level", default_value="warn"),
            DeclareLaunchArgument("reactive_upper_log_level", default_value="warn"),
            DeclareLaunchArgument("raceline_guard_log_level", default_value="warn"),
            DeclareLaunchArgument(
                "drive_arbitrator_log_level", default_value="info"
            ),
            DeclareLaunchArgument("lower_safety_log_level", default_value="info"),
            OpaqueFunction(function=_launch_stack),
        ]
    )
