import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parameter_files(context, include_integration):
    parameters = [LaunchConfiguration("config").perform(context)]
    for argument in ("platform_config",):
        value = LaunchConfiguration(argument).perform(context)
        if value:
            parameters.append(value)
    if include_integration:
        for argument in ("integration_config", "integration_platform_config"):
            value = LaunchConfiguration(argument).perform(context)
            if value:
                parameters.append(value)
    return parameters


def _launch_nodes(context):
    drive_command_source = LaunchConfiguration("drive_command_source").perform(
        context
    )
    if drive_command_source not in ("stack", "lower", "upper"):
        raise RuntimeError(
            "drive_command_source must be 'stack', 'lower', or 'upper', got: "
            + drive_command_source
        )

    drive_topic = LaunchConfiguration("drive_topic").perform(context)
    nominal_cmd_topic = LaunchConfiguration("nominal_cmd_topic").perform(context)
    upper_log_level = LaunchConfiguration("upper_log_level").perform(context)
    lower_log_level = LaunchConfiguration("lower_log_level").perform(context)
    nodes = []

    if drive_command_source in ("stack", "upper"):
        upper_remappings = []
        if drive_command_source == "upper":
            upper_remappings.append(
                ("/reactive_control_v2/nominal_cmd", drive_topic)
            )
        nodes.append(
            Node(
                package="reactive_control_v2",
                executable="upper_corridor_follower",
                name="upper_corridor_follower",
                output="screen",
                parameters=_parameter_files(context, include_integration=False),
                remappings=upper_remappings,
                arguments=["--ros-args", "--log-level", upper_log_level],
            )
        )

    if drive_command_source in ("stack", "lower"):
        selected_command_topic = nominal_cmd_topic
        if drive_command_source == "lower":
            selected_command_topic = "/reactive_control_v2/lower_debug_unused_cmd"
        nodes.append(
            Node(
                package="reactive_control_v2",
                executable="lower_safety_controller",
                name="lower_safety_controller",
                output="screen",
                parameters=_parameter_files(context, include_integration=True),
                remappings=[
                    (
                        "/reactive_control_v2/selected_cmd",
                        selected_command_topic,
                    ),
                    ("/reactive_control_v2/safe_cmd", drive_topic),
                ],
                arguments=["--ros-args", "--log-level", lower_log_level],
            )
        )

    return nodes


def generate_launch_description():
    package_share = get_package_share_directory("reactive_control_v2")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=os.path.join(
                    package_share, "config", "reactive_control_v2.yaml"
                ),
                description="Canonical reactive-control behavior profile.",
            ),
            DeclareLaunchArgument(
                "platform_config",
                default_value="",
                description="Optional sparse runtime adapter loaded after config.",
            ),
            DeclareLaunchArgument(
                "integration_config",
                default_value="",
                description="Optional integrated-stack safety configuration.",
            ),
            DeclareLaunchArgument(
                "integration_platform_config",
                default_value="",
                description="Optional platform safety adapter loaded last.",
            ),
            DeclareLaunchArgument("drive_topic", default_value="/drive"),
            DeclareLaunchArgument(
                "nominal_cmd_topic",
                default_value="/reactive_control_v2/nominal_cmd",
            ),
            DeclareLaunchArgument(
                "drive_command_source",
                default_value="stack",
                description="Controller path: stack, upper, or lower.",
            ),
            DeclareLaunchArgument("upper_log_level", default_value="warn"),
            DeclareLaunchArgument("lower_log_level", default_value="info"),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
