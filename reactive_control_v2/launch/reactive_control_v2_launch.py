from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_nodes(context):
    """Start the requested debug controller path with one /drive publisher."""
    config = LaunchConfiguration("config")
    drive_topic = LaunchConfiguration("drive_topic")
    nominal_cmd_topic = LaunchConfiguration("nominal_cmd_topic")
    drive_command_source = LaunchConfiguration("drive_command_source").perform(context)

    if drive_command_source not in ("stack", "lower", "upper"):
        raise RuntimeError(
            "drive_command_source must be 'stack', 'lower', or 'upper', got: "
            + drive_command_source
        )

    nodes = []

    if drive_command_source == "upper":
        # Upper-only debug: do not start the lower controller.
        nodes.append(
            Node(
                package="reactive_control_v2",
                executable="upper_corridor_follower",
                name="upper_corridor_follower",
                output="screen",
                parameters=[config],
                remappings=[
                    ("/reactive_control_v2/nominal_cmd", drive_topic),
                ],
            )
        )

    if drive_command_source == "lower":
        # Lower-only debug: do not start the upper controller. Remap the
        # selected-command input to an intentionally unused topic so the lower
        # controller continuously exercises its built-in FTG fallback.
        nodes.append(
            Node(
                package="reactive_control_v2",
                executable="lower_safety_controller",
                name="lower_safety_controller",
                output="screen",
                parameters=[config],
                remappings=[
                    (
                        "/reactive_control_v2/selected_cmd",
                        "/reactive_control_v2/lower_debug_unused_cmd",
                    ),
                    ("/reactive_control_v2/safe_cmd", drive_topic),
                ],
            )
        )

    if drive_command_source == "stack":
        # Normal complete path: upper nominal command passes through the lower
        # safety gateway, and only the lower controller publishes to /drive.
        nodes.extend([
            Node(
                package="reactive_control_v2",
                executable="upper_corridor_follower",
                name="upper_corridor_follower",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="reactive_control_v2",
                executable="lower_safety_controller",
                name="lower_safety_controller",
                output="screen",
                parameters=[config],
                remappings=[
                    ("/reactive_control_v2/selected_cmd", nominal_cmd_topic),
                    ("/reactive_control_v2/safe_cmd", drive_topic),
                ],
            ),
        ])
    return nodes


def generate_launch_description():
    package_share = get_package_share_directory("reactive_control_v2")
    default_config = os.path.join(
        package_share, "config", "reactive_control_v2.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to the reactive_control_v2 parameter YAML.",
        ),
        DeclareLaunchArgument(
            "drive_topic",
            default_value="/drive",
            description="Final lower-safety output used by the simulator.",
        ),
        DeclareLaunchArgument(
            "nominal_cmd_topic",
            default_value="/reactive_control_v2/nominal_cmd",
            description=(
                "Selected upper command entering the lower controller. Use the "
                "arbitrator output topic when drive_arbitration is running."
            ),
        ),
        DeclareLaunchArgument(
            "drive_command_source",
            default_value="stack",
            description=(
                "Controller path: 'stack' runs upper through lower (normal); "
                "'upper' runs only upper; 'lower' runs only lower FTG (debug only)."
            ),
        ),
        OpaqueFunction(function=launch_nodes),
    ])
