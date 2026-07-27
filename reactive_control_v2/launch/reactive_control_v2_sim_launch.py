from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("reactive_control_v2")
    default_config = os.path.join(
        package_share, "config", "reactive_control_v2_sim.yaml"
    )
    config = LaunchConfiguration("config")
    drive_topic = LaunchConfiguration("drive_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to the reactive_control_v2 parameter YAML.",
        ),
        DeclareLaunchArgument(
            "drive_topic",
            default_value="/drive",
            description="Final command topic used during standalone simulator tests.",
        ),
        Node(
            package="reactive_control_v2",
            executable="corridor_planner_node",
            name="corridor_planner",
            output="screen",
            parameters=[config],
            remappings=[
                ("/reactive_control_v2/nominal_cmd", drive_topic),
            ],
        ),
    ])
