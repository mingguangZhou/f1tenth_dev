import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    centerline_share = get_package_share_directory("centerline_tools")
    path_share = get_package_share_directory("path_following_v2")
    reactive_share = get_package_share_directory("reactive_control_v2")
    arbitration_share = get_package_share_directory("drive_arbitration_v2")
    bringup_share = get_package_share_directory("oudtra_driver_bringup")

    path_config = LaunchConfiguration("path_config")
    reactive_config = LaunchConfiguration("reactive_config")
    arbitration_config = LaunchConfiguration("arbitration_config")
    integration_config = LaunchConfiguration("integration_config")
    final_drive_topic = LaunchConfiguration("final_drive_topic")
    path_generator_log_level = LaunchConfiguration("path_generator_log_level")
    local_trajectory_planner_log_level = LaunchConfiguration(
        "local_trajectory_planner_log_level"
    )
    path_follower_log_level = LaunchConfiguration("path_follower_log_level")
    reactive_upper_log_level = LaunchConfiguration("reactive_upper_log_level")
    raceline_guard_log_level = LaunchConfiguration("raceline_guard_log_level")
    drive_arbitrator_log_level = LaunchConfiguration("drive_arbitrator_log_level")
    lower_safety_log_level = LaunchConfiguration("lower_safety_log_level")

    return LaunchDescription([
        DeclareLaunchArgument(
            "raceline_csv_path",
            default_value="/sim_ws/src/centerline_tools/output_backup/V0_reward_ppo_speed_spielberg_1000k_20260612/raceline_points_smooth.csv",
            description="Relative or absolute simulator raceline CSV path.",
        ),
        DeclareLaunchArgument(
            "raceline_direction",
            default_value="csv",
            description="Raceline traversal direction: csv/normal or reverse.",
        ),
        DeclareLaunchArgument(
            "path_config",
            default_value=os.path.join(
                path_share, "config", "path_following_v2_sim.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "reactive_config",
            default_value=os.path.join(
                reactive_share, "config", "reactive_control_v2_sim.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "arbitration_config",
            default_value=os.path.join(
                arbitration_share, "config", "drive_arbitration_v2_sim.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "integration_config",
            default_value=os.path.join(
                bringup_share, "config", "full_stack_sim.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "final_drive_topic",
            default_value="/drive",
            description="Only lower_safety_controller publishes this topic.",
        ),
        DeclareLaunchArgument("path_generator_log_level", default_value="warn"),
        DeclareLaunchArgument(
            "local_trajectory_planner_log_level", default_value="info"
        ),
        DeclareLaunchArgument("path_follower_log_level", default_value="warn"),
        DeclareLaunchArgument("reactive_upper_log_level", default_value="warn"),
        DeclareLaunchArgument("raceline_guard_log_level", default_value="warn"),
        DeclareLaunchArgument("drive_arbitrator_log_level", default_value="info"),
        DeclareLaunchArgument("lower_safety_log_level", default_value="info"),

        # Simulator deliberately does not start particle_filter. The simulator's
        # ground-truth map -> ego_racecar/base_link transform drives path following.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    centerline_share, "launch", "raceline_publisher_sim_launch.py"
                )
            ),
            launch_arguments={
                "csv_path": LaunchConfiguration("raceline_csv_path"),
                "direction": LaunchConfiguration("raceline_direction"),
                "use_sim_time": "true",
            }.items(),
        ),

        Node(
            package="path_following_v2",
            executable="path_generator_node",
            name="path_generator",
            output="screen",
            parameters=[path_config],
            arguments=["--ros-args", "--log-level", path_generator_log_level],
        ),
        # The generator publishes the raw raceline window. The persistent local
        # trajectory planner converts it into the final path consumed by the follower
        # and the raceline guard. Keep this node in the master launch because
        # those two consumers must never bypass obstacle validation.
        Node(
            package="path_following_v2",
            executable="local_trajectory_planner_node",
            name="local_trajectory_planner",
            output="screen",
            parameters=[path_config],
            arguments=[
                "--ros-args",
                "--log-level",
                local_trajectory_planner_log_level,
            ],
        ),
        Node(
            package="path_following_v2",
            executable="path_following_v2_node",
            name="path_following_v2",
            output="screen",
            parameters=[path_config],
            arguments=["--ros-args", "--log-level", path_follower_log_level],
        ),
        Node(
            package="reactive_control_v2",
            executable="upper_corridor_follower",
            name="upper_corridor_follower",
            output="screen",
            parameters=[reactive_config],
            arguments=["--ros-args", "--log-level", reactive_upper_log_level],
        ),
        Node(
            package="drive_arbitration_v2",
            executable="raceline_guard_node",
            name="raceline_guard",
            output="screen",
            parameters=[arbitration_config],
            arguments=["--ros-args", "--log-level", raceline_guard_log_level],
        ),
        Node(
            package="drive_arbitration_v2",
            executable="drive_arbitrator_node",
            name="drive_arbitrator",
            output="screen",
            parameters=[arbitration_config],
            arguments=["--ros-args", "--log-level", drive_arbitrator_log_level],
        ),
        Node(
            package="reactive_control_v2",
            executable="lower_safety_controller",
            name="lower_safety_controller",
            output="screen",
            parameters=[reactive_config, integration_config],
            arguments=["--ros-args", "--log-level", lower_safety_log_level],
            remappings=[
                (
                    "/reactive_control_v2/selected_cmd",
                    "/drive_arbitration_v2/selected_cmd",
                ),
                ("/reactive_control_v2/safe_cmd", final_drive_topic),
            ],
        ),
    ])
