# Copyright 2026 F1TENTH Development Contributors
# SPDX-License-Identifier: MIT

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("f1tenth_gym_ros")
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "gym_bridge_launch.py")
        ),
        launch_arguments={
            "config_file": os.path.join(package_share, "config", "sim_two_agent.yaml"),
            "rviz_config": os.path.join(
                package_share, "launch", "gym_bridge_two_agent.rviz"
            ),
        }.items(),
    )
    controller = Node(
        package="f1tenth_gym_ros",
        executable="slow_agent_controller",
        name="slow_agent_controller",
        output="screen",
        parameters=[os.path.join(package_share, "config", "slow_agent.yaml")],
    )
    return LaunchDescription([simulator, controller])
