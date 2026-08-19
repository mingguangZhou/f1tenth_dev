# MIT License
#
# Copyright (c) 2026 F1TENTH Development Contributors
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Launch ego and ten route-following cars on the Spielberg fixture."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from f1tenth_gym_ros.multi_agent_fixture import controller_parameters
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the deterministic eleven-vehicle Spielberg scenario."""
    package_share = get_package_share_directory("f1tenth_gym_ros")
    simulator_config = os.path.join(
        package_share, "config", "sim_multi_agent.yaml"
    )
    with open(simulator_config, "r", encoding="utf-8") as stream:
        simulator_parameters = yaml.safe_load(stream)["bridge"][
            "ros__parameters"
        ]

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "gym_bridge_launch.py")
        ),
        launch_arguments={
            "config_file": simulator_config,
            "rviz_config": os.path.join(
                package_share, "launch", "gym_bridge_multi_agent.rviz"
            ),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    controller_config = os.path.join(
        package_share, "config", "slow_agent.yaml"
    )
    with open(controller_config, "r", encoding="utf-8") as stream:
        common_parameters = yaml.safe_load(stream)["slow_agent_controller"][
            "ros__parameters"
        ]

    controllers = []
    for parameters in controller_parameters(
        common_parameters, simulator_parameters
    ):
        # Every car follows the same fixed, obstacle-nudged route at the same
        # target speed. Spatial phase, not behavior tuning, distributes them.
        parameters["maximum_speed_mps"] = 1.40
        parameters["minimum_speed_mps"] = 1.40
        controllers.append(
            Node(
                package="f1tenth_gym_ros",
                executable="slow_agent_controller",
                name=f"{parameters['agent_name']}_controller",
                output="screen",
                parameters=[parameters],
            )
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the eleven-vehicle fixture.",
            ),
            simulator,
            *controllers,
        ]
    )
