# MIT License

# Copyright 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    package_share = get_package_share_directory("f1tenth_gym_ros")
    config = LaunchConfiguration("config_file").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    with open(config, "r", encoding="utf-8") as stream:
        config_dict = yaml.safe_load(stream)
    parameters = config_dict["bridge"]["ros__parameters"]
    has_opponent = int(parameters["num_agent"]) > 1

    nodes = [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", rviz_config],
        ),
        Node(
            package="f1tenth_gym_ros",
            executable="gym_bridge",
            name="bridge",
            parameters=[config],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                {"autostart": True},
                {"node_names": ["map_server"]},
            ],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            parameters=[
                {"yaml_filename": parameters["map_path"] + ".yaml"},
                {"topic": "map"},
                {"frame_id": "map"},
                {"output": "screen"},
                {"use_sim_time": True},
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="ego_robot_state_publisher",
            parameters=[
                {
                    "robot_description": Command(
                        ["xacro ", os.path.join(package_share, "launch", "ego_racecar.xacro")]
                    )
                }
            ],
            remappings=[("/robot_description", "ego_robot_description")],
        ),
    ]
    if has_opponent:
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="opp_robot_state_publisher",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                os.path.join(package_share, "launch", "opp_racecar.xacro"),
                            ]
                        )
                    }
                ],
                remappings=[("/robot_description", "opp_robot_description")],
            )
        )
    return nodes


def generate_launch_description():
    package_share = get_package_share_directory("f1tenth_gym_ros")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(package_share, "config", "sim.yaml"),
                description="Simulator bridge parameter file.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(package_share, "launch", "gym_bridge.rviz"),
                description="RViz display configuration.",
            ),
            OpaqueFunction(function=launch_nodes),
        ]
    )
