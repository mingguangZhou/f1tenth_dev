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

"""Tests for simulator mode selection in the f1 command."""

from pathlib import Path
import subprocess

import yaml


REPOSITORY = Path(__file__).resolve().parents[2]
F1_COMMAND = next(
    candidate
    for candidate in (REPOSITORY / "f1", Path("/usr/local/bin/f1"))
    if candidate.is_file()
)


def run_sim(*arguments):
    command = r'''
source "$1"
shift
source_workspace() { :; }
configure_rendering() { :; }
ros2() { printf '%s\n' "$@"; }
run_sim "$@"
'''
    return subprocess.run(
        ["bash", "-c", command, "bash", str(F1_COMMAND), *arguments],
        check=False,
        capture_output=True,
        text=True,
    )


def test_sim_defaults_to_ifac_moving_agent_fixture():
    result = run_sim("use_rviz:=false")
    assert result.returncode == 0
    assert result.stdout.splitlines() == [
        "launch",
        "f1tenth_gym_ros",
        "ifac_roboracer_moving_agent_launch.py",
        "use_rviz:=false",
    ]


def test_agents_flag_explicitly_selects_default_fixture():
    result = run_sim("--agents")
    assert result.returncode == 0
    assert result.stdout.splitlines()[2] == (
        "ifac_roboracer_moving_agent_launch.py"
    )
    assert "--agents" not in result.stdout


def test_no_agents_selects_ego_only_launch_and_preserves_ros_arguments():
    result = run_sim(
        "--no-agents",
        "config_file:=/tmp/single.yaml",
        "use_rviz:=false",
    )
    assert result.returncode == 0
    assert result.stdout.splitlines() == [
        "launch",
        "f1tenth_gym_ros",
        "gym_bridge_launch.py",
        "config_file:=/tmp/single.yaml",
        "use_rviz:=false",
    ]


def test_no_agents_defaults_to_ifac_static_obstacle_config():
    result = run_sim("--no-agents", "use_rviz:=false")
    assert result.returncode == 0
    assert result.stdout.splitlines() == [
        "launch",
        "f1tenth_gym_ros",
        "gym_bridge_launch.py",
        (
            "config_file:=/sim_ws/src/f1tenth_gym_ros/config/"
            "sim_ifac_roboracer_obstacles.yaml"
        ),
        (
            "rviz_config:=/sim_ws/src/f1tenth_gym_ros/launch/"
            "gym_bridge_ifac_roboracer.rviz"
        ),
        "use_rviz:=false",
    ]


def test_conflicting_agent_flags_are_rejected():
    result = run_sim("--agents", "--no-agents")
    assert result.returncode == 2
    assert "Cannot combine --agents and --no-agents." in result.stderr
    assert result.stdout == ""


def test_default_agent_free_config_contains_only_ego():
    config = yaml.safe_load(
        (
            REPOSITORY
            / "f1tenth_gym_ros/config/sim_ifac_roboracer_obstacles.yaml"
        ).read_text(
            encoding="utf-8"
        )
    )
    assert int(config["bridge"]["ros__parameters"]["num_agent"]) == 1
    assert config["bridge"]["ros__parameters"]["map_path"].endswith(
        "/ifac_roboracer_obstacles"
    )


def test_f1_shell_syntax():
    subprocess.run(["bash", "-n", str(F1_COMMAND)], check=True)
