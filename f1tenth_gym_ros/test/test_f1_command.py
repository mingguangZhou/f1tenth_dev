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

import os
from pathlib import Path
import subprocess

import pytest
import yaml


REPOSITORY = Path(__file__).resolve().parents[2]
DK_COMMAND = REPOSITORY / "dk.sh"
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


def test_no_obstacle_defaults_to_clean_ifac_config():
    result = run_sim("--no-obstacle", "use_rviz:=false")
    assert result.returncode == 0
    assert result.stdout.splitlines() == [
        "launch",
        "f1tenth_gym_ros",
        "gym_bridge_launch.py",
        (
            "config_file:=/sim_ws/src/f1tenth_gym_ros/config/"
            "sim_ifac_roboracer.yaml"
        ),
        (
            "rviz_config:=/sim_ws/src/f1tenth_gym_ros/launch/"
            "gym_bridge_ifac_roboracer.rviz"
        ),
        "use_rviz:=false",
    ]


def test_no_obstacle_preserves_explicit_ros_overrides():
    result = run_sim(
        "--no-obstacle",
        "config_file:=/tmp/clean.yaml",
        "rviz_config:=/tmp/clean.rviz",
        "use_rviz:=false",
    )
    assert result.returncode == 0
    assert result.stdout.splitlines() == [
        "launch",
        "f1tenth_gym_ros",
        "gym_bridge_launch.py",
        "config_file:=/tmp/clean.yaml",
        "rviz_config:=/tmp/clean.rviz",
        "use_rviz:=false",
    ]


@pytest.mark.parametrize(
    "mode_flags",
    [
        ("--agents", "--no-agents"),
        ("--agents", "--no-obstacle"),
        ("--no-agents", "--no-obstacle"),
    ],
)
def test_conflicting_simulator_mode_flags_are_rejected(mode_flags):
    result = run_sim(*mode_flags)
    assert result.returncode == 2
    assert "Cannot combine --agents, --no-agents, and --no-obstacle." in result.stderr
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


def test_no_obstacle_config_contains_only_ego_on_clean_map():
    config = yaml.safe_load(
        (
            REPOSITORY / "f1tenth_gym_ros/config/sim_ifac_roboracer.yaml"
        ).read_text(encoding="utf-8")
    )["bridge"]["ros__parameters"]
    assert int(config["num_agent"]) == 1
    assert config["map_path"].endswith("/maps/ifac_roboracer")


def test_f1_shell_syntax():
    subprocess.run(["bash", "-n", str(F1_COMMAND)], check=True)


def test_docker_startup_does_not_install_ros_dependencies(tmp_path):
    if not DK_COMMAND.is_file():
        pytest.skip("repository-root dk.sh is not mounted in this container")

    docker_calls = tmp_path / "docker-calls.txt"
    fake_docker = tmp_path / "docker"
    fake_docker.write_text(
        "#!/usr/bin/env bash\n"
        'printf "%s\\n" "$*" >> "$DOCKER_CALL_LOG"\n'
        'if [[ "${1:-}" == "inspect" ]]; then printf "true\\n"; fi\n',
        encoding="utf-8",
    )
    fake_docker.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = f"{tmp_path}:{environment['PATH']}"
    environment["DOCKER_CALL_LOG"] = str(docker_calls)

    result = subprocess.run(
        [str(DK_COMMAND), "--cpu", "up"],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert result.returncode == 0
    calls = docker_calls.read_text(encoding="utf-8")
    assert " up -d --remove-orphans sim novnc" in calls
    assert " f1 deps" not in calls
