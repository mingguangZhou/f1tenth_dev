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

"""Tests for the deterministic multi-agent fixture contract."""

import csv
import math
from pathlib import Path

import pytest
import yaml

from f1tenth_gym_ros.multi_agent_fixture import controller_parameters
from f1tenth_gym_ros.multi_agent_fixture import traffic_agent_specs


def fixture_parameters(count=10):
    """Return a compact valid fixture with ``count`` traffic cars."""
    return {
        "num_agent": count + 1,
        "opp_odom_topic": "odom",
        "traffic_namespaces": [f"traffic_{index}" for index in range(count)],
        "traffic_scan_topics": [f"scan_{index}" for index in range(count)],
        "traffic_drive_topics": [f"drive_{index}" for index in range(count)],
        "traffic_start_x": [float(index) for index in range(count)],
        "traffic_start_y": [0.0] * count,
        "traffic_start_theta": [0.0] * count,
    }


def test_eleven_vehicle_fixture_builds_ten_independent_specs():
    """Eleven total vehicles require ten unique traffic specifications."""
    specs = traffic_agent_specs(fixture_parameters())
    assert len(specs) == 10
    assert len({spec.namespace for spec in specs}) == 10


def test_multi_agent_rviz_shows_raceline_and_selected_trajectory():
    """Verify raceline and selected-trajectory displays in the RViz config."""
    repository = Path(__file__).resolve().parents[2]
    rviz_path = (
        repository
        / "f1tenth_gym_ros/launch/gym_bridge_multi_agent.rviz"
    )
    with rviz_path.open("r", encoding="utf-8") as stream:
        rviz = yaml.safe_load(stream)

    displays = {
        display["Name"]: display
        for display in rviz["Visualization Manager"]["Displays"]
    }

    raceline = displays["Raceline Markers"]
    assert raceline["Enabled"] is True
    assert raceline["Class"] == "rviz_default_plugins/MarkerArray"
    assert raceline["Topic"]["Value"] == "/raceline_markers"
    assert raceline["Topic"]["Durability Policy"] == "Transient Local"

    selected = displays["Ultimate Chosen Local Trajectory"]
    assert selected["Enabled"] is True
    assert selected["Class"] == "rviz_default_plugins/Marker"
    assert selected["Topic"]["Value"] == (
        "/drive_arbitration_v2/ultimate_chosen_local_trajectory"
    )


def test_array_length_mismatch_is_rejected():
    """Index-aligned bridge arrays may not silently truncate with zip()."""
    parameters = fixture_parameters()
    parameters["traffic_start_y"].pop()
    with pytest.raises(ValueError, match="traffic_start_y"):
        traffic_agent_specs(parameters)


def test_duplicate_identity_or_topic_is_rejected():
    """Every traffic car owns a unique ROS identity and command topic."""
    parameters = fixture_parameters()
    parameters["traffic_namespaces"][1] = parameters["traffic_namespaces"][0]
    with pytest.raises(ValueError, match="traffic_namespaces must be unique"):
        traffic_agent_specs(parameters)

    parameters = fixture_parameters()
    parameters["traffic_drive_topics"][1] = parameters[
        "traffic_drive_topics"
    ][0]
    with pytest.raises(
        ValueError, match="traffic_drive_topics must be unique"
    ):
        traffic_agent_specs(parameters)


def test_resolved_topic_alias_and_cross_type_collision_are_rejected():
    """Different spellings may not resolve to the same ROS topic."""
    parameters = fixture_parameters()
    parameters["traffic_scan_topics"][1] = "/scan_0"
    with pytest.raises(ValueError, match="traffic_scan_topics must be unique"):
        traffic_agent_specs(parameters)

    parameters = fixture_parameters()
    parameters["traffic_drive_topics"][1] = parameters[
        "traffic_scan_topics"
    ][0]
    with pytest.raises(ValueError, match="resolved scan, drive"):
        traffic_agent_specs(parameters)


def test_hierarchical_and_reserved_traffic_names_are_rejected():
    """Traffic namespaces must also be valid unique launch-node tokens."""
    parameters = fixture_parameters()
    parameters["traffic_namespaces"][0] = "fleet/traffic"
    with pytest.raises(ValueError, match="single ROS name tokens"):
        traffic_agent_specs(parameters)

    parameters = fixture_parameters()
    parameters["traffic_namespaces"][0] = "ego"
    with pytest.raises(ValueError, match="reserved"):
        traffic_agent_specs(parameters)


def test_controller_topics_and_reference_path_owner_are_deterministic():
    """Controllers use isolated topics and one shared route marker."""
    controllers = controller_parameters(
        {"maximum_speed_mps": 1.4}, fixture_parameters(3)
    )
    assert [item["agent_name"] for item in controllers] == [
        "traffic_0",
        "traffic_1",
        "traffic_2",
    ]
    assert [item["odom_topic"] for item in controllers] == [
        "/traffic_0/odom",
        "/traffic_1/odom",
        "/traffic_2/odom",
    ]
    assert [item["publish_reference_path"] for item in controllers] == [
        True,
        False,
        False,
    ]


def _closed_route(path):
    with path.open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    points = [(float(row["x"]), float(row["y"])) for row in rows]
    segments = []
    cumulative = [0.0]
    for index, point in enumerate(points):
        following = points[(index + 1) % len(points)]
        length = math.hypot(following[0] - point[0], following[1] - point[1])
        segments.append(length)
        if index + 1 < len(points):
            cumulative.append(cumulative[-1] + length)
    return points, segments, cumulative, sum(segments)


def _project_to_route(point, points, segments, cumulative):
    best = (math.inf, 0.0, 0.0)
    for index, start in enumerate(points):
        following = points[(index + 1) % len(points)]
        dx = following[0] - start[0]
        dy = following[1] - start[1]
        length_squared = dx * dx + dy * dy
        fraction = max(
            0.0,
            min(
                1.0,
                ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
                / length_squared,
            ),
        )
        projected = (start[0] + fraction * dx, start[1] + fraction * dy)
        distance = math.hypot(point[0] - projected[0], point[1] - projected[1])
        if distance < best[0]:
            best = (
                distance,
                cumulative[index] + fraction * segments[index],
                math.atan2(dy, dx),
            )
    return best


def _angle_error(first, second):
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def test_spielberg_fixture_is_uniform_and_route_validated():
    """The fixture proves count, phase uniformity, and route safety."""
    repository = Path(__file__).resolve().parents[2]
    config_path = (
        repository / "f1tenth_gym_ros/config/sim_multi_agent.yaml"
    )
    route_directory = (
        repository
        / "centerline_tools/moving_agent_output/spielberg_slow_agent"
    )
    route_path = route_directory / "spielberg_slow_agent_path.csv"
    report_path = (
        route_directory / "spielberg_slow_agent_path_validation.yaml"
    )

    with config_path.open("r", encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)["bridge"]["ros__parameters"]
    specs = traffic_agent_specs(parameters)
    assert int(parameters["num_agent"]) == 11
    assert len(specs) == 10

    points, segments, cumulative, lap_length = _closed_route(route_path)
    projections = [
        _project_to_route(
            (spec.start_x, spec.start_y), points, segments, cumulative
        )
        for spec in specs
    ]
    assert max(item[0] for item in projections) <= 1.0e-5
    assert max(
        _angle_error(spec.start_theta, projection[2])
        for spec, projection in zip(specs, projections)
    ) <= 1.0e-4

    stations = sorted(item[1] for item in projections)
    gaps = [
        stations[(index + 1) % len(stations)] - station
        for index, station in enumerate(stations)
    ]
    gaps[-1] += lap_length
    expected_gap = lap_length / len(specs)
    assert max(abs(gap - expected_gap) for gap in gaps) <= 1.0e-5

    with report_path.open("r", encoding="utf-8") as stream:
        report = yaml.safe_load(stream)
    assert report["metrics"]["validated"] is True
    assert (
        report["metrics"]["minimum_continuous_map_clearance_m"]
        >= report["parameters"]["minimum_map_clearance_m"]
    )
    assert (
        report["metrics"]["minimum_nudge_hold_clearance_m"]
        >= report["parameters"]["minimum_nudge_clearance_m"]
    )
    assert report["metrics"]["maximum_abs_curvature_inv_m"] <= report[
        "parameters"
    ]["maximum_curvature_inv_m"]
    assert report["metrics"]["exact_raceline_point_fraction"] >= 0.70
    assert report["metrics"]["nudge_count"] == 9
    assert len(report["nudges"]) == 9
    assert report["inputs"]["raceline"].endswith(
        "raceline_points_smooth.csv"
    )
