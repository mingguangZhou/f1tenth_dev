"""Contracts for the IFAC Roboracer map and optimized racing line."""

import csv
import hashlib
import math
from pathlib import Path

import pytest
import yaml


REPOSITORY = Path(__file__).resolve().parents[2]
MAP_DIRECTORY = REPOSITORY / "f1tenth_gym_ros/maps"
ARTIFACT_DIRECTORY = (
    REPOSITORY / "centerline_tools/output_backup/ifac_roboracer"
)
OBSTACLE_DIRECTORY = (
    REPOSITORY
    / "centerline_tools/obstacle_output/ifac_roboracer_key_turns"
)
MOVING_AGENT_DIRECTORY = (
    REPOSITORY
    / "centerline_tools/moving_agent_output/ifac_roboracer_moving_agent"
)


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_ifac_clean_and_static_obstacle_sim_configs_are_consistent():
    map_yaml_path = MAP_DIRECTORY / "ifac_roboracer.yaml"
    map_metadata = yaml.safe_load(map_yaml_path.read_text(encoding="utf-8"))
    map_image = map_yaml_path.parent / map_metadata["image"]
    assert map_image.is_file()
    assert map_metadata["resolution"] == pytest.approx(0.025)
    assert map_metadata["origin"] == pytest.approx([-17.9339, -8.42608, 0.0])

    obstacle_yaml = yaml.safe_load(
        (OBSTACLE_DIRECTORY / "ifac_roboracer_obstacles.yaml").read_text(
            encoding="utf-8"
        )
    )
    obstacle_image = OBSTACLE_DIRECTORY / obstacle_yaml["image"]
    assert obstacle_image.is_file()
    assert obstacle_image.read_bytes() != map_image.read_bytes()
    assert obstacle_yaml["resolution"] == map_metadata["resolution"]
    assert obstacle_yaml["origin"] == map_metadata["origin"]

    clean_config = yaml.safe_load(
        (
            REPOSITORY / "f1tenth_gym_ros/config/sim_ifac_roboracer.yaml"
        ).read_text(encoding="utf-8")
    )["bridge"]["ros__parameters"]
    assert clean_config["map_path"] == (
        "/sim_ws/src/f1tenth_gym_ros/maps/ifac_roboracer"
    )
    assert clean_config["num_agent"] == 1

    config = yaml.safe_load(
        (
            REPOSITORY
            / "f1tenth_gym_ros/config/sim_ifac_roboracer_obstacles.yaml"
        ).read_text(encoding="utf-8")
    )["bridge"]["ros__parameters"]
    expected_obstacle_map = (
        "/sim_ws/src/centerline_tools/obstacle_output/"
        "ifac_roboracer_key_turns/ifac_roboracer_obstacles"
    )
    assert config["map_path"] == expected_obstacle_map
    assert config["map_img_ext"] == ".png"
    assert config["num_agent"] == 1

    with (ARTIFACT_DIRECTORY / "raceline_points_optimized.csv").open(
        "r", newline="", encoding="utf-8"
    ) as stream:
        raceline = list(csv.DictReader(stream))
    start = raceline[300]
    assert config["sx"] == pytest.approx(float(start["x"]), abs=1.0e-12)
    assert config["sy"] == pytest.approx(float(start["y"]), abs=1.0e-12)
    assert config["stheta"] == pytest.approx(
        float(start["yaw"]), abs=1.0e-12
    )


def test_ifac_key_turn_obstacles_are_equal_size_and_block_the_raceline():
    spec = yaml.safe_load(
        (
            REPOSITORY
            / "centerline_tools/obstacle_specs/ifac_roboracer_key_turns.yaml"
        ).read_text(encoding="utf-8")
    )["obstacles"]
    assert len(spec) == 3
    assert {float(obstacle["size_m"]) for obstacle in spec} == {0.40}
    assert {float(obstacle["requested_gap_m"]) for obstacle in spec} == {
        0.50
    }
    assert len({obstacle["label"] for obstacle in spec}) == 3
    with (ARTIFACT_DIRECTORY / "raceline_points_optimized.csv").open(
        "r", newline="", encoding="utf-8"
    ) as stream:
        raceline = list(csv.DictReader(stream))
    assert all(
        float(raceline[int(obstacle["centerline_index"])]["curvature_abs"])
        >= 0.30
        for obstacle in spec
    )

    with (
        OBSTACLE_DIRECTORY / "ifac_roboracer_obstacles_clearance.csv"
    ).open("r", newline="", encoding="utf-8") as stream:
        obstacles = list(csv.DictReader(stream))
    assert len(obstacles) == 3
    assert {row["label"] for row in obstacles} == {
        obstacle["label"] for obstacle in spec
    }
    assert all(row["raceline_blocked"] == "True" for row in obstacles)
    assert all(float(row["size_m"]) == pytest.approx(0.40) for row in obstacles)
    assert all(float(row["min_clearance_m"]) >= 0.47 for row in obstacles)


def test_ifac_moving_agent_fixture_uses_the_validated_safe_route():
    single_config = yaml.safe_load(
        (
            REPOSITORY
            / "f1tenth_gym_ros/config/sim_ifac_roboracer_obstacles.yaml"
        ).read_text(encoding="utf-8")
    )["bridge"]["ros__parameters"]
    moving_config = yaml.safe_load(
        (
            REPOSITORY
            / "f1tenth_gym_ros/config/sim_ifac_roboracer_moving_agent.yaml"
        ).read_text(encoding="utf-8")
    )["bridge"]["ros__parameters"]
    report = yaml.safe_load(
        (
            MOVING_AGENT_DIRECTORY
            / "ifac_roboracer_moving_agent_path_validation.yaml"
        ).read_text(encoding="utf-8")
    )
    route = (
        MOVING_AGENT_DIRECTORY / "ifac_roboracer_moving_agent_path.csv"
    )

    assert moving_config["num_agent"] == 2
    assert moving_config["map_path"] == single_config["map_path"]
    for field in ("sx", "sy", "stheta"):
        assert moving_config[field] == pytest.approx(single_config[field])

    spawn = report["spawn"]["poses"]
    assert len(spawn) == 1
    assert moving_config["sx1"] == pytest.approx(spawn[0]["x"])
    assert moving_config["sy1"] == pytest.approx(spawn[0]["y"])
    assert moving_config["stheta1"] == pytest.approx(spawn[0]["yaw"])

    metrics = report["metrics"]
    parameters = report["parameters"]
    assert metrics["validated"] is True
    assert metrics["nudge_count"] == 3
    assert metrics["minimum_continuous_map_clearance_m"] >= parameters[
        "minimum_map_clearance_m"
    ]
    assert metrics["minimum_nudge_hold_clearance_m"] >= parameters[
        "minimum_nudge_clearance_m"
    ]
    assert metrics["maximum_abs_curvature_inv_m"] <= parameters[
        "maximum_curvature_inv_m"
    ]
    required_steering = math.atan(
        0.33 * metrics["maximum_abs_curvature_inv_m"]
    )
    assert required_steering < 0.36

    with route.open("r", newline="", encoding="utf-8") as stream:
        assert len(list(csv.DictReader(stream))) == metrics["point_count"]
    slow_config = yaml.safe_load(
        (
            REPOSITORY
            / "f1tenth_gym_ros/config/slow_agent_ifac_roboracer.yaml"
        ).read_text(encoding="utf-8")
    )["slow_agent_controller"]["ros__parameters"]
    assert slow_config["path_csv"] == (
        "/sim_ws/src/centerline_tools/moving_agent_output/"
        "ifac_roboracer_moving_agent/ifac_roboracer_moving_agent_path.csv"
    )
    assert slow_config["maximum_steering_rad"] >= required_steering

    launch = (
        REPOSITORY
        / "f1tenth_gym_ros/launch/ifac_roboracer_moving_agent_launch.py"
    ).read_text(encoding="utf-8")
    assert "sim_ifac_roboracer_moving_agent.yaml" in launch
    assert "slow_agent_ifac_roboracer.yaml" in launch
    assert "gym_bridge_ifac_roboracer.rviz" in launch
    assert 'LaunchConfiguration("use_rviz")' in launch


def test_ifac_optimized_raceline_has_portable_validated_artifacts():
    report_path = (
        ARTIFACT_DIRECTORY / "raceline_points_optimized_validation.yaml"
    )
    report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
    optimizer_config = yaml.safe_load(
        (
            REPOSITORY
            / "centerline_tools"
            / "config"
            / "global_raceline_optimizer_ifac_roboracer.yaml"
        ).read_text(encoding="utf-8")
    )["parameters"]
    raceline = ARTIFACT_DIRECTORY / "raceline_points_optimized.csv"
    centerline = ARTIFACT_DIRECTORY / "centerline_points_smooth.csv"

    assert report["outputs"]["raceline"] == (
        "centerline_tools/output_backup/ifac_roboracer/"
        "raceline_points_optimized.csv"
    )
    assert not Path(report["outputs"]["raceline"]).is_absolute()
    assert report["outputs"]["raceline_sha256"] == _sha256(raceline)
    assert report["inputs"]["centerline_sha256"] == _sha256(centerline)
    assert optimizer_config["curvature_weight"] == pytest.approx(1.22)
    assert optimizer_config["safety_margin_m"] == pytest.approx(0.30)
    assert optimizer_config["curvature_rate_weight"] == pytest.approx(0.025)
    assert report["parameters"]["curvature_weight"] == pytest.approx(
        optimizer_config["curvature_weight"]
    )
    assert report["parameters"]["safety_margin_m"] == pytest.approx(
        optimizer_config["safety_margin_m"]
    )
    assert report["parameters"]["curvature_rate_weight"] == pytest.approx(
        optimizer_config["curvature_rate_weight"]
    )
    assert raceline.read_bytes() != centerline.read_bytes()
    assert all(report["validation"].values())
    assert report["metrics"]["optimized_output"][
        "minimum_dense_nearest_cell_clearance_m"
    ] >= 0.25
    assert report["metrics"]["optimized_after_common_sampling"][
        "curvature_rms_inv_m"
    ] < report["metrics"]["centerline_before_common_sampling"][
        "curvature_rms_inv_m"
    ]

    metadata = yaml.safe_load(
        (ARTIFACT_DIRECTORY / "centerline_metadata.yaml").read_text(
            encoding="utf-8"
        )
    )
    loop_validation = metadata["ordered_loop_validation"]
    assert loop_validation["closed"] is True
    assert loop_validation["coverage_ratio"] >= metadata[
        "pipeline_parameters"
    ]["minimum_ordered_loop_coverage_ratio"]
    assert loop_validation["max_step_m"] <= (
        2.0 ** 0.5 * float(metadata["map_metadata"]["resolution"])
        + 1.0e-12
    )
