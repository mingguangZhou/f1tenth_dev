"""Contracts for the IFAC Roboracer map and optimized racing line."""

import csv
import hashlib
from pathlib import Path

import pytest
import yaml


REPOSITORY = Path(__file__).resolve().parents[2]
MAP_DIRECTORY = REPOSITORY / "f1tenth_gym_ros/maps"
ARTIFACT_DIRECTORY = (
    REPOSITORY / "centerline_tools/output_backup/ifac_roboracer"
)


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_ifac_map_pair_and_ego_only_sim_config_are_consistent():
    map_yaml_path = MAP_DIRECTORY / "ifac_roboracer.yaml"
    map_metadata = yaml.safe_load(map_yaml_path.read_text(encoding="utf-8"))
    map_image = map_yaml_path.parent / map_metadata["image"]
    assert map_image.is_file()
    assert map_metadata["resolution"] == pytest.approx(0.025)
    assert map_metadata["origin"] == pytest.approx([-17.9339, -8.42608, 0.0])

    config = yaml.safe_load(
        (
            REPOSITORY / "f1tenth_gym_ros/config/sim_ifac_roboracer.yaml"
        ).read_text(encoding="utf-8")
    )["bridge"]["ros__parameters"]
    expected_runtime_map = "/sim_ws/src/f1tenth_gym_ros/maps/ifac_roboracer"
    assert config["map_path"] == expected_runtime_map
    assert config["map_img_ext"] == ".png"
    assert config["num_agent"] == 1

    with (
        ARTIFACT_DIRECTORY / "raceline_points_optimized.csv"
    ).open("r", newline="", encoding="utf-8") as stream:
        first = next(csv.DictReader(stream))
    assert config["sx"] == pytest.approx(float(first["x"]), abs=1.0e-12)
    assert config["sy"] == pytest.approx(float(first["y"]), abs=1.0e-12)
    assert config["stheta"] == pytest.approx(
        float(first["yaw"]), abs=1.0e-12
    )


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
