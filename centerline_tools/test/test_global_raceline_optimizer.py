#!/usr/bin/env python3
"""Tests for the deterministic offline global raceline optimizer."""

import csv
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from centerline_tools import (  # noqa: E402
    global_raceline_optimizer as optimizer,
)


def wavy_circle(count=240):
    """Return a periodic reference with deterministic curvature ripple."""
    angle = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
    radius = 5.0 + 0.20 * np.sin(7.0 * angle) + 0.05 * np.sin(13.0 * angle)
    return np.column_stack((radius * np.cos(angle), radius * np.sin(angle)))


def write_path_csv(path, points):
    """Write the minimum accepted input schema."""
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("index", "x", "y"))
        for index, point in enumerate(points):
            writer.writerow((index, point[0], point[1]))


def annulus_fixture(tmp_path):
    """Create a ROS-aligned annular drivable mask and metadata."""
    resolution = 0.05
    size = 320
    origin = -8.0
    coordinates = origin + np.arange(size) * resolution
    grid_x, grid_y = np.meshgrid(coordinates, coordinates)
    radius = np.hypot(grid_x, grid_y)
    mask = ((radius >= 3.5) & (radius <= 6.5)).astype(np.uint8)
    mask_path = tmp_path / "drivable_region.npy"
    yaml_path = tmp_path / "map.yaml"
    np.save(mask_path, mask)
    yaml_path.write_text(
        yaml.safe_dump(
            {
                "image": "unused.png",
                "resolution": resolution,
                "origin": [origin, origin, 0.0],
            }
        ),
        encoding="utf-8",
    )
    return mask_path, yaml_path


def test_objective_gradient_and_periodic_shift_invariance():
    reference = wavy_circle(37)
    normals = optimizer.path_normals(reference)
    spacing = float(
        np.mean(
            np.linalg.norm(np.roll(reference, -1, axis=0) - reference, axis=1)
        )
    )
    config = optimizer.OptimizerConfig()
    phase = np.arange(len(reference), dtype=np.float64)
    offsets = 0.04 * np.sin(0.73 * phase)
    direction = np.cos(0.41 * phase)
    direction /= np.linalg.norm(direction)
    value, gradient = optimizer.smoothness_objective(
        offsets,
        reference,
        normals,
        spacing,
        config,
    )
    epsilon = 1.0e-6
    upper = optimizer.smoothness_objective(
        offsets + epsilon * direction,
        reference,
        normals,
        spacing,
        config,
    )[0]
    lower = optimizer.smoothness_objective(
        offsets - epsilon * direction,
        reference,
        normals,
        spacing,
        config,
    )[0]
    numerical = (upper - lower) / (2.0 * epsilon)
    assert numerical == pytest.approx(float(gradient @ direction), rel=1.0e-6)

    shift = 11
    shifted_value, shifted_gradient = optimizer.smoothness_objective(
        np.roll(offsets, shift),
        np.roll(reference, shift, axis=0),
        np.roll(normals, shift, axis=0),
        spacing,
        config,
    )
    assert shifted_value == pytest.approx(value, rel=1.0e-12, abs=1.0e-12)
    np.testing.assert_allclose(
        shifted_gradient,
        np.roll(gradient, shift),
        rtol=1.0e-10,
        atol=1.0e-10,
    )


def test_bounded_optimizer_reduces_curvature_and_curvature_rate():
    reference = wavy_circle()
    config = optimizer.OptimizerConfig()
    lower = np.full(len(reference), -0.80)
    upper = np.full(len(reference), 0.80)
    first = optimizer.optimize_global_raceline(reference, lower, upper, config)
    second = optimizer.optimize_global_raceline(
        reference, lower, upper, config
    )

    before = optimizer.path_metrics(reference)
    after = optimizer.path_metrics(first.points)
    assert all(stage["success"] for stage in first.stages)
    assert after["curvature_rms_inv_m"] <= 0.85 * before["curvature_rms_inv_m"]
    assert (
        after["curvature_rate_rms_inv_m2"]
        <= 0.70 * before["curvature_rate_rms_inv_m2"]
    )
    assert np.all(first.offsets_m >= lower - 1.0e-10)
    assert np.all(first.offsets_m <= upper + 1.0e-10)
    assert np.sign(optimizer.signed_area(first.points)) == np.sign(
        optimizer.signed_area(reference)
    )
    np.testing.assert_allclose(
        first.points, second.points, rtol=0.0, atol=1.0e-12
    )


def test_dense_validator_rejects_segment_crossing_unsafe_hole():
    metadata = optimizer.MapMetadata(0.05, -3.0, -3.0, 0.0)
    safe = np.ones((120, 120), dtype=bool)
    clearance = np.full((120, 120), 10.0)
    safe[55:66, 55:66] = False
    points = np.array(
        [
            [-2.0, 0.0],
            [2.0, 0.0],
            [2.0, 2.0],
            [1.0, 2.0],
            [0.0, 2.0],
            [-1.0, 2.0],
            [-2.0, 2.0],
            [-2.0, 1.0],
        ]
    )
    assert np.all(optimizer.points_in_mask(points, safe, metadata))
    config = replace(
        optimizer.OptimizerConfig(),
        maximum_curvature_inv_m=100.0,
        validation_step_m=0.02,
    )
    with pytest.raises(RuntimeError, match="leaves the eroded corridor"):
        optimizer.validate_optimized_path(
            points,
            safe,
            clearance,
            metadata,
            config,
            optimizer.signed_area(points),
        )


def test_topology_validator_rejects_nonadjacent_touch():
    points = np.array(
        [
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 2.0],
            [0.0, 2.0],
            [0.0, 0.0],
            [-2.0, 0.0],
            [-2.0, -2.0],
            [0.0, -2.0],
        ]
    )
    assert optimizer.first_self_intersection(points) is not None


def test_lateral_raycast_never_exceeds_configured_maximum():
    angle = np.linspace(0.0, 2.0 * np.pi, 16, endpoint=False)
    centerline = np.column_stack((np.cos(angle), np.sin(angle)))
    normals = optimizer.path_normals(centerline)
    metadata = optimizer.MapMetadata(0.10, -3.0, -3.0, 0.0)
    mask = np.ones((60, 60), dtype=bool)
    lower, upper = optimizer.lateral_offset_bounds(
        centerline,
        normals,
        mask,
        metadata,
        maximum_offset_m=0.15,
        ray_step_fraction=0.40,
    )
    assert np.min(lower) >= -0.15
    assert np.max(upper) <= 0.15


def test_cli_writes_csv_report_without_overwriting_source(tmp_path):
    centerline_path = tmp_path / "centerline_points_smooth.csv"
    write_path_csv(centerline_path, wavy_circle())
    source_before = centerline_path.read_bytes()
    mask_path, yaml_path = annulus_fixture(tmp_path)
    output_path = tmp_path / "raceline_points_optimized.csv"
    report_path = tmp_path / "raceline_points_optimized_validation.yaml"

    exit_code = optimizer.main(
        [
            "--centerline",
            str(centerline_path),
            "--drivable-mask",
            str(mask_path),
            "--map-yaml",
            str(yaml_path),
            "--output",
            str(output_path),
            "--report",
            str(report_path),
            "--maximum-lateral-offset-m",
            "0.8",
            "--output-spacing-m",
            "0.08",
        ]
    )
    assert exit_code == 0
    assert centerline_path.read_bytes() == source_before
    with output_path.open("r", newline="", encoding="utf-8") as stream:
        header = tuple(stream.readline().strip().split(","))
        stream.seek(0)
        rows = list(csv.DictReader(stream))
    assert header == optimizer.CSV_FIELDS
    assert [int(row["index"]) for row in rows] == list(range(len(rows)))
    for row in rows:
        assert abs(float(row["curvature"])) == pytest.approx(
            float(row["curvature_abs"]),
            abs=1.0e-12,
        )

    report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
    assert report["validation"] == {
        "solver_converged": True,
        "dense_corridor_clear": True,
        "self_intersection_free": True,
        "direction_preserved": True,
        "maximum_curvature_satisfied": True,
        "curvature_rms_improved": True,
        "curvature_rate_rms_improved": True,
    }
    assert report["outputs"]["raceline_sha256"] == optimizer.file_sha256(
        output_path
    )
    assert report["environment"]["python"]
    assert report["environment"]["numpy"]
    assert report["environment"]["scipy"]
    assert report["solver"]["stages"][-1]["success"] is True
    assert report["metrics"]["optimized_output"]["point_count"] == len(rows)


def test_invalid_input_and_unsafe_output_do_not_replace_source(tmp_path):
    invalid = tmp_path / "invalid.csv"
    invalid.write_text("index,x,y\n0,nan,0\n", encoding="utf-8")
    with pytest.raises(ValueError, match="at least eight"):
        optimizer.load_path_csv(invalid)

    valid = tmp_path / "valid.csv"
    write_path_csv(valid, wavy_circle())
    mask_path, yaml_path = annulus_fixture(tmp_path)
    with pytest.raises(ValueError, match="must not overwrite"):
        optimizer.run_optimization(
            valid,
            mask_path,
            yaml_path,
            valid,
            tmp_path / "report.yaml",
            optimizer.OptimizerConfig(),
        )


def test_output_pair_rolls_back_both_files_on_second_write_failure(
    tmp_path,
    monkeypatch,
):
    csv_path = tmp_path / "raceline.csv"
    report_path = tmp_path / "report.yaml"
    csv_path.write_bytes(b"old csv\n")
    report_path.write_bytes(b"old report\n")
    atomic_write = optimizer._atomic_write_text
    call_count = 0

    def fail_second_write(path, text):
        nonlocal call_count
        call_count += 1
        if call_count == 2:
            raise OSError("injected report failure")
        atomic_write(path, text)

    monkeypatch.setattr(optimizer, "_atomic_write_text", fail_second_write)
    with pytest.raises(OSError, match="injected report failure"):
        optimizer._write_output_pair(
            csv_path,
            "new csv\n",
            report_path,
            "new report\n",
        )
    assert csv_path.read_bytes() == b"old csv\n"
    assert report_path.read_bytes() == b"old report\n"
