#!/usr/bin/env python3
"""Offline periodic raceline optimization with map-clearance validation."""

import argparse
import csv
from dataclasses import asdict, dataclass
import hashlib
import math
import os
from pathlib import Path
import platform
import tempfile

import numpy as np
import scipy
from scipy.interpolate import CubicSpline
from scipy.ndimage import distance_transform_edt
from scipy.optimize import minimize
import yaml


CSV_FIELDS = ("index", "x", "y", "yaw", "curvature", "curvature_abs")


@dataclass(frozen=True)
class OptimizerConfig:
    """Numerical and geometric settings for one optimization run."""

    optimization_spacing_m: float = 0.15
    output_spacing_m: float = 0.03
    safety_margin_m: float = 0.25
    maximum_lateral_offset_m: float = 1.50
    ray_step_fraction: float = 0.40
    validation_step_m: float = 0.005
    curvature_weight: float = 1.0
    curvature_rate_weight: float = 0.02
    curvature_limit_weight: float = 0.10
    comfortable_curvature_inv_m: float = 0.90
    segment_uniformity_weight: float = 1.0
    path_length_weight: float = 0.005
    offset_weight: float = 0.001
    offset_slope_weight: float = 0.03
    maximum_curvature_inv_m: float = 1.082
    maximum_iterations_per_stage: int = 2500
    function_tolerance: float = 1.0e-9
    gradient_tolerance: float = 2.0e-4

    def validate(self):
        """Reject settings that weaken validation or make solving undefined."""
        positive = {
            "optimization_spacing_m": self.optimization_spacing_m,
            "output_spacing_m": self.output_spacing_m,
            "safety_margin_m": self.safety_margin_m,
            "maximum_lateral_offset_m": self.maximum_lateral_offset_m,
            "ray_step_fraction": self.ray_step_fraction,
            "validation_step_m": self.validation_step_m,
            "maximum_curvature_inv_m": self.maximum_curvature_inv_m,
            "maximum_iterations_per_stage": self.maximum_iterations_per_stage,
            "function_tolerance": self.function_tolerance,
            "gradient_tolerance": self.gradient_tolerance,
        }
        for name, value in positive.items():
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0.0 < self.ray_step_fraction <= 1.0:
            raise ValueError("ray_step_fraction must be in (0, 1]")
        nonnegative = {
            "curvature_weight": self.curvature_weight,
            "curvature_rate_weight": self.curvature_rate_weight,
            "curvature_limit_weight": self.curvature_limit_weight,
            "comfortable_curvature_inv_m": self.comfortable_curvature_inv_m,
            "segment_uniformity_weight": self.segment_uniformity_weight,
            "path_length_weight": self.path_length_weight,
            "offset_weight": self.offset_weight,
            "offset_slope_weight": self.offset_slope_weight,
        }
        for name, value in nonnegative.items():
            if not math.isfinite(float(value)) or float(value) < 0.0:
                raise ValueError(f"{name} must be finite and nonnegative")


@dataclass(frozen=True)
class MapMetadata:
    """ROS occupancy-grid geometry for a ROS-aligned NumPy mask."""

    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    origin_yaw_rad: float

    @classmethod
    def from_yaml(cls, path):
        with Path(path).open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
        if (
            not isinstance(raw, dict)
            or "resolution" not in raw
            or "origin" not in raw
        ):
            raise ValueError("map YAML must contain resolution and origin")
        origin = raw["origin"]
        if not isinstance(origin, (list, tuple)) or len(origin) < 3:
            raise ValueError("map YAML origin must contain x, y, and yaw")
        values = [float(raw["resolution"]), *(float(v) for v in origin[:3])]
        if not all(math.isfinite(v) for v in values) or values[0] <= 0.0:
            raise ValueError("map YAML contains invalid geometry")
        return cls(*values)

    def world_to_grid(self, points):
        """Return floating row/column coordinates for world XY points."""
        points = np.asarray(points, dtype=np.float64)
        delta = points - np.array([self.origin_x_m, self.origin_y_m])
        cosine = math.cos(self.origin_yaw_rad)
        sine = math.sin(self.origin_yaw_rad)
        local_x = cosine * delta[:, 0] + sine * delta[:, 1]
        local_y = -sine * delta[:, 0] + cosine * delta[:, 1]
        return np.column_stack(
            (local_y / self.resolution_m, local_x / self.resolution_m)
        )


@dataclass
class OptimizationResult:
    """Optimized coarse path and solver details."""

    points: np.ndarray
    offsets_m: np.ndarray
    stages: list


def _strip_duplicate_closure(points, tolerance=1.0e-7):
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("path must be an N x 2 array")
    if (
        len(points) >= 2
        and np.linalg.norm(points[-1] - points[0]) <= tolerance
    ):
        points = points[:-1]
    return points


def validate_input_path(points):
    """Validate closed-loop input invariants and return its core points."""
    points = _strip_duplicate_closure(points)
    if len(points) < 8:
        raise ValueError(
            "closed path must contain at least eight unique points"
        )
    if not np.all(np.isfinite(points)):
        raise ValueError("path contains non-finite coordinates")
    segments = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    if float(np.min(segments)) <= 1.0e-6:
        raise ValueError(
            "path contains duplicate or degenerate adjacent points"
        )
    if abs(signed_area(points)) <= 1.0e-6:
        raise ValueError("closed path has zero signed area")
    return points


def load_path_csv(path):
    """Load x/y columns from the standard centerline/raceline CSV format."""
    rows = []
    with Path(path).open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or not {"x", "y"}.issubset(
            reader.fieldnames
        ):
            raise ValueError("path CSV must contain x and y columns")
        for row in reader:
            rows.append((float(row["x"]), float(row["y"])))
    return validate_input_path(np.asarray(rows, dtype=np.float64))


def signed_area(points):
    """Return twice-normalized signed polygon area."""
    points = np.asarray(points, dtype=np.float64)
    following = np.roll(points, -1, axis=0)
    return 0.5 * float(
        np.sum(points[:, 0] * following[:, 1] - following[:, 0] * points[:, 1])
    )


def resample_closed_loop(points, spacing_m):
    """Periodically resample a loop at approximately uniform arc spacing."""
    points = validate_input_path(points)
    segment_lengths = np.linalg.norm(
        np.roll(points, -1, axis=0) - points,
        axis=1,
    )
    total_length = float(np.sum(segment_lengths))
    count = max(8, int(round(total_length / float(spacing_m))))
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    closed = np.vstack((points, points[0]))
    sample_s = np.arange(count, dtype=np.float64) * total_length / count
    x_spline = CubicSpline(cumulative, closed[:, 0], bc_type="periodic")
    y_spline = CubicSpline(cumulative, closed[:, 1], bc_type="periodic")
    result = np.column_stack((x_spline(sample_s), y_spline(sample_s)))
    return validate_input_path(result), total_length / count


def path_normals(points):
    """Compute unit left normals using periodic central tangents."""
    tangents = np.roll(points, -1, axis=0) - np.roll(points, 1, axis=0)
    lengths = np.linalg.norm(tangents, axis=1)
    if float(np.min(lengths)) <= 1.0e-9:
        raise ValueError("cannot compute normals for a degenerate path")
    tangents /= lengths[:, None]
    return np.column_stack((-tangents[:, 1], tangents[:, 0]))


def build_safe_mask(drivable_mask, metadata, safety_margin_m):
    """Erode the corridor by metric clearance plus pixel-center uncertainty."""
    drivable_mask = np.asarray(drivable_mask)
    if drivable_mask.ndim != 2 or drivable_mask.size == 0:
        raise ValueError("drivable mask must be a non-empty 2D array")
    binary = drivable_mask.astype(bool)
    pixel_uncertainty = metadata.resolution_m / math.sqrt(2.0)
    required = float(safety_margin_m) + pixel_uncertainty
    clearance = distance_transform_edt(binary) * metadata.resolution_m
    safe = clearance >= required
    if not np.any(safe):
        raise ValueError("safety margin removes the entire drivable region")
    return safe, clearance, required


def points_in_mask(points, mask, metadata):
    """Test world points against a ROS-aligned binary mask."""
    grid = metadata.world_to_grid(points)
    rows = np.rint(grid[:, 0]).astype(np.int64)
    columns = np.rint(grid[:, 1]).astype(np.int64)
    inside = (
        (rows >= 0)
        & (rows < mask.shape[0])
        & (columns >= 0)
        & (columns < mask.shape[1])
    )
    result = np.zeros(len(points), dtype=bool)
    result[inside] = mask[rows[inside], columns[inside]]
    return result


def lateral_offset_bounds(
    centerline,
    normals,
    safe_mask,
    metadata,
    maximum_offset_m,
    ray_step_fraction,
):
    """Ray-cast hard per-station lateral bounds inside the safe corridor."""
    if not np.all(points_in_mask(centerline, safe_mask, metadata)):
        raise ValueError("centerline is outside the eroded drivable region")
    step = metadata.resolution_m * float(ray_step_fraction)
    full_steps = int(math.floor(maximum_offset_m / step))
    distances = step * np.arange(1, full_steps + 1, dtype=np.float64)
    if len(distances) == 0 or maximum_offset_m - distances[-1] > 1.0e-12:
        distances = np.append(distances, maximum_offset_m)
    lower = np.zeros(len(centerline), dtype=np.float64)
    upper = np.zeros(len(centerline), dtype=np.float64)
    for direction, destination in ((-1.0, lower), (1.0, upper)):
        active = np.ones(len(centerline), dtype=bool)
        last_safe = np.zeros(len(centerline), dtype=np.float64)
        for distance in distances:
            candidates = centerline + direction * distance * normals
            active &= points_in_mask(candidates, safe_mask, metadata)
            last_safe[active] = distance
        destination[:] = direction * last_safe
    if np.any(lower > 0.0) or np.any(upper < 0.0):
        raise RuntimeError("invalid lateral bounds")
    return lower, upper


def _curvature_with_offset_jacobian(reference, normals, offsets):
    """Return exact three-point curvature and its local offset derivatives."""
    points = reference + normals * offsets[:, None]
    previous = np.roll(points, 1, axis=0)
    following = np.roll(points, -1, axis=0)
    incoming = points - previous
    outgoing = following - points
    chord = incoming + outgoing
    incoming_sq = np.sum(incoming * incoming, axis=1)
    outgoing_sq = np.sum(outgoing * outgoing, axis=1)
    chord_sq = np.sum(chord * chord, axis=1)
    minimum_sq = min(incoming_sq.min(), outgoing_sq.min(), chord_sq.min())
    if float(minimum_sq) <= 1.0e-12:
        raise FloatingPointError(
            "optimizer produced degenerate local geometry"
        )
    denominator = np.sqrt(incoming_sq * outgoing_sq * chord_sq)
    cross = incoming[:, 0] * outgoing[:, 1] - incoming[:, 1] * outgoing[:, 0]
    curvature = 2.0 * cross / denominator

    scale = 2.0 / denominator[:, None]
    cross_incoming = np.column_stack((outgoing[:, 1], -outgoing[:, 0]))
    cross_outgoing = np.column_stack((-incoming[:, 1], incoming[:, 0]))
    grad_incoming = scale * (
        cross_incoming
        - cross[:, None]
        * (incoming / incoming_sq[:, None] + chord / chord_sq[:, None])
    )
    grad_outgoing = scale * (
        cross_outgoing
        - cross[:, None]
        * (outgoing / outgoing_sq[:, None] + chord / chord_sq[:, None])
    )
    jac_previous = np.sum(
        -grad_incoming * np.roll(normals, 1, axis=0),
        axis=1,
    )
    jac_current = np.sum((grad_incoming - grad_outgoing) * normals, axis=1)
    jac_following = np.sum(
        grad_outgoing * np.roll(normals, -1, axis=0),
        axis=1,
    )
    segment_lengths = np.sqrt(outgoing_sq)
    return (
        points,
        curvature,
        segment_lengths,
        jac_previous,
        jac_current,
        jac_following,
    )


def smoothness_objective(
    offsets,
    reference,
    normals,
    spacing_m,
    config,
    include_curvature=True,
    include_curvature_rate=True,
):
    """Evaluate the periodic nonlinear objective and its analytic gradient."""
    (
        points,
        curvature,
        segment_lengths,
        jac_previous,
        jac_current,
        jac_following,
    ) = _curvature_with_offset_jacobian(reference, normals, offsets)
    count = len(offsets)
    value = 0.0
    curvature_coefficients = np.zeros(count, dtype=np.float64)

    if include_curvature:
        weight = config.curvature_weight
        value += weight * float(np.mean(curvature * curvature))
        curvature_coefficients += 2.0 * weight * curvature / count

        excess = np.maximum(
            np.abs(curvature) - config.comfortable_curvature_inv_m,
            0.0,
        )
        limit_weight = config.curvature_limit_weight
        value += limit_weight * float(np.mean(excess**4))
        curvature_coefficients += (
            4.0 * limit_weight * excess**3 * np.sign(curvature) / count
        )

    if include_curvature_rate:
        curvature_rate = (np.roll(curvature, -1) - curvature) / spacing_m
        weight = config.curvature_rate_weight
        value += weight * float(np.mean(curvature_rate * curvature_rate))
        curvature_coefficients += (
            2.0
            * weight
            * (np.roll(curvature_rate, 1) - curvature_rate)
            / (count * spacing_m)
        )

    gradient = curvature_coefficients * jac_current
    gradient += np.roll(curvature_coefficients, -1) * np.roll(jac_previous, -1)
    gradient += np.roll(curvature_coefficients, 1) * np.roll(jac_following, 1)

    relative_length_error = segment_lengths / spacing_m - 1.0
    value += config.segment_uniformity_weight * float(
        np.mean(relative_length_error * relative_length_error)
    )
    value += config.path_length_weight * float(
        np.mean(segment_lengths / spacing_m)
    )
    length_derivative = (
        2.0 * config.segment_uniformity_weight * relative_length_error
        + config.path_length_weight
    ) / (count * spacing_m)
    edge_force = length_derivative[:, None] * (
        (np.roll(points, -1, axis=0) - points) / segment_lengths[:, None]
    )
    point_gradient = np.roll(edge_force, 1, axis=0) - edge_force
    gradient += np.sum(point_gradient * normals, axis=1)

    value += config.offset_weight * float(np.mean(offsets * offsets))
    gradient += 2.0 * config.offset_weight * offsets / count
    offset_rate = (np.roll(offsets, -1) - offsets) / spacing_m
    value += config.offset_slope_weight * float(
        np.mean(offset_rate * offset_rate)
    )
    gradient += (
        2.0
        * config.offset_slope_weight
        * (np.roll(offset_rate, 1) - offset_rate)
        / (count * spacing_m)
    )
    if not math.isfinite(value) or not np.all(np.isfinite(gradient)):
        raise FloatingPointError("objective produced non-finite values")
    return value, gradient


def optimize_global_raceline(reference, lower_bounds, upper_bounds, config):
    """Run three deterministic bounded continuation stages."""
    config.validate()
    reference = validate_input_path(reference)
    lower_bounds = np.asarray(lower_bounds, dtype=np.float64)
    upper_bounds = np.asarray(upper_bounds, dtype=np.float64)
    if lower_bounds.shape != (len(reference),) or upper_bounds.shape != (
        len(reference),
    ):
        raise ValueError("lateral bounds must match the reference path")
    if not np.all(np.isfinite(lower_bounds)) or not np.all(
        np.isfinite(upper_bounds)
    ):
        raise ValueError("lateral bounds contain non-finite values")
    if np.any(lower_bounds > 0.0) or np.any(upper_bounds < 0.0):
        raise ValueError("zero offset must be feasible at every station")

    segment_lengths = np.linalg.norm(
        np.roll(reference, -1, axis=0) - reference,
        axis=1,
    )
    spacing_m = float(np.mean(segment_lengths))
    normals = path_normals(reference)
    offsets = np.zeros(len(reference), dtype=np.float64)
    stages = []
    stage_definitions = (
        ("length_initialization", False, False),
        ("minimum_curvature", True, False),
        ("curvature_rate_smoothing", True, True),
    )
    bounds = list(zip(lower_bounds, upper_bounds))
    for name, include_curvature, include_rate in stage_definitions:
        result = minimize(
            lambda values: smoothness_objective(
                values,
                reference,
                normals,
                spacing_m,
                config,
                include_curvature=include_curvature,
                include_curvature_rate=include_rate,
            ),
            offsets,
            jac=True,
            bounds=bounds,
            method="L-BFGS-B",
            options={
                "maxiter": config.maximum_iterations_per_stage,
                "ftol": config.function_tolerance,
                "gtol": config.gradient_tolerance,
                "maxcor": 30,
                "maxls": 40,
            },
        )
        stage = {
            "name": name,
            "success": bool(result.success),
            "status": int(result.status),
            "iterations": int(result.nit),
            "objective": float(result.fun),
            "message": str(result.message),
        }
        stages.append(stage)
        if not result.success or not np.all(np.isfinite(result.x)):
            raise RuntimeError(
                f"optimizer stage {name} failed: {result.message}"
            )
        offsets = np.asarray(result.x, dtype=np.float64)

    points = reference + normals * offsets[:, None]
    return OptimizationResult(points=points, offsets_m=offsets, stages=stages)


def path_geometry(points):
    """Return yaw and exact periodic three-point curvature."""
    points = validate_input_path(points)
    previous = np.roll(points, 1, axis=0)
    following = np.roll(points, -1, axis=0)
    tangent = following - previous
    yaw = np.arctan2(tangent[:, 1], tangent[:, 0])
    incoming = points - previous
    outgoing = following - points
    chord = following - previous
    denominator = (
        np.linalg.norm(incoming, axis=1)
        * np.linalg.norm(outgoing, axis=1)
        * np.linalg.norm(chord, axis=1)
    )
    if float(np.min(denominator)) <= 1.0e-12:
        raise ValueError(
            "path geometry contains a degenerate three-point span"
        )
    cross = incoming[:, 0] * outgoing[:, 1] - incoming[:, 1] * outgoing[:, 0]
    curvature = 2.0 * cross / denominator
    return yaw, curvature


def path_metrics(points):
    """Compute sampling-independent smoothness metrics for a periodic loop."""
    points = validate_input_path(points)
    segments = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    _, curvature = path_geometry(points)
    station_step = 0.5 * (segments + np.roll(segments, -1))
    curvature_rate = (np.roll(curvature, -1) - curvature) / station_step
    length = float(np.sum(segments))
    return {
        "point_count": int(len(points)),
        "length_m": length,
        "spacing_min_m": float(np.min(segments)),
        "spacing_max_m": float(np.max(segments)),
        "spacing_cv": float(np.std(segments) / np.mean(segments)),
        "curvature_rms_inv_m": float(np.sqrt(np.mean(curvature**2))),
        "curvature_abs_p95_inv_m": float(
            np.percentile(np.abs(curvature), 95.0)
        ),
        "curvature_abs_p99_inv_m": float(
            np.percentile(np.abs(curvature), 99.0)
        ),
        "curvature_abs_max_inv_m": float(np.max(np.abs(curvature))),
        "curvature_squared_integral_inv_m": float(
            np.sum(curvature**2 * segments)
        ),
        "curvature_rate_rms_inv_m2": float(
            np.sqrt(np.mean(curvature_rate**2))
        ),
        "curvature_rate_squared_integral_inv_m3": float(
            np.sum(curvature_rate**2 * segments)
        ),
        "signed_area_m2": signed_area(points),
    }


def densify_closed_path(points, maximum_step_m):
    """Sample all segments, including the seam, at a bounded interval."""
    points = validate_input_path(points)
    chunks = []
    for start, end in zip(points, np.roll(points, -1, axis=0)):
        length = float(np.linalg.norm(end - start))
        count = max(1, int(math.ceil(length / maximum_step_m)))
        fractions = np.arange(count, dtype=np.float64) / count
        chunks.append(start + fractions[:, None] * (end - start))
    return np.vstack(chunks)


def minimum_mask_clearance(points, clearance_grid, metadata):
    """Return minimum nearest-cell clearance for world points."""
    grid = metadata.world_to_grid(points)
    rows = np.rint(grid[:, 0]).astype(np.int64)
    columns = np.rint(grid[:, 1]).astype(np.int64)
    inside = (
        (rows >= 0)
        & (rows < clearance_grid.shape[0])
        & (columns >= 0)
        & (columns < clearance_grid.shape[1])
    )
    if not np.all(inside):
        return 0.0
    return float(np.min(clearance_grid[rows, columns]))


def _segments_intersect(a, b, c, d, epsilon=1.0e-10):
    def orient(p, q, r):
        first = q - p
        second = r - p
        return float(first[0] * second[1] - first[1] * second[0])

    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)
    proper_crossing = (
        (o1 > epsilon and o2 < -epsilon) or (o1 < -epsilon and o2 > epsilon)
    ) and (
        (o3 > epsilon and o4 < -epsilon) or (o3 < -epsilon and o4 > epsilon)
    )
    if proper_crossing:
        return True

    def on_segment(start, end, point):
        return (
            min(start[0], end[0]) - epsilon
            <= point[0]
            <= max(start[0], end[0]) + epsilon
            and min(start[1], end[1]) - epsilon
            <= point[1]
            <= max(start[1], end[1]) + epsilon
        )

    return (
        (abs(o1) <= epsilon and on_segment(a, b, c))
        or (abs(o2) <= epsilon and on_segment(a, b, d))
        or (abs(o3) <= epsilon and on_segment(c, d, a))
        or (abs(o4) <= epsilon and on_segment(c, d, b))
    )


def first_self_intersection(points):
    """Return the first nonadjacent segment intersection or None."""
    points = validate_input_path(points)
    segments = np.roll(points, -1, axis=0) - points
    median_length = float(np.median(np.linalg.norm(segments, axis=1)))
    cell_size = max(0.25, 4.0 * median_length)
    buckets = {}
    count = len(points)
    for index, (start, end) in enumerate(
        zip(points, np.roll(points, -1, axis=0))
    ):
        low = np.floor(np.minimum(start, end) / cell_size).astype(int)
        high = np.floor(np.maximum(start, end) / cell_size).astype(int)
        candidates = set()
        cells = []
        for cell_x in range(low[0], high[0] + 1):
            for cell_y in range(low[1], high[1] + 1):
                cell = (cell_x, cell_y)
                cells.append(cell)
                candidates.update(buckets.get(cell, ()))
        for other in candidates:
            if abs(index - other) <= 1 or {index, other} == {0, count - 1}:
                continue
            if _segments_intersect(
                start,
                end,
                points[other],
                points[(other + 1) % count],
            ):
                return other, index
        for cell in cells:
            buckets.setdefault(cell, []).append(index)
    return None


def validate_optimized_path(
    points,
    safe_mask,
    clearance_grid,
    metadata,
    config,
    input_area_sign,
):
    """Run continuous clearance, topology, direction, and curvature gates."""
    points = validate_input_path(points)
    dense = densify_closed_path(points, config.validation_step_m)
    safe = points_in_mask(dense, safe_mask, metadata)
    if not np.all(safe):
        unsafe_count = int(np.sum(~safe))
        raise RuntimeError(
            "optimized path leaves the eroded corridor at "
            f"{unsafe_count} dense samples"
        )
    intersection = first_self_intersection(points)
    if intersection is not None:
        raise RuntimeError(
            "optimized path self-intersects at segments "
            f"{intersection[0]} and {intersection[1]}"
        )
    if math.copysign(1.0, signed_area(points)) != math.copysign(
        1.0, input_area_sign
    ):
        raise RuntimeError("optimized path reverses the input loop direction")
    metrics = path_metrics(points)
    if metrics["curvature_abs_max_inv_m"] > config.maximum_curvature_inv_m:
        raise RuntimeError(
            "optimized path exceeds maximum curvature: "
            f"{metrics['curvature_abs_max_inv_m']:.6f} > "
            f"{config.maximum_curvature_inv_m:.6f} 1/m"
        )
    metrics["minimum_dense_nearest_cell_clearance_m"] = (
        minimum_mask_clearance(
            dense,
            clearance_grid,
            metadata,
        )
    )
    metrics["dense_validation_point_count"] = int(len(dense))
    return metrics


def _atomic_write_text(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".",
        suffix=".tmp",
        dir=str(path.parent),
    )
    try:
        with os.fdopen(
            file_descriptor, "w", encoding="utf-8", newline=""
        ) as stream:
            stream.write(text)
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def save_path_csv(points, path):
    """Atomically save the standard six-column geometry CSV."""
    _atomic_write_text(path, path_csv_text(points))


def path_csv_text(points):
    """Serialize the standard six-column geometry CSV deterministically."""
    yaw, curvature = path_geometry(points)
    lines = [",".join(CSV_FIELDS)]
    for index, ((x_value, y_value), yaw_value, curvature_value) in enumerate(
        zip(points, yaw, curvature)
    ):
        lines.append(
            f"{index},{x_value:.15g},{y_value:.15g},{yaw_value:.15g},"
            f"{curvature_value:.15g},{abs(curvature_value):.15g}"
        )
    return "\n".join(lines) + "\n"


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _plain(value):
    if isinstance(value, dict):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def save_report(report, path):
    """Atomically save a deterministic validation report."""
    text = yaml.safe_dump(_plain(report), sort_keys=False)
    _atomic_write_text(path, text)


def _atomic_write_bytes(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".",
        suffix=".tmp",
        dir=str(path.parent),
    )
    try:
        with os.fdopen(file_descriptor, "wb") as stream:
            stream.write(content)
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def _write_output_pair(csv_path, csv_text, report_path, report_text):
    """Replace CSV/report together and restore both if either write fails."""
    paths = (Path(csv_path), Path(report_path))
    originals = {
        path: path.read_bytes() if path.exists() else None for path in paths
    }
    try:
        _atomic_write_text(paths[0], csv_text)
        _atomic_write_text(paths[1], report_text)
    except BaseException:
        restore_errors = []
        for path in paths:
            try:
                original = originals[path]
                if original is None:
                    try:
                        path.unlink()
                    except FileNotFoundError:
                        pass
                else:
                    _atomic_write_bytes(path, original)
            except BaseException as error:
                restore_errors.append(f"{path}: {error}")
        if restore_errors:
            raise RuntimeError(
                "output write failed and rollback was incomplete: "
                + "; ".join(restore_errors)
            )
        raise


def save_debug_plot(
    centerline, optimized, safe_mask, metadata, path, baseline=None
):
    """Save a map-frame comparison plot without affecting core optimization."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    height, width = safe_mask.shape
    corners = np.array(
        [
            [0.0, 0.0],
            [width * metadata.resolution_m, 0.0],
            [width * metadata.resolution_m, height * metadata.resolution_m],
            [0.0, height * metadata.resolution_m],
        ]
    )
    cosine = math.cos(metadata.origin_yaw_rad)
    sine = math.sin(metadata.origin_yaw_rad)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    world_corners = corners @ rotation.T + np.array(
        [metadata.origin_x_m, metadata.origin_y_m]
    )
    extent = [
        float(np.min(world_corners[:, 0])),
        float(np.max(world_corners[:, 0])),
        float(np.min(world_corners[:, 1])),
        float(np.max(world_corners[:, 1])),
    ]
    figure, axis = plt.subplots(figsize=(12, 8))
    if abs(metadata.origin_yaw_rad) <= 1.0e-12:
        axis.imshow(
            safe_mask,
            origin="lower",
            extent=extent,
            cmap="Greys",
            alpha=0.30,
        )
    axis.plot(
        centerline[:, 0],
        centerline[:, 1],
        "--",
        linewidth=1.0,
        label="centerline",
    )
    if baseline is not None:
        axis.plot(
            baseline[:, 0],
            baseline[:, 1],
            linewidth=1.0,
            label="baseline raceline",
        )
    axis.plot(
        optimized[:, 0],
        optimized[:, 1],
        linewidth=1.5,
        label="optimized raceline",
    )
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("map x [m]")
    axis.set_ylabel("map y [m]")
    axis.set_title("Global periodic raceline optimization")
    axis.legend(loc="best")
    figure.tight_layout()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=150)
    plt.close(figure)


def _config_from_mapping(mapping):
    if mapping is None:
        return OptimizerConfig()
    if not isinstance(mapping, dict):
        raise ValueError("optimizer config YAML must contain a mapping")
    if "parameters" in mapping:
        mapping = mapping["parameters"]
    known = set(OptimizerConfig.__dataclass_fields__)
    unknown = sorted(set(mapping) - known)
    if unknown:
        raise ValueError(
            f"unknown optimizer config keys: {', '.join(unknown)}"
        )
    return OptimizerConfig(**mapping)


def load_optimizer_config(path):
    if path is None:
        return OptimizerConfig()
    with Path(path).open("r", encoding="utf-8") as stream:
        return _config_from_mapping(yaml.safe_load(stream))


def run_optimization(
    centerline_path,
    drivable_mask_path,
    map_yaml_path,
    output_path,
    report_path,
    config,
    baseline_path=None,
    debug_plot_path=None,
):
    """Run the complete fail-closed offline optimization transaction."""
    config.validate()
    centerline_path = Path(centerline_path).resolve()
    drivable_mask_path = Path(drivable_mask_path).resolve()
    map_yaml_path = Path(map_yaml_path).resolve()
    output_path = Path(output_path).resolve()
    report_path = Path(report_path).resolve()
    baseline_resolved = (
        Path(baseline_path).resolve() if baseline_path is not None else None
    )
    protected_paths = {
        centerline_path,
        drivable_mask_path,
        map_yaml_path,
    }
    if baseline_resolved is not None:
        protected_paths.add(baseline_resolved)
    write_paths = {output_path, report_path}
    if debug_plot_path is not None:
        write_paths.add(Path(debug_plot_path).resolve())
    expected_write_count = 2 + int(debug_plot_path is not None)
    if len(write_paths) != expected_write_count:
        raise ValueError(
            "output, report, and debug plot paths must be distinct"
        )
    if protected_paths & write_paths:
        raise ValueError("generated files must not overwrite any input")

    centerline = load_path_csv(centerline_path)
    metadata = MapMetadata.from_yaml(map_yaml_path)
    drivable_mask = np.load(drivable_mask_path, allow_pickle=False)
    safe_mask, clearance_grid, effective_margin = build_safe_mask(
        drivable_mask,
        metadata,
        config.safety_margin_m,
    )
    reference, spacing = resample_closed_loop(
        centerline,
        config.optimization_spacing_m,
    )
    normals = path_normals(reference)
    lower, upper = lateral_offset_bounds(
        reference,
        normals,
        safe_mask,
        metadata,
        config.maximum_lateral_offset_m,
        config.ray_step_fraction,
    )
    optimization = optimize_global_raceline(reference, lower, upper, config)
    optimized, _ = resample_closed_loop(
        optimization.points,
        config.output_spacing_m,
    )

    comparison_spacing = max(config.output_spacing_m, 0.05)
    centerline_comparison, _ = resample_closed_loop(
        centerline, comparison_spacing
    )
    optimized_comparison, _ = resample_closed_loop(
        optimized, comparison_spacing
    )
    before_metrics = path_metrics(centerline_comparison)
    after_metrics = validate_optimized_path(
        optimized,
        safe_mask,
        clearance_grid,
        metadata,
        config,
        signed_area(centerline),
    )
    common_after_metrics = path_metrics(optimized_comparison)
    if common_after_metrics["curvature_rms_inv_m"] > before_metrics[
        "curvature_rms_inv_m"
    ] * (1.0 + 1.0e-6) or common_after_metrics[
        "curvature_rate_rms_inv_m2"
    ] > before_metrics[
        "curvature_rate_rms_inv_m2"
    ] * (
        1.0 + 1.0e-6
    ):
        raise RuntimeError(
            "optimized path does not improve both smoothness metrics"
        )

    baseline = None
    baseline_metrics = None
    if baseline_path is not None:
        baseline = load_path_csv(baseline_resolved)
        baseline_comparison, _ = resample_closed_loop(
            baseline, comparison_spacing
        )
        baseline_metrics = path_metrics(baseline_comparison)

    report = {
        "format_version": 1,
        "environment": {
            "python": platform.python_version(),
            "python_implementation": platform.python_implementation(),
            "numpy": np.__version__,
            "scipy": scipy.__version__,
        },
        "inputs": {
            "centerline": str(centerline_path),
            "centerline_sha256": file_sha256(centerline_path),
            "drivable_mask": str(drivable_mask_path),
            "drivable_mask_sha256": file_sha256(drivable_mask_path),
            "map_yaml": str(map_yaml_path),
            "map_yaml_sha256": file_sha256(map_yaml_path),
            "baseline_raceline": (
                str(baseline_resolved) if baseline_resolved else None
            ),
            "baseline_raceline_sha256": (
                file_sha256(baseline_resolved) if baseline_resolved else None
            ),
        },
        "outputs": {
            "raceline": str(output_path),
            "validation_report": str(report_path),
            "debug_plot": (
                str(Path(debug_plot_path).resolve())
                if debug_plot_path
                else None
            ),
        },
        "parameters": asdict(config),
        "derived": {
            "optimization_point_count": int(len(reference)),
            "optimization_spacing_m": float(spacing),
            "effective_eroded_margin_m": float(effective_margin),
            "minimum_left_bound_m": float(np.min(upper)),
            "minimum_right_bound_m": float(np.min(-lower)),
            "maximum_abs_offset_used_m": float(
                np.max(np.abs(optimization.offsets_m))
            ),
            "comparison_spacing_m": float(comparison_spacing),
        },
        "solver": {"stages": optimization.stages},
        "metrics": {
            "centerline_before_common_sampling": before_metrics,
            "optimized_after_common_sampling": common_after_metrics,
            "optimized_output": after_metrics,
            "baseline_raceline_common_sampling": baseline_metrics,
        },
        "validation": {
            "solver_converged": True,
            "dense_corridor_clear": True,
            "self_intersection_free": True,
            "direction_preserved": True,
            "maximum_curvature_satisfied": True,
            "curvature_rms_improved": True,
            "curvature_rate_rms_improved": True,
        },
    }

    if debug_plot_path is not None:
        save_debug_plot(
            centerline,
            optimized,
            safe_mask,
            metadata,
            debug_plot_path,
            baseline=baseline,
        )
    csv_text = path_csv_text(optimized)
    report["outputs"]["raceline_sha256"] = hashlib.sha256(
        csv_text.encode("utf-8")
    ).hexdigest()
    report_text = yaml.safe_dump(_plain(report), sort_keys=False)
    _write_output_pair(output_path, csv_text, report_path, report_text)
    return report


def _parser(default_config):
    parser = argparse.ArgumentParser(
        description=(
            "Optimize a periodic global raceline for smoothness while "
            "remaining inside an eroded drivable-region mask."
        )
    )
    parser.add_argument(
        "--centerline", required=True, help="Phase-9 centerline CSV"
    )
    parser.add_argument(
        "--drivable-mask", required=True, help="Phase-9 drivable_region.npy"
    )
    parser.add_argument(
        "--map-yaml", required=True, help="ROS map metadata YAML"
    )
    parser.add_argument(
        "--output", help="output CSV; defaults beside the centerline"
    )
    parser.add_argument(
        "--report", help="validation YAML; defaults beside the output"
    )
    parser.add_argument(
        "--baseline-raceline", help="optional existing raceline for metrics"
    )
    parser.add_argument("--debug-plot", help="optional PNG comparison plot")
    parser.add_argument("--config", help="optimizer parameter YAML")
    for name, field in OptimizerConfig.__dataclass_fields__.items():
        default = getattr(default_config, name)
        argument = "--" + name.replace("_", "-")
        value_type = (
            int if field.type is int or isinstance(default, int) else float
        )
        parser.add_argument(argument, type=value_type, default=default)
    return parser


def main(argv=None):
    """CLI entry point."""
    preliminary = argparse.ArgumentParser(add_help=False)
    preliminary.add_argument("--config")
    known, _ = preliminary.parse_known_args(argv)
    config = load_optimizer_config(known.config)
    arguments = _parser(config).parse_args(argv)
    resolved = {
        name: getattr(arguments, name)
        for name in OptimizerConfig.__dataclass_fields__
    }
    config = OptimizerConfig(**resolved)
    centerline = Path(arguments.centerline)
    output = (
        Path(arguments.output)
        if arguments.output
        else centerline.with_name("raceline_points_optimized.csv")
    )
    report = (
        Path(arguments.report)
        if arguments.report
        else output.with_name(output.stem + "_validation.yaml")
    )
    result = run_optimization(
        centerline,
        arguments.drivable_mask,
        arguments.map_yaml,
        output,
        report,
        config,
        baseline_path=arguments.baseline_raceline,
        debug_plot_path=arguments.debug_plot,
    )
    before = result["metrics"]["centerline_before_common_sampling"]
    after = result["metrics"]["optimized_after_common_sampling"]
    print(f"Optimized raceline: {output}")
    print(f"Validation report: {report}")
    print(
        "Curvature RMS: "
        f"{before['curvature_rms_inv_m']:.6f} -> "
        f"{after['curvature_rms_inv_m']:.6f} 1/m"
    )
    print(
        "Curvature-rate RMS: "
        f"{before['curvature_rate_rms_inv_m2']:.6f} -> "
        f"{after['curvature_rate_rms_inv_m2']:.6f} 1/m^2"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
