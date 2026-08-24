#!/usr/bin/env python3
# Copyright 2026 F1TENTH Development Contributors
# SPDX-License-Identifier: MIT

"""Generate a raceline-covering, obstacle-safe simulator traffic route."""

import argparse
import csv
import math
import os

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline
import yaml


def smoothstep5(value):
    """Return a quintic smoothstep with flat first/second derivatives."""
    value = np.clip(value, 0.0, 1.0)
    return value**3 * (10.0 + value * (-15.0 + 6.0 * value))


def load_reference_path(path, label="Reference"):
    """Load a finite x/y/yaw closed-path CSV."""
    with open(path, "r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    required = {"x", "y", "yaw"}
    if len(rows) < 3 or not required.issubset(rows[0]):
        raise RuntimeError(
            f"{label} CSV must contain {sorted(required)}: {path}"
        )
    points = np.asarray(
        [[float(row["x"]), float(row["y"])] for row in rows],
        dtype=np.float64,
    )
    yaw = np.asarray([float(row["yaw"]) for row in rows], dtype=np.float64)
    if not np.all(np.isfinite(points)) or not np.all(np.isfinite(yaw)):
        raise RuntimeError(f"{label} CSV contains non-finite values: {path}")
    return points, yaw


def load_centerline(path):
    """Retain the original public loader name for existing callers."""
    return load_reference_path(path, "Centerline")


def load_obstacles(path):
    """Load fixed-obstacle centers and their centerline correspondence."""
    with open(path, "r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    required = {
        "obstacle_id",
        "label",
        "centerline_index",
        "center_x",
        "center_y",
    }
    if not rows or not required.issubset(rows[0]):
        raise RuntimeError(
            f"Obstacle CSV must contain {sorted(required)}: {path}"
        )
    return rows


def closed_geometry(points):
    """Return segment lengths, vertex progress, and closed-loop length."""
    segment_lengths = np.linalg.norm(
        np.roll(points, -1, axis=0) - points,
        axis=1,
    )
    if np.any(segment_lengths <= 1e-9):
        raise RuntimeError(
            "Reference path contains a duplicate adjacent point."
        )
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths[:-1])))
    return segment_lengths, cumulative, float(np.sum(segment_lengths))


def wrapped_signed_distance(progress, center, lap_length):
    """Return shortest signed arc distance from center on a closed loop."""
    return (
        (progress - center + 0.5 * lap_length) % lap_length
        - 0.5 * lap_length
    )


def path_yaw_and_curvature(points):
    """Estimate tangent yaw and signed curvature on a closed sampled path."""
    previous = np.roll(points, 1, axis=0)
    following = np.roll(points, -1, axis=0)
    chord = following - previous
    yaw = np.arctan2(chord[:, 1], chord[:, 0])

    length_a = np.linalg.norm(points - previous, axis=1)
    length_b = np.linalg.norm(following - points, axis=1)
    length_c = np.linalg.norm(following - previous, axis=1)
    cross = (
        (points[:, 0] - previous[:, 0]) * (following[:, 1] - previous[:, 1])
        - (points[:, 1] - previous[:, 1]) * (following[:, 0] - previous[:, 0])
    )
    denominator = length_a * length_b * length_c
    curvature = np.divide(
        2.0 * cross,
        denominator,
        out=np.zeros_like(cross),
        where=denominator > 1e-10,
    )
    return yaw, curvature


def build_offset_path(
    centerline,
    centerline_yaw,
    obstacles,
    cumulative,
    lap_length,
    passing_offset_m,
    hold_half_length_m,
    transition_length_m,
):
    """Build a deterministic centerline-based guide around fixed obstacles."""
    offsets = np.zeros(len(centerline), dtype=np.float64)
    obstacle_sides = []
    for obstacle in obstacles:
        index = int(obstacle["centerline_index"])
        if index < 0 or index >= len(centerline):
            raise RuntimeError(
                f"Obstacle index is outside the centerline: {index}"
            )
        obstacle_center = np.asarray(
            [float(obstacle["center_x"]), float(obstacle["center_y"])],
            dtype=np.float64,
        )
        left_normal = np.asarray(
            [
                -math.sin(centerline_yaw[index]),
                math.cos(centerline_yaw[index]),
            ],
            dtype=np.float64,
        )
        obstacle_side = (
            1.0
            if np.dot(obstacle_center - centerline[index], left_normal) >= 0.0
            else -1.0
        )
        obstacle_sides.append(obstacle_side)

        absolute_distance = np.abs(
            wrapped_signed_distance(cumulative, cumulative[index], lap_length)
        )
        transition_fraction = (
            absolute_distance - hold_half_length_m
        ) / transition_length_m
        weight = np.where(
            absolute_distance <= hold_half_length_m,
            1.0,
            np.where(
                absolute_distance >= hold_half_length_m + transition_length_m,
                0.0,
                1.0 - smoothstep5(transition_fraction),
            ),
        )
        offsets += -obstacle_side * passing_offset_m * weight

    normals = np.column_stack(
        (-np.sin(centerline_yaw), np.cos(centerline_yaw))
    )
    return centerline + normals * offsets[:, None], offsets, obstacle_sides


def periodic_resample_by_lap_phase(source, target_progress, target_lap_length):
    """Cubic-resample a closed path at another path's lap phase."""
    _, source_progress, source_lap_length = closed_geometry(source)
    source_phase = np.concatenate((source_progress / source_lap_length, [1.0]))
    closed_source = np.vstack((source, source[0]))
    target_phase = (
        np.asarray(target_progress, dtype=np.float64) / target_lap_length
    )
    x_spline = CubicSpline(
        source_phase, closed_source[:, 0], bc_type="periodic"
    )
    y_spline = CubicSpline(
        source_phase, closed_source[:, 1], bc_type="periodic"
    )
    return np.column_stack((x_spline(target_phase), y_spline(target_phase)))


def nearest_vertex(points, query):
    """Return the closest sampled vertex to one query point."""
    differences = points - np.asarray(query, dtype=np.float64)
    return int(np.argmin(np.sum(differences * differences, axis=1)))


def build_raceline_nudge_route(
    raceline,
    guide_on_raceline_phase,
    obstacles,
    raceline_progress,
    raceline_lap_length,
    hold_half_length_m,
    transition_length_m,
):
    """Use the guide in smooth windows around projected obstacles."""
    combined_weight = np.zeros(len(raceline), dtype=np.float64)
    nudge_centers = []
    for obstacle in obstacles:
        center = np.asarray(
            [float(obstacle["center_x"]), float(obstacle["center_y"])],
            dtype=np.float64,
        )
        index = nearest_vertex(raceline, center)
        center_progress = float(raceline_progress[index])
        nudge_centers.append((index, center_progress))

        absolute_distance = np.abs(
            wrapped_signed_distance(
                raceline_progress,
                center_progress,
                raceline_lap_length,
            )
        )
        transition_fraction = (
            absolute_distance - hold_half_length_m
        ) / transition_length_m
        weight = np.where(
            absolute_distance <= hold_half_length_m,
            1.0,
            np.where(
                absolute_distance >= hold_half_length_m + transition_length_m,
                0.0,
                1.0 - smoothstep5(transition_fraction),
            ),
        )
        combined_weight = np.maximum(combined_weight, weight)

    route = raceline + combined_weight[:, None] * (
        guide_on_raceline_phase - raceline
    )
    return route, combined_weight, nudge_centers


def load_clearance_grid(map_image_path, map_yaml_path):
    """Load the occupancy image and its metric distance-to-occupancy field."""
    with open(map_yaml_path, "r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"Could not load map image: {map_image_path}")
    image_world = np.flipud(image)
    free_mask = (image_world >= 250).astype(np.uint8)
    resolution = float(metadata["resolution"])
    clearance = cv2.distanceTransform(free_mask * 255, cv2.DIST_L2, 5)
    clearance *= resolution
    origin = np.asarray(metadata["origin"][:2], dtype=np.float64)
    extent = [
        float(origin[0]),
        float(origin[0]) + image_world.shape[1] * resolution,
        float(origin[1]),
        float(origin[1]) + image_world.shape[0] * resolution,
    ]
    return {
        "clearance": clearance,
        "extent": extent,
        "image_world": image_world,
        "origin": origin,
        "resolution": resolution,
    }


def sample_map_clearance(points, clearance_grid):
    """Sample metric occupancy clearance using the map's pixel convention."""
    points = np.asarray(points, dtype=np.float64)
    pixel = np.rint(
        (points - clearance_grid["origin"]) / clearance_grid["resolution"]
    ).astype(int)
    clearance = clearance_grid["clearance"]
    valid = (
        (pixel[:, 0] >= 0)
        & (pixel[:, 0] < clearance.shape[1])
        & (pixel[:, 1] >= 0)
        & (pixel[:, 1] < clearance.shape[0])
    )
    if not np.all(valid):
        raise RuntimeError("Generated path leaves the occupancy-map extent.")
    return clearance[pixel[:, 1], pixel[:, 0]]


def map_clearance(path, map_image_path, map_yaml_path):
    """Retain the original map-clearance interface for existing callers."""
    grid = load_clearance_grid(map_image_path, map_yaml_path)
    return (
        sample_map_clearance(path, grid),
        grid["image_world"],
        grid["extent"],
    )


def densify_closed_path(points, maximum_step_m):
    """Sample every closed-path segment densely for continuous map checks."""
    dense = []
    for start, end in zip(points, np.roll(points, -1, axis=0)):
        length = float(np.linalg.norm(end - start))
        count = max(1, int(math.ceil(length / maximum_step_m)))
        fractions = np.arange(count, dtype=np.float64) / count
        dense.extend(start + fractions[:, None] * (end - start))
    return np.asarray(dense, dtype=np.float64)


def project_point_to_closed_path(points, query):
    """Return closest segment progress on a closed polyline."""
    segment_lengths, cumulative, lap_length = closed_geometry(points)
    vectors = np.roll(points, -1, axis=0) - points
    lengths_squared = segment_lengths * segment_lengths
    query = np.asarray(query, dtype=np.float64)
    fractions = np.sum((query - points) * vectors, axis=1) / lengths_squared
    fractions = np.clip(fractions, 0.0, 1.0)
    projections = points + fractions[:, None] * vectors
    distances_squared = np.sum((projections - query) ** 2, axis=1)
    index = int(np.argmin(distances_squared))
    progress = float(
        cumulative[index] + fractions[index] * segment_lengths[index]
    )
    distance = math.sqrt(float(distances_squared[index]))
    return progress % lap_length, distance, index


def pose_at_progress(points, progress):
    """Interpolate position and tangent at closed-loop progress."""
    segment_lengths, cumulative, lap_length = closed_geometry(points)
    progress = float(progress) % lap_length
    index = int(np.searchsorted(cumulative, progress, side="right") - 1)
    index = max(0, index)
    fraction = (progress - cumulative[index]) / segment_lengths[index]
    following = (index + 1) % len(points)
    position = points[index] + fraction * (points[following] - points[index])
    direction = points[following] - points[index]
    yaw = math.atan2(float(direction[1]), float(direction[0]))
    return position, yaw, index, float(fraction)


def evenly_spaced_spawn_poses(path, reference_point, count):
    """Return exact equal-arc poses anchored nearest a reference point."""
    if count < 1:
        raise RuntimeError("spawn_count must be at least one.")
    _, _, lap_length = closed_geometry(path)
    anchor, _, _ = project_point_to_closed_path(path, reference_point)
    spacing = lap_length / count
    poses = []
    for ordinal in range(count):
        progress = (anchor + ordinal * spacing) % lap_length
        position, yaw, segment_index, segment_fraction = pose_at_progress(
            path, progress
        )
        poses.append(
            {
                "ordinal": ordinal + 1,
                "progress_m": float(progress),
                "x": float(position[0]),
                "y": float(position[1]),
                "yaw": float(yaw),
                "segment_index": int(segment_index),
                "segment_fraction": float(segment_fraction),
            }
        )
    return poses, float(anchor), float(spacing)


def obstacle_nudge_metrics(
    route,
    clearance,
    curvature,
    raceline_progress,
    raceline_lap_length,
    obstacles,
    obstacle_sides,
    nudge_centers,
    passing_offset_m,
    hold_half_length_m,
):
    """Summarize each fixed obstacle's fully committed safe hold region."""
    results = []
    for obstacle, side, (index, center_progress) in zip(
        obstacles, obstacle_sides, nudge_centers
    ):
        absolute_distance = np.abs(
            wrapped_signed_distance(
                raceline_progress,
                center_progress,
                raceline_lap_length,
            )
        )
        hold = absolute_distance <= hold_half_length_m + 1e-9
        obstacle_center = np.asarray(
            [float(obstacle["center_x"]), float(obstacle["center_y"])],
            dtype=np.float64,
        )
        results.append(
            {
                "obstacle_id": str(obstacle["obstacle_id"]),
                "label": str(obstacle["label"]),
                "raceline_index": int(index),
                "raceline_progress_m": float(center_progress),
                "obstacle_side": "left" if side > 0.0 else "right",
                "guide_offset_m": float(-side * passing_offset_m),
                "minimum_hold_map_clearance_m": float(np.min(clearance[hold])),
                "maximum_hold_curvature_inv_m": float(
                    np.max(np.abs(curvature[hold]))
                ),
                "minimum_hold_obstacle_center_distance_m": float(
                    np.min(
                        np.linalg.norm(
                            route[hold] - obstacle_center,
                            axis=1,
                        )
                    )
                ),
            }
        )
    return results


def write_outputs(
    output_dir,
    output_name,
    path,
    yaw,
    curvature,
    signed_deviation,
    deviation,
    nudge_weight,
    raceline,
    guide,
    obstacles,
    obstacle_sides,
    clearance_grid,
    metrics,
    inputs,
    parameters,
    nudges,
    spawn_summary,
):
    """Write the route, validation evidence, and debug overlay."""
    os.makedirs(output_dir, exist_ok=True)
    csv_path = os.path.join(output_dir, f"{output_name}.csv")
    report_path = os.path.join(output_dir, f"{output_name}_validation.yaml")
    debug_path = os.path.join(output_dir, f"{output_name}_debug.png")

    with open(csv_path, "w", newline="", encoding="utf-8") as stream:
        fieldnames = [
            "index",
            "x",
            "y",
            "yaw",
            "curvature",
            "curvature_abs",
            "offset_m",
            "raceline_deviation_m",
            "nudge_weight",
        ]
        writer = csv.DictWriter(
            stream,
            fieldnames=fieldnames,
            lineterminator="\n",
        )
        writer.writeheader()
        for index in range(len(path)):
            writer.writerow(
                {
                    "index": index,
                    "x": float(path[index, 0]),
                    "y": float(path[index, 1]),
                    "yaw": float(yaw[index]),
                    "curvature": float(curvature[index]),
                    "curvature_abs": float(abs(curvature[index])),
                    "offset_m": float(signed_deviation[index]),
                    "raceline_deviation_m": float(deviation[index]),
                    "nudge_weight": float(nudge_weight[index]),
                }
            )

    report = {
        "inputs": inputs,
        "parameters": parameters,
        "metrics": metrics,
        "nudges": nudges,
        "spawn": spawn_summary,
    }
    with open(report_path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(report, stream, sort_keys=False)

    figure, axis = plt.subplots(figsize=(11, 8))
    axis.imshow(
        clearance_grid["image_world"],
        cmap="gray",
        origin="lower",
        extent=clearance_grid["extent"],
    )
    axis.plot(
        raceline[:, 0],
        raceline[:, 1],
        color="tab:red",
        linestyle="--",
        linewidth=0.8,
        label="ego raceline",
    )
    axis.plot(
        guide[:, 0],
        guide[:, 1],
        color="tab:gray",
        linewidth=0.6,
        alpha=0.7,
        label="validated safety guide",
    )
    axis.plot(
        path[:, 0],
        path[:, 1],
        color="tab:orange",
        linewidth=1.2,
        label="traffic route",
    )
    for obstacle, obstacle_side in zip(obstacles, obstacle_sides):
        center_x = float(obstacle["center_x"])
        center_y = float(obstacle["center_y"])
        axis.scatter([center_x], [center_y], color="tab:red", s=18)
        axis.text(
            center_x,
            center_y,
            (
                f"O{obstacle['obstacle_id']} / "
                f"{'L' if obstacle_side > 0 else 'R'}"
            ),
            fontsize=7,
        )
    spawn_x = [pose["x"] for pose in spawn_summary["poses"]]
    spawn_y = [pose["y"] for pose in spawn_summary["poses"]]
    axis.scatter(
        spawn_x,
        spawn_y,
        color="lime",
        edgecolor="black",
        s=32,
        label="traffic spawn",
    )
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title("Raceline-covering traffic route")
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(debug_path, dpi=180)
    plt.close(figure)

    return csv_path, report_path, debug_path


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--centerline", required=True)
    parser.add_argument("--raceline", required=True)
    parser.add_argument("--obstacles", required=True)
    parser.add_argument("--map-image", required=True)
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--output-name", default="spielberg_slow_agent_path")
    parser.add_argument("--passing-offset-m", type=float, default=0.48)
    parser.add_argument(
        "--guide-hold-half-length-m", type=float, default=0.55
    )
    parser.add_argument(
        "--guide-transition-length-m", type=float, default=3.50
    )
    parser.add_argument(
        "--nudge-hold-half-length-m", type=float, default=0.50
    )
    parser.add_argument(
        "--nudge-transition-length-m", type=float, default=5.00
    )
    parser.add_argument(
        "--minimum-map-clearance-m", type=float, default=0.35
    )
    parser.add_argument(
        "--minimum-nudge-clearance-m", type=float, default=0.45
    )
    parser.add_argument("--maximum-curvature-inv-m", type=float, default=0.95)
    parser.add_argument("--spawn-index", type=int, default=3126)
    parser.add_argument("--spawn-count", type=int, default=10)
    return parser.parse_args()


def main():
    args = parse_args()
    for path in (
        args.centerline,
        args.raceline,
        args.obstacles,
        args.map_image,
        args.map_yaml,
    ):
        if not os.path.isfile(path):
            raise RuntimeError(f"Required input does not exist: {path}")
    positive = (
        args.passing_offset_m,
        args.guide_hold_half_length_m,
        args.guide_transition_length_m,
        args.nudge_hold_half_length_m,
        args.nudge_transition_length_m,
        args.minimum_map_clearance_m,
        args.minimum_nudge_clearance_m,
        args.maximum_curvature_inv_m,
    )
    if any(value <= 0.0 for value in positive):
        raise RuntimeError(
            "Route geometry and validation limits must be positive."
        )
    if args.spawn_count < 1:
        raise RuntimeError("spawn_count must be at least one.")

    centerline, centerline_yaw = load_reference_path(
        args.centerline, "Centerline"
    )
    raceline, raceline_input_yaw = load_reference_path(
        args.raceline, "Raceline"
    )
    obstacles = load_obstacles(args.obstacles)
    _, centerline_progress, centerline_lap_length = closed_geometry(centerline)
    _, raceline_progress, raceline_lap_length = closed_geometry(raceline)

    guide, _, obstacle_sides = build_offset_path(
        centerline,
        centerline_yaw,
        obstacles,
        centerline_progress,
        centerline_lap_length,
        args.passing_offset_m,
        args.guide_hold_half_length_m,
        args.guide_transition_length_m,
    )
    guide_on_raceline_phase = periodic_resample_by_lap_phase(
        guide,
        raceline_progress,
        raceline_lap_length,
    )
    path, nudge_weight, nudge_centers = build_raceline_nudge_route(
        raceline,
        guide_on_raceline_phase,
        obstacles,
        raceline_progress,
        raceline_lap_length,
        args.nudge_hold_half_length_m,
        args.nudge_transition_length_m,
    )
    yaw, curvature = path_yaw_and_curvature(path)
    segment_lengths, _, path_lap_length = closed_geometry(path)

    clearance_grid = load_clearance_grid(args.map_image, args.map_yaml)
    clearance = sample_map_clearance(path, clearance_grid)
    dense_path = densify_closed_path(
        path,
        maximum_step_m=0.25 * clearance_grid["resolution"],
    )
    dense_minimum_clearance = float(
        np.min(sample_map_clearance(dense_path, clearance_grid))
    )
    maximum_curvature = float(np.max(np.abs(curvature)))

    displacement = path - raceline
    raceline_normals = np.column_stack(
        (-np.sin(raceline_input_yaw), np.cos(raceline_input_yaw))
    )
    signed_deviation = np.sum(displacement * raceline_normals, axis=1)
    deviation = np.linalg.norm(displacement, axis=1)
    exact_mask = nudge_weight <= 1e-12

    nudges = obstacle_nudge_metrics(
        path,
        clearance,
        curvature,
        raceline_progress,
        raceline_lap_length,
        obstacles,
        obstacle_sides,
        nudge_centers,
        args.passing_offset_m,
        args.nudge_hold_half_length_m,
    )
    minimum_nudge_clearance = min(
        nudge["minimum_hold_map_clearance_m"] for nudge in nudges
    )

    if args.spawn_index < 0 or args.spawn_index >= len(centerline):
        raise RuntimeError("spawn_index is outside the centerline.")
    spawn_poses, anchor_progress, spawn_spacing = evenly_spaced_spawn_poses(
        path,
        centerline[args.spawn_index],
        args.spawn_count,
    )
    spawn_summary = {
        "anchor_reference_index": int(args.spawn_index),
        "anchor_progress_m": anchor_progress,
        "count": int(args.spawn_count),
        "spacing_m": spawn_spacing,
        "poses": spawn_poses,
    }

    if dense_minimum_clearance < args.minimum_map_clearance_m:
        raise RuntimeError(
            f"Generated path clearance {dense_minimum_clearance:.3f} m "
            "is below "
            f"{args.minimum_map_clearance_m:.3f} m."
        )
    if minimum_nudge_clearance < args.minimum_nudge_clearance_m:
        raise RuntimeError(
            f"Obstacle-hold clearance {minimum_nudge_clearance:.3f} m "
            "is below "
            f"{args.minimum_nudge_clearance_m:.3f} m."
        )
    if maximum_curvature > args.maximum_curvature_inv_m:
        raise RuntimeError(
            f"Generated path curvature {maximum_curvature:.3f} 1/m exceeds "
            f"{args.maximum_curvature_inv_m:.3f} 1/m."
        )

    metrics = {
        "point_count": int(len(path)),
        "lap_length_m": path_lap_length,
        "minimum_sampled_map_clearance_m": float(np.min(clearance)),
        "minimum_continuous_map_clearance_m": dense_minimum_clearance,
        "minimum_nudge_hold_clearance_m": float(minimum_nudge_clearance),
        "maximum_abs_curvature_inv_m": maximum_curvature,
        "maximum_raceline_deviation_m": float(np.max(deviation)),
        "mean_raceline_deviation_m": float(np.mean(deviation)),
        "exact_raceline_point_fraction": float(np.mean(exact_mask)),
        "exact_raceline_length_m": float(np.sum(segment_lengths[exact_mask])),
        "nudge_count": int(len(nudges)),
        "validated": True,
    }
    inputs = {
        "centerline": args.centerline,
        "raceline": args.raceline,
        "obstacles": args.obstacles,
        "map_image": args.map_image,
        "map_yaml": args.map_yaml,
    }
    parameters = {
        "passing_offset_m": args.passing_offset_m,
        "guide_hold_half_length_m": args.guide_hold_half_length_m,
        "guide_transition_length_m": args.guide_transition_length_m,
        "nudge_hold_half_length_m": args.nudge_hold_half_length_m,
        "nudge_transition_length_m": args.nudge_transition_length_m,
        "minimum_map_clearance_m": args.minimum_map_clearance_m,
        "minimum_nudge_clearance_m": args.minimum_nudge_clearance_m,
        "maximum_curvature_inv_m": args.maximum_curvature_inv_m,
        "continuous_clearance_step_m": 0.25 * clearance_grid["resolution"],
        "spawn_count": int(args.spawn_count),
    }
    paths = write_outputs(
        args.output_dir,
        args.output_name,
        path,
        yaw,
        curvature,
        signed_deviation,
        deviation,
        nudge_weight,
        raceline,
        guide_on_raceline_phase,
        obstacles,
        obstacle_sides,
        clearance_grid,
        metrics,
        inputs,
        parameters,
        nudges,
        spawn_summary,
    )
    print(
        f"Validated raceline traffic route: points={len(path)} "
        f"lap={path_lap_length:.2f} m "
        f"continuous_clearance={dense_minimum_clearance:.3f} m "
        f"nudge_clearance={minimum_nudge_clearance:.3f} m "
        f"maximum_curvature={maximum_curvature:.3f} 1/m "
        f"exact_raceline={np.mean(exact_mask):.1%}"
    )
    for path in paths:
        print(path)


if __name__ == "__main__":
    main()
