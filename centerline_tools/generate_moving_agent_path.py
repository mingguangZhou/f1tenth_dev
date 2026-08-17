#!/usr/bin/env python3
# Copyright 2026 F1TENTH Development Contributors
# SPDX-License-Identifier: MIT

"""Generate and validate a closed path for the simulator's slower second car."""

import argparse
import csv
import math
import os

import cv2
import matplotlib.pyplot as plt
import numpy as np
import yaml


def smoothstep5(value):
    value = np.clip(value, 0.0, 1.0)
    return value**3 * (10.0 + value * (-15.0 + 6.0 * value))


def load_centerline(path):
    with open(path, "r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    required = {"x", "y", "yaw"}
    if not rows or not required.issubset(rows[0]):
        raise RuntimeError(f"Centerline CSV must contain {sorted(required)}: {path}")
    points = np.asarray(
        [[float(row["x"]), float(row["y"])] for row in rows], dtype=np.float64
    )
    yaw = np.asarray([float(row["yaw"]) for row in rows], dtype=np.float64)
    return points, yaw


def load_obstacles(path):
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
        raise RuntimeError(f"Obstacle CSV must contain {sorted(required)}: {path}")
    return rows


def closed_geometry(points):
    segment_lengths = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    if np.any(segment_lengths <= 1e-9):
        raise RuntimeError("Reference path contains a duplicate adjacent point.")
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths[:-1])))
    return segment_lengths, cumulative, float(np.sum(segment_lengths))


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
    offsets = np.zeros(len(centerline), dtype=np.float64)
    obstacle_sides = []
    for obstacle in obstacles:
        index = int(obstacle["centerline_index"])
        if index < 0 or index >= len(centerline):
            raise RuntimeError(f"Obstacle index is outside the centerline: {index}")
        obstacle_center = np.asarray(
            [float(obstacle["center_x"]), float(obstacle["center_y"])],
            dtype=np.float64,
        )
        left_normal = np.asarray(
            [-math.sin(centerline_yaw[index]), math.cos(centerline_yaw[index])],
            dtype=np.float64,
        )
        obstacle_side = 1.0 if np.dot(
            obstacle_center - centerline[index], left_normal
        ) >= 0.0 else -1.0
        obstacle_sides.append(obstacle_side)

        signed_distance = (
            cumulative - cumulative[index] + 0.5 * lap_length
        ) % lap_length - 0.5 * lap_length
        absolute_distance = np.abs(signed_distance)
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

    normals = np.column_stack((-np.sin(centerline_yaw), np.cos(centerline_yaw)))
    return centerline + normals * offsets[:, None], offsets, obstacle_sides


def path_yaw_and_curvature(points):
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


def map_clearance(path, map_image_path, map_yaml_path):
    with open(map_yaml_path, "r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"Could not load map image: {map_image_path}")
    image_world = np.flipud(image)
    free_mask = (image_world >= 250).astype(np.uint8)
    clearance = cv2.distanceTransform(free_mask * 255, cv2.DIST_L2, 5)
    clearance *= float(metadata["resolution"])

    origin = metadata["origin"]
    resolution = float(metadata["resolution"])
    pixel_x = np.rint((path[:, 0] - float(origin[0])) / resolution).astype(int)
    pixel_y = np.rint((path[:, 1] - float(origin[1])) / resolution).astype(int)
    valid = (
        (pixel_x >= 0)
        & (pixel_x < clearance.shape[1])
        & (pixel_y >= 0)
        & (pixel_y < clearance.shape[0])
    )
    if not np.all(valid):
        raise RuntimeError("Generated path leaves the occupancy-map extent.")
    sampled = clearance[pixel_y, pixel_x]
    extent = [
        float(origin[0]),
        float(origin[0]) + image_world.shape[1] * resolution,
        float(origin[1]),
        float(origin[1]) + image_world.shape[0] * resolution,
    ]
    return sampled, image_world, extent


def write_outputs(
    output_dir,
    output_name,
    path,
    yaw,
    curvature,
    offsets,
    centerline,
    obstacles,
    obstacle_sides,
    map_image,
    map_extent,
    metrics,
    inputs,
    parameters,
    spawn_index,
):
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
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
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
                    "offset_m": float(offsets[index]),
                }
            )

    report = {
        "inputs": inputs,
        "parameters": parameters,
        "metrics": metrics,
        "spawn": {
            "index": int(spawn_index),
            "x": float(path[spawn_index, 0]),
            "y": float(path[spawn_index, 1]),
            "yaw": float(yaw[spawn_index]),
        },
    }
    with open(report_path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(report, stream, sort_keys=False)

    figure, axis = plt.subplots(figsize=(11, 8))
    axis.imshow(map_image, cmap="gray", origin="lower", extent=map_extent)
    axis.plot(
        centerline[:, 0],
        centerline[:, 1],
        color="tab:gray",
        linestyle="--",
        linewidth=0.7,
        label="centerline",
    )
    axis.plot(
        path[:, 0],
        path[:, 1],
        color="tab:orange",
        linewidth=1.2,
        label="slow-agent path",
    )
    for obstacle, obstacle_side in zip(obstacles, obstacle_sides):
        center_x = float(obstacle["center_x"])
        center_y = float(obstacle["center_y"])
        axis.scatter([center_x], [center_y], color="tab:red", s=18)
        axis.text(
            center_x,
            center_y,
            f"O{obstacle['obstacle_id']} / {'L' if obstacle_side > 0 else 'R'}",
            fontsize=7,
        )
    axis.scatter(
        [path[spawn_index, 0]],
        [path[spawn_index, 1]],
        color="lime",
        edgecolor="black",
        s=45,
        label="second-car start",
    )
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title("Spielberg slow-agent path validation")
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(debug_path, dpi=180)
    plt.close(figure)

    return csv_path, report_path, debug_path


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--centerline", required=True)
    parser.add_argument("--obstacles", required=True)
    parser.add_argument("--map-image", required=True)
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--output-name", default="spielberg_slow_agent_path")
    parser.add_argument("--passing-offset-m", type=float, default=0.48)
    parser.add_argument("--hold-half-length-m", type=float, default=0.55)
    parser.add_argument("--transition-length-m", type=float, default=3.50)
    parser.add_argument("--minimum-map-clearance-m", type=float, default=0.45)
    parser.add_argument("--maximum-curvature-inv-m", type=float, default=0.95)
    parser.add_argument("--spawn-index", type=int, default=3126)
    return parser.parse_args()


def main():
    args = parse_args()
    for path in (args.centerline, args.obstacles, args.map_image, args.map_yaml):
        if not os.path.isfile(path):
            raise RuntimeError(f"Required input does not exist: {path}")
    if args.passing_offset_m <= 0.0 or args.transition_length_m <= 0.0:
        raise RuntimeError("Offset and transition length must be positive.")

    centerline, centerline_yaw = load_centerline(args.centerline)
    obstacles = load_obstacles(args.obstacles)
    _, cumulative, reference_lap_length = closed_geometry(centerline)
    path, offsets, obstacle_sides = build_offset_path(
        centerline,
        centerline_yaw,
        obstacles,
        cumulative,
        reference_lap_length,
        args.passing_offset_m,
        args.hold_half_length_m,
        args.transition_length_m,
    )
    yaw, curvature = path_yaw_and_curvature(path)
    _, _, path_lap_length = closed_geometry(path)
    clearance, map_image, map_extent = map_clearance(
        path, args.map_image, args.map_yaml
    )

    if args.spawn_index < 0 or args.spawn_index >= len(path):
        raise RuntimeError("Spawn index is outside the generated path.")
    minimum_clearance = float(np.min(clearance))
    maximum_curvature = float(np.max(np.abs(curvature)))
    if minimum_clearance < args.minimum_map_clearance_m:
        raise RuntimeError(
            f"Generated path clearance {minimum_clearance:.3f} m is below "
            f"{args.minimum_map_clearance_m:.3f} m."
        )
    if maximum_curvature > args.maximum_curvature_inv_m:
        raise RuntimeError(
            f"Generated path curvature {maximum_curvature:.3f} 1/m exceeds "
            f"{args.maximum_curvature_inv_m:.3f} 1/m."
        )

    metrics = {
        "point_count": int(len(path)),
        "lap_length_m": path_lap_length,
        "minimum_map_clearance_m": minimum_clearance,
        "maximum_abs_curvature_inv_m": maximum_curvature,
        "maximum_abs_offset_m": float(np.max(np.abs(offsets))),
        "validated": True,
    }
    inputs = {
        "centerline": args.centerline,
        "obstacles": args.obstacles,
        "map_image": args.map_image,
        "map_yaml": args.map_yaml,
    }
    parameters = {
        "passing_offset_m": args.passing_offset_m,
        "hold_half_length_m": args.hold_half_length_m,
        "transition_length_m": args.transition_length_m,
        "minimum_map_clearance_m": args.minimum_map_clearance_m,
        "maximum_curvature_inv_m": args.maximum_curvature_inv_m,
    }
    paths = write_outputs(
        args.output_dir,
        args.output_name,
        path,
        yaw,
        curvature,
        offsets,
        centerline,
        obstacles,
        obstacle_sides,
        map_image,
        map_extent,
        metrics,
        inputs,
        parameters,
        args.spawn_index,
    )
    print(
        f"Validated path: points={len(path)} lap={path_lap_length:.2f} m "
        f"minimum_clearance={minimum_clearance:.3f} m "
        f"maximum_curvature={maximum_curvature:.3f} 1/m"
    )
    for path in paths:
        print(path)


if __name__ == "__main__":
    main()
