#!/usr/bin/env python3
"""
Interactive obstacle creation tool for centerline_tools.

Typical use from the package root:
    cd /sim_ws/src/centerline_tools
    python3 obstacle_creation_tool.py Spielberg_map.png Spielberg_map.yaml

Optional explicit centerline/output paths:
    python3 obstacle_creation_tool.py Spielberg_map.png Spielberg_map.yaml \
        --centerline centerline_output/centerline_points_smooth.csv \
        --output-dir obstacle_output

UI model:
  1. Set requested boundary gap [m].
  2. Set square obstacle size [m].
  3. Choose Left or Right relative to centerline driving direction/yaw.
  4. Left-click near the centerline to add an obstacle.
  5. Repeat as needed; Undo/Reset are available.
  6. Save writes a new map image + map YAML, plus a clearance CSV and debug PNG.

The original input map is never overwritten.
"""

import argparse
import csv
import os
from collections import Counter

import cv2
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.widgets import Button, RadioButtons, TextBox
import numpy as np
import yaml


# =============================================================================
# USER-EDITABLE PARAMETERS
# =============================================================================

# Announced orientation constraint. Keep this easy to adjust if the competition
# specification changes later.
MIN_BOUNDARY_GAP_M = 0.50

# Competition obstacle size limit for the current square V0 model.
MAX_OBSTACLE_SIZE_M = 0.50

# Default UI values.
DEFAULT_REQUESTED_GAP_M = 0.50
DEFAULT_OBSTACLE_SIZE_M = 0.40
DEFAULT_SIDE = "left"  # "left" or "right"

# A map click must be this close to the centerline before it is accepted.
CENTERLINE_CLICK_MAX_DISTANCE_M = 0.50

# The selected-side realized gap may differ slightly from the requested value
# because placement is rasterized on a finite-resolution occupancy map.
MAX_REQUESTED_GAP_ERROR_M = 0.10

# Lateral placement search resolution. The effective step is the smaller of
# this value and half a map pixel, with a small lower bound.
LATERAL_SEARCH_STEP_M = 0.02

# Raycast step for locating left/right track boundaries.
BOUNDARY_RAY_STEP_M = 0.01
BOUNDARY_RAY_MAX_DISTANCE_M = 20.0

# Numerical/raster tolerance used only for the global minimum-clearance check.
# The actual displayed lateral gaps are not reduced by this tolerance.
CLEARANCE_TOLERANCE_M = 0.03

# Output defaults relative to /sim_ws/src/centerline_tools when run there.
DEFAULT_CENTERLINE_CSV = "centerline_output/centerline_points_smooth.csv"
DEFAULT_OUTPUT_DIR = "obstacle_output"


# =============================================================================
# IO + COORDINATE HELPERS
# =============================================================================

def load_map(map_path, yaml_path):
    """Load map image and YAML; return ROS/world-aligned image (Y flipped)."""
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    img = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise RuntimeError(f"Failed to load map image: {map_path}")

    if img.ndim == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()

    return np.flipud(img), np.flipud(gray), meta


def load_centerline_csv(csv_path):
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        required = {"index", "x", "y", "yaw"}
        if reader.fieldnames is None or not required.issubset(set(reader.fieldnames)):
            raise RuntimeError(
                f"Centerline CSV must contain columns {sorted(required)}; "
                f"got {reader.fieldnames}"
            )

        for row in reader:
            rows.append({
                "index": int(row["index"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "yaw": float(row["yaw"]),
            })

    if len(rows) < 3:
        raise RuntimeError(f"Centerline CSV has too few rows: {csv_path}")
    return rows


def get_world_extent(img_shape, meta):
    h, w = img_shape[:2]
    res = float(meta["resolution"])
    origin = meta["origin"]
    return [
        float(origin[0]),
        float(origin[0]) + w * res,
        float(origin[1]),
        float(origin[1]) + h * res,
    ]


def world_to_pixel_float(x, y, meta):
    res = float(meta["resolution"])
    origin = meta["origin"]
    px = (float(x) - float(origin[0])) / res
    py = (float(y) - float(origin[1])) / res
    return py, px


def world_to_pixel(x, y, meta):
    py, px = world_to_pixel_float(x, y, meta)
    return int(round(py)), int(round(px))


def centerline_xy_array(rows):
    return np.asarray([[r["x"], r["y"]] for r in rows], dtype=np.float64)


def nearest_centerline_index(centerline_xy, x, y):
    q = np.asarray([float(x), float(y)], dtype=np.float64)
    d2 = np.sum((centerline_xy - q) ** 2, axis=1)
    idx = int(np.argmin(d2))
    return idx, float(np.sqrt(d2[idx]))


# =============================================================================
# DRIVABLE REGION + BOUNDARY HELPERS
# =============================================================================

def build_free_mask(gray_ros):
    """
    Match the package's current map convention:
      near-white = free, dark = occupied, mid-gray = unknown.
    """
    return (gray_ros >= 250).astype(np.uint8)


def select_drivable_component_from_centerline(free_mask, centerline_rows, meta):
    """
    Select the free-space connected component traversed by the centerline.

    This avoids re-running the whole centerline/skeleton pipeline. The connected
    component label seen by the largest number of centerline samples is used.
    """
    num_labels, labels = cv2.connectedComponents(free_mask, connectivity=8)
    if num_labels <= 1:
        raise RuntimeError("No free-space connected component found in map.")

    h, w = free_mask.shape
    sampled_labels = []
    for row in centerline_rows:
        py, px = world_to_pixel(row["x"], row["y"], meta)
        if 0 <= py < h and 0 <= px < w:
            label_id = int(labels[py, px])
            if label_id > 0:
                sampled_labels.append(label_id)

    if not sampled_labels:
        raise RuntimeError(
            "Centerline does not overlap any free-space component. "
            "Check map/YAML/centerline coordinate consistency."
        )

    counts = Counter(sampled_labels)
    best_label, best_count = counts.most_common(1)[0]
    coverage = best_count / float(len(centerline_rows))

    if coverage < 0.90:
        print(
            f"Warning: only {coverage:.1%} of centerline samples identify the selected "
            "drivable component. Check map consistency if placement behaves unexpectedly."
        )

    mask = (labels == best_label).astype(np.uint8)
    return mask, best_label, coverage


def point_in_mask(x, y, mask, meta):
    py, px = world_to_pixel(x, y, meta)
    h, w = mask.shape
    if py < 0 or py >= h or px < 0 or px >= w:
        return False
    return bool(mask[py, px] > 0)


def raycast_distance_to_boundary(x, y, direction_xy, drivable_mask, meta):
    """Return distance from centerline point to the last drivable sample."""
    direction = np.asarray(direction_xy, dtype=np.float64)
    norm = float(np.linalg.norm(direction))
    if norm < 1e-12:
        raise ValueError("Boundary ray direction has zero norm.")
    direction /= norm

    if not point_in_mask(x, y, drivable_mask, meta):
        return None

    step = min(float(BOUNDARY_RAY_STEP_M), max(0.005, float(meta["resolution"]) * 0.25))
    d = 0.0
    last_valid = 0.0
    while d <= BOUNDARY_RAY_MAX_DISTANCE_M:
        tx = float(x) + direction[0] * d
        ty = float(y) + direction[1] * d
        if not point_in_mask(tx, ty, drivable_mask, meta):
            return last_valid
        last_valid = d
        d += step

    return None


def local_frame_from_row(row):
    yaw = float(row["yaw"])
    tangent = np.asarray([np.cos(yaw), np.sin(yaw)], dtype=np.float64)
    left_normal = np.asarray([-np.sin(yaw), np.cos(yaw)], dtype=np.float64)
    return tangent, left_normal


# =============================================================================
# OBSTACLE GEOMETRY + VALIDATION
# =============================================================================

def square_corners_world(center_xy, tangent, left_normal, size_m):
    center = np.asarray(center_xy, dtype=np.float64)
    half = 0.5 * float(size_m)
    return np.asarray([
        center + tangent * half + left_normal * half,
        center - tangent * half + left_normal * half,
        center - tangent * half - left_normal * half,
        center + tangent * half - left_normal * half,
    ], dtype=np.float64)


def polygon_mask_from_world_corners(corners_xy, shape, meta):
    pts = []
    for x, y in corners_xy:
        py, px = world_to_pixel(x, y, meta)
        pts.append([px, py])
    pts = np.asarray(pts, dtype=np.int32)

    mask = np.zeros(shape, dtype=np.uint8)
    cv2.fillPoly(mask, [pts], 1)
    return mask


def compute_lateral_gaps(left_boundary_dist, right_boundary_dist, lateral_center, size_m):
    """
    Lateral coordinate convention: +left, -right.
    Return gap from square edge to left and right boundary along local normal.
    """
    half = 0.5 * float(size_m)
    left_gap = float(left_boundary_dist) - (float(lateral_center) + half)
    right_gap = float(right_boundary_dist) + (float(lateral_center) - half)
    return left_gap, right_gap


def parse_positive_float(text, field_name):
    try:
        value = float(text)
    except Exception as exc:
        raise ValueError(f"{field_name} must be a number.") from exc
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError(f"{field_name} must be > 0 m.")
    return value


def validate_user_values(requested_gap_m, size_m):
    if size_m > MAX_OBSTACLE_SIZE_M + 1e-9:
        return False, (
            f"Obstacle size {size_m:.3f} m exceeds maximum "
            f"{MAX_OBSTACLE_SIZE_M:.3f} m."
        )
    if requested_gap_m < MIN_BOUNDARY_GAP_M - 1e-9:
        return False, (
            f"Requested gap {requested_gap_m:.3f} m is below minimum "
            f"{MIN_BOUNDARY_GAP_M:.3f} m."
        )
    return True, ""


def try_place_obstacle(
    centerline_row,
    side,
    requested_gap_m,
    size_m,
    drivable_mask,
    clearance_map_m,
    occupied_by_new_obstacles,
    meta,
):
    """
    Search the selected half of the local track cross-section for the valid square
    whose selected-side lateral gap best matches requested_gap_m.
    """
    valid_values, reason = validate_user_values(requested_gap_m, size_m)
    if not valid_values:
        return None, reason

    side = str(side).lower()
    if side not in ("left", "right"):
        return None, f"Unknown side: {side}"

    x = float(centerline_row["x"])
    y = float(centerline_row["y"])
    tangent, left_normal = local_frame_from_row(centerline_row)

    left_dist = raycast_distance_to_boundary(x, y, left_normal, drivable_mask, meta)
    right_dist = raycast_distance_to_boundary(x, y, -left_normal, drivable_mask, meta)
    if left_dist is None or right_dist is None:
        return None, "Could not determine both track boundaries at this centerline location."

    half = 0.5 * size_m
    l_min = -right_dist + half
    l_max = left_dist - half
    if l_min > l_max:
        return None, "Track cross-section is too narrow for this obstacle size."

    search_step = min(float(LATERAL_SEARCH_STEP_M), max(0.005, float(meta["resolution"]) * 0.5))
    candidate_offsets = np.arange(l_min, l_max + 0.5 * search_step, search_step)

    # Keep obstacle center on the selected side of the centerline.
    if side == "left":
        candidate_offsets = candidate_offsets[candidate_offsets >= -1e-9]
    else:
        candidate_offsets = candidate_offsets[candidate_offsets <= 1e-9]

    if len(candidate_offsets) == 0:
        return None, f"No geometric room exists on the {side} side for this obstacle."

    scored = []
    for lateral_center in candidate_offsets:
        left_gap, right_gap = compute_lateral_gaps(
            left_dist, right_dist, lateral_center, size_m
        )
        selected_gap = left_gap if side == "left" else right_gap
        gap_error = abs(selected_gap - requested_gap_m)
        scored.append((gap_error, float(lateral_center), left_gap, right_gap))

    scored.sort(key=lambda item: item[0])

    best_invalid_reason = None
    for gap_error, lateral_center, left_gap, right_gap in scored:
        if gap_error > MAX_REQUESTED_GAP_ERROR_M + 1e-12:
            break

        if left_gap < MIN_BOUNDARY_GAP_M - 1e-9:
            best_invalid_reason = (
                f"left gap would be {left_gap:.3f} m < {MIN_BOUNDARY_GAP_M:.3f} m"
            )
            continue
        if right_gap < MIN_BOUNDARY_GAP_M - 1e-9:
            best_invalid_reason = (
                f"right gap would be {right_gap:.3f} m < {MIN_BOUNDARY_GAP_M:.3f} m"
            )
            continue

        obstacle_center = np.asarray([x, y], dtype=np.float64) + left_normal * lateral_center
        corners = square_corners_world(obstacle_center, tangent, left_normal, size_m)
        obstacle_mask = polygon_mask_from_world_corners(corners, drivable_mask.shape, meta)
        pix = obstacle_mask > 0

        if not np.any(pix):
            best_invalid_reason = "Obstacle rasterization produced an empty footprint."
            continue

        if np.any((drivable_mask == 0) & pix):
            best_invalid_reason = "Obstacle footprint extends outside the drivable region."
            continue

        min_clearance = float(np.min(clearance_map_m[pix]))
        if min_clearance < MIN_BOUNDARY_GAP_M - CLEARANCE_TOLERANCE_M:
            best_invalid_reason = (
                f"nearest-boundary clearance would be {min_clearance:.3f} m, "
                f"below minimum {MIN_BOUNDARY_GAP_M:.3f} m"
            )
            continue

        if np.any((occupied_by_new_obstacles > 0) & pix):
            best_invalid_reason = "Obstacle would overlap an obstacle already added."
            continue

        selected_gap = left_gap if side == "left" else right_gap
        opposite_gap = right_gap if side == "left" else left_gap

        return {
            "side": side,
            "requested_gap_m": float(requested_gap_m),
            "size_m": float(size_m),
            "centerline_index": int(centerline_row["index"]),
            "anchor_x": x,
            "anchor_y": y,
            "yaw": float(centerline_row["yaw"]),
            "lateral_center_m": float(lateral_center),
            "center_x": float(obstacle_center[0]),
            "center_y": float(obstacle_center[1]),
            "corners": corners,
            "mask": obstacle_mask,
            "left_gap_m": float(left_gap),
            "right_gap_m": float(right_gap),
            "selected_gap_m": float(selected_gap),
            "opposite_gap_m": float(opposite_gap),
            "min_clearance_m": float(min_clearance),
            "left_boundary_dist_m": float(left_dist),
            "right_boundary_dist_m": float(right_dist),
        }, ""

    closest = scored[0]
    closest_selected_gap = closest[2] if side == "left" else closest[3]
    msg = (
        f"No valid {side}-side placement found. Closest selected-side gap was "
        f"{closest_selected_gap:.3f} m for requested {requested_gap_m:.3f} m."
    )
    if best_invalid_reason:
        msg += f" Last validation issue: {best_invalid_reason}."
    return None, msg


# =============================================================================
# OUTPUT
# =============================================================================

def build_output_paths(map_path, output_dir):
    base = os.path.basename(map_path)
    stem, ext = os.path.splitext(base)
    ext = ext.lower()
    if ext not in (".png", ".pgm"):
        raise RuntimeError(
            f"Unsupported map image extension '{ext}'. V0 supports .png and .pgm."
        )

    image_name = f"{stem}_obstacles{ext}"
    yaml_name = f"{stem}_obstacles.yaml"
    return {
        "image": os.path.join(output_dir, image_name),
        "yaml": os.path.join(output_dir, yaml_name),
        "summary_csv": os.path.join(output_dir, f"{stem}_obstacles_clearance.csv"),
        "debug_png": os.path.join(output_dir, f"{stem}_obstacles_debug.png"),
    }


def write_obstacle_outputs(
    img_ros,
    gray_ros,
    meta,
    centerline_xy,
    obstacles,
    map_path,
    output_dir,
):
    os.makedirs(output_dir, exist_ok=True)
    paths = build_output_paths(map_path, output_dir)

    modified = img_ros.copy()
    combined_mask = np.zeros(gray_ros.shape, dtype=np.uint8)
    for obs in obstacles:
        combined_mask = np.maximum(combined_mask, obs["mask"])

    if modified.ndim == 2:
        modified[combined_mask > 0] = 0
    else:
        modified[combined_mask > 0] = 0

    # Convert ROS/world-aligned array back to normal image-file row order.
    modified_file_order = np.flipud(modified)
    if not cv2.imwrite(paths["image"], modified_file_order):
        raise RuntimeError(f"Failed to write output map image: {paths['image']}")

    output_meta = dict(meta)
    output_meta["image"] = os.path.basename(paths["image"])
    with open(paths["yaml"], "w") as f:
        yaml.safe_dump(output_meta, f, sort_keys=False)

    fieldnames = [
        "obstacle_id",
        "centerline_index",
        "side",
        "requested_gap_m",
        "selected_gap_m",
        "opposite_gap_m",
        "left_gap_m",
        "right_gap_m",
        "min_clearance_m",
        "size_m",
        "center_x",
        "center_y",
        "yaw",
    ]
    with open(paths["summary_csv"], "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for i, obs in enumerate(obstacles, start=1):
            writer.writerow({
                "obstacle_id": i,
                "centerline_index": obs["centerline_index"],
                "side": obs["side"],
                "requested_gap_m": obs["requested_gap_m"],
                "selected_gap_m": obs["selected_gap_m"],
                "opposite_gap_m": obs["opposite_gap_m"],
                "left_gap_m": obs["left_gap_m"],
                "right_gap_m": obs["right_gap_m"],
                "min_clearance_m": obs["min_clearance_m"],
                "size_m": obs["size_m"],
                "center_x": obs["center_x"],
                "center_y": obs["center_y"],
                "yaw": obs["yaw"],
            })

    save_debug_overlay(
        modified,
        meta,
        centerline_xy,
        obstacles,
        paths["debug_png"],
    )

    print("\nObstacle map saved:")
    print(f"  Map image:        {paths['image']}")
    print(f"  Map YAML:         {paths['yaml']}")
    print(f"  Clearance CSV:    {paths['summary_csv']}")
    print(f"  Debug overlay:    {paths['debug_png']}")
    print(f"  Obstacles:        {len(obstacles)}")
    return paths


def save_debug_overlay(img_ros, meta, centerline_xy, obstacles, path):
    if img_ros.ndim == 3:
        base = cv2.cvtColor(img_ros, cv2.COLOR_BGR2GRAY)
    else:
        base = img_ros
    base = base.astype(np.float32)
    if base.max() > 1.0:
        base /= 255.0

    extent = get_world_extent(base.shape, meta)
    fig, ax = plt.subplots(figsize=(11, 8))
    ax.imshow(base, cmap="gray", origin="lower", extent=extent)
    ax.plot(centerline_xy[:, 0], centerline_xy[:, 1], linewidth=1.0, label="smoothed centerline")

    for i, obs in enumerate(obstacles, start=1):
        patch = Polygon(obs["corners"], closed=True, alpha=0.45)
        ax.add_patch(patch)
        ax.text(
            obs["center_x"],
            obs["center_y"],
            f"O{i}\nL {obs['left_gap_m']:.2f} m\nR {obs['right_gap_m']:.2f} m",
            fontsize=8,
            ha="center",
            va="center",
            bbox=dict(facecolor="white", alpha=0.78, edgecolor="none", pad=1.5),
        )

    ax.set_title("Obstacle Map Debug Overlay")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.axis("equal")
    ax.grid(True, linewidth=0.3, alpha=0.35)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


# =============================================================================
# INTERACTIVE UI
# =============================================================================

def try_enable_interactive_backend():
    current = str(plt.get_backend()).lower()
    if "agg" not in current:
        return True
    for backend in ("TkAgg", "Qt5Agg", "QtAgg"):
        try:
            plt.switch_backend(backend)
            return True
        except Exception:
            pass
    return False


def run_editor_ui(
    img_ros,
    gray_ros,
    meta,
    centerline_rows,
    drivable_mask,
    map_path,
    output_dir,
):
    if not try_enable_interactive_backend():
        raise RuntimeError(
            "No interactive Matplotlib backend is available. "
            "Run this tool in the same X11/VNC GUI environment used by the raceline keypoint editor."
        )

    centerline_xy = centerline_xy_array(centerline_rows)
    extent = get_world_extent(gray_ros.shape, meta)
    base = gray_ros.astype(np.float32) / 255.0

    # Distance to nearest non-drivable pixel, used for global clearance validation.
    clearance_px = cv2.distanceTransform(
        (drivable_mask * 255).astype(np.uint8), cv2.DIST_L2, 5
    )
    clearance_map_m = clearance_px * float(meta["resolution"])

    state = {
        "obstacles": [],
        "occupied_mask": np.zeros(drivable_mask.shape, dtype=np.uint8),
        "side": DEFAULT_SIDE,
        "status": "Set gap/size/side, then left-click near the centerline.",
        "status_ok": True,
        "last_anchor": None,
        "view_initialized": False,
        "saved": False,
        "cancelled": False,
    }

    fig = plt.figure(figsize=(15, 9))
    ax = fig.add_axes([0.05, 0.18, 0.68, 0.77])
    info_ax = fig.add_axes([0.76, 0.20, 0.22, 0.74])
    try:
        fig.canvas.manager.set_window_title("RoboRacer Obstacle Creation Tool")
    except Exception:
        pass

    def toolbar_is_active():
        toolbar = getattr(fig.canvas, "toolbar", None)
        mode = getattr(toolbar, "mode", "") if toolbar is not None else ""
        return bool(mode)

    def set_status(message, ok=True):
        state["status"] = str(message)
        state["status_ok"] = bool(ok)
        print(("OK: " if ok else "INVALID: ") + str(message))

    def rebuild_occupied_mask():
        state["occupied_mask"][:] = 0
        for obs in state["obstacles"]:
            state["occupied_mask"] = np.maximum(state["occupied_mask"], obs["mask"])

    def draw_info_panel():
        info_ax.clear()
        info_ax.axis("off")
        info_ax.text(0.0, 0.98, "Obstacle creation", fontsize=13, weight="bold", va="top")
        info_ax.text(
            0.0,
            0.91,
            f"Minimum boundary gap: {MIN_BOUNDARY_GAP_M:.2f} m\n"
            f"Maximum square size:  {MAX_OBSTACLE_SIZE_M:.2f} m\n"
            f"Selected side:        {state['side'].upper()}",
            fontsize=10,
            va="top",
            family="monospace",
        )

        y = 0.76
        info_ax.text(0.0, y, "Placed obstacles", fontsize=11, weight="bold", va="top")
        y -= 0.045
        if not state["obstacles"]:
            info_ax.text(0.0, y, "None", fontsize=9, va="top")
            y -= 0.05
        else:
            for i, obs in enumerate(state["obstacles"], start=1):
                text = (
                    f"O{i}: idx {obs['centerline_index']} | {obs['side']}\n"
                    f"    size {obs['size_m']:.2f} m | req {obs['requested_gap_m']:.2f} m\n"
                    f"    L {obs['left_gap_m']:.2f} m | R {obs['right_gap_m']:.2f} m"
                )
                info_ax.text(0.0, y, text, fontsize=8.4, va="top", family="monospace")
                y -= 0.105
                if y < 0.25:
                    remaining = len(state["obstacles"]) - i
                    if remaining > 0:
                        info_ax.text(0.0, y, f"... {remaining} more", fontsize=8.5, va="top")
                    break

        status_color = "tab:green" if state["status_ok"] else "tab:red"
        info_ax.text(0.0, 0.18, "Status", fontsize=11, weight="bold", va="top")
        info_ax.text(
            0.0,
            0.14,
            state["status"],
            fontsize=9,
            va="top",
            wrap=True,
            color=status_color,
        )

    def draw():
        old_xlim = ax.get_xlim() if state["view_initialized"] else None
        old_ylim = ax.get_ylim() if state["view_initialized"] else None

        ax.clear()
        ax.imshow(base, cmap="gray", origin="lower", extent=extent)
        ax.plot(
            centerline_xy[:, 0],
            centerline_xy[:, 1],
            linewidth=1.2,
            label="smoothed centerline",
        )

        if state["last_anchor"] is not None:
            idx = state["last_anchor"]
            ax.scatter(
                [centerline_rows[idx]["x"]],
                [centerline_rows[idx]["y"]],
                s=55,
                marker="x",
                zorder=6,
                label="last clicked anchor",
            )

        for i, obs in enumerate(state["obstacles"], start=1):
            patch = Polygon(obs["corners"], closed=True, alpha=0.55, zorder=5)
            ax.add_patch(patch)
            ax.text(
                obs["center_x"],
                obs["center_y"],
                f"O{i}\nL:{obs['left_gap_m']:.2f}\nR:{obs['right_gap_m']:.2f}",
                fontsize=8,
                ha="center",
                va="center",
                zorder=7,
                bbox=dict(facecolor="white", alpha=0.75, edgecolor="none", pad=1.0),
            )

        ax.set_title(
            "Click near centerline to add square obstacle | "
            "Left/Right are relative to centerline yaw"
        )
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.axis("equal")
        ax.grid(True, linewidth=0.3, alpha=0.35)
        ax.legend(loc="best")

        if old_xlim is not None and old_ylim is not None:
            ax.set_xlim(old_xlim)
            ax.set_ylim(old_ylim)
        else:
            state["view_initialized"] = True

        draw_info_panel()
        fig.canvas.draw_idle()

    def read_ui_values():
        requested_gap = parse_positive_float(gap_box.text, "Boundary gap")
        size_m = parse_positive_float(size_box.text, "Obstacle size")
        return requested_gap, size_m

    def on_map_click(event):
        if event.inaxes != ax or toolbar_is_active():
            return
        if getattr(event, "button", None) != 1:
            return
        if event.xdata is None or event.ydata is None:
            return

        idx, click_distance = nearest_centerline_index(
            centerline_xy, event.xdata, event.ydata
        )
        if click_distance > CENTERLINE_CLICK_MAX_DISTANCE_M:
            set_status(
                f"Click is {click_distance:.2f} m from centerline; maximum allowed is "
                f"{CENTERLINE_CLICK_MAX_DISTANCE_M:.2f} m. Nothing added.",
                ok=False,
            )
            draw()
            return

        state["last_anchor"] = idx
        try:
            requested_gap, size_m = read_ui_values()
        except ValueError as exc:
            set_status(str(exc) + " Nothing added.", ok=False)
            draw()
            return

        obstacle, reason = try_place_obstacle(
            centerline_rows[idx],
            state["side"],
            requested_gap,
            size_m,
            drivable_mask,
            clearance_map_m,
            state["occupied_mask"],
            meta,
        )

        if obstacle is None:
            set_status(reason + " Nothing added.", ok=False)
        else:
            state["obstacles"].append(obstacle)
            state["occupied_mask"] = np.maximum(
                state["occupied_mask"], obstacle["mask"]
            )
            set_status(
                f"Added O{len(state['obstacles'])} at centerline idx {idx}: "
                f"L={obstacle['left_gap_m']:.2f} m, "
                f"R={obstacle['right_gap_m']:.2f} m.",
                ok=True,
            )
        draw()

    def undo_last():
        if not state["obstacles"]:
            set_status("No obstacle to undo.", ok=False)
        else:
            removed_id = len(state["obstacles"])
            state["obstacles"].pop()
            rebuild_occupied_mask()
            set_status(f"Removed O{removed_id}.", ok=True)
        draw()

    def reset_all():
        state["obstacles"].clear()
        state["last_anchor"] = None
        rebuild_occupied_mask()
        set_status("All obstacles cleared.", ok=True)
        draw()

    def save_and_close():
        if not state["obstacles"]:
            set_status("No obstacles placed; nothing was saved.", ok=False)
            draw()
            return
        write_obstacle_outputs(
            img_ros,
            gray_ros,
            meta,
            centerline_xy,
            state["obstacles"],
            map_path,
            output_dir,
        )
        state["saved"] = True
        plt.close(fig)

    def cancel_and_close():
        state["cancelled"] = True
        print("Obstacle creation cancelled; no files written.")
        plt.close(fig)

    def on_side_changed(label):
        state["side"] = str(label).lower()
        set_status(f"Placement side set to {state['side'].upper()}.", ok=True)
        draw()

    def apply_scroll_zoom(event):
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        scale_factor = 0.80 if event.button == "up" else 1.25
        xdata = float(event.xdata)
        ydata = float(event.ydata)
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        new_width = (xlim[1] - xlim[0]) * scale_factor
        new_height = (ylim[1] - ylim[0]) * scale_factor
        relx = (xlim[1] - xdata) / (xlim[1] - xlim[0])
        rely = (ylim[1] - ydata) / (ylim[1] - ylim[0])
        ax.set_xlim([xdata - new_width * (1 - relx), xdata + new_width * relx])
        ax.set_ylim([ydata - new_height * (1 - rely), ydata + new_height * rely])
        fig.canvas.draw_idle()

    def on_key(event):
        if event.key in ("u", "U", "ctrl+z"):
            undo_last()
        elif event.key in ("r", "R"):
            reset_all()
        elif event.key in ("enter", "return", "s", "S"):
            save_and_close()
        elif event.key == "escape":
            cancel_and_close()

    fig.canvas.mpl_connect("button_press_event", on_map_click)
    fig.canvas.mpl_connect("scroll_event", apply_scroll_zoom)
    fig.canvas.mpl_connect("key_press_event", on_key)

    # Controls.
    ax_gap = plt.axes([0.06, 0.065, 0.17, 0.05])
    ax_size = plt.axes([0.28, 0.065, 0.17, 0.05])
    gap_box = TextBox(ax_gap, "Gap [m] ", initial=f"{DEFAULT_REQUESTED_GAP_M:.2f}")
    size_box = TextBox(ax_size, "Size [m] ", initial=f"{DEFAULT_OBSTACLE_SIZE_M:.2f}")

    ax_side = plt.axes([0.50, 0.035, 0.10, 0.10])
    side_radio = RadioButtons(ax_side, ("left", "right"), active=0 if DEFAULT_SIDE == "left" else 1)
    side_radio.on_clicked(on_side_changed)

    ax_undo = plt.axes([0.63, 0.065, 0.08, 0.05])
    ax_reset = plt.axes([0.72, 0.065, 0.08, 0.05])
    ax_save = plt.axes([0.81, 0.065, 0.08, 0.05])
    ax_cancel = plt.axes([0.90, 0.065, 0.08, 0.05])
    b_undo = Button(ax_undo, "Undo")
    b_reset = Button(ax_reset, "Reset")
    b_save = Button(ax_save, "Save")
    b_cancel = Button(ax_cancel, "Cancel")
    b_undo.on_clicked(lambda event: undo_last())
    b_reset.on_clicked(lambda event: reset_all())
    b_save.on_clicked(lambda event: save_and_close())
    b_cancel.on_clicked(lambda event: cancel_and_close())

    # Keep strong references for GUI backends.
    state["widgets"] = [
        gap_box,
        size_box,
        side_radio,
        b_undo,
        b_reset,
        b_save,
        b_cancel,
    ]

    draw()

    print("\nObstacle Creation Tool UI")
    print(f"  Minimum boundary gap: {MIN_BOUNDARY_GAP_M:.2f} m")
    print(f"  Maximum obstacle size: {MAX_OBSTACLE_SIZE_M:.2f} m")
    print("  Left/right are relative to centerline yaw/driving direction.")
    print("  Left-click near centerline to add; u=undo, r=reset, Enter/s=save, Esc=cancel.")
    plt.show(block=True)

    return state["saved"]


# =============================================================================
# MAIN
# =============================================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="Interactive square-obstacle map creation tool for centerline_tools."
    )
    parser.add_argument("map_image", help="Input .png or .pgm map image")
    parser.add_argument("map_yaml", help="Input ROS map YAML")
    parser.add_argument(
        "--centerline",
        default=DEFAULT_CENTERLINE_CSV,
        help=f"Smoothed centerline CSV (default: {DEFAULT_CENTERLINE_CSV})",
    )
    parser.add_argument(
        "--output-dir",
        default=DEFAULT_OUTPUT_DIR,
        help=f"Output directory (default: {DEFAULT_OUTPUT_DIR})",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    for path in (args.map_image, args.map_yaml, args.centerline):
        if not os.path.exists(path):
            raise RuntimeError(f"Required input does not exist: {path}")

    img_ros, gray_ros, meta = load_map(args.map_image, args.map_yaml)
    centerline_rows = load_centerline_csv(args.centerline)
    centerline_xy = centerline_xy_array(centerline_rows)

    free_mask = build_free_mask(gray_ros)
    drivable_mask, label_id, centerline_coverage = select_drivable_component_from_centerline(
        free_mask, centerline_rows, meta
    )

    print("Loaded obstacle creation inputs:")
    print(f"  Map image:          {args.map_image}")
    print(f"  Map YAML:           {args.map_yaml}")
    print(f"  Centerline CSV:     {args.centerline}")
    print(f"  Centerline points:  {len(centerline_rows)}")
    print(f"  Map shape:          {gray_ros.shape}")
    print(f"  Resolution:         {float(meta['resolution']):.5f} m/pixel")
    print(f"  Drivable label:     {label_id}")
    print(f"  Centerline coverage:{centerline_coverage:.1%}")
    print(f"  Output directory:   {args.output_dir}")

    # Basic coordinate consistency check.
    extent = get_world_extent(gray_ros.shape, meta)
    inside = (
        (centerline_xy[:, 0] >= extent[0]) &
        (centerline_xy[:, 0] <= extent[1]) &
        (centerline_xy[:, 1] >= extent[2]) &
        (centerline_xy[:, 1] <= extent[3])
    )
    if float(np.mean(inside)) < 0.99:
        raise RuntimeError(
            "Centerline/map coordinate consistency check failed: fewer than 99% "
            "of centerline points lie inside the map extent."
        )

    run_editor_ui(
        img_ros,
        gray_ros,
        meta,
        centerline_rows,
        drivable_mask,
        args.map_image,
        args.output_dir,
    )


if __name__ == "__main__":
    main()
