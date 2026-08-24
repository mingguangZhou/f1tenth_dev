#!/usr/bin/env python3

import sys
import os
import csv
import yaml
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
from skimage.measure import label
from scipy.interpolate import splprep, splev

# =========================
# CONFIG
# =========================
DEBUG = True

# Phase 2
INFLATION_RADIUS_M = 0.10

# Phase 5
SPUR_DILATION_RADIUS = 1

# Phase 8
RESAMPLE_SPACING_M = 0.05
SMOOTHING_WINDOW = 9       # must be odd and >= 3
SMOOTHING_PASSES = 2

# Experimental Phase 8 replacement: periodic cubic B-spline centerline smoothing.
USE_BSPLINE_CENTERLINE_SMOOTHING = True
BSPLINE_DEGREE = 3
# Larger value = smoother centerline; 0.0002 worked reasonably in the previous test.
BSPLINE_SMOOTHING_FACTOR_PER_POINT = 0.0005

# Final direction for exported centerline geometry.
# normal/csv: keep generated order; reverse: flip order before yaw/curvature export.
CENTERLINE_DIRECTION = "normal"

# Drivable-region selection tuning
MIN_COMPONENT_AREA_ABS = 50
MIN_COMPONENT_AREA_RATIO = 0.01

# A clean single-loop skeleton normally covers at least 98% of its pixels.
# Scanned maps with wide bays can leave short medial-axis branches after the
# conservative spur-pruning fallback. Keep the strict default, but allow an
# explicit per-generation override that is recorded in the output metadata.
MIN_ORDERED_LOOP_COVERAGE_RATIO = float(
    os.environ.get("CENTERLINE_MIN_ORDERED_LOOP_COVERAGE_RATIO", "0.98")
)

# Phase 9 outputs
OUTPUT_DIR = "centerline_output"
RAW_CSV_NAME = "centerline_points_raw.csv"
SMOOTH_CSV_NAME = "centerline_points_smooth.csv"
SMOOTH_NPY_NAME = "centerline_points_smooth.npy"
METADATA_YAML_NAME = "centerline_metadata.yaml"

# Debug image names
DEBUG_DRIVABLE_REGION = "debug_drivable_region.png"
DEBUG_MAIN_LOOP_OVERLAY = "debug_main_loop_overlay.png"
DEBUG_ORDERED_LOOP_OVERLAY = "debug_ordered_loop_overlay.png"
DEBUG_PHASE8_PATHS = "debug_phase8_world_paths.png"

# Phase 10: corner detection only, no raceline generation yet
DETECT_CORNERS = True
CORNERS_CSV_NAME = "corner_key_points.csv"
DEBUG_CORNER_KEYPOINTS = "debug_corner_keypoints.png"
DEBUG_CORNER_CURVATURE = "debug_corner_curvature.png"

# Corner detection tuning
# Curvature is in 1/m.
# Low threshold defines entrance/exit candidate regions.
# High threshold validates that a candidate region contains a real apex.
CORNER_CURVATURE_SMOOTHING_METHOD = "moving_average"  # "gaussian" or "moving_average"
CORNER_CURVATURE_SMOOTHING_WINDOW = 21  # must be odd and >= 3
CORNER_CURVATURE_GAUSSIAN_SIGMA_POINTS = 2.5  # smaller keeps sharper apex peaks
CORNER_APEX_THRESHOLD = 0.15
CORNER_ENTRY_EXIT_THRESHOLD = 0.03
CORNER_MIN_LENGTH_POINTS = 5
CORNER_MERGE_GAP_POINTS = 8
CORNER_APEX_MODE = "max_abs_curvature"

# Phase 11: moved corner keypoints only
MOVE_CORNER_KEYPOINTS = True
MOVED_APEX_SAFETY_MARGIN_M = 0.30
MOVED_ENTRY_EXIT_SAFETY_MARGIN_M = 0.20
MOVED_KEYPOINT_RAY_STEP_M = 0.02
MOVED_KEYPOINTS_CSV_NAME = "moved_corner_keypoints.csv"
DEBUG_MOVED_KEYPOINTS = "debug_moved_corner_keypoints.png"

# Phase 12: raceline generation from moved keypoints
# Main output: corner-aware local patch raceline.
# Comparison output: global periodic B-spline using moved keypoints as soft guide points.
GENERATE_PIECEWISE_RACELINE = True
GENERATE_GLOBAL_GUIDE_SPLINE_RACELINE = False

RACELINE_SAFETY_REGION_MARGIN_M = 0.20
RACELINE_RESAMPLE_SPACING_M = 0.03

# Offset-field raceline settings.
# The moved entrance/apex/exit keypoints are converted into signed lateral
# offset anchors on the smoothed centerline. The full closed-loop offset
# signal is interpolated, smoothed, then applied along the centerline normals.
RACELINE_OFFSET_SMOOTHING_METHOD = "gaussian"  # "gaussian" or "moving_average"
RACELINE_OFFSET_SMOOTHING_WINDOW = 81          # must be odd and >= 3
RACELINE_OFFSET_GAUSSIAN_SIGMA_POINTS = 12.0
RACELINE_OFFSET_SCALE_START = 0.85
RACELINE_OFFSET_SCALE_MIN = 0.20
RACELINE_OFFSET_SCALE_SHRINK = 0.85

# Offset-field robustness filters.
# Curvature limit prevents normal-offset curves from folding/cusping in tight turns.
# It limits inside offsets so roughly |offset * centerline_curvature| <= factor.
RACELINE_USE_CURVATURE_OFFSET_LIMIT = True
RACELINE_CURVATURE_OFFSET_LIMIT_FACTOR = 0.55
RACELINE_CURVATURE_EPS = 1e-3

# Gradient limit prevents outside->inside->outside offset transitions from changing
# too quickly over a short distance. Unit: meters lateral change per meter forward.
RACELINE_USE_OFFSET_GRADIENT_LIMIT = True
RACELINE_MAX_OFFSET_CHANGE_PER_M = 0.80
RACELINE_OFFSET_GRADIENT_LIMIT_PASSES = 3

# Optional final geometric smoothing of the generated raceline.
# This is applied after offset limiting and is validated against the safety mask again.
USE_BSPLINE_RACELINE_SMOOTHING = True
RACELINE_BSPLINE_DEGREE = 3
RACELINE_BSPLINE_SMOOTHING_FACTOR_PER_POINT = 0.00090 # defualt 0.00015
DEBUG_RACELINE_CURVATURE = "debug_raceline_curvature.png"

# Corner-aware local patch settings.
# For each corner, build a local C1 curve:
#   blend-before-entrance -> strong apex -> blend-after-exit
# then connect corner patches with straight links.
RACELINE_CORNER_BLEND_RATIO = 0.35       # fraction of local E/A/X distance used for blend length
RACELINE_CORNER_BLEND_MIN_M = 0.15
RACELINE_CORNER_BLEND_MAX_M = 0.75
RACELINE_CORNER_TANGENT_SCALE = 0.55
RACELINE_CORNER_MIN_TANGENT_SCALE = 0.08
RACELINE_CORNER_TANGENT_SHRINK = 0.70
RACELINE_CORNER_SAMPLES_PER_HALF = 35

# Optional apex softening. 1.0 = pass exactly through moved apex.
# Smaller values pull apex toward the E-X chord midpoint.
RACELINE_APEX_PULL = 1.00
RACELINE_MIN_APEX_PULL = 0.55
RACELINE_APEX_PULL_SHRINK = 0.85

# Global guide spline comparison settings.
RACELINE_GLOBAL_BSPLINE_DEGREE = 5
RACELINE_GLOBAL_BSPLINE_SMOOTHING_FACTOR_PER_POINT = 0.0050

# Final smoothed raceline export used by the ROS 2 publisher by default.
# It has the same geometry columns as centerline_points_smooth.csv:
# index,x,y,yaw,curvature,curvature_abs
RACELINE_CSV_NAME = "raceline_points_smooth.csv"

# Kept as a compatibility alias for older scripts/debug workflows that still
# look for the previous output name.
RACELINE_LEGACY_CSV_NAME = "raceline_offset_field.csv"
RACELINE_GLOBAL_CSV_NAME = "raceline_global_soft_spline.csv"
DEBUG_RACELINE_PIECEWISE = "debug_raceline_offset_field.png"
DEBUG_RACELINE_GLOBAL = "debug_raceline_global_soft_spline.png"

# =========================
# IO + PREPROCESS
# =========================
def load_map(map_path, yaml_path):
    """
    Load the map image and YAML metadata.

    The image is flipped vertically once so that pixel indexing matches
    ROS-style map coordinates:
      - x increases to the right
      - y increases upward
    """
    with open(yaml_path, "r") as f:
        yaml_data = yaml.safe_load(f)

    img = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise RuntimeError(f"Failed to load image: {map_path}")

    img_ros = np.flipud(img)
    return img_ros, yaml_data


def ensure_output_dir(output_dir):
    os.makedirs(output_dir, exist_ok=True)


# =========================
# WORLD / MAP COORD HELPERS
# =========================
def get_world_extent(img_shape, yaml_data):
    """
    Return matplotlib extent in map/world coordinates.
    """
    height, width = img_shape
    resolution = yaml_data["resolution"]
    origin = yaml_data["origin"]

    x_min = origin[0]
    x_max = origin[0] + width * resolution
    y_min = origin[1]
    y_max = origin[1] + height * resolution

    return [x_min, x_max, y_min, y_max]


def pixel_path_to_world(path_pixels, yaml_data):
    """
    Convert ordered pixel coordinates [(y, x), ...] to world coordinates [(X, Y), ...].
    """
    resolution = yaml_data["resolution"]
    origin = yaml_data["origin"]

    world_pts = []
    for y, x in path_pixels:
        X = origin[0] + x * resolution
        Y = origin[1] + y * resolution
        world_pts.append((X, Y))

    return world_pts


def compute_world_path_lengths(world_pts):
    """
    Compute Euclidean segment lengths between consecutive world points.
    """
    if len(world_pts) < 2:
        return np.array([], dtype=np.float32)

    pts = np.array(world_pts, dtype=np.float64)
    diffs = pts[1:] - pts[:-1]
    return np.linalg.norm(diffs, axis=1)


def compute_closed_loop_length(world_pts):
    """
    Sum segment lengths of a closed loop path.
    Assumes the final point already closes back to the first.
    """
    seg_lengths = compute_world_path_lengths(world_pts)
    return float(np.sum(seg_lengths)) if len(seg_lengths) > 0 else 0.0


def apply_centerline_direction(world_pts, direction):
    """
    Apply final canonical direction before computing yaw/curvature.

    normal/csv:
      keep generated order

    reverse:
      reverse point order
    """
    direction = direction.lower()
    if direction == "csv":
        direction = "normal"
    if direction not in ("normal", "reverse"):
        raise ValueError("CENTERLINE_DIRECTION must be 'normal', 'csv', or 'reverse'.")

    closed_pts = ensure_closed_loop_world(world_pts)
    core = list(closed_pts[:-1])

    if direction == "reverse":
        core.reverse()

    core.append(core[0])
    return core


def wrap_angle(angle):
    """Wrap angle to [-pi, pi]."""
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def compute_yaw_and_curvature(world_pts):
    """
    Compute yaw, signed curvature, and absolute curvature for a closed loop.

    Returns rows with:
      index, x, y, yaw, curvature, curvature_abs
    """
    closed_pts = ensure_closed_loop_world(world_pts)
    pts = np.array(closed_pts[:-1], dtype=np.float64)
    n = len(pts)

    rows = []
    for i in range(n):
        p_prev = pts[(i - 1) % n]
        p = pts[i]
        p_next = pts[(i + 1) % n]

        tangent = p_next - p_prev
        yaw = wrap_angle(np.arctan2(tangent[1], tangent[0]))

        a = p - p_prev
        b = p_next - p
        c = p_next - p_prev

        la = np.linalg.norm(a)
        lb = np.linalg.norm(b)
        lc = np.linalg.norm(c)

        if la < 1e-9 or lb < 1e-9 or lc < 1e-9:
            curvature = 0.0
        else:
            cross = a[0] * b[1] - a[1] * b[0]
            curvature = float(2.0 * cross / (la * lb * lc))

        rows.append({
            "index": i,
            "x": float(p[0]),
            "y": float(p[1]),
            "yaw": float(yaw),
            "curvature": float(curvature),
            "curvature_abs": float(abs(curvature)),
        })

    return rows


def rows_to_xy_points(rows):
    """Convert rich centerline rows back to [(x, y), ...], explicitly closed."""
    pts = [(float(row["x"]), float(row["y"])) for row in rows]
    if pts and pts[0] != pts[-1]:
        pts.append(pts[0])
    return pts


def rows_to_numpy(rows):
    """Convert rich centerline rows to [N, 6] NumPy array."""
    return np.array(
        [[
            row["index"],
            row["x"],
            row["y"],
            row["yaw"],
            row["curvature"],
            row["curvature_abs"],
        ] for row in rows],
        dtype=np.float64,
    )


# =========================
# CORE PIPELINE
# =========================
def build_free_space(img):
    """
    Classify the ROS-aligned map into free / occupied / unknown masks.

    For both PGM occupancy maps and black-line PNG maps:
      - near-white  -> free
      - near-black  -> occupied
      - mid-gray    -> unknown
    """
    free = (img >= 250).astype(np.uint8)
    occupied = (img <= 5).astype(np.uint8)
    unknown = ((img > 5) & (img < 250)).astype(np.uint8)
    return free, occupied, unknown


def touches_image_border(mask):
    """
    Return True if a binary component touches the image border.
    """
    return bool(
        np.any(mask[0, :]) or
        np.any(mask[-1, :]) or
        np.any(mask[:, 0]) or
        np.any(mask[:, -1])
    )


def count_neighbors(img):
    """
    Count 8-neighbors for each foreground pixel in a binary image.
    """
    kernel = np.array([
        [1, 1, 1],
        [1, 10, 1],
        [1, 1, 1]
    ], dtype=np.uint8)

    conv = cv2.filter2D(img, -1, kernel)
    return np.where(img > 0, conv - 10, 0)


def find_features(skeleton):
    """
    Identify endpoints and junctions in the skeleton.

    endpoint: exactly 1 neighbor
    junction: 3 or more neighbors
    """
    n = count_neighbors(skeleton)
    endpoints = ((skeleton > 0) & (n == 1)).astype(np.uint8)
    junctions = ((skeleton > 0) & (n >= 3)).astype(np.uint8)
    return endpoints, junctions


def evaluate_component_as_track(mask):
    """
    Evaluate a connected free-space component as a possible drivable corridor.
    """
    skel = skeletonize(mask.astype(bool)).astype(np.uint8)
    endpoints, junctions = find_features(skel)

    return {
        "area": int(np.sum(mask)),
        "skeleton_pixels": int(np.sum(skel)),
        "endpoint_count": int(np.sum(endpoints)),
        "junction_count": int(np.sum(junctions)),
    }


def select_drivable_region(free_mask):
    """
    Select the drivable region from the free mask.
    """
    num_labels, labels, _, _ = cv2.connectedComponentsWithStats(free_mask, connectivity=8)

    enclosed_candidates = []

    for label_id in range(1, num_labels):
        mask = (labels == label_id).astype(np.uint8)

        if touches_image_border(mask):
            continue

        metrics = evaluate_component_as_track(mask)
        metrics["label_id"] = label_id
        enclosed_candidates.append(metrics)

    if len(enclosed_candidates) == 0:
        selection_info = {
            "mode": "whole_free_mask",
            "reason": "no enclosed free-space component found"
        }
        return free_mask.copy().astype(np.uint8), selection_info

    largest_enclosed_area = max(c["area"] for c in enclosed_candidates)
    filtered_candidates = [
        c for c in enclosed_candidates
        if c["area"] >= max(MIN_COMPONENT_AREA_ABS, int(MIN_COMPONENT_AREA_RATIO * largest_enclosed_area))
    ]

    if len(filtered_candidates) == 0:
        best = max(enclosed_candidates, key=lambda c: c["area"])
        mode = "largest_enclosed_fallback"
    else:
        best = min(
            filtered_candidates,
            key=lambda c: (
                c["endpoint_count"] > 0,
                c["endpoint_count"],
                c["junction_count"],
                -c["skeleton_pixels"],
                -c["area"]
            )
        )
        mode = "filtered_enclosed_component"

    drivable_free = (labels == best["label_id"]).astype(np.uint8)

    selection_info = {
        "mode": mode,
        "label_id": best["label_id"],
        "area": best["area"],
        "skeleton_pixels": best["skeleton_pixels"],
        "endpoint_count": best["endpoint_count"],
        "junction_count": best["junction_count"],
        "num_enclosed_candidates": len(enclosed_candidates),
        "num_filtered_candidates": len(filtered_candidates),
        "largest_enclosed_area": largest_enclosed_area,
        "min_area_threshold": max(MIN_COMPONENT_AREA_ABS, int(MIN_COMPONENT_AREA_RATIO * largest_enclosed_area)),
    }

    return drivable_free, selection_info


def inflate_obstacles(binary_free, resolution, radius_m):
    """
    Inflate obstacles by shrinking free space.

    Input:
      binary_free: 1 = free, 0 = blocked

    Output:
      inflated_free: 1 = free, 0 = blocked
      radius_px: inflation radius in pixels
    """
    radius_px = max(1, int(np.ceil(radius_m / resolution)))

    blocked = (binary_free == 0).astype(np.uint8)
    kernel = np.ones((2 * radius_px + 1, 2 * radius_px + 1), dtype=np.uint8)

    blocked_inflated = cv2.dilate(blocked, kernel, iterations=1)
    inflated_free = (blocked_inflated == 0).astype(np.uint8)

    return inflated_free, radius_px


def compute_distance_map(inflated_free, resolution):
    """
    Compute Euclidean distance transform.
    """
    dist_px = cv2.distanceTransform(
        (inflated_free * 255).astype(np.uint8),
        cv2.DIST_L2,
        5
    )
    return dist_px, dist_px * resolution


def compute_skeleton(inflated_free):
    """
    Compute a one-pixel-wide skeleton from the inflated free-space corridor.
    """
    return skeletonize(inflated_free.astype(bool)).astype(np.uint8)


# =========================
# SKELETON ANALYSIS
# =========================
def remove_junctions(skeleton, junctions):
    """
    Remove junction pixels temporarily so the skeleton splits into simpler segments.
    """
    s = skeleton.copy()
    s[junctions > 0] = 0
    return s


def analyze_segments(skeleton_wo_junctions, endpoints):
    """
    Label connected components after junction removal.
    """
    labels = label(skeleton_wo_junctions > 0, connectivity=2)

    segments = []
    for i in range(1, labels.max() + 1):
        mask = (labels == i)
        segments.append({
            "label_id": i,
            "pixel_count": int(np.sum(mask)),
            "touches_endpoint": bool(np.any(mask & (endpoints > 0)))
        })

    return labels, segments


def remove_spurs(skeleton, labels, segments, dilation=1):
    """
    Remove spur segments directly from the original skeleton.
    """
    spur_mask = np.zeros_like(skeleton, dtype=np.uint8)

    for seg in segments:
        if seg["touches_endpoint"]:
            spur_mask[labels == seg["label_id"]] = 1

    if dilation > 0:
        k = np.ones((2 * dilation + 1, 2 * dilation + 1), dtype=np.uint8)
        spur_mask = cv2.dilate(spur_mask, k, iterations=1)

    result = skeleton.copy()
    result[spur_mask > 0] = 0
    return result


def keep_largest_component(binary):
    """
    Keep only the largest connected component in a binary image.
    """
    labels = label(binary > 0, connectivity=2)
    if labels.max() == 0:
        return binary

    sizes = [(labels == i).sum() for i in range(1, labels.max() + 1)]
    largest = np.argmax(sizes) + 1
    return (labels == largest).astype(np.uint8)


# =========================
# PHASE 6: ORDERED LOOP TRAVERSAL
# =========================
def get_8_neighbors(y, x, height, width):
    neighbors = []
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            ny, nx = y + dy, x + dx
            if 0 <= ny < height and 0 <= nx < width:
                neighbors.append((ny, nx))
    return neighbors


def get_foreground_neighbors(binary_img, y, x):
    """
    Return foreground 8-neighbors of pixel (y, x).
    """
    h, w = binary_img.shape
    return [(ny, nx) for ny, nx in get_8_neighbors(y, x, h, w) if binary_img[ny, nx] > 0]


def choose_start_pixel(loop_img):
    """
    Deterministic start pixel:
    choose the top-most, then left-most loop pixel.
    """
    ys, xs = np.where(loop_img > 0)
    if len(xs) == 0:
        raise RuntimeError("Main loop is empty; cannot traverse.")

    candidates = sorted(zip(ys, xs), key=lambda p: (-p[0], p[1]))
    return candidates[0]


def trace_ordered_loop(loop_img):
    """
    Traverse a one-pixel-wide closed loop in order.
    Returns:
      ordered_pixels: list of (y, x) tuples
    """
    ys, xs = np.where(loop_img > 0)
    total_pixels = len(xs)
    if total_pixels == 0:
        raise RuntimeError("Loop image has no foreground pixels.")

    start = choose_start_pixel(loop_img)
    start_neighbors = get_foreground_neighbors(loop_img, start[0], start[1])

    if len(start_neighbors) < 2:
        raise RuntimeError(
            f"Start pixel {start} has fewer than 2 neighbors; loop is not properly closed."
        )

    current = start_neighbors[0]
    previous = start

    ordered = [start, current]
    visited = {start, current}

    max_steps = total_pixels * 3
    for _ in range(max_steps):
        nbrs = get_foreground_neighbors(loop_img, current[0], current[1])
        candidates = [p for p in nbrs if p != previous]

        if start in candidates and len(visited) >= total_pixels - 2:
            ordered.append(start)
            return ordered

        if len(candidates) == 0:
            break

        if len(candidates) == 1:
            nxt = candidates[0]
        else:
            vy = current[0] - previous[0]
            vx = current[1] - previous[1]

            best_score = None
            nxt = None
            for cand in candidates:
                cy = cand[0] - current[0]
                cx = cand[1] - current[1]

                score = vy * cy + vx * cx
                visited_penalty = 1000 if cand in visited else 0
                final_score = (visited_penalty, -score, cand[0], cand[1])

                if best_score is None or final_score < best_score:
                    best_score = final_score
                    nxt = cand

        previous, current = current, nxt

        if current == start:
            ordered.append(start)
            return ordered

        ordered.append(current)
        visited.add(current)

        if len(visited) >= total_pixels - 1:
            current_neighbors = get_foreground_neighbors(loop_img, current[0], current[1])
            if start in current_neighbors:
                ordered.append(start)
                return ordered

    raise RuntimeError(
        f"Loop traversal failed to close. Visited {len(visited)} / {total_pixels} pixels."
    )


def compute_path_step_lengths(path_pixels):
    """
    Return Euclidean step lengths between consecutive path pixels, in pixel units.
    """
    if len(path_pixels) < 2:
        return np.array([], dtype=np.float32)

    pts = np.array(path_pixels, dtype=np.float64)
    diffs = pts[1:] - pts[:-1]
    return np.linalg.norm(diffs, axis=1)


# =========================
# VALIDATION
# =========================
def count_connected_components(binary_img):
    """
    Count connected foreground components in a binary image.
    """
    labels = label(binary_img > 0, connectivity=2)
    return int(labels.max())


def validate_main_loop(loop_img):
    """
    Validate structural assumptions before traversal.
    """
    endpoints, junctions = find_features(loop_img)

    metrics = {
        "connected_components": count_connected_components(loop_img),
        "endpoint_count": int(np.sum(endpoints)),
        "junction_count": int(np.sum(junctions)),
        "loop_pixels": int(np.sum(loop_img)),
    }

    return metrics


def validate_ordered_loop(loop_img, ordered_loop, resolution):
    """
    Validate traversal result after ordering.
    """
    unique_points = len(set(ordered_loop))
    loop_pixels = int(np.sum(loop_img))

    step_lengths_px = compute_path_step_lengths(ordered_loop)
    step_lengths_m = step_lengths_px * resolution

    closed = False
    if len(ordered_loop) >= 2:
        closed = (ordered_loop[0] == ordered_loop[-1])

    metrics = {
        "ordered_points": len(ordered_loop),
        "unique_ordered_points": unique_points,
        "loop_pixels": loop_pixels,
        "coverage_ratio": (unique_points / loop_pixels) if loop_pixels > 0 else 0.0,
        "closed": closed,
        "min_step_m": float(step_lengths_m.min()) if len(step_lengths_m) > 0 else 0.0,
        "mean_step_m": float(step_lengths_m.mean()) if len(step_lengths_m) > 0 else 0.0,
        "max_step_m": float(step_lengths_m.max()) if len(step_lengths_m) > 0 else 0.0,
    }

    return metrics


# =========================
# PHASE 7: WORLD POINT EXPORT
# =========================
def save_centerline_csv(world_pts, csv_path):
    """
    Save ordered centerline points to CSV.

    Columns:
      index, x, y
    """
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "x", "y"])
        for i, (x, y) in enumerate(world_pts):
            writer.writerow([i, x, y])


# =========================
# PHASE 8: RESAMPLING + SMOOTHING
# =========================
def ensure_closed_loop_world(world_pts):
    """
    Ensure the world-point path is explicitly closed.
    """
    if len(world_pts) == 0:
        raise RuntimeError("World point list is empty.")

    if world_pts[0] != world_pts[-1]:
        world_pts = list(world_pts) + [world_pts[0]]

    return world_pts


def cumulative_arc_lengths(world_pts):
    """
    Compute cumulative arc length along a closed loop path.
    Assumes the final point already closes back to the first.
    """
    pts = np.array(world_pts, dtype=np.float64)
    diffs = pts[1:] - pts[:-1]
    seg_lengths = np.linalg.norm(diffs, axis=1)

    s = np.zeros(len(pts), dtype=np.float64)
    s[1:] = np.cumsum(seg_lengths)
    return s, seg_lengths


def resample_closed_loop(world_pts, spacing_m):
    """
    Resample a closed loop at approximately constant arc-length spacing.
    """
    if spacing_m <= 0:
        raise ValueError("spacing_m must be positive.")

    world_pts = ensure_closed_loop_world(world_pts)
    pts = np.array(world_pts, dtype=np.float64)

    s, _ = cumulative_arc_lengths(pts)
    total_length = s[-1]

    if total_length <= 1e-9:
        raise RuntimeError("Closed loop length is zero; cannot resample.")

    num_samples = max(4, int(np.round(total_length / spacing_m)))
    sample_s = np.linspace(0.0, total_length, num_samples, endpoint=False)

    x = pts[:, 0]
    y = pts[:, 1]

    resampled_x = np.interp(sample_s, s, x)
    resampled_y = np.interp(sample_s, s, y)

    resampled_pts = list(zip(resampled_x, resampled_y))
    resampled_pts.append(resampled_pts[0])

    return resampled_pts


def circular_moving_average(values, window_size):
    """
    Circular moving average for 1D closed-loop data.

    window_size must be odd.
    """
    if window_size < 3 or window_size % 2 == 0:
        raise ValueError("window_size must be odd and >= 3.")

    n = len(values)
    half = window_size // 2
    out = np.zeros(n, dtype=np.float64)

    for i in range(n):
        acc = 0.0
        for k in range(-half, half + 1):
            acc += values[(i + k) % n]
        out[i] = acc / window_size

    return out



def circular_gaussian_smooth(values, window_size, sigma_points):
    """
    Circular Gaussian smoothing for 1D closed-loop data.

    Compared with a box/moving average, this keeps the local apex peak better
    because samples near the target index get higher weight than distant samples.
    """
    if window_size < 3 or window_size % 2 == 0:
        raise ValueError("window_size must be odd and >= 3.")
    if sigma_points <= 0:
        raise ValueError("sigma_points must be positive.")

    values = np.asarray(values, dtype=np.float64)
    n = len(values)
    half = window_size // 2

    offsets = np.arange(-half, half + 1, dtype=np.float64)
    weights = np.exp(-0.5 * (offsets / float(sigma_points)) ** 2)
    weights = weights / np.sum(weights)

    out = np.zeros(n, dtype=np.float64)
    for i in range(n):
        acc = 0.0
        for offset, weight in zip(range(-half, half + 1), weights):
            acc += float(weight) * values[(i + offset) % n]
        out[i] = acc

    return out


def smooth_curvature_for_corner_detection(curv):
    """
    Selectable curvature-signal smoothing for Phase 10.

    Keep the geometric centerline unchanged; only filter the 1D curvature signal.
    """
    if CORNER_CURVATURE_SMOOTHING_METHOD == "moving_average":
        return circular_moving_average(curv, CORNER_CURVATURE_SMOOTHING_WINDOW)
    if CORNER_CURVATURE_SMOOTHING_METHOD == "gaussian":
        return circular_gaussian_smooth(
            curv,
            CORNER_CURVATURE_SMOOTHING_WINDOW,
            CORNER_CURVATURE_GAUSSIAN_SIGMA_POINTS
        )
    raise ValueError("CORNER_CURVATURE_SMOOTHING_METHOD must be 'moving_average' or 'gaussian'.")


def smooth_closed_loop(world_pts, window_size=9, passes=2):
    """
    Smooth a closed loop using circular moving average with wrap-around.
    """
    if passes < 0:
        raise ValueError("passes must be >= 0")

    world_pts = ensure_closed_loop_world(world_pts)

    core_pts = np.array(world_pts[:-1], dtype=np.float64)
    x = core_pts[:, 0]
    y = core_pts[:, 1]

    for _ in range(passes):
        x = circular_moving_average(x, window_size)
        y = circular_moving_average(y, window_size)

    smoothed_pts = list(zip(x, y))
    smoothed_pts.append(smoothed_pts[0])

    return smoothed_pts



def smooth_closed_loop_bspline(world_pts, spacing_m, smoothing_factor_per_point=0.0004, degree=3):
    """
    Smooth a closed loop using a periodic cubic B-spline and resample it.

    Difference from smooth_closed_loop():
      - smooth_closed_loop() averages x/y point coordinates directly.
      - this function fits a continuous periodic spline curve through/near the points,
        then samples that curve back into discrete points.

    The spline is periodic (per=True), so the loop closes smoothly.
    The smoothing factor is scaled by the number of points:
        s = smoothing_factor_per_point * N

    Rule of thumb:
      smoothing_factor_per_point = 0.0001  -> about 1 cm RMS allowed fitting error
      smoothing_factor_per_point = 0.0004  -> about 2 cm RMS allowed fitting error
      smoothing_factor_per_point = 0.0025  -> about 5 cm RMS allowed fitting error
    """
    if spacing_m <= 0:
        raise ValueError("spacing_m must be positive.")

    closed_pts = ensure_closed_loop_world(world_pts)
    pts = np.array(closed_pts[:-1], dtype=np.float64)

    # Remove consecutive duplicate points, because splprep may fail with repeated samples.
    if len(pts) >= 2:
        keep = [0]
        for i in range(1, len(pts)):
            if np.linalg.norm(pts[i] - pts[keep[-1]]) > 1e-9:
                keep.append(i)
        pts = pts[keep]

    k = min(int(degree), len(pts) - 1)
    if len(pts) < 4 or k < 2:
        # Fallback to existing moving-average smoothing for very short loops.
        return smooth_closed_loop(world_pts, window_size=SMOOTHING_WINDOW, passes=SMOOTHING_PASSES)

    s_value = max(0.0, float(smoothing_factor_per_point) * len(pts))

    # Fit periodic parametric spline: x(u), y(u), u in [0, 1].
    tck, _ = splprep(
        [pts[:, 0], pts[:, 1]],
        s=s_value,
        per=True,
        k=k
    )

    # Estimate spline length with dense sampling, then resample at requested spacing.
    dense_count = max(1000, len(pts) * 4)
    u_dense = np.linspace(0.0, 1.0, dense_count, endpoint=False)
    x_dense, y_dense = splev(u_dense, tck)
    dense_pts = np.column_stack([x_dense, y_dense])

    dense_closed = np.vstack([dense_pts, dense_pts[0]])
    dense_len = float(np.sum(np.linalg.norm(dense_closed[1:] - dense_closed[:-1], axis=1)))

    if dense_len <= 1e-9:
        raise RuntimeError("B-spline loop length is zero; cannot resample.")

    num_samples = max(4, int(np.round(dense_len / spacing_m)))
    u_sample = np.linspace(0.0, 1.0, num_samples, endpoint=False)
    x_sample, y_sample = splev(u_sample, tck)

    smoothed_pts = list(zip(np.asarray(x_sample, dtype=np.float64), np.asarray(y_sample, dtype=np.float64)))
    smoothed_pts.append(smoothed_pts[0])
    return smoothed_pts


def validate_smoothed_loop(world_pts, resampled_pts, smoothed_pts):
    """
    Print basic diagnostics for Phase 8 outputs.
    """
    raw_length = compute_closed_loop_length(world_pts)
    resampled_length = compute_closed_loop_length(resampled_pts)
    smoothed_length = compute_closed_loop_length(smoothed_pts)

    resampled_seg = compute_world_path_lengths(resampled_pts)
    smoothed_seg = compute_world_path_lengths(smoothed_pts)

    metrics = {
        "raw_length_m": raw_length,
        "resampled_length_m": resampled_length,
        "smoothed_length_m": smoothed_length,
        "resampled_points": len(resampled_pts),
        "smoothed_points": len(smoothed_pts),
        "resampled_step_min_m": float(resampled_seg.min()) if len(resampled_seg) > 0 else 0.0,
        "resampled_step_mean_m": float(resampled_seg.mean()) if len(resampled_seg) > 0 else 0.0,
        "resampled_step_max_m": float(resampled_seg.max()) if len(resampled_seg) > 0 else 0.0,
        "smoothed_step_min_m": float(smoothed_seg.min()) if len(smoothed_seg) > 0 else 0.0,
        "smoothed_step_mean_m": float(smoothed_seg.mean()) if len(smoothed_seg) > 0 else 0.0,
        "smoothed_step_max_m": float(smoothed_seg.max()) if len(smoothed_seg) > 0 else 0.0,
    }

    return metrics


def save_world_centerline_csv(world_pts, csv_path):
    """
    Save ordered world-coordinate centerline points to CSV.

    Columns:
      index, x, y
    """
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "x", "y"])
        for i, (x, y) in enumerate(world_pts):
            writer.writerow([i, x, y])


def save_centerline_geometry_csv(rows, csv_path):
    """
    Save final centerline with geometry fields.

    Columns:
      index, x, y, yaw, curvature, curvature_abs
    """
    fieldnames = ["index", "x", "y", "yaw", "curvature", "curvature_abs"]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


# =========================
# PHASE 9: FINAL EXPORT PACKAGE
# =========================
def save_centerline_npy(centerline_rows, npy_path):
    """
    Save final centerline rows to NumPy binary format.
    Shape: [N, 6] with columns:
      index, x, y, yaw, curvature, curvature_abs
    """
    np.save(npy_path, rows_to_numpy(centerline_rows))


def save_metadata_yaml(
    metadata_path,
    map_path,
    yaml_path,
    map_meta,
    region_info,
    traversal_metrics,
    raw_loop,
    smooth_loop,
    centerline_rows
):
    """
    Save pipeline metadata for reproducibility and later ROS integration.
    """
    raw_len = compute_closed_loop_length(raw_loop)
    smooth_len = compute_closed_loop_length(smooth_loop)

    metadata = {
        "source_files": {
            "map_image": map_path,
            "map_yaml": yaml_path,
        },
        "map_metadata": {
            "resolution": float(map_meta["resolution"]),
            "origin": [float(v) for v in map_meta["origin"]],
        },
        "pipeline_parameters": {
            "inflation_radius_m": float(INFLATION_RADIUS_M),
            "spur_dilation_radius": int(SPUR_DILATION_RADIUS),
            "resample_spacing_m": float(RESAMPLE_SPACING_M),
            "smoothing_window": int(SMOOTHING_WINDOW),
            "smoothing_passes": int(SMOOTHING_PASSES),
            "use_bspline_centerline_smoothing": bool(USE_BSPLINE_CENTERLINE_SMOOTHING),
            "bspline_degree": int(BSPLINE_DEGREE),
            "bspline_smoothing_factor_per_point": float(BSPLINE_SMOOTHING_FACTOR_PER_POINT),
            "centerline_direction": str(CENTERLINE_DIRECTION),
            "geometry_columns": ["index", "x", "y", "yaw", "curvature", "curvature_abs"],
            "min_component_area_abs": int(MIN_COMPONENT_AREA_ABS),
            "min_component_area_ratio": float(MIN_COMPONENT_AREA_RATIO),
            "minimum_ordered_loop_coverage_ratio": float(
                MIN_ORDERED_LOOP_COVERAGE_RATIO
            ),
        },
        "drivable_region_selection": region_info,
        "ordered_loop_validation": traversal_metrics,
        "outputs": {
            "raw_csv": RAW_CSV_NAME,
            "smooth_csv": SMOOTH_CSV_NAME,
            "smooth_npy": SMOOTH_NPY_NAME,
            "raw_point_count": int(len(raw_loop)),
            "smooth_point_count": int(len(centerline_rows)),
            "raw_loop_length_m": float(raw_len),
            "smooth_loop_length_m": float(smooth_len),
        }
    }

    with open(metadata_path, "w") as f:
        yaml.safe_dump(metadata, f, sort_keys=False)



# =========================
# PHASE 10: CORNER KEY-POINT DETECTION
# =========================
def circular_index_distance(i, j, n):
    """
    Forward circular distance in index steps from i to j on [0, n).
    """
    return int((j - i) % n)


def circular_segment_indices(start, end, n):
    """
    Return indices from start to end inclusive on a circular array.
    """
    if n <= 0:
        return []
    if start <= end:
        return list(range(start, end + 1))
    return list(range(start, n)) + list(range(0, end + 1))


def find_true_segments_circular(mask):
    """
    Convert a circular boolean mask into continuous true segments.

    Returns a list of (start_index, end_index), inclusive.
    Handles the case where a corner segment crosses index 0.
    """
    mask = np.asarray(mask, dtype=bool)
    n = len(mask)
    if n == 0 or not np.any(mask):
        return []
    if np.all(mask):
        return [(0, n - 1)]

    # Rotate so index 0 is guaranteed to be in a False region.
    false_indices = np.where(~mask)[0]
    start_scan = int((false_indices[0] + 1) % n)

    segments = []
    in_seg = False
    seg_start = None
    prev_idx = None

    for step in range(n):
        idx = (start_scan + step) % n
        if mask[idx] and not in_seg:
            in_seg = True
            seg_start = idx
        elif (not mask[idx]) and in_seg:
            segments.append((seg_start, prev_idx))
            in_seg = False
        prev_idx = idx

    if in_seg:
        segments.append((seg_start, prev_idx))

    return segments


def merge_short_gaps_circular(segments, n, max_gap_points):
    """
    Merge neighboring corner segments if the straight gap between them is short.
    This prevents one real corner being split by a tiny curvature dip.
    """
    if len(segments) <= 1:
        return segments

    segments = sorted(segments, key=lambda ab: ab[0])
    changed = True

    while changed and len(segments) > 1:
        changed = False
        merged = []
        used = [False] * len(segments)

        i = 0
        while i < len(segments):
            if used[i]:
                i += 1
                continue

            start_i, end_i = segments[i]
            j = (i + 1) % len(segments)
            start_j, end_j = segments[j]

            if i == len(segments) - 1:
                # Circular gap from last segment end to first segment start.
                first_start, first_end = segments[0]
                gap = circular_index_distance(end_i, first_start, n) - 1
                if gap <= max_gap_points:
                    # Merge last and first into a wrap-around segment.
                    merged.append((start_i, first_end))
                    used[i] = True
                    used[0] = True
                    changed = True
                else:
                    merged.append((start_i, end_i))
                    used[i] = True
            else:
                gap = start_j - end_i - 1
                if gap <= max_gap_points:
                    merged.append((start_i, end_j))
                    used[i] = True
                    used[j] = True
                    changed = True
                    i += 1
                else:
                    merged.append((start_i, end_i))
                    used[i] = True
            i += 1

        segments = sorted(merged, key=lambda ab: ab[0])

    return segments


def detect_corner_keypoints(centerline_rows):
    """
    Detect corner entrance, apex, and exit from smoothed centerline curvature.

    Definition used here:
      entrance: first index where abs(smoothed_curvature) exceeds threshold
      apex:     index with maximum abs(smoothed_curvature) inside the corner segment
      exit:     last index where abs(smoothed_curvature) exceeds threshold

    This detects geometry key points only. It does not generate a raceline yet.
    """
    if CORNER_CURVATURE_SMOOTHING_WINDOW < 3 or CORNER_CURVATURE_SMOOTHING_WINDOW % 2 == 0:
        raise RuntimeError("CORNER_CURVATURE_SMOOTHING_WINDOW must be odd and >= 3.")

    curv = np.array([float(r["curvature"]) for r in centerline_rows], dtype=np.float64)
    n = len(curv)
    curv_smooth = smooth_curvature_for_corner_detection(curv)
    abs_smooth = np.abs(curv_smooth)

    # Low threshold defines entrance/exit candidate region.
    corner_mask = abs_smooth >= CORNER_ENTRY_EXIT_THRESHOLD

    segments = find_true_segments_circular(corner_mask)
    segments = merge_short_gaps_circular(segments, n, CORNER_MERGE_GAP_POINTS)

    corners = []
    for seg_id, (start, end) in enumerate(segments):
        idxs = circular_segment_indices(start, end, n)
        if len(idxs) < CORNER_MIN_LENGTH_POINTS:
            continue

        idxs_np = np.array(idxs, dtype=int)
        local_abs = abs_smooth[idxs_np]
        apex_idx = int(idxs_np[int(np.argmax(local_abs))])
        # Reject weak bends: a valid corner must contain a strong enough apex.
        if float(abs_smooth[apex_idx]) < CORNER_APEX_THRESHOLD:
            continue

        entrance = int(start)
        exit_idx = int(end)

        sign = 1 if curv_smooth[apex_idx] > 0 else -1
        direction = "left" if sign > 0 else "right"

        # Arc-length approximation. The centerline is resampled at near-constant spacing.
        approx_length_m = float(len(idxs) * RESAMPLE_SPACING_M)

        corners.append({
            "corner_id": len(corners),
            "turn_direction": direction,
            "turn_sign": int(sign),
            "entrance_index": entrance,
            "apex_index": apex_idx,
            "exit_index": exit_idx,
            "num_points": int(len(idxs)),
            "approx_length_m": approx_length_m,
            "apex_curvature": float(curv_smooth[apex_idx]),
            "apex_curvature_abs": float(abs_smooth[apex_idx]),
            "entrance_x": float(centerline_rows[entrance]["x"]),
            "entrance_y": float(centerline_rows[entrance]["y"]),
            "apex_x": float(centerline_rows[apex_idx]["x"]),
            "apex_y": float(centerline_rows[apex_idx]["y"]),
            "exit_x": float(centerline_rows[exit_idx]["x"]),
            "exit_y": float(centerline_rows[exit_idx]["y"]),
        })

    return corners, curv, curv_smooth, corner_mask


def save_corner_keypoints_csv(corners, csv_path):
    fieldnames = [
        "corner_id", "turn_direction", "turn_sign",
        "entrance_index", "apex_index", "exit_index",
        "num_points", "approx_length_m",
        "apex_curvature", "apex_curvature_abs",
        "entrance_x", "entrance_y", "apex_x", "apex_y", "exit_x", "exit_y",
    ]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for c in corners:
            writer.writerow(c)


def overlay_corner_keypoints(img, yaml_data, centerline_rows, corners, path):
    if not DEBUG:
        return

    extent = get_world_extent(img.shape, yaml_data)
    base = img.astype(np.float32) / 255.0
    cpts = np.array([[r["x"], r["y"]] for r in centerline_rows], dtype=np.float64)

    plt.figure(figsize=(9, 7))
    plt.imshow(base, cmap="gray", origin="lower", extent=extent)
    plt.plot(cpts[:, 0], cpts[:, 1], linewidth=1.0, label="centerline")

    for c in corners:
        ex, ey = c["entrance_x"], c["entrance_y"]
        ax, ay = c["apex_x"], c["apex_y"]
        xx, xy = c["exit_x"], c["exit_y"]
        cid = c["corner_id"]
        direction = c["turn_direction"]

        plt.scatter([ex], [ey], s=40, marker="^", label="entrance" if cid == 0 else None)
        plt.scatter([ax], [ay], s=60, marker="*", label="apex" if cid == 0 else None)
        plt.scatter([xx], [xy], s=40, marker="s", label="exit" if cid == 0 else None)
        plt.text(ax, ay, f"C{cid} {direction}", fontsize=8)

    plt.title("Detected Corner Key Points")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.axis("equal")
    plt.legend()
    plt.savefig(path, dpi=140)
    plt.close()


def plot_corner_curvature(curv_raw, curv_smooth, corner_mask, corners, path):
    if not DEBUG:
        return

    plt.figure(figsize=(11, 4))
    plt.plot(curv_raw, linewidth=0.8, label="raw centerline curvature")
    plt.plot(curv_smooth, linewidth=1.4, label="smoothed curvature for corner detection")
    plt.axhline(CORNER_ENTRY_EXIT_THRESHOLD, linewidth=0.8, linestyle="--", label="entry/exit threshold")
    plt.axhline(-CORNER_ENTRY_EXIT_THRESHOLD, linewidth=0.8, linestyle="--")
    plt.axhline(CORNER_APEX_THRESHOLD, linewidth=0.8, linestyle=":", label="apex threshold")
    plt.axhline(-CORNER_APEX_THRESHOLD, linewidth=0.8, linestyle=":")
    plt.axhline(0.0, linewidth=0.8)

    # Use both raw and smoothed curvature for the visible range so raw spikes
    # are not clipped. Use robust percentiles to avoid one extreme artifact making
    # the plot unreadable.
    all_curv_for_ylim = np.concatenate([
        np.asarray(curv_raw, dtype=np.float64),
        np.asarray(curv_smooth, dtype=np.float64)
    ])
    y_abs = max(
        float(np.percentile(np.abs(all_curv_for_ylim), 99.5)),
        float(CORNER_APEX_THRESHOLD),
        float(CORNER_ENTRY_EXIT_THRESHOLD),
        1e-6
    )
    y_abs *= 1.15
    plt.ylim(-y_abs, y_abs)


    # Lightly shade detected corner regions.
    n = len(curv_smooth)
    for c in corners:
        start = c["entrance_index"]
        end = c["exit_index"]
        if start <= end:
            plt.axvspan(start, end, alpha=0.15)
        else:
            plt.axvspan(start, n - 1, alpha=0.15)
            plt.axvspan(0, end, alpha=0.15)
        plt.axvline(c["apex_index"], linewidth=0.8, linestyle=":")
        plt.text(c["apex_index"], 0.92 * y_abs, f"C{c['corner_id']}", fontsize=8, ha="center")

    plt.title("Corner Detection from Centerline Curvature")
    plt.xlabel("Centerline index")
    plt.ylabel("Curvature (1/m)")
    plt.legend(loc="best")
    plt.tight_layout()
    plt.savefig(path, dpi=140)
    plt.close()

def world_to_pixel(x, y, yaml_data):
    res = float(yaml_data["resolution"])
    origin = yaml_data["origin"]
    px = int(round((float(x) - origin[0]) / res))
    py = int(round((float(y) - origin[1]) / res))
    return py, px


def is_world_point_in_mask(x, y, mask, yaml_data):
    py, px = world_to_pixel(x, y, yaml_data)
    h, w = mask.shape
    if py < 0 or py >= h or px < 0 or px >= w:
        return False
    return bool(mask[py, px] > 0)


def validate_points_in_mask(points, mask, yaml_data):
    """
    Validate a list of world-coordinate points against a binary mask.

    Returns:
      inside_ratio: fraction of points inside the mask
      outside_count: number of points outside the mask
    """
    if len(points) == 0:
        return 0.0, 0

    flags = [is_world_point_in_mask(x, y, mask, yaml_data) for x, y in points]
    outside_count = int(np.sum(np.logical_not(flags)))
    inside_ratio = float(np.mean(flags))
    return inside_ratio, outside_count


def raycast_available_lateral_distance(x, y, yaw, lateral_sign, drivable_mask, yaml_data, step_m):
    nx = -np.sin(float(yaw)) * float(lateral_sign)
    ny =  np.cos(float(yaw)) * float(lateral_sign)

    dist = 0.0
    last_valid = 0.0
    max_dist = 20.0

    while dist <= max_dist:
        tx = float(x) + nx * dist
        ty = float(y) + ny * dist

        if not is_world_point_in_mask(tx, ty, drivable_mask, yaml_data):
            break

        last_valid = dist
        dist += step_m

    return float(last_valid)


def move_corner_keypoint_to_safe_side(centerline_rows, index, lateral_sign, safety_margin_m, drivable_mask, yaml_data):
    row = centerline_rows[int(index)]
    x = float(row["x"])
    y = float(row["y"])
    yaw = float(row["yaw"])

    available = raycast_available_lateral_distance(
        x, y, yaw,
        lateral_sign,
        drivable_mask,
        yaml_data,
        MOVED_KEYPOINT_RAY_STEP_M
    )

    move_dist = max(0.0, available - float(safety_margin_m))

    nx = -np.sin(yaw) * float(lateral_sign)
    ny =  np.cos(yaw) * float(lateral_sign)

    return {
        "x": float(x + nx * move_dist),
        "y": float(y + ny * move_dist),
        "center_x": x,
        "center_y": y,
        "center_yaw": yaw,
        "center_index": int(index),
        "lateral_sign": int(lateral_sign),
        "available_to_limit_m": float(available),
        "safety_margin_m": float(safety_margin_m),
        "move_dist_m": float(move_dist),
    }


def build_moved_corner_keypoints(centerline_rows, corners, drivable_mask, yaml_data):
    keypoints = []

    for c in corners:
        cid = int(c["corner_id"])
        turn_sign = int(c["turn_sign"])

        inside_sign = turn_sign
        outside_sign = -turn_sign

        specs = [
            ("entrance_outside", int(c["entrance_index"]), outside_sign, MOVED_ENTRY_EXIT_SAFETY_MARGIN_M),
            ("apex_inside",      int(c["apex_index"]),     inside_sign,  MOVED_APEX_SAFETY_MARGIN_M),
            ("exit_outside",     int(c["exit_index"]),     outside_sign, MOVED_ENTRY_EXIT_SAFETY_MARGIN_M),
        ]

        for role, idx, lateral_sign, safety_margin_m in specs:
            moved = move_corner_keypoint_to_safe_side(
                centerline_rows,
                idx,
                lateral_sign,
                safety_margin_m,
                drivable_mask,
                yaml_data
            )
            moved.update({
                "moved_keypoint_id": len(keypoints),
                "corner_id": cid,
                "role": role,
                "turn_direction": c["turn_direction"],
                "turn_sign": turn_sign,
            })
            keypoints.append(moved)

    return keypoints


def save_moved_corner_keypoints_csv(keypoints, csv_path):
    fieldnames = [
        "moved_keypoint_id", "corner_id", "role", "turn_direction", "turn_sign",
        "center_index", "x", "y", "center_x", "center_y", "center_yaw",
        "lateral_sign", "available_to_limit_m", "safety_margin_m", "move_dist_m",
    ]

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for kp in keypoints:
            writer.writerow(kp)


def overlay_moved_corner_keypoints(img, yaml_data, centerline_rows, keypoints, path):
    if not DEBUG:
        return

    extent = get_world_extent(img.shape, yaml_data)
    base = img.astype(np.float32) / 255.0
    cpts = np.array([[r["x"], r["y"]] for r in centerline_rows], dtype=np.float64)

    plt.figure(figsize=(9, 7))
    plt.imshow(base, cmap="gray", origin="lower", extent=extent)
    plt.plot(cpts[:, 0], cpts[:, 1], linewidth=1.0, label="centerline")

    marker_by_role = {
        "entrance_outside": "^",
        "apex_inside": "*",
        "exit_outside": "s",
    }

    used = set()
    for kp in keypoints:
        role = kp["role"]
        label = role if role not in used else None
        used.add(role)

        plt.scatter(
            [kp["x"]], [kp["y"]],
            s=90 if role == "apex_inside" else 50,
            marker=marker_by_role[role],
            label=label
        )

        plt.plot(
            [kp["center_x"], kp["x"]],
            [kp["center_y"], kp["y"]],
            linewidth=0.7,
            linestyle=":"
        )

        plt.text(kp["x"], kp["y"], f"C{kp['corner_id']}", fontsize=8)

    plt.title("Moved Corner Keypoints: Entrance/Exit Outside, Apex Inside")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.axis("equal")
    plt.legend()
    plt.savefig(path, dpi=140)
    plt.close()





def build_safety_region_mask(drivable_mask, resolution, safety_margin_m):
    """
    Shrink the selected drivable corridor by safety_margin_m.

    Output mask: 1 means a raceline point is at least safety_margin_m away
    from the edge of the drivable corridor.
    """
    radius_px = max(1, int(np.ceil(float(safety_margin_m) / float(resolution))))
    kernel = np.ones((2 * radius_px + 1, 2 * radius_px + 1), dtype=np.uint8)
    safety_mask = cv2.erode(drivable_mask.astype(np.uint8), kernel, iterations=1)
    return safety_mask.astype(np.uint8), radius_px


def order_moved_keypoints_for_raceline(moved_keypoints):
    """
    Build the reference sequence:
      entrance_0 -> apex_0 -> exit_0 -> entrance_1 -> apex_1 -> exit_1 -> ...
    """
    role_order = {
        "entrance_outside": 0,
        "apex_inside": 1,
        "exit_outside": 2,
    }
    return sorted(
        moved_keypoints,
        key=lambda kp: (int(kp["corner_id"]), role_order.get(str(kp["role"]), 99))
    )


def _centerline_xy_yaw_arrays(centerline_rows):
    pts = np.array([[float(r["x"]), float(r["y"])] for r in centerline_rows], dtype=np.float64)
    yaw = np.array([float(r["yaw"]) for r in centerline_rows], dtype=np.float64)
    return pts, yaw


def _centerline_normals_from_yaw(yaw):
    """
    Left normal of the centerline tangent.
    A signed lateral offset d generates p_race = p_center + d * normal.
    """
    yaw = np.asarray(yaw, dtype=np.float64)
    return np.column_stack([-np.sin(yaw), np.cos(yaw)])


def _anchor_offset_from_moved_keypoint(centerline_rows, kp):
    """
    Convert one moved keypoint into a signed lateral offset anchor.

    The moved keypoint still stores its original centerline index. We project
    the vector centerline_point -> moved_point onto that centerline point's
    local normal, giving a signed offset in meters.
    """
    idx = int(kp["center_index"])
    row = centerline_rows[idx]
    c = np.array([float(row["x"]), float(row["y"])], dtype=np.float64)
    m = np.array([float(kp["x"]), float(kp["y"])], dtype=np.float64)
    yaw = float(row["yaw"])
    normal = np.array([-np.sin(yaw), np.cos(yaw)], dtype=np.float64)
    offset = float(np.dot(m - c, normal))
    return {
        "index": idx,
        "offset": offset,
        "role": str(kp.get("role", "unknown")),
        "corner_id": int(kp.get("corner_id", -1)),
        "turn_direction": str(kp.get("turn_direction", "unknown")),
        "turn_sign": int(kp.get("turn_sign", 0)),
        "source_kp": kp,
    }


def build_offset_anchors_from_moved_keypoints(centerline_rows, moved_keypoints):
    anchors = [_anchor_offset_from_moved_keypoint(centerline_rows, kp) for kp in moved_keypoints]

    # Multiple anchors can theoretically map to the same centerline index.
    # Keep one averaged anchor per index to avoid zero-length interpolation segments.
    by_index = {}
    for a in anchors:
        by_index.setdefault(int(a["index"]), []).append(a)

    merged = []
    for idx in sorted(by_index.keys()):
        group = by_index[idx]
        base = dict(group[0])
        base["offset"] = float(np.mean([g["offset"] for g in group]))
        base["role"] = "+".join(g["role"] for g in group)
        merged.append(base)

    return merged


def _smoothstep(t):
    t = np.clip(np.asarray(t, dtype=np.float64), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def interpolate_closed_offset_signal(n, anchors):
    """
    Interpolate a closed-loop offset signal from sparse offset anchors.

    Anchors are sorted by centerline index. Between every pair of neighboring
    anchors, offset is smoothly interpolated with smoothstep. This fills the
    whole centerline, including long non-corner sections, so they follow the
    centerline backbone instead of becoming straight geometric chords.
    """
    if n <= 0:
        return np.array([], dtype=np.float64)
    if len(anchors) == 0:
        return np.zeros(n, dtype=np.float64)
    if len(anchors) == 1:
        return np.full(n, float(anchors[0]["offset"]), dtype=np.float64)

    anchors = sorted(anchors, key=lambda a: int(a["index"]))
    out = np.zeros(n, dtype=np.float64)

    for i, a0 in enumerate(anchors):
        a1 = anchors[(i + 1) % len(anchors)]
        idx0 = int(a0["index"])
        idx1 = int(a1["index"])
        off0 = float(a0["offset"])
        off1 = float(a1["offset"])

        if i == len(anchors) - 1:
            idx1_ext = idx1 + n
        else:
            idx1_ext = idx1

        if idx1_ext <= idx0:
            idx1_ext += n

        span = idx1_ext - idx0
        if span <= 0:
            continue

        for k_ext in range(idx0, idx1_ext + 1):
            k = k_ext % n
            t = (k_ext - idx0) / float(span)
            w = float(_smoothstep(t))
            out[k] = (1.0 - w) * off0 + w * off1

    return out


def smooth_closed_offset_signal(offsets):
    """
    Smooth the offset signal only, not the centerline geometry.
    This is the main mechanism that blends entrance/apex/exit anchors into a
    continuous raceline.
    """
    if RACELINE_OFFSET_SMOOTHING_WINDOW < 3 or RACELINE_OFFSET_SMOOTHING_WINDOW % 2 == 0:
        raise RuntimeError("RACELINE_OFFSET_SMOOTHING_WINDOW must be odd and >= 3.")

    if RACELINE_OFFSET_SMOOTHING_METHOD == "moving_average":
        return circular_moving_average(offsets, RACELINE_OFFSET_SMOOTHING_WINDOW)
    if RACELINE_OFFSET_SMOOTHING_METHOD == "gaussian":
        return circular_gaussian_smooth(
            offsets,
            RACELINE_OFFSET_SMOOTHING_WINDOW,
            RACELINE_OFFSET_GAUSSIAN_SIGMA_POINTS
        )
    raise ValueError("RACELINE_OFFSET_SMOOTHING_METHOD must be 'moving_average' or 'gaussian'.")


def apply_curvature_aware_offset_limit(centerline_rows, offsets):
    """
    Limit dangerous inside offsets in high-curvature regions.

    For a normal-offset curve r_offset = r + d*N, a cusp/fold can appear
    when d * curvature approaches 1 for an inside offset. This limiter keeps
    approximately |d * curvature| <= RACELINE_CURVATURE_OFFSET_LIMIT_FACTOR
    for inside offsets only. Outside offsets are much less prone to folding.
    """
    offsets = np.asarray(offsets, dtype=np.float64).copy()
    if not RACELINE_USE_CURVATURE_OFFSET_LIMIT:
        return offsets

    curv = np.array([float(r["curvature"]) for r in centerline_rows], dtype=np.float64)
    eps = float(RACELINE_CURVATURE_EPS)
    factor = float(RACELINE_CURVATURE_OFFSET_LIMIT_FACTOR)

    for i in range(len(offsets)):
        k = float(curv[i])
        d = float(offsets[i])

        # Same sign means the offset goes to the inside of the turn.
        if abs(k) > eps and d * k > 0.0:
            limit = factor / max(abs(k), eps)
            if abs(d) > limit:
                offsets[i] = np.sign(d) * limit

    return offsets


def limit_offset_gradient_circular(offsets, spacing_m):
    """
    Limit how quickly the lateral offset can change along the closed loop.

    This prevents aggressive outside->inside->outside transitions near tight
    corners, which can create local S-shaped folds even if every point is
    technically still inside the safety mask.
    """
    offsets = np.asarray(offsets, dtype=np.float64).copy()
    if not RACELINE_USE_OFFSET_GRADIENT_LIMIT or len(offsets) < 3:
        return offsets

    max_delta = float(RACELINE_MAX_OFFSET_CHANGE_PER_M) * float(spacing_m)
    if max_delta <= 0.0:
        return offsets

    n = len(offsets)
    passes = max(1, int(RACELINE_OFFSET_GRADIENT_LIMIT_PASSES))

    for _ in range(passes):
        # Forward circular pass.
        for i in range(n):
            j = (i + 1) % n
            diff = offsets[j] - offsets[i]
            if diff > max_delta:
                offsets[j] = offsets[i] + max_delta
            elif diff < -max_delta:
                offsets[j] = offsets[i] - max_delta

        # Backward circular pass.
        for i in range(n - 1, -1, -1):
            j = (i - 1) % n
            diff = offsets[j] - offsets[i]
            if diff > max_delta:
                offsets[j] = offsets[i] + max_delta
            elif diff < -max_delta:
                offsets[j] = offsets[i] - max_delta

    return offsets


def postprocess_offset_signal(centerline_rows, offsets):
    """
    Apply the two robustness filters to the smoothed offset signal:
      1. curvature-aware inside-offset clamp;
      2. offset-gradient clamp;
      3. one more curvature clamp after gradient limiting.
    """
    out = apply_curvature_aware_offset_limit(centerline_rows, offsets)
    out = limit_offset_gradient_circular(out, RESAMPLE_SPACING_M)
    out = apply_curvature_aware_offset_limit(centerline_rows, out)
    return out


def generate_raceline_from_offsets(centerline_rows, offsets):
    center_pts, yaw = _centerline_xy_yaw_arrays(centerline_rows)
    normals = _centerline_normals_from_yaw(yaw)
    offsets = np.asarray(offsets, dtype=np.float64)

    race_pts = center_pts + offsets[:, None] * normals
    out = [(float(p[0]), float(p[1])) for p in race_pts]
    if len(out) > 0:
        out.append(out[0])
    return out


def maybe_smooth_raceline_bspline(raceline_points):
    """
    Optionally smooth the final XY raceline with a periodic B-spline.
    Safety validation is intentionally done after this function in the caller.
    """
    if not USE_BSPLINE_RACELINE_SMOOTHING or len(raceline_points) < 5:
        return raceline_points

    return smooth_closed_loop_bspline(
        raceline_points,
        spacing_m=RACELINE_RESAMPLE_SPACING_M,
        smoothing_factor_per_point=RACELINE_BSPLINE_SMOOTHING_FACTOR_PER_POINT,
        degree=RACELINE_BSPLINE_DEGREE
    )


def build_offset_field_raceline_from_moved_keypoints(centerline_rows, moved_keypoints, safety_mask, yaml_data):
    """
    Build raceline using the smoothed centerline as the geometric backbone.

    Steps:
      1. Convert moved corner keypoints into signed lateral offset anchors.
      2. Interpolate a full closed-loop offset signal along the centerline.
      3. Smooth the offset signal.
      4. Shift every centerline point by offset * local normal.
      5. If unsafe, globally scale offsets down and retry.
    """
    n = len(centerline_rows)
    anchors = build_offset_anchors_from_moved_keypoints(centerline_rows, moved_keypoints)
    if n < 4 or len(anchors) < 3:
        return [], [], np.zeros(n, dtype=np.float64), np.zeros(n, dtype=np.float64), 0.0

    raw_offsets = interpolate_closed_offset_signal(n, anchors)
    smoothed_offsets_base = smooth_closed_offset_signal(raw_offsets)

    scale = float(RACELINE_OFFSET_SCALE_START)
    last_points = []
    last_offsets = np.zeros(n, dtype=np.float64)
    last_outside = 0
    last_inside_ratio = 0.0

    while scale >= float(RACELINE_OFFSET_SCALE_MIN) - 1e-9:
        # Scale first, then apply geometric safety filters.
        candidate_offsets = scale * smoothed_offsets_base
        candidate_offsets = postprocess_offset_signal(centerline_rows, candidate_offsets)

        candidate_points = generate_raceline_from_offsets(centerline_rows, candidate_offsets)
        candidate_points = maybe_smooth_raceline_bspline(candidate_points)

        # Validate AFTER optional final B-spline smoothing, because smoothing may
        # move points outside the eroded safety region.
        inside_ratio, outside_count = validate_points_in_mask(candidate_points, safety_mask, yaml_data)

        last_points = candidate_points
        last_offsets = candidate_offsets
        last_outside = outside_count
        last_inside_ratio = inside_ratio

        if outside_count == 0:
            reports = summarize_offset_field_reports(anchors, raw_offsets, candidate_offsets, scale, inside_ratio, outside_count)
            return candidate_points, reports, raw_offsets, candidate_offsets, scale

        scale *= float(RACELINE_OFFSET_SCALE_SHRINK)

    reports = summarize_offset_field_reports(anchors, raw_offsets, last_offsets, scale, last_inside_ratio, last_outside)
    return last_points, reports, raw_offsets, last_offsets, scale


def summarize_offset_field_reports(anchors, raw_offsets, final_offsets, scale, inside_ratio, outside_count):
    reports = []
    corner_ids = sorted(set(int(a["corner_id"]) for a in anchors if int(a["corner_id"]) >= 0))
    for cid in corner_ids:
        a_c = [a for a in anchors if int(a["corner_id"]) == cid]
        turn_direction = a_c[0]["turn_direction"] if a_c else "unknown"
        roles = ",".join(a["role"] for a in a_c)
        reports.append({
            "corner_id": int(cid),
            "turn_direction": str(turn_direction),
            "apex_pull": 1.0,
            "used_tangent_scale": float(scale),
            "blend_dist_m": float(np.max(np.abs(final_offsets))) if len(final_offsets) else 0.0,
            "corner_inside_ratio": float(inside_ratio),
            "corner_outside_count": int(outside_count),
            "corner_points": int(len(final_offsets)),
            "link_points_to_next": 0,
            "roles": roles,
        })
    return reports


def build_piecewise_raceline_from_moved_keypoints(centerline_rows, moved_keypoints, safety_mask, yaml_data):
    """
    Compatibility wrapper for main(). Despite the old name, this now builds the
    offset-field raceline on the smoothed centerline backbone.
    """
    raceline, reports, raw_offsets, final_offsets, used_scale = build_offset_field_raceline_from_moved_keypoints(
        centerline_rows,
        moved_keypoints,
        safety_mask,
        yaml_data
    )
    return raceline, reports, raw_offsets, final_offsets, used_scale

def save_raceline_csv(points, csv_path):
    """
    Save final smoothed raceline with the same geometry fields as
    centerline_points_smooth.csv.

    Columns:
      index, x, y, yaw, curvature, curvature_abs
    """
    rows = compute_yaw_and_curvature(points)
    save_centerline_geometry_csv(rows, csv_path)


def save_legacy_raceline_xy_csv(points, csv_path):
    """Save x/y-only raceline CSV for backward compatibility."""
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "x", "y"])
        for i, (x, y) in enumerate(points):
            writer.writerow([i, float(x), float(y)])


def overlay_raceline_common(
    img,
    yaml_data,
    centerline_rows,
    moved_keypoints,
    raceline_points,
    safety_mask,
    path,
    title,
    raceline_label
):
    if not DEBUG:
        return

    extent = get_world_extent(img.shape, yaml_data)
    base = img.astype(np.float32) / 255.0
    cpts = np.array([[r["x"], r["y"]] for r in centerline_rows], dtype=np.float64)

    plt.figure(figsize=(9, 7))
    plt.imshow(base, cmap="gray", origin="lower", extent=extent)

    safety_overlay = np.ma.masked_where(safety_mask <= 0, safety_mask)
    plt.imshow(safety_overlay, cmap="Greens", origin="lower", extent=extent, alpha=0.18)

    plt.plot(cpts[:, 0], cpts[:, 1], linewidth=1.0, label="centerline")

    if len(moved_keypoints) > 0:
        ref = np.array([[kp["x"], kp["y"]] for kp in order_moved_keypoints_for_raceline(moved_keypoints)], dtype=np.float64)
        ref_closed = np.vstack([ref, ref[0]])
        plt.plot(ref_closed[:, 0], ref_closed[:, 1], linewidth=1.0, linestyle="--", label="moved keypoint polygon")
        plt.scatter(ref[:, 0], ref[:, 1], s=35, label="moved keypoints")

    if len(raceline_points) > 0:
        rp = np.array(raceline_points, dtype=np.float64)
        plt.plot(rp[:, 0], rp[:, 1], linewidth=2.0, label=raceline_label)

    plt.title(title)
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.axis("equal")
    plt.legend()
    plt.savefig(path, dpi=140)
    plt.close()


def overlay_piecewise_raceline(img, yaml_data, centerline_rows, moved_keypoints, raceline_points, safety_mask, path):
    overlay_raceline_common(
        img,
        yaml_data,
        centerline_rows,
        moved_keypoints,
        raceline_points,
        safety_mask,
        path,
        title="Offset-field Raceline from Centerline Backbone",
        raceline_label="offset-field raceline"
    )


def overlay_global_spline_raceline(img, yaml_data, centerline_rows, moved_keypoints, raceline_points, safety_mask, path):
    overlay_raceline_common(
        img,
        yaml_data,
        centerline_rows,
        moved_keypoints,
        raceline_points,
        safety_mask,
        path,
        title="Global Soft B-spline Raceline from Moved Keypoints",
        raceline_label="global soft spline"
    )


def plot_raceline_curvature(raceline_points, path):
    """
    Plot final raceline curvature after all offset limiting and optional B-spline smoothing.
    This helps verify whether the final exported raceline is smoother than the raw
    offset-field result and whether tight turns still contain curvature spikes.
    """
    if not DEBUG or len(raceline_points) < 5:
        return

    rows = compute_yaw_and_curvature(raceline_points)
    curv = np.array([float(r["curvature"]) for r in rows], dtype=np.float64)

    # Display-only smoothing; does not modify the exported raceline.
    smooth_window = 31 if len(curv) >= 31 else (len(curv) // 2) * 2 - 1
    if smooth_window >= 3:
        curv_smooth = circular_gaussian_smooth(curv, smooth_window, max(2.0, smooth_window / 5.0))
    else:
        curv_smooth = curv.copy()

    plt.figure(figsize=(11, 4))
    plt.plot(curv, linewidth=0.8, label="raw raceline curvature")
    plt.plot(curv_smooth, linewidth=1.4, label="smoothed raceline curvature")
    plt.axhline(0.0, linewidth=0.8)

    all_curv = np.concatenate([curv, curv_smooth])
    y_abs = max(float(np.percentile(np.abs(all_curv), 99.5)), 1e-6) * 1.15
    plt.ylim(-y_abs, y_abs)

    plt.title("Final Raceline Curvature")
    plt.xlabel("Raceline index")
    plt.ylabel("Curvature (1/m)")
    plt.legend(loc="best")
    plt.tight_layout()
    plt.savefig(path, dpi=140)
    plt.close()

# =========================
# VISUALIZATION
# =========================
def save_img(img, path, yaml_data, cmap="gray", title=None):
    """
    Save an image in map/world coordinates.
    """
    if not DEBUG:
        return

    extent = get_world_extent(img.shape, yaml_data)

    plt.figure(figsize=(8, 6))
    plt.imshow(img, cmap=cmap, origin="lower", extent=extent)
    plt.title(title if title is not None else os.path.basename(path))
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.axis("equal")
    plt.savefig(path, dpi=120)
    plt.close()


def overlay_mask(img, mask, path, yaml_data, title=None):
    """
    Overlay a binary mask on the grayscale map in world coordinates.
    """
    if not DEBUG:
        return

    extent = get_world_extent(img.shape, yaml_data)

    base = img.astype(np.float32) / 255.0
    rgb = np.dstack([base, base, base])
    rgb[:, :, 0] = np.where(mask > 0, 1, rgb[:, :, 0])
    rgb[:, :, 1] = np.where(mask > 0, 0, rgb[:, :, 1])
    rgb[:, :, 2] = np.where(mask > 0, 0, rgb[:, :, 2])

    plt.figure(figsize=(8, 6))
    plt.imshow(rgb, origin="lower", extent=extent)
    plt.title(title if title is not None else os.path.basename(path))
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.axis("equal")
    plt.savefig(path, dpi=120)
    plt.close()


def overlay_ordered_path(img, path_pixels, path, yaml_data):
    """
    Overlay the ordered loop path on the grayscale map in world coordinates.
    """
    if not DEBUG:
        return

    extent = get_world_extent(img.shape, yaml_data)
    base = img.astype(np.float32) / 255.0
    world_pts = np.array(pixel_path_to_world(path_pixels, yaml_data))

    plt.figure(figsize=(8, 6))
    plt.imshow(base, cmap="gray", origin="lower", extent=extent)
    plt.plot(world_pts[:, 0], world_pts[:, 1], linewidth=1.2)
    plt.scatter([world_pts[0, 0]], [world_pts[0, 1]], s=30, marker="o")

    plt.title("Ordered Loop Path Overlay")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.axis("equal")
    plt.savefig(path, dpi=120)
    plt.close()


def overlay_world_paths(img, yaml_data, raw_pts, resampled_pts, smoothed_pts, path):
    """
    Overlay raw, resampled, and smoothed centerlines in world coordinates.
    """
    if not DEBUG:
        return

    extent = get_world_extent(img.shape, yaml_data)
    base = img.astype(np.float32) / 255.0

    plt.figure(figsize=(8, 6))
    plt.imshow(base, cmap="gray", origin="lower", extent=extent)

    raw = np.array(raw_pts)
    resamp = np.array(resampled_pts)
    smooth = np.array(smoothed_pts)

    plt.plot(raw[:, 0], raw[:, 1], linewidth=0.8, label="raw")
    plt.plot(resamp[:, 0], resamp[:, 1], linewidth=1.0, label="resampled")
    plt.plot(smooth[:, 0], smooth[:, 1], linewidth=1.5, label="smoothed")
    plt.scatter([smooth[0, 0]], [smooth[0, 1]], s=30, marker="o")

    plt.title("Phase 8: Raw / Resampled / Smoothed Centerline")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.axis("equal")
    plt.legend()
    plt.savefig(path, dpi=120)
    plt.close()


# =========================
# MAIN PIPELINE
# =========================
def main():
    if len(sys.argv) != 3:
        print("Usage: python3 generate_centerline.py <map_image> <map.yaml>")
        return

    map_path, yaml_path = sys.argv[1], sys.argv[2]
    ensure_output_dir(OUTPUT_DIR)

    img, meta = load_map(map_path, yaml_path)
    res = meta["resolution"]

    print(f"Map shape: {img.shape}, resolution={res:.3f} m/pixel")

    # Phase 2: free-space extraction
    free_raw, occ, unk = build_free_space(img)

    free, region_info = select_drivable_region(free_raw)
    print("Drivable region selection:")
    for k, v in region_info.items():
        print(f"  {k}: {v}")

    inflated, rpx = inflate_obstacles(free, res, INFLATION_RADIUS_M)
    print(f"Inflation radius: {INFLATION_RADIUS_M:.3f} m ({rpx} px)")

    # Phase 3: distance field
    dist_px, dist_m = compute_distance_map(inflated, res)
    print(f"Max distance to obstacle: {float(np.max(dist_m)):.3f} m")

    # Phase 4: skeleton
    skeleton = compute_skeleton(inflated)
    print(f"Skeleton pixels: {int(np.sum(skeleton))}")

    # Phase 5: prune spurs and isolate main loop
    endpoints, junctions = find_features(skeleton)
    print(f"Endpoint pixels: {int(np.sum(endpoints))}")
    print(f"Junction pixels: {int(np.sum(junctions))}")

    sk_wo_j = remove_junctions(skeleton, junctions)
    labels, segments = analyze_segments(sk_wo_j, endpoints)

    print("Connected segments after junction removal:")
    for seg in segments:
        print(
            f"  label={seg['label_id']:2d}, "
            f"pixels={seg['pixel_count']:4d}, "
            f"touches_endpoint={seg['touches_endpoint']}"
        )

    pruned = remove_spurs(skeleton, labels, segments, SPUR_DILATION_RADIUS)
    main_loop = keep_largest_component(pruned)
    print(f"Main loop pixels: {int(np.sum(main_loop))}")

    # Validation before traversal
    loop_metrics = validate_main_loop(main_loop)
    print("Main loop validation:")
    print(f"  Connected components: {loop_metrics['connected_components']}")
    print(f"  Endpoint count:       {loop_metrics['endpoint_count']}")
    print(f"  Junction count:       {loop_metrics['junction_count']}")
    print(f"  Loop pixels:          {loop_metrics['loop_pixels']}")

    if loop_metrics["connected_components"] != 1:
        raise RuntimeError("Main loop validation failed: expected exactly 1 connected component.")
    if loop_metrics["endpoint_count"] != 0:
        raise RuntimeError("Main loop validation failed: expected 0 endpoints for a closed loop.")
    if loop_metrics["junction_count"] != 0:
        print("Warning: main loop still has junction pixels; traversal may be ambiguous.")

    # Phase 6: ordered traversal
    ordered_loop = trace_ordered_loop(main_loop)

    traversal_metrics = validate_ordered_loop(main_loop, ordered_loop, res)
    print("Ordered loop validation:")
    print(f"  Ordered points:       {traversal_metrics['ordered_points']}")
    print(f"  Unique ordered pts:   {traversal_metrics['unique_ordered_points']}")
    print(f"  Coverage ratio:       {traversal_metrics['coverage_ratio']:.3f}")
    print(f"  Required coverage:    {MIN_ORDERED_LOOP_COVERAGE_RATIO:.3f}")
    print(f"  Closed:               {traversal_metrics['closed']}")
    print(f"  Step length min:      {traversal_metrics['min_step_m']:.3f} m")
    print(f"  Step length mean:     {traversal_metrics['mean_step_m']:.3f} m")
    print(f"  Step length max:      {traversal_metrics['max_step_m']:.3f} m")

    if not traversal_metrics["closed"]:
        raise RuntimeError("Ordered loop validation failed: loop is not closed.")
    if not 0.0 < MIN_ORDERED_LOOP_COVERAGE_RATIO <= 1.0:
        raise RuntimeError(
            "MIN_ORDERED_LOOP_COVERAGE_RATIO must be in the interval (0, 1]."
        )
    if traversal_metrics["coverage_ratio"] < MIN_ORDERED_LOOP_COVERAGE_RATIO:
        raise RuntimeError("Ordered loop validation failed: traversal coverage is too low.")
    if traversal_metrics["max_step_m"] > np.sqrt(2) * res + 1e-6:
        raise RuntimeError("Ordered loop validation failed: detected a traversal jump.")

    # Phase 7: raw world centerline
    world_loop_raw = pixel_path_to_world(ordered_loop, meta)
    raw_loop_length_m = compute_closed_loop_length(world_loop_raw)

    print("World-coordinate raw centerline:")
    print(f"  Raw world points:     {len(world_loop_raw)}")
    print(f"  Approx loop length:   {raw_loop_length_m:.3f} m")

    raw_csv_path = os.path.join(OUTPUT_DIR, RAW_CSV_NAME)
    save_centerline_csv(world_loop_raw, raw_csv_path)
    print(f"Saved raw centerline:   {raw_csv_path}")

    # Phase 8: resample + smooth
    if SMOOTHING_WINDOW < 3 or SMOOTHING_WINDOW % 2 == 0:
        raise RuntimeError("SMOOTHING_WINDOW must be odd and >= 3.")

    world_loop_resampled = resample_closed_loop(world_loop_raw, RESAMPLE_SPACING_M)

    if USE_BSPLINE_CENTERLINE_SMOOTHING:
        world_loop_smoothed = smooth_closed_loop_bspline(
            world_loop_resampled,
            spacing_m=RESAMPLE_SPACING_M,
            smoothing_factor_per_point=BSPLINE_SMOOTHING_FACTOR_PER_POINT,
            degree=BSPLINE_DEGREE
        )
    else:
        world_loop_smoothed = smooth_closed_loop(
            world_loop_resampled,
            window_size=SMOOTHING_WINDOW,
            passes=SMOOTHING_PASSES
        )

    phase8_metrics = validate_smoothed_loop(
        world_loop_raw,
        world_loop_resampled,
        world_loop_smoothed
    )

    print("Phase 8: resampling + smoothing")
    print(f"  Resample spacing:     {RESAMPLE_SPACING_M:.3f} m")
    print(f"  Smoothing window:     {SMOOTHING_WINDOW}")
    print(f"  Smoothing passes:     {SMOOTHING_PASSES}")
    print(f"  B-spline smoothing:   {USE_BSPLINE_CENTERLINE_SMOOTHING}")
    if USE_BSPLINE_CENTERLINE_SMOOTHING:
        print(f"  B-spline degree:      {BSPLINE_DEGREE}")
        print(f"  B-spline s/point:     {BSPLINE_SMOOTHING_FACTOR_PER_POINT:.6f}")
    print(f"  Raw length:           {phase8_metrics['raw_length_m']:.3f} m")
    print(f"  Resampled length:     {phase8_metrics['resampled_length_m']:.3f} m")
    print(f"  Smoothed length:      {phase8_metrics['smoothed_length_m']:.3f} m")
    print(f"  Resampled points:     {phase8_metrics['resampled_points']}")
    print(f"  Smoothed points:      {phase8_metrics['smoothed_points']}")
    print(f"  Resampled step mean:  {phase8_metrics['resampled_step_mean_m']:.3f} m")
    print(f"  Smoothed step mean:   {phase8_metrics['smoothed_step_mean_m']:.3f} m")

    # Apply final direction and compute yaw/curvature for exported smooth centerline.
    world_loop_final = apply_centerline_direction(world_loop_smoothed, CENTERLINE_DIRECTION)
    centerline_rows = compute_yaw_and_curvature(world_loop_final)
    world_loop_final_xy = rows_to_xy_points(centerline_rows)

    smooth_csv_path = os.path.join(OUTPUT_DIR, SMOOTH_CSV_NAME)
    save_centerline_geometry_csv(centerline_rows, smooth_csv_path)
    print(f"Saved smooth centerline:{smooth_csv_path}")
    print(f"  Geometry columns:      index,x,y,yaw,curvature,curvature_abs")
    print(f"  Export direction:      {CENTERLINE_DIRECTION}")

    # Phase 9: final export package
    smooth_npy_path = os.path.join(OUTPUT_DIR, SMOOTH_NPY_NAME)
    metadata_yaml_path = os.path.join(OUTPUT_DIR, METADATA_YAML_NAME)

    save_centerline_npy(centerline_rows, smooth_npy_path)
    save_metadata_yaml(
        metadata_yaml_path,
        map_path=map_path,
        yaml_path=yaml_path,
        map_meta=meta,
        region_info=region_info,
        traversal_metrics=traversal_metrics,
        raw_loop=world_loop_raw,
        smooth_loop=world_loop_final_xy,
        centerline_rows=centerline_rows
    )

    print("Phase 9: final export package")
    print(f"  Saved NumPy array:    {smooth_npy_path}")
    print(f"  Saved metadata YAML:  {metadata_yaml_path}")

    # Phase 10: detect corner entrance/apex/exit points only
    corner_debug = None
    if DETECT_CORNERS:
        print("Phase 10: corner key-point detection")
        corners, curv_raw, curv_smooth, corner_mask = detect_corner_keypoints(centerline_rows)
        corners_csv_path = os.path.join(OUTPUT_DIR, CORNERS_CSV_NAME)
        save_corner_keypoints_csv(corners, corners_csv_path)

        print(f"  Saved corner CSV:      {corners_csv_path}")
        print(f"  Curvature smoothing:   {CORNER_CURVATURE_SMOOTHING_METHOD}, window={CORNER_CURVATURE_SMOOTHING_WINDOW}, sigma={CORNER_CURVATURE_GAUSSIAN_SIGMA_POINTS}")
        print(f"  Entry/exit threshold:  {CORNER_ENTRY_EXIT_THRESHOLD:.3f} 1/m")
        print(f"  Apex threshold:        {CORNER_APEX_THRESHOLD:.3f} 1/m")
        print(f"  Min corner length:     {CORNER_MIN_LENGTH_POINTS} points")
        print(f"  Merge gap:             {CORNER_MERGE_GAP_POINTS} points")
        print(f"  Detected corners:      {len(corners)}")
        for c in corners:
            print(
                f"    C{c['corner_id']:02d}: {c['turn_direction']:5s} "
                f"entrance={c['entrance_index']:4d}, "
                f"apex={c['apex_index']:4d}, "
                f"exit={c['exit_index']:4d}, "
                f"|k|max={c['apex_curvature_abs']:.3f}, "
                f"length≈{c['approx_length_m']:.2f} m"
            )

        corner_debug = {
            "corners": corners,
            "curv_raw": curv_raw,
            "curv_smooth": curv_smooth,
            "corner_mask": corner_mask,
        }

    moved_keypoints = []

    if MOVE_CORNER_KEYPOINTS:
        print("Phase 11: move detected corner keypoints")

        moved_keypoints = build_moved_corner_keypoints(
            centerline_rows,
            corner_debug["corners"],
            free,
            meta
        )

        moved_csv_path = os.path.join(OUTPUT_DIR, MOVED_KEYPOINTS_CSV_NAME)
        save_moved_corner_keypoints_csv(moved_keypoints, moved_csv_path)

        print(f"  Saved moved keypoints CSV: {moved_csv_path}")
        print(f"  Apex safety margin:        {MOVED_APEX_SAFETY_MARGIN_M:.3f} m")
        print(f"  Entry/exit safety margin:  {MOVED_ENTRY_EXIT_SAFETY_MARGIN_M:.3f} m")
        print(f"  Ray step:                  {MOVED_KEYPOINT_RAY_STEP_M:.3f} m")

    raceline_points = []
    raceline_raw_offsets = []
    raceline_final_offsets = []
    raceline_offset_scale = 0.0
    raceline_safety_mask = None
    corner_reports = []

    if GENERATE_PIECEWISE_RACELINE:
        if not MOVE_CORNER_KEYPOINTS or len(moved_keypoints) < 3:
            print("Phase 12: skipped raceline generation; not enough moved keypoints")
        else:
            print("Phase 12: raceline generation from moved keypoints")
            raceline_safety_mask, safety_radius_px = build_safety_region_mask(
                free,
                res,
                RACELINE_SAFETY_REGION_MARGIN_M
            )
            print(f"  Safety region margin:         {RACELINE_SAFETY_REGION_MARGIN_M:.3f} m ({safety_radius_px} px erosion)")

            if GENERATE_PIECEWISE_RACELINE:
                raceline_points, corner_reports, raceline_raw_offsets, raceline_final_offsets, raceline_offset_scale = build_piecewise_raceline_from_moved_keypoints(
                    centerline_rows,
                    moved_keypoints,
                    raceline_safety_mask,
                    meta
                )

                inside_ratio, outside_count = validate_points_in_mask(
                    raceline_points,
                    raceline_safety_mask,
                    meta
                )
                raceline_csv_path = os.path.join(OUTPUT_DIR, RACELINE_CSV_NAME)
                save_raceline_csv(raceline_points, raceline_csv_path)

                legacy_raceline_csv_path = os.path.join(OUTPUT_DIR, RACELINE_LEGACY_CSV_NAME)
                save_legacy_raceline_xy_csv(raceline_points, legacy_raceline_csv_path)

                print(f"  Saved final smoothed raceline CSV: {raceline_csv_path}")
                print(f"  Geometry columns:                 index,x,y,yaw,curvature,curvature_abs")
                print(f"  Saved legacy x/y raceline CSV:    {legacy_raceline_csv_path}")
                print(f"  Offset smoothing:               {RACELINE_OFFSET_SMOOTHING_METHOD}, window={RACELINE_OFFSET_SMOOTHING_WINDOW}, sigma={RACELINE_OFFSET_GAUSSIAN_SIGMA_POINTS}")
                print(f"  Offset scale start/min:         {RACELINE_OFFSET_SCALE_START:.3f} / {RACELINE_OFFSET_SCALE_MIN:.3f}")
                print(f"  Offset scale used:              {raceline_offset_scale:.3f}")
                print(f"  Curvature offset limit:         {RACELINE_USE_CURVATURE_OFFSET_LIMIT}, factor={RACELINE_CURVATURE_OFFSET_LIMIT_FACTOR:.3f}")
                print(f"  Offset gradient limit:          {RACELINE_USE_OFFSET_GRADIENT_LIMIT}, max={RACELINE_MAX_OFFSET_CHANGE_PER_M:.3f} m/m")
                print(f"  Final B-spline smoothing:       {USE_BSPLINE_RACELINE_SMOOTHING}, s/point={RACELINE_BSPLINE_SMOOTHING_FACTOR_PER_POINT:.6f}")
                print(f"  Offset raw range:               {float(np.min(raceline_raw_offsets)):.3f} .. {float(np.max(raceline_raw_offsets)):.3f} m")
                print(f"  Offset final range:             {float(np.min(raceline_final_offsets)):.3f} .. {float(np.max(raceline_final_offsets)):.3f} m")
                print(f"  Raceline points:                {len(raceline_points)}")
                print(f"  Raceline inside ratio:          {inside_ratio:.3f}")
                print(f"  Raceline outside count:         {outside_count}")
                for rep in corner_reports:
                    print(
                        f"    C{rep['corner_id']:02d} {rep['turn_direction']:5s}: "
                        f"scale={rep['used_tangent_scale']:.3f}, "
                        f"roles={rep.get('roles', '')}, "
                        f"inside={rep['corner_inside_ratio']:.3f}, "
                        f"outside={rep['corner_outside_count']}"
                    )

    # ================= DEBUG OUTPUT =================
    if DEBUG:
        save_img(
            free * 255,
            os.path.join(OUTPUT_DIR, DEBUG_DRIVABLE_REGION),
            meta,
            title="Selected Drivable Region"
        )
        overlay_mask(
            img,
            main_loop,
            os.path.join(OUTPUT_DIR, DEBUG_MAIN_LOOP_OVERLAY),
            meta,
            title="Main Loop Overlay"
        )
        overlay_ordered_path(
            img,
            ordered_loop,
            os.path.join(OUTPUT_DIR, DEBUG_ORDERED_LOOP_OVERLAY),
            meta
        )
        overlay_world_paths(
            img,
            meta,
            world_loop_raw,
            world_loop_resampled,
            world_loop_final_xy,
            os.path.join(OUTPUT_DIR, DEBUG_PHASE8_PATHS)
        )

        if DETECT_CORNERS and corner_debug is not None:
            overlay_corner_keypoints(
                img,
                meta,
                centerline_rows,
                corner_debug["corners"],
                os.path.join(OUTPUT_DIR, DEBUG_CORNER_KEYPOINTS)
            )
            plot_corner_curvature(
                corner_debug["curv_raw"],
                corner_debug["curv_smooth"],
                corner_debug["corner_mask"],
                corner_debug["corners"],
                os.path.join(OUTPUT_DIR, DEBUG_CORNER_CURVATURE)
            )

        if MOVE_CORNER_KEYPOINTS and len(moved_keypoints) > 0:
            overlay_moved_corner_keypoints(
                img,
                meta,
                centerline_rows,
                moved_keypoints,
                os.path.join(OUTPUT_DIR, DEBUG_MOVED_KEYPOINTS)
            )

        if GENERATE_PIECEWISE_RACELINE and len(raceline_points) > 0 and raceline_safety_mask is not None:
            overlay_piecewise_raceline(
                img,
                meta,
                centerline_rows,
                moved_keypoints,
                raceline_points,
                raceline_safety_mask,
                os.path.join(OUTPUT_DIR, DEBUG_RACELINE_PIECEWISE)
            )
            plot_raceline_curvature(
                raceline_points,
                os.path.join(OUTPUT_DIR, DEBUG_RACELINE_CURVATURE)
            )


    print("Phase 12 completed successfully." if GENERATE_PIECEWISE_RACELINE else ("Phase 11 completed successfully." if MOVE_CORNER_KEYPOINTS else ("Phase 10 completed successfully." if DETECT_CORNERS else "Phase 9 completed successfully.")))


if __name__ == "__main__":
    main()
