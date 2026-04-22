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

# =========================
# CONFIG
# =========================
DEBUG = True

# Phase 2
INFLATION_RADIUS_M = 0.10

# Phase 5
SPUR_DILATION_RADIUS = 1

# Phase 8
RESAMPLE_SPACING_M = 0.10
SMOOTHING_WINDOW = 9       # must be odd and >= 3
SMOOTHING_PASSES = 2

# Drivable-region selection tuning
MIN_COMPONENT_AREA_ABS = 50
MIN_COMPONENT_AREA_RATIO = 0.01

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


# =========================
# PHASE 9: FINAL EXPORT PACKAGE
# =========================
def save_centerline_npy(world_pts, npy_path):
    """
    Save world-coordinate centerline points to NumPy binary format.
    Shape: [N, 2]
    """
    arr = np.array(world_pts, dtype=np.float64)
    np.save(npy_path, arr)


def save_metadata_yaml(
    metadata_path,
    map_path,
    yaml_path,
    map_meta,
    region_info,
    raw_loop,
    smooth_loop
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
            "min_component_area_abs": int(MIN_COMPONENT_AREA_ABS),
            "min_component_area_ratio": float(MIN_COMPONENT_AREA_RATIO),
        },
        "drivable_region_selection": region_info,
        "outputs": {
            "raw_csv": RAW_CSV_NAME,
            "smooth_csv": SMOOTH_CSV_NAME,
            "smooth_npy": SMOOTH_NPY_NAME,
            "raw_point_count": int(len(raw_loop)),
            "smooth_point_count": int(len(smooth_loop)),
            "raw_loop_length_m": float(raw_len),
            "smooth_loop_length_m": float(smooth_len),
        }
    }

    with open(metadata_path, "w") as f:
        yaml.safe_dump(metadata, f, sort_keys=False)


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
    print(f"  Closed:               {traversal_metrics['closed']}")
    print(f"  Step length min:      {traversal_metrics['min_step_m']:.3f} m")
    print(f"  Step length mean:     {traversal_metrics['mean_step_m']:.3f} m")
    print(f"  Step length max:      {traversal_metrics['max_step_m']:.3f} m")

    if not traversal_metrics["closed"]:
        raise RuntimeError("Ordered loop validation failed: loop is not closed.")
    if traversal_metrics["coverage_ratio"] < 0.98:
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
    print(f"  Raw length:           {phase8_metrics['raw_length_m']:.3f} m")
    print(f"  Resampled length:     {phase8_metrics['resampled_length_m']:.3f} m")
    print(f"  Smoothed length:      {phase8_metrics['smoothed_length_m']:.3f} m")
    print(f"  Resampled points:     {phase8_metrics['resampled_points']}")
    print(f"  Smoothed points:      {phase8_metrics['smoothed_points']}")
    print(f"  Resampled step mean:  {phase8_metrics['resampled_step_mean_m']:.3f} m")
    print(f"  Smoothed step mean:   {phase8_metrics['smoothed_step_mean_m']:.3f} m")

    smooth_csv_path = os.path.join(OUTPUT_DIR, SMOOTH_CSV_NAME)
    save_world_centerline_csv(world_loop_smoothed, smooth_csv_path)
    print(f"Saved smooth centerline:{smooth_csv_path}")

    # Phase 9: final export package
    smooth_npy_path = os.path.join(OUTPUT_DIR, SMOOTH_NPY_NAME)
    metadata_yaml_path = os.path.join(OUTPUT_DIR, METADATA_YAML_NAME)

    save_centerline_npy(world_loop_smoothed, smooth_npy_path)
    save_metadata_yaml(
        metadata_yaml_path,
        map_path=map_path,
        yaml_path=yaml_path,
        map_meta=meta,
        region_info=region_info,
        raw_loop=world_loop_raw,
        smooth_loop=world_loop_smoothed
    )

    print("Phase 9: final export package")
    print(f"  Saved NumPy array:    {smooth_npy_path}")
    print(f"  Saved metadata YAML:  {metadata_yaml_path}")

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
            world_loop_smoothed,
            os.path.join(OUTPUT_DIR, DEBUG_PHASE8_PATHS)
        )

    print("Phase 9 completed successfully.")


if __name__ == "__main__":
    main()
