#!/usr/bin/env python3
"""
Manual raceline editor/generator.

Purpose
-------
Use the Phase 9 centerline/drivable-region outputs from
centerline_reference_generator.py, but do NOT auto-detect corners and do NOT
auto-move E/A/X points laterally.

The user manually:
  1. adds E/A/X corner groups on the centerline;
  2. selects a point without moving it;
  3. explicitly enters Centerline Ref Move mode to change longitudinal index;
  4. explicitly enters Lateral Move mode to change signed lateral offset;
  5. optionally flips the driving direction in-memory and shows it in the UI;
  6. previews the raceline using the same raceline-generation/smoothing backend
     as raceline_generator.py;
  7. saves the standard raceline output files.

When run standalone, the script resumes the most recent compatible manual
session by default. Use --fresh to deliberately start with no manual corners.

If no manual corner points are added, the saved raceline is the centerline.
"""

import csv
import os
import sys

import numpy as np
import yaml

import raceline_generator as rg

plt = rg.plt
Button = rg.Button


# =========================
# MANUAL UI TUNING
# =========================
MANUAL_KEYPOINT_PICK_RADIUS_M = 0.45
MANUAL_LATERAL_KEY_STEP_M = 0.02
MANUAL_CENTERLINE_KEY_STEP_POINTS = 1
MANUAL_NEW_CORNER_HALF_SPAN_POINTS = 20
MANUAL_RESUME_IF_NEWER_THAN_CENTERLINE = True
MANUAL_PREVIEW_PNG = "debug_manual_raceline_preview.png"
MANUAL_EDIT_FALLBACK_PNG = "debug_manual_raceline_editor.png"

# The user may inspect the full physical corridor, but manual movement itself is
# clamped to the eroded safety region used by the final raceline validator.
MANUAL_CLAMP_TO_SAFETY_MASK = True

ROLE_ORDER = ("E", "A", "X")
ROLE_TO_BACKEND = {
    "E": "entrance_outside",
    "A": "apex_inside",
    "X": "exit_outside",
}
ROLE_MARKER = {
    "E": ("^", "tab:green", "entrance"),
    "A": ("*", "tab:red", "apex"),
    "X": ("s", "tab:purple", "exit"),
}


def _index_key(role):
    return {"E": "entrance_index", "A": "apex_index", "X": "exit_index"}[role]


def _xy_keys(role):
    return {
        "E": ("entrance_x", "entrance_y"),
        "A": ("apex_x", "apex_y"),
        "X": ("exit_x", "exit_y"),
    }[role]


def _offset_key(role):
    return f"manual_{role.lower()}_offset_m"


def _corner_role_index(corner, role):
    return int(corner[_index_key(role)])


def _set_corner_role_index(corner, role, index, centerline_rows, preserve_offset=True):
    """Move an E/A/X reference longitudinally along the centerline."""
    idx = int(index) % len(centerline_rows)
    xk, yk = _xy_keys(role)
    row = centerline_rows[idx]
    corner[_index_key(role)] = idx
    corner[xk] = float(row["x"])
    corner[yk] = float(row["y"])
    if not preserve_offset:
        corner[_offset_key(role)] = 0.0

    n = len(centerline_rows)
    idxs = rg.circular_segment_indices(
        int(corner["entrance_index"]),
        int(corner["exit_index"]),
        n,
    )
    corner["num_points"] = int(len(idxs))
    corner["approx_length_m"] = float(len(idxs) * rg.RESAMPLE_SPACING_M)


def _make_manual_corner(centerline_rows, apex_index):
    """Create a centerline E/A/X group without assigning a turn direction."""
    n = len(centerline_rows)
    apex = int(apex_index) % n
    half = int(MANUAL_NEW_CORNER_HALF_SPAN_POINTS)
    entrance = (apex - half) % n
    exit_idx = (apex + half) % n

    c = {
        "corner_id": -1,
        "turn_direction": "manual",
        "turn_sign": 0,
        "entrance_index": int(entrance),
        "apex_index": int(apex),
        "exit_index": int(exit_idx),
        "num_points": 0,
        "approx_length_m": 0.0,
        "apex_curvature": 0.0,
        "apex_curvature_abs": 0.0,
        "entrance_x": 0.0,
        "entrance_y": 0.0,
        "apex_x": 0.0,
        "apex_y": 0.0,
        "exit_x": 0.0,
        "exit_y": 0.0,
        "manual_e_offset_m": 0.0,
        "manual_a_offset_m": 0.0,
        "manual_x_offset_m": 0.0,
    }
    for role in ROLE_ORDER:
        _set_corner_role_index(c, role, _corner_role_index(c, role), centerline_rows)
    return c


def _refresh_corner_ids(corners):
    corners.sort(key=lambda c: int(c["entrance_index"]))
    for i, c in enumerate(corners):
        c["corner_id"] = int(i)
    return corners


def _point_center_and_normal(centerline_rows, corner, role):
    idx = _corner_role_index(corner, role)
    row = centerline_rows[idx]
    center = np.array([float(row["x"]), float(row["y"])], dtype=np.float64)
    yaw = float(row["yaw"])
    left_normal = np.array([-np.sin(yaw), np.cos(yaw)], dtype=np.float64)
    return idx, row, center, left_normal


def _point_world_xy(centerline_rows, corner, role):
    _, _, center, normal = _point_center_and_normal(centerline_rows, corner, role)
    offset = float(corner.get(_offset_key(role), 0.0))
    p = center + offset * normal
    return float(p[0]), float(p[1])


def _lateral_limits(centerline_rows, corner, role, drivable_mask, safety_mask, yaml_data):
    """
    Return raw and safe lateral limits about the selected centerline point.

    Sign convention:
      + offset = left of driving direction
      - offset = right of driving direction
    """
    _, row, _, _ = _point_center_and_normal(centerline_rows, corner, role)
    x = float(row["x"])
    y = float(row["y"])
    yaw = float(row["yaw"])
    step = float(rg.MOVED_KEYPOINT_RAY_STEP_M)

    raw_left = rg.raycast_available_lateral_distance(x, y, yaw, +1, drivable_mask, yaml_data, step)
    raw_right = rg.raycast_available_lateral_distance(x, y, yaw, -1, drivable_mask, yaml_data, step)

    safe_left = rg.raycast_available_lateral_distance(x, y, yaw, +1, safety_mask, yaml_data, step)
    safe_right = rg.raycast_available_lateral_distance(x, y, yaw, -1, safety_mask, yaml_data, step)

    return {
        "raw_min": -float(raw_right),
        "raw_max": +float(raw_left),
        "safe_min": -float(safe_right),
        "safe_max": +float(safe_left),
    }


def _clamp_point_offset(centerline_rows, corner, role, drivable_mask, safety_mask, yaml_data, requested_offset):
    limits = _lateral_limits(centerline_rows, corner, role, drivable_mask, safety_mask, yaml_data)
    if MANUAL_CLAMP_TO_SAFETY_MASK:
        lo, hi = limits["safe_min"], limits["safe_max"]
    else:
        lo, hi = limits["raw_min"], limits["raw_max"]
    return float(np.clip(float(requested_offset), lo, hi)), limits


def build_manual_moved_keypoints(centerline_rows, corners, drivable_mask, safety_mask, yaml_data):
    """Convert the manual E/A/X state into the backend moved-keypoint schema."""
    keypoints = []
    for c in corners:
        cid = int(c["corner_id"])
        for role in ROLE_ORDER:
            idx, row, center, normal = _point_center_and_normal(centerline_rows, c, role)
            offset, limits = _clamp_point_offset(
                centerline_rows,
                c,
                role,
                drivable_mask,
                safety_mask,
                yaml_data,
                c.get(_offset_key(role), 0.0),
            )
            c[_offset_key(role)] = offset
            moved = center + offset * normal
            lateral_sign = 0 if abs(offset) < 1e-9 else (1 if offset > 0.0 else -1)
            available = limits["raw_max"] if lateral_sign > 0 else (-limits["raw_min"] if lateral_sign < 0 else max(limits["raw_max"], -limits["raw_min"]))

            keypoints.append({
                "moved_keypoint_id": len(keypoints),
                "corner_id": cid,
                "role": ROLE_TO_BACKEND[role],
                "turn_direction": "manual",
                "turn_sign": 0,
                "center_index": int(idx),
                "x": float(moved[0]),
                "y": float(moved[1]),
                "center_x": float(center[0]),
                "center_y": float(center[1]),
                "center_yaw": float(row["yaw"]),
                "lateral_sign": int(lateral_sign),
                "available_to_limit_m": float(available),
                "safety_margin_m": float(rg.RACELINE_SAFETY_REGION_MARGIN_M),
                "move_dist_m": float(abs(offset)),
            })
    return keypoints


def _centerline_as_closed_points(centerline_rows):
    pts = [(float(r["x"]), float(r["y"])) for r in centerline_rows]
    if pts and not np.allclose(pts[0], pts[-1]):
        pts.append(pts[0])
    return pts


def generate_preview(centerline_rows, corners, drivable_mask, safety_mask, yaml_data):
    """Generate a preview using exactly the current raceline_generator backend."""
    # Keep IDs synchronized before converting manual points to backend anchors.
    # This also makes saved moved-keypoint files reliably resumable.
    if len(corners) > 0:
        _refresh_corner_ids(corners)

    if len(corners) == 0:
        points = _centerline_as_closed_points(centerline_rows)
        moved = []
        raw_offsets = np.zeros(len(centerline_rows), dtype=np.float64)
        final_offsets = raw_offsets.copy()
        scale = 0.0
        reports = []
    else:
        moved = build_manual_moved_keypoints(
            centerline_rows,
            corners,
            drivable_mask,
            safety_mask,
            yaml_data,
        )
        points, reports, raw_offsets, final_offsets, scale = rg.build_piecewise_raceline_from_moved_keypoints(
            centerline_rows,
            moved,
            safety_mask,
            yaml_data,
        )

    inside_ratio, outside_count = rg.validate_points_in_mask(points, safety_mask, yaml_data)
    curvature_ok, curvature_summary, curvature_limit = rg.raceline_curvature_is_ok(points)
    return {
        "points": points,
        "moved_keypoints": moved,
        "reports": reports,
        "raw_offsets": raw_offsets,
        "final_offsets": final_offsets,
        "scale": float(scale),
        "inside_ratio": float(inside_ratio),
        "outside_count": int(outside_count),
        "curvature_ok": bool(curvature_ok),
        "curvature_summary": curvature_summary,
        "curvature_limit": float(curvature_limit),
    }


def _corners_for_standard_csv(corners, centerline_rows):
    """Build the normal corner CSV schema while keeping direction explicitly manual."""
    out = []
    for c in _refresh_corner_ids([dict(x) for x in corners]):
        cc = dict(c)
        cc["turn_direction"] = "manual"
        cc["turn_sign"] = 0
        apex_idx = int(cc["apex_index"])
        k = float(centerline_rows[apex_idx].get("curvature", 0.0))
        cc["apex_curvature"] = k
        cc["apex_curvature_abs"] = abs(k)
        # CSV E/A/X positions intentionally mean the original centerline
        # reference positions. Actual lateral moves are in moved_keypoints.csv.
        for role in ROLE_ORDER:
            idx = _corner_role_index(cc, role)
            xk, yk = _xy_keys(role)
            cc[xk] = float(centerline_rows[idx]["x"])
            cc[yk] = float(centerline_rows[idx]["y"])
        out.append(cc)
    return out


def save_manual_outputs(output_dir, centerline_rows, corners, preview, selected_direction="normal"):
    """Save the same principal files used by the existing raceline workflow."""
    os.makedirs(output_dir, exist_ok=True)
    standard_corners = _corners_for_standard_csv(corners, centerline_rows)

    edited_path = os.path.join(output_dir, rg.EDITED_CORNERS_CSV_NAME)
    base_fields = [
        "corner_id", "turn_direction", "turn_sign",
        "entrance_index", "apex_index", "exit_index",
        "num_points", "approx_length_m",
        "apex_curvature", "apex_curvature_abs",
        "entrance_x", "entrance_y", "apex_x", "apex_y", "exit_x", "exit_y",
    ]
    with open(edited_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=base_fields + ["selected_direction", "keypoint_reference"])
        writer.writeheader()
        for c in standard_corners:
            row = {k: c.get(k, "") for k in base_fields}
            row["selected_direction"] = selected_direction
            row["keypoint_reference"] = "manual_centerline_index_plus_lateral_offset"
            writer.writerow(row)

    metadata_path = os.path.join(output_dir, rg.EDITED_CORNERS_METADATA_YAML_NAME)
    with open(metadata_path, "w") as f:
        yaml.safe_dump({
            "selected_direction": selected_direction,
            "corner_count": int(len(standard_corners)),
            "source": "manual_raceline_generator",
            "keypoint_reference": "manual_centerline_index_plus_lateral_offset",
            "lateral_offsets_file": rg.MOVED_KEYPOINTS_CSV_NAME,
        }, f, sort_keys=False)

    direction_path = os.path.join(output_dir, "raceline_selected_direction.txt")
    with open(direction_path, "w") as f:
        f.write(str(selected_direction) + "\n")

    active_corner_path = os.path.join(output_dir, rg.CORNERS_CSV_NAME)
    rg.save_corner_keypoints_csv(standard_corners, active_corner_path)

    moved_path = os.path.join(output_dir, rg.MOVED_KEYPOINTS_CSV_NAME)
    rg.save_moved_corner_keypoints_csv(preview["moved_keypoints"], moved_path)

    raceline_path = os.path.join(output_dir, rg.RACELINE_CSV_NAME)
    rg.save_raceline_csv(preview["points"], raceline_path)

    legacy_path = os.path.join(output_dir, rg.RACELINE_LEGACY_CSV_NAME)
    rg.save_legacy_raceline_xy_csv(preview["points"], legacy_path)

    print("Manual raceline outputs saved:")
    print(f"  Edited corner refs:   {edited_path}")
    print(f"  Active corner refs:   {active_corner_path}")
    print(f"  Moved keypoints:      {moved_path}")
    print(f"  Final raceline:       {raceline_path}")
    print(f"  Legacy raceline:      {legacy_path}")
    print(f"  Direction:            {direction_path}")



def _load_saved_raceline_points(csv_path):
    """Load an already-exported raceline as XY points for resume preview."""
    points = []
    if not os.path.exists(csv_path):
        return points
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append((float(row["x"]), float(row["y"])))
    return points


def _load_saved_moved_keypoint_rows(csv_path):
    rows = []
    if not os.path.exists(csv_path):
        return rows
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(dict(row))
    return rows


def load_manual_session_if_available(
    output_dir,
    centerline_csv_path,
    centerline_rows,
    drivable_mask,
    safety_mask,
    yaml_data,
):
    """
    Resume a previous manual-raceline editing session when this script is run
    by itself.

    The manual corner E/A/X indices come from corner_key_points_edited.csv.
    Signed lateral offsets are reconstructed from moved_corner_keypoints.csv by
    projecting each saved moved XY point onto the current centerline normal.

    To avoid applying stale manual data to a newly regenerated centerline, the
    edited manual CSV must be at least as new as centerline_points_smooth.csv
    when MANUAL_RESUME_IF_NEWER_THAN_CENTERLINE is enabled.
    """
    edited_path = os.path.join(output_dir, rg.EDITED_CORNERS_CSV_NAME)
    metadata_path = os.path.join(output_dir, rg.EDITED_CORNERS_METADATA_YAML_NAME)
    moved_path = os.path.join(output_dir, rg.MOVED_KEYPOINTS_CSV_NAME)
    raceline_path = os.path.join(output_dir, rg.RACELINE_CSV_NAME)

    if not os.path.exists(edited_path) or not os.path.exists(metadata_path):
        return None

    try:
        with open(metadata_path, "r") as f:
            meta = yaml.safe_load(f) or {}
    except Exception as exc:
        print(f"Manual UI resume: ignored metadata because it could not be read: {exc}")
        return None

    source = str(meta.get("source", ""))
    keypoint_reference = str(meta.get("keypoint_reference", ""))
    if source != "manual_raceline_generator" and "manual_centerline_index" not in keypoint_reference:
        # Do not reinterpret the automatic raceline editor's files as a manual
        # lateral-offset session.
        return None

    selected_direction = str(meta.get("selected_direction", "normal")).lower()
    if selected_direction not in ("normal", "reverse"):
        selected_direction = "normal"

    if MANUAL_RESUME_IF_NEWER_THAN_CENTERLINE and os.path.exists(centerline_csv_path):
        if os.path.getmtime(edited_path) + 1e-6 < os.path.getmtime(centerline_csv_path):
            print("Manual UI resume: previous manual session is older than the current centerline; starting fresh.")
            return None

    active_centerline_rows = centerline_rows
    if selected_direction == "reverse":
        active_centerline_rows = rg.reverse_centerline_rows_for_raceline_editor(centerline_rows)

    try:
        corners = rg.load_corner_keypoints_csv(edited_path)
    except Exception as exc:
        print(f"Manual UI resume: failed to load edited corner references: {exc}")
        return None

    # Ensure manual offset fields exist even for files written by the first V1.
    for c in corners:
        c["turn_direction"] = "manual"
        c["turn_sign"] = 0
        for role in ROLE_ORDER:
            c[_offset_key(role)] = 0.0

    # Restore lateral offsets from the saved moved XY points. This is robust to
    # small floating-point differences and does not depend on move_dist_m sign.
    role_from_backend = {v: k for k, v in ROLE_TO_BACKEND.items()}
    by_corner_id = {int(c["corner_id"]): c for c in corners}
    for row in _load_saved_moved_keypoint_rows(moved_path):
        try:
            cid = int(row["corner_id"])
            role = role_from_backend.get(str(row.get("role", "")))
            c = by_corner_id.get(cid)
            if role is None:
                continue
            # Compatibility fallback for an early manual-generator prototype
            # that could save moved rows before corner IDs were refreshed.
            if c is None:
                saved_idx = int(row.get("center_index", -1))
                for candidate in corners:
                    if _corner_role_index(candidate, role) == saved_idx:
                        c = candidate
                        break
            if c is None:
                continue
            _, _, center, normal = _point_center_and_normal(active_centerline_rows, c, role)
            moved = np.array([float(row["x"]), float(row["y"])], dtype=np.float64)
            offset = float(np.dot(moved - center, normal))
            offset, _ = _clamp_point_offset(
                active_centerline_rows, c, role,
                drivable_mask, safety_mask, yaml_data,
                offset,
            )
            c[_offset_key(role)] = offset
        except Exception:
            continue

    _refresh_corner_ids(corners)
    moved_keypoints = build_manual_moved_keypoints(
        active_centerline_rows,
        corners,
        drivable_mask,
        safety_mask,
        yaml_data,
    )

    preview = None
    saved_points = _load_saved_raceline_points(raceline_path)
    if len(saved_points) >= 4:
        inside_ratio, outside_count = rg.validate_points_in_mask(saved_points, safety_mask, yaml_data)
        curvature_ok, curvature_summary, curvature_limit = rg.raceline_curvature_is_ok(saved_points)
        preview = {
            "points": saved_points,
            "moved_keypoints": moved_keypoints,
            "reports": [],
            "raw_offsets": np.array([], dtype=np.float64),
            "final_offsets": np.array([], dtype=np.float64),
            "scale": None,
            "inside_ratio": float(inside_ratio),
            "outside_count": int(outside_count),
            "curvature_ok": bool(curvature_ok),
            "curvature_summary": curvature_summary,
            "curvature_limit": float(curvature_limit),
            "source": "saved",
        }

    print(f"Manual UI resume: restored {len(corners)} manual corners from the previous session.")
    print(f"Manual UI resume: restored driving direction: {selected_direction}")
    if preview is not None:
        print(f"Manual UI resume: loaded existing raceline preview: {raceline_path}")
    return active_centerline_rows, corners, preview, selected_direction

def show_manual_raceline_editor(
    img,
    yaml_data,
    centerline_rows,
    drivable_mask,
    output_dir,
    initial_corners=None,
    initial_preview=None,
    initial_direction="normal",
):
    """
    Manual E/A/X raceline editor with explicit editing modes.

    Modes are mutually exclusive:
      NORMAL          select only; clicking/dragging does not move a point
      ADD             next map click creates a new E/A/X group
      CENTERLINE_MOVE move selected point longitudinally; lateral offset kept
      LATERAL_MOVE    move selected point along local centerline normal

    Right-click finishes the active ADD/CENTERLINE/LATERAL mode while keeping
    the selected point. In NORMAL mode, right-click deselects.
    """
    interactive_ok = rg.try_enable_interactive_matplotlib_backend()
    res = float(yaml_data["resolution"])
    safety_mask, safety_radius_px = rg.build_safety_region_mask(
        drivable_mask,
        res,
        rg.RACELINE_SAFETY_REGION_MARGIN_M,
    )

    MODE_NORMAL = "NORMAL"
    MODE_ADD = "ADD CORNER"
    MODE_CENTERLINE = "CENTERLINE REF MOVE"
    MODE_LATERAL = "LATERAL MOVE"

    state = {
        "corners": [dict(c) for c in (initial_corners or [])],
        "selected": None,       # (corner list index, role E/A/X)
        "mode": MODE_NORMAL,
        "dragging": False,
        "preview": initial_preview,
        "preview_dirty": False if initial_preview is not None else True,
        "selected_direction": (
            str(initial_direction).lower()
            if str(initial_direction).lower() in ("normal", "reverse")
            else "normal"
        ),
        "saved": False,
        "view_initialized": False,
    }
    _refresh_corner_ids(state["corners"])

    extent = rg.get_world_extent(img.shape, yaml_data)
    base = img.astype(np.float32) / 255.0

    fig = plt.figure(figsize=(14.5, 8.5))
    ax = fig.add_axes([0.050, 0.175, 0.685, 0.755])
    info_ax = fig.add_axes([0.755, 0.175, 0.225, 0.755])
    try:
        fig.canvas.manager.set_window_title("Manual Raceline Generator")
    except Exception:
        pass

    def toolbar_active():
        toolbar = getattr(fig.canvas, "toolbar", None)
        return bool(getattr(toolbar, "mode", "")) if toolbar is not None else False

    def set_mode(mode, keep_selection=True):
        state["mode"] = mode
        state["dragging"] = False
        if not keep_selection:
            state["selected"] = None

    def mark_dirty():
        state["preview_dirty"] = True

    def selected_item():
        if state["selected"] is None:
            return None
        ci, role = state["selected"]
        if ci < 0 or ci >= len(state["corners"]):
            return None
        return ci, role, state["corners"][ci]

    def current_limits():
        item = selected_item()
        if item is None:
            return None
        _, role, c = item
        return _lateral_limits(centerline_rows, c, role, drivable_mask, safety_mask, yaml_data)

    def clamp_selected_offset(requested):
        item = selected_item()
        if item is None:
            return
        _, role, c = item
        value, _ = _clamp_point_offset(
            centerline_rows,
            c,
            role,
            drivable_mask,
            safety_mask,
            yaml_data,
            requested,
        )
        c[_offset_key(role)] = value
        mark_dirty()

    def mode_short():
        return state["mode"]

    def draw_info():
        info_ax.clear()
        info_ax.axis("off")
        lines = [
            "Manual raceline editor",
            "",
            f"Corners: {len(state['corners'])}",
            f"Direction: {state['selected_direction'].upper()}",
            f"Mode: {mode_short()}",
            f"Safety margin: {rg.RACELINE_SAFETY_REGION_MARGIN_M:.2f} m",
            "",
        ]

        item = selected_item()
        if item is None:
            lines += ["Selected: none", ""]
        else:
            _, role, c = item
            idx = _corner_role_index(c, role)
            offset = float(c.get(_offset_key(role), 0.0))
            limits = current_limits()
            lines += [
                f"Selected: C{int(c['corner_id']):02d} {role}",
                f"Center idx: {idx}",
                f"Lateral offset: {offset:+.3f} m",
                f"Raw right/left: {-limits['raw_min']:.3f} / {limits['raw_max']:.3f} m",
                f"Safe right/left: {-limits['safe_min']:.3f} / {limits['safe_max']:.3f} m",
                "(+ offset = LEFT)",
                "",
            ]

        p = state.get("preview")
        if p is not None:
            cs = p["curvature_summary"]
            scale = p.get("scale")
            scale_text = "saved" if scale is None else f"{float(scale):.3f}"
            lines += [
                "Last preview:",
                f"  scale: {scale_text}",
                f"  inside: {p['inside_ratio']:.3f}",
                f"  outside: {p['outside_count']}",
                f"  kappa p99: {cs['p_abs_curvature']:.3f} 1/m",
                f"  kappa max: {cs['max_abs_curvature']:.3f} 1/m",
                f"  limit: {p['curvature_limit']:.3f} 1/m",
                f"  accepted: {p['curvature_ok']}",
                "",
            ]

        lines += ["Current-mode controls:"]
        if state["mode"] == MODE_NORMAL:
            lines += [
                "  Left-click E/A/X = select only",
                "  Right-click = deselect",
            ]
        elif state["mode"] == MODE_ADD:
            lines += [
                "  Left-click centerline = add E/A/X",
                "  Right-click = cancel Add mode",
            ]
        elif state["mode"] == MODE_CENTERLINE:
            lines += [
                "  Drag selected point = move along centerline",
                "  Up/Down = centerline index +/-1",
                "  Existing lateral offset is preserved",
                "  Right-click = finish mode",
            ]
        elif state["mode"] == MODE_LATERAL:
            lines += [
                "  Drag selected point = move laterally",
                f"  Left/Right = lateral +/-{MANUAL_LATERAL_KEY_STEP_M:.2f} m",
                "  Right-click = finish mode",
            ]

        lines += [
            "",
            "Other controls:",
            "  Flip Direction = reverse CSV driving order",
            "  Reset Lateral = selected offset 0",
            "  Delete/d = clear selected corner",
            "  Enter = Preview Raceline",
            "  Mouse wheel = zoom at cursor",
            "  Toolbar = pan / zoom / home",
            "  Save = write files and exit",
        ]
        info_ax.text(
            0.0, 1.0, "\n".join(lines),
            transform=info_ax.transAxes,
            va="top", ha="left",
            fontsize=8.6, family="monospace",
            bbox=dict(facecolor="white", alpha=0.93, edgecolor="black", pad=6),
        )

    def draw():
        old_xlim = ax.get_xlim() if state["view_initialized"] else None
        old_ylim = ax.get_ylim() if state["view_initialized"] else None
        ax.clear()

        ax.imshow(base, cmap="gray", origin="lower", extent=extent)
        safety_overlay = np.ma.masked_where(safety_mask <= 0, safety_mask)
        ax.imshow(safety_overlay, cmap="Greens", origin="lower", extent=extent, alpha=0.14)

        cpts = rg.rows_xy_array(centerline_rows)
        ax.plot(cpts[:, 0], cpts[:, 1], linewidth=1.2, color="tab:blue", label="centerline")
        rg.draw_direction_arrows_on_path(
            ax, cpts,
            stride=max(12, len(cpts) // 36),
            color="tab:blue",
            label="direction",
            zorder=4,
        )

        used = set()
        for ci, c in enumerate(state["corners"]):
            for role in ROLE_ORDER:
                x, y = _point_world_xy(centerline_rows, c, role)
                _, _, center, _ = _point_center_and_normal(centerline_rows, c, role)
                marker, color, role_name = ROLE_MARKER[role]
                selected = state["selected"] == (ci, role)
                label = role_name if role_name not in used else None
                used.add(role_name)
                size = 180 if role == "A" else 110
                if selected:
                    size *= 1.35
                ax.plot([center[0], x], [center[1], y], linestyle=":", linewidth=1.0, color=color, alpha=0.8)
                ax.scatter(
                    [x], [y], s=size, marker=marker, color=color,
                    edgecolors="yellow" if selected else "black",
                    linewidths=2.0 if selected else 0.8,
                    label=label, zorder=6,
                )
                ax.text(
                    x, y, f"C{int(c['corner_id'])}{role}", fontsize=8,
                    bbox=dict(facecolor="white", alpha=0.80, edgecolor="none", pad=1.1),
                    zorder=7,
                )

        # Emphasize the centerline reference of the selected off-center point.
        item = selected_item()
        if item is not None:
            _, role, c = item
            _, _, center, normal = _point_center_and_normal(centerline_rows, c, role)
            ax.scatter([center[0]], [center[1]], s=55, marker="+", color="black", linewidths=1.4, zorder=8)

            if state["mode"] == MODE_LATERAL:
                limits = current_limits()
                raw_a = center + limits["raw_min"] * normal
                raw_b = center + limits["raw_max"] * normal
                safe_a = center + limits["safe_min"] * normal
                safe_b = center + limits["safe_max"] * normal
                ax.plot(
                    [raw_a[0], raw_b[0]], [raw_a[1], raw_b[1]],
                    "--", linewidth=1.2, color="tab:gray", label="physical lateral limit",
                )
                ax.plot(
                    [safe_a[0], safe_b[0]], [safe_a[1], safe_b[1]],
                    linewidth=3.0, alpha=0.65, color="tab:green", label="allowed safe range",
                )

        p = state.get("preview")
        if p is not None and len(p["points"]) > 0:
            rp = np.asarray(p["points"], dtype=np.float64)
            ax.plot(rp[:, 0], rp[:, 1], linewidth=2.2, color="tab:red", label="preview raceline", zorder=5)

        ax.set_title(
            "Manual Raceline Generator\n"
            f"Direction={state['selected_direction'].upper()} | "
            f"Mode={mode_short()} | Preview={'stale' if state['preview_dirty'] else 'current'}"
        )
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.axis("equal")
        ax.grid(True, linewidth=0.3, alpha=0.35)
        ax.legend(loc="best")
        draw_info()

        if old_xlim is not None and old_ylim is not None:
            ax.set_xlim(old_xlim)
            ax.set_ylim(old_ylim)
        else:
            state["view_initialized"] = True
        fig.canvas.draw_idle()

    def pick_manual_point(event):
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return None
        q = np.array([float(event.xdata), float(event.ydata)], dtype=np.float64)
        best = None
        best_d = None
        for ci, c in enumerate(state["corners"]):
            for role in ROLE_ORDER:
                p = np.array(_point_world_xy(centerline_rows, c, role), dtype=np.float64)
                d = float(np.linalg.norm(q - p))
                if best_d is None or d < best_d:
                    best_d = d
                    best = (ci, role)
        if best_d is not None and best_d <= float(MANUAL_KEYPOINT_PICK_RADIUS_M):
            return best
        return None

    def add_corner_from_event(event):
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        idx = rg.nearest_centerline_index(centerline_rows, event.xdata, event.ydata)
        c = _make_manual_corner(centerline_rows, idx)
        state["corners"].append(c)
        _refresh_corner_ids(state["corners"])
        sel = None
        for ci, cc in enumerate(state["corners"]):
            if cc is c:
                sel = (ci, "A")
                break
        state["selected"] = sel
        set_mode(MODE_NORMAL, keep_selection=True)
        mark_dirty()
        print(f"Manual UI: added corner around apex index {idx}; apex selected.")
        draw()

    def _restore_selected_after_sort(c, role):
        for new_ci, cc in enumerate(state["corners"]):
            if cc is c:
                state["selected"] = (new_ci, role)
                return
        state["selected"] = None

    def move_selected_along_centerline(event):
        item = selected_item()
        if item is None or event.xdata is None or event.ydata is None:
            return
        _, role, c = item
        idx = rg.nearest_centerline_index(centerline_rows, event.xdata, event.ydata)
        _set_corner_role_index(c, role, idx, centerline_rows, preserve_offset=True)
        clamp_selected_offset(c.get(_offset_key(role), 0.0))
        _refresh_corner_ids(state["corners"])
        _restore_selected_after_sort(c, role)
        mark_dirty()
        draw()

    def step_selected_along_centerline(delta):
        item = selected_item()
        if item is None:
            print("Manual UI: select E/A/X first.")
            return
        _, role, c = item
        old_idx = _corner_role_index(c, role)
        new_idx = (old_idx + int(delta)) % len(centerline_rows)
        _set_corner_role_index(c, role, new_idx, centerline_rows, preserve_offset=True)
        clamp_selected_offset(c.get(_offset_key(role), 0.0))
        _refresh_corner_ids(state["corners"])
        _restore_selected_after_sort(c, role)
        mark_dirty()
        print(
            f"Manual UI: C{int(c['corner_id']):02d} {role} centerline ref "
            f"idx {old_idx} -> {new_idx}; lateral offset preserved."
        )
        draw()

    def move_selected_laterally_from_event(event):
        item = selected_item()
        if item is None or event.xdata is None or event.ydata is None:
            return
        _, role, c = item
        _, _, center, normal = _point_center_and_normal(centerline_rows, c, role)
        q = np.array([float(event.xdata), float(event.ydata)], dtype=np.float64)
        requested = float(np.dot(q - center, normal))
        clamp_selected_offset(requested)
        draw()

    def apply_scroll_zoom(event, scale_factor):
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        xdata = float(event.xdata)
        ydata = float(event.ydata)
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        width = float(xlim[1] - xlim[0])
        height = float(ylim[1] - ylim[0])
        if abs(width) < 1e-12 or abs(height) < 1e-12:
            return
        new_width = width * float(scale_factor)
        new_height = height * float(scale_factor)
        relx = (xdata - xlim[0]) / width
        rely = (ydata - ylim[0]) / height
        ax.set_xlim(xdata - relx * new_width, xdata + (1.0 - relx) * new_width)
        ax.set_ylim(ydata - rely * new_height, ydata + (1.0 - rely) * new_height)
        fig.canvas.draw_idle()

    def on_press(event):
        if event.inaxes != ax or toolbar_active():
            return

        # Right click is the common "finish current mode" operation. The
        # selected point stays selected. In NORMAL mode it acts as deselect.
        if getattr(event, "button", None) == 3:
            if state["mode"] != MODE_NORMAL:
                finished = state["mode"]
                set_mode(MODE_NORMAL, keep_selection=True)
                print(f"Manual UI: finished {finished}; current point position kept.")
            else:
                state["selected"] = None
                state["dragging"] = False
            draw()
            return

        if getattr(event, "button", None) != 1:
            return

        if state["mode"] == MODE_ADD:
            add_corner_from_event(event)
            return

        picked = pick_manual_point(event)

        if state["mode"] == MODE_NORMAL:
            state["selected"] = picked
            state["dragging"] = False
            draw()
            return

        # In a movement mode, only the already-selected point may be dragged.
        # Clicking another point selects it but returns to NORMAL, forcing an
        # explicit movement-mode button press for that new point.
        if picked != state["selected"]:
            if picked is not None:
                state["selected"] = picked
                set_mode(MODE_NORMAL, keep_selection=True)
                print("Manual UI: selected another point; choose a movement mode for it.")
                draw()
            return

        if picked is None:
            return

        state["dragging"] = True
        if state["mode"] == MODE_CENTERLINE:
            move_selected_along_centerline(event)
        elif state["mode"] == MODE_LATERAL:
            move_selected_laterally_from_event(event)

    def on_motion(event):
        if not state["dragging"] or toolbar_active():
            return
        if state["mode"] == MODE_CENTERLINE:
            move_selected_along_centerline(event)
        elif state["mode"] == MODE_LATERAL:
            move_selected_laterally_from_event(event)

    def on_release(event):
        state["dragging"] = False

    def on_scroll(event):
        if event.button == "up":
            apply_scroll_zoom(event, 0.80)
        elif event.button == "down":
            apply_scroll_zoom(event, 1.25)

    def on_key(event):
        if event.key == "up" and state["mode"] == MODE_CENTERLINE and selected_item() is not None:
            step_selected_along_centerline(+MANUAL_CENTERLINE_KEY_STEP_POINTS)
        elif event.key == "down" and state["mode"] == MODE_CENTERLINE and selected_item() is not None:
            step_selected_along_centerline(-MANUAL_CENTERLINE_KEY_STEP_POINTS)
        elif event.key in ("left", "right") and state["mode"] == MODE_LATERAL and selected_item() is not None:
            _, role, c = selected_item()
            current = float(c.get(_offset_key(role), 0.0))
            delta = float(MANUAL_LATERAL_KEY_STEP_M) * (1.0 if event.key == "left" else -1.0)
            clamp_selected_offset(current + delta)
            print(f"Manual UI: lateral offset -> {float(c.get(_offset_key(role), 0.0)):+.3f} m")
            draw()
        elif event.key in ("delete", "backspace", "d", "D"):
            clear_selected_corner()
        elif event.key in ("enter", "return"):
            preview_raceline()
        elif event.key == "escape":
            if state["mode"] != MODE_NORMAL:
                set_mode(MODE_NORMAL, keep_selection=True)
            else:
                state["selected"] = None
            draw()

    def toggle_add():
        if state["mode"] == MODE_ADD:
            set_mode(MODE_NORMAL, keep_selection=True)
            print("Manual UI: Add Corner mode OFF.")
        else:
            set_mode(MODE_ADD, keep_selection=False)
            print("Manual UI: Add Corner mode ON. Click near the centerline.")
        draw()

    def start_centerline_move():
        if selected_item() is None:
            print("Manual UI: select E/A/X first, then press Centerline Ref Move.")
            return
        set_mode(MODE_CENTERLINE, keep_selection=True)
        print("Manual UI: CENTERLINE REF MOVE. Drag selected point or use Up/Down; right-click to finish.")
        draw()

    def start_lateral_move():
        if selected_item() is None:
            print("Manual UI: select E/A/X first, then press Lateral Move.")
            return
        set_mode(MODE_LATERAL, keep_selection=True)
        print("Manual UI: LATERAL MOVE. Drag along the shown line or use Left/Right; right-click to finish.")
        draw()

    def flip_direction():
        """
        Flip the in-memory driving direction while preserving the manually
        positioned geometry as closely as possible.

        E/X swap roles because entrance and exit reverse with driving direction.
        Signed lateral offsets also change sign because the left normal reverses.
        The Phase 9 centerline CSV on disk is never modified.
        """
        set_mode(MODE_NORMAL, keep_selection=True)

        selected_corner = None
        selected_role = None
        item = selected_item()
        if item is not None:
            _, selected_role, selected_corner = item

        # Preserve each role's centerline-reference world point and signed
        # offset before reversing the centerline order.
        saved = []
        for c in state["corners"]:
            role_data = {}
            for role in ROLE_ORDER:
                _, row, _, _ = _point_center_and_normal(centerline_rows, c, role)
                role_data[role] = {
                    "center_xy": (float(row["x"]), float(row["y"])),
                    "offset": float(c.get(_offset_key(role), 0.0)),
                }
            saved.append((c, role_data))

        reversed_rows = rg.reverse_centerline_rows_for_raceline_editor(centerline_rows)
        centerline_rows[:] = [dict(r) for r in reversed_rows]

        # In the reversed driving direction:
        #   new entrance = old exit
        #   new apex     = old apex
        #   new exit     = old entrance
        # and +left/-right changes sign because yaw reverses by pi.
        source_role_for_new = {"E": "X", "A": "A", "X": "E"}
        for c, role_data in saved:
            for new_role in ROLE_ORDER:
                old_role = source_role_for_new[new_role]
                ref_x, ref_y = role_data[old_role]["center_xy"]
                new_idx = rg.nearest_centerline_index(centerline_rows, ref_x, ref_y)
                _set_corner_role_index(
                    c, new_role, new_idx, centerline_rows, preserve_offset=False
                )
                requested = -float(role_data[old_role]["offset"])
                clamped, _ = _clamp_point_offset(
                    centerline_rows, c, new_role,
                    drivable_mask, safety_mask, yaml_data,
                    requested,
                )
                c[_offset_key(new_role)] = clamped

        _refresh_corner_ids(state["corners"])

        # Keep the same physical selected point selected. E and X swap roles.
        if selected_corner is not None:
            new_selected_role = {"E": "X", "A": "A", "X": "E"}[selected_role]
            state["selected"] = None
            for ci, c in enumerate(state["corners"]):
                if c is selected_corner:
                    state["selected"] = (ci, new_selected_role)
                    break

        state["selected_direction"] = (
            "reverse" if state["selected_direction"] == "normal" else "normal"
        )

        # A preview generated in the old direction is no longer current.
        state["preview"] = None
        mark_dirty()
        print(
            f"Manual UI: driving direction flipped to "
            f"{state['selected_direction'].upper()}. "
            "E/X roles and lateral offset signs were remapped."
        )
        draw()

    def reset_lateral():
        item = selected_item()
        if item is None:
            print("Manual UI: select E/A/X first.")
            return
        _, role, c = item
        c[_offset_key(role)] = 0.0
        mark_dirty()
        print("Manual UI: selected point lateral offset reset to 0.000 m; centerline index unchanged.")
        draw()

    def clear_selected_corner():
        item = selected_item()
        if item is None:
            print("Manual UI: no selected corner to clear.")
            return
        ci, _, c = item
        cid = int(c["corner_id"])
        del state["corners"][ci]
        _refresh_corner_ids(state["corners"])
        state["selected"] = None
        set_mode(MODE_NORMAL, keep_selection=True)
        mark_dirty()
        print(f"Manual UI: cleared corner C{cid:02d}.")
        draw()

    def clear_all():
        state["corners"].clear()
        state["selected"] = None
        set_mode(MODE_NORMAL, keep_selection=True)
        mark_dirty()
        print("Manual UI: cleared all corners. Final raceline will equal centerline unless new points are added.")
        draw()

    def preview_raceline():
        set_mode(MODE_NORMAL, keep_selection=True)
        print("Manual UI: generating raceline preview...")
        state["preview"] = generate_preview(
            centerline_rows,
            state["corners"],
            drivable_mask,
            safety_mask,
            yaml_data,
        )
        state["preview"]["source"] = "generated"
        state["preview_dirty"] = False
        p = state["preview"]
        cs = p["curvature_summary"]
        print(
            f"  preview: points={len(p['points'])}, scale={p['scale']:.3f}, "
            f"inside={p['inside_ratio']:.3f}, outside={p['outside_count']}, "
            f"kappa_p99={cs['p_abs_curvature']:.3f}, "
            f"kappa_max={cs['max_abs_curvature']:.3f}, "
            f"limit={p['curvature_limit']:.3f}, accepted={p['curvature_ok']}"
        )
        draw()

    def save_and_close():
        set_mode(MODE_NORMAL, keep_selection=True)
        if state["preview"] is None or state["preview_dirty"]:
            preview_raceline()
        save_manual_outputs(
            output_dir,
            centerline_rows,
            state["corners"],
            state["preview"],
            selected_direction=state["selected_direction"],
        )

        try:
            rg.overlay_raceline_with_direction(
                img,
                yaml_data,
                centerline_rows,
                state["preview"]["moved_keypoints"],
                state["preview"]["points"],
                safety_mask,
                os.path.join(output_dir, MANUAL_PREVIEW_PNG),
                selected_direction=state["selected_direction"],
            )
            rg.plot_raceline_curvature(
                state["preview"]["points"],
                os.path.join(output_dir, rg.DEBUG_RACELINE_CURVATURE),
            )
        except Exception as exc:
            print(f"Manual UI: warning: debug-image export failed: {exc}")

        state["saved"] = True
        plt.close(fig)

    def cancel():
        print("Manual UI: cancelled; no manual raceline outputs saved by this UI session.")
        plt.close(fig)

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("scroll_event", on_scroll)

    # Group labels and buttons: corner/direction | point movement | raceline/output.
    fig.text(0.180, 0.125, "Corner / direction", ha="center", va="bottom", fontsize=9, weight="bold")
    fig.text(0.555, 0.125, "Selected point movement", ha="center", va="bottom", fontsize=9, weight="bold")
    fig.text(0.875, 0.125, "Raceline / output", ha="center", va="bottom", fontsize=9, weight="bold")

    ax_add = plt.axes([0.020, 0.050, 0.075, 0.055])
    ax_clear = plt.axes([0.100, 0.050, 0.080, 0.055])
    ax_clearall = plt.axes([0.185, 0.050, 0.070, 0.055])
    ax_flip = plt.axes([0.260, 0.050, 0.100, 0.055])

    ax_centerline = plt.axes([0.375, 0.050, 0.145, 0.055])
    ax_lateral = plt.axes([0.525, 0.050, 0.105, 0.055])
    ax_reset = plt.axes([0.635, 0.050, 0.120, 0.055])

    ax_preview = plt.axes([0.775, 0.050, 0.115, 0.055])
    ax_save = plt.axes([0.895, 0.050, 0.045, 0.055])
    ax_cancel = plt.axes([0.945, 0.050, 0.050, 0.055])

    b_add = Button(ax_add, "Add Corner")
    b_clear = Button(ax_clear, "Clear Corner")
    b_clearall = Button(ax_clearall, "Clear All")
    b_flip = Button(ax_flip, "Flip Direction")
    b_centerline = Button(ax_centerline, "Centerline Ref Move")
    b_lateral = Button(ax_lateral, "Lateral Move")
    b_reset = Button(ax_reset, "Reset Lateral")
    b_preview = Button(ax_preview, "Preview Raceline")
    b_save = Button(ax_save, "Save")
    b_cancel = Button(ax_cancel, "Cancel")

    b_add.on_clicked(lambda event: toggle_add())
    b_clear.on_clicked(lambda event: clear_selected_corner())
    b_clearall.on_clicked(lambda event: clear_all())
    b_flip.on_clicked(lambda event: flip_direction())
    b_centerline.on_clicked(lambda event: start_centerline_move())
    b_lateral.on_clicked(lambda event: start_lateral_move())
    b_reset.on_clicked(lambda event: reset_lateral())
    b_preview.on_clicked(lambda event: preview_raceline())
    b_save.on_clicked(lambda event: save_and_close())
    b_cancel.on_clicked(lambda event: cancel())
    state["buttons"] = [
        b_add, b_clear, b_clearall, b_flip,
        b_centerline, b_lateral, b_reset,
        b_preview, b_save, b_cancel,
    ]

    print("Manual raceline editor")
    print(f"  Centerline points:       {len(centerline_rows)}")
    print(f"  Raceline safety margin:  {rg.RACELINE_SAFETY_REGION_MARGIN_M:.3f} m ({safety_radius_px} px erosion)")
    print(f"  Driving direction:       {state['selected_direction'].upper()} (relative to centerline CSV order)")
    print("  Flip Direction reverses the in-memory centerline; the Phase 9 CSV is unchanged.")
    print("  NORMAL: click E/A/X to select only; no accidental point movement.")
    print("  Centerline Ref Move: drag selected point or Up/Down; lateral offset is preserved.")
    print("  Lateral Move: drag along local normal or Left/Right; Reset Lateral returns offset to zero.")
    print("  Right-click: finish current movement/add mode while keeping the point selected.")
    print("  Mouse wheel zooms around the cursor; Matplotlib toolbar pan/zoom/home remains available.")
    print("  Preview Raceline uses the current raceline_generator.py backend.")
    print("  Save writes standard raceline outputs and exits.")
    print("  With zero corners, saved raceline equals the centerline.")

    draw()

    if interactive_ok:
        try:
            plt.show(block=True)
        except Exception as exc:
            fallback = os.path.join(output_dir, MANUAL_EDIT_FALLBACK_PNG)
            fig.savefig(fallback, dpi=160)
            print(f"Manual UI: interactive display failed: {exc}")
            print(f"Manual UI: saved fallback PNG: {fallback}")
            plt.close(fig)
    else:
        fallback = os.path.join(output_dir, MANUAL_EDIT_FALLBACK_PNG)
        fig.savefig(fallback, dpi=160)
        print("Manual UI: no interactive matplotlib backend available.")
        print(f"Manual UI: saved fallback PNG: {fallback}")
        plt.close(fig)

    return bool(state["saved"])

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 manual_raceline_generator.py <map_image> <map.yaml> [--fresh]")
        print("Note: Phase 9 outputs must already exist in centerline_output/.")
        print("Default standalone behavior: resume the last compatible manual session when available.")
        return

    map_path, yaml_path = sys.argv[1], sys.argv[2]
    resume_existing = True
    for arg in sys.argv[3:]:
        if arg in ("--fresh", "--no-resume"):
            resume_existing = False
        elif arg == "--resume":
            resume_existing = True
        else:
            raise RuntimeError(f"Unknown optional argument: {arg}")

    rg.ensure_output_dir(rg.OUTPUT_DIR)
    paths = rg.resolve_centerline_output_paths(rg.OUTPUT_DIR)
    rg.ensure_required_phase9_outputs(paths)

    img, meta = rg.load_map(map_path, yaml_path)
    centerline_rows = rg.load_centerline_geometry_csv(paths["centerline_csv"])
    drivable_mask = np.load(paths["drivable_mask"]).astype(np.uint8)
    res = float(meta["resolution"])
    safety_mask, _ = rg.build_safety_region_mask(
        drivable_mask,
        res,
        rg.RACELINE_SAFETY_REGION_MARGIN_M,
    )

    print("Loaded Phase 9 reference data for manual raceline generation")
    print(f"  Centerline CSV:       {paths['centerline_csv']}")
    print(f"  Centerline points:    {len(centerline_rows)}")
    print(f"  Drivable mask:        {paths['drivable_mask']}")
    print(f"  Map shape:            {img.shape}, resolution={res:.3f} m/pixel")

    initial_corners = []
    initial_preview = None
    initial_direction = "normal"
    if resume_existing:
        resumed = load_manual_session_if_available(
            rg.OUTPUT_DIR,
            paths["centerline_csv"],
            centerline_rows,
            drivable_mask,
            safety_mask,
            meta,
        )
        if resumed is not None:
            centerline_rows, initial_corners, initial_preview, initial_direction = resumed
        else:
            print("Manual UI resume: no compatible recent manual session found; starting fresh.")
    else:
        print("Manual UI: --fresh selected; starting with no manual corners.")

    saved = show_manual_raceline_editor(
        img,
        meta,
        centerline_rows,
        drivable_mask,
        rg.OUTPUT_DIR,
        initial_corners=initial_corners,
        initial_preview=initial_preview,
        initial_direction=initial_direction,
    )

    if saved:
        print("Manual raceline generation completed successfully.")
    else:
        print("Manual raceline generator closed without Save.")


if __name__ == "__main__":
    main()