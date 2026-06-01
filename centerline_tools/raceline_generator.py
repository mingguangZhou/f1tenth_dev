#!/usr/bin/env python3
"""
Separated Phase 10-12 raceline generator.

Inputs are the Phase 9 outputs from centerline_reference_generator.py:
  - centerline_points_smooth.csv
  - centerline_metadata.yaml
  - drivable_region.npy

This file intentionally reuses helpers/constants from centerline_reference_generator.py
so the centerline/skeleton pipeline is not recalculated.
"""

from centerline_reference_generator import *

from matplotlib.widgets import Button

# =========================
# PHASE 0 UI CONFIG
# =========================
# Review-only UI. It does not modify keypoints or change any exported CSV.
# Enable from command line with:
#   python3 raceline_generator.py <map_image> <map.yaml> --review-keypoints
REVIEW_CORNER_KEYPOINTS_UI = False
KEYPOINT_REVIEW_FALLBACK_PNG = "debug_corner_keypoints_review_ui.png"

# Phase 1+ edit UI. Default ON so run_centerline_and_raceline.py can call normally.
EDIT_CORNER_KEYPOINTS_UI = True
EDITED_CORNERS_CSV_NAME = "corner_key_points_edited.csv"
KEYPOINT_EDIT_FALLBACK_PNG = "debug_corner_keypoints_edit_ui.png"
KEYPOINT_PICK_RADIUS_M = 0.45
KEYPOINT_KEYBOARD_STEP_POINTS = 1
NEW_CORNER_HALF_SPAN_POINTS = 20
DEBUG_EDITED_CORNER_KEYPOINTS = "debug_edited_corner_keypoints.png"
DEBUG_RACELINE_WITH_DIRECTION = "debug_raceline_with_direction.png"
EDITED_CORNERS_METADATA_YAML_NAME = "corner_key_points_edited_metadata.yaml"
RESUME_KEYPOINTS_IF_NEWER_THAN_CENTERLINE = True





# =========================
# PHASE 0: REVIEW-ONLY KEYPOINT UI
# =========================
def try_enable_interactive_matplotlib_backend():
    """
    The centerline reference module uses Agg for normal batch/debug output.
    For the review UI, try to switch to a GUI backend. If no GUI backend is
    available, the caller will save a clear fallback PNG instead.
    """
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


def show_corner_keypoint_review_ui(img, yaml_data, centerline_rows, corners, output_dir):
    """
    Phase 0 UI: display detected entrance/apex/exit keypoints clearly.

    This version is intentionally read-only:
      - no moving
      - no adding
      - no deleting
      - no effect on saved corner/moved/raceline CSV files

    Purpose:
      Validate that a GUI window can open in your environment and that the
      visual encoding of keypoints is clear enough before implementing editing.
    """
    if len(corners) == 0:
        print("Phase 0 UI: skipped; no detected corners to display.")
        return

    interactive_ok = try_enable_interactive_matplotlib_backend()

    extent = get_world_extent(img.shape, yaml_data)
    base = img.astype(np.float32) / 255.0
    cpts = np.array([[r["x"], r["y"]] for r in centerline_rows], dtype=np.float64)

    fig, ax = plt.subplots(figsize=(11, 8))
    fig.canvas.manager.set_window_title("Raceline Keypoint Review UI - Phase 0")

    ax.imshow(base, cmap="gray", origin="lower", extent=extent)
    ax.plot(cpts[:, 0], cpts[:, 1], linewidth=1.2, color="tab:blue", label="smoothed centerline")

    # Optional: show corner centerline segments with light highlight.
    n = len(centerline_rows)
    for c in corners:
        start = int(c["entrance_index"])
        end = int(c["exit_index"])
        if start <= end:
            idxs = list(range(start, end + 1))
        else:
            idxs = list(range(start, n)) + list(range(0, end + 1))

        seg = cpts[idxs]
        ax.plot(seg[:, 0], seg[:, 1], linewidth=3.0, alpha=0.35, color="tab:orange")

    for c in corners:
        cid = int(c["corner_id"])
        direction = str(c["turn_direction"])

        ex, ey = float(c["entrance_x"]), float(c["entrance_y"])
        axp, ayp = float(c["apex_x"]), float(c["apex_y"])
        xx, xy = float(c["exit_x"]), float(c["exit_y"])

        ax.scatter([ex], [ey], s=80, marker="^", color="tab:green", edgecolors="black",
                   label="entrance" if cid == 0 else None, zorder=4)
        ax.scatter([axp], [ayp], s=130, marker="*", color="tab:red", edgecolors="black",
                   label="apex" if cid == 0 else None, zorder=5)
        ax.scatter([xx], [xy], s=80, marker="s", color="tab:purple", edgecolors="black",
                   label="exit" if cid == 0 else None, zorder=4)

        ax.text(ex, ey, f"C{cid} E", fontsize=8, color="black",
                bbox=dict(facecolor="white", alpha=0.75, edgecolor="none", pad=1.5))
        ax.text(axp, ayp, f"C{cid} A {direction}", fontsize=8, color="black",
                bbox=dict(facecolor="white", alpha=0.85, edgecolor="none", pad=1.5))
        ax.text(xx, xy, f"C{cid} X", fontsize=8, color="black",
                bbox=dict(facecolor="white", alpha=0.75, edgecolor="none", pad=1.5))

    ax.set_title(
        "Phase 0 Review UI: detected corner keypoints only\\n"
        "E=entrance, A=apex, X=exit. Close the window to continue."
    )
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.axis("equal")
    ax.grid(True, linewidth=0.3, alpha=0.35)
    ax.legend(loc="best")

    instruction = (
        "Phase 0 is read-only.\\n"
        "Close window to continue.\\n"
        "Next phase: click/select/edit."
    )
    ax.text(
        0.01, 0.01, instruction,
        transform=ax.transAxes,
        fontsize=9,
        va="bottom",
        ha="left",
        bbox=dict(facecolor="white", alpha=0.85, edgecolor="black", pad=4)
    )

    plt.tight_layout()

    if interactive_ok:
        print("Phase 0 UI: opening interactive keypoint review window.")
        print("  Close the window to continue Phase 11/12.")
        try:
            plt.show(block=True)
        except Exception as exc:
            print(f"Phase 0 UI: interactive display failed: {exc}")
            fallback_path = os.path.join(output_dir, KEYPOINT_REVIEW_FALLBACK_PNG)
            fig.savefig(fallback_path, dpi=160)
            print(f"Phase 0 UI: saved fallback PNG: {fallback_path}")
    else:
        fallback_path = os.path.join(output_dir, KEYPOINT_REVIEW_FALLBACK_PNG)
        fig.savefig(fallback_path, dpi=160)
        print("Phase 0 UI: no interactive matplotlib backend available.")
        print(f"Phase 0 UI: saved fallback PNG instead: {fallback_path}")

    plt.close(fig)



# =========================
# PHASE 1: EDIT EXISTING KEYPOINT UI
# =========================
def rows_xy_array(centerline_rows):
    return np.array([[float(r["x"]), float(r["y"])] for r in centerline_rows], dtype=np.float64)


def nearest_centerline_index(centerline_rows, x, y):
    pts = rows_xy_array(centerline_rows)
    q = np.array([float(x), float(y)], dtype=np.float64)
    d2 = np.sum((pts - q) ** 2, axis=1)
    return int(np.argmin(d2))


def reverse_centerline_rows_for_raceline_editor(centerline_rows):
    """
    Reverse track driving direction for the raceline editor.

    The Phase 9 centerline CSV is left unchanged on disk. This function only
    changes the in-memory centerline used by Phase 10-12 in this run. Yaw and
    signed curvature are recomputed after reversing, so downstream normal-offset
    logic remains consistent with the selected direction.
    """
    pts = [(float(r["x"]), float(r["y"])) for r in centerline_rows]
    pts.reverse()
    pts.append(pts[0])
    return compute_yaw_and_curvature(pts)


def update_corner_point_from_index(corner, role_short, index, centerline_rows):
    idx = int(index)
    row = centerline_rows[idx]
    if role_short == "E":
        corner["entrance_index"] = idx
        corner["entrance_x"] = float(row["x"])
        corner["entrance_y"] = float(row["y"])
    elif role_short == "A":
        corner["apex_index"] = idx
        corner["apex_x"] = float(row["x"])
        corner["apex_y"] = float(row["y"])
        corner["apex_curvature"] = float(row.get("curvature", 0.0))
        corner["apex_curvature_abs"] = float(abs(row.get("curvature", 0.0)))
        sign = 1 if float(row.get("curvature", 0.0)) >= 0.0 else -1
        corner["turn_sign"] = int(sign)
        corner["turn_direction"] = "left" if sign > 0 else "right"
    elif role_short == "X":
        corner["exit_index"] = idx
        corner["exit_x"] = float(row["x"])
        corner["exit_y"] = float(row["y"])
    else:
        raise ValueError(f"Unknown role_short: {role_short}")

    n = len(centerline_rows)
    idxs = circular_segment_indices(int(corner["entrance_index"]), int(corner["exit_index"]), n)
    corner["num_points"] = int(len(idxs))
    corner["approx_length_m"] = float(len(idxs) * RESAMPLE_SPACING_M)


def refresh_corner_ids(corners):
    for i, c in enumerate(corners):
        c["corner_id"] = int(i)
    return corners


def make_corner_from_apex_index(centerline_rows, apex_index, half_span_points=NEW_CORNER_HALF_SPAN_POINTS):
    """
    Create one editable corner triplet around a user-selected apex index.

    The new triplet is intentionally centerline-index based:
      entrance = apex - half_span
      apex     = clicked/specified index
      exit     = apex + half_span

    The user can then drag E/A/X to refine the triplet before accepting.
    """
    n = len(centerline_rows)
    if n <= 0:
        raise RuntimeError("Cannot create corner from empty centerline.")

    apex_index = int(apex_index) % n
    entrance_index = int((apex_index - int(half_span_points)) % n)
    exit_index = int((apex_index + int(half_span_points)) % n)

    curv = float(centerline_rows[apex_index].get("curvature", 0.0))
    sign = 1 if curv >= 0.0 else -1
    direction = "left" if sign > 0 else "right"

    c = {
        "corner_id": -1,
        "turn_direction": direction,
        "turn_sign": int(sign),
        "entrance_index": entrance_index,
        "apex_index": apex_index,
        "exit_index": exit_index,
        "num_points": 0,
        "approx_length_m": 0.0,
        "apex_curvature": curv,
        "apex_curvature_abs": abs(curv),
        "entrance_x": 0.0,
        "entrance_y": 0.0,
        "apex_x": 0.0,
        "apex_y": 0.0,
        "exit_x": 0.0,
        "exit_y": 0.0,
    }
    update_corner_point_from_index(c, "E", entrance_index, centerline_rows)
    update_corner_point_from_index(c, "A", apex_index, centerline_rows)
    update_corner_point_from_index(c, "X", exit_index, centerline_rows)
    return c


def sort_corners_by_entrance_index(corners):
    return sorted(corners, key=lambda c: int(c["entrance_index"]))


def build_editor_keypoint_list(corners):
    items = []
    for ci, c in enumerate(corners):
        items.append((ci, "E", float(c["entrance_x"]), float(c["entrance_y"])))
        items.append((ci, "A", float(c["apex_x"]), float(c["apex_y"])))
        items.append((ci, "X", float(c["exit_x"]), float(c["exit_y"])))
    return items


def save_editor_corners_csv(corners, output_dir, selected_direction="normal"):
    path = os.path.join(output_dir, EDITED_CORNERS_CSV_NAME)
    direction = str(selected_direction).lower()
    if direction not in ("normal", "reverse"):
        direction = "normal"

    # Save edited CSV with extra direction columns. csv.DictReader users that
    # only expect the old columns still work; the extra columns make direction
    # visible and recoverable even if the YAML metadata is missed.
    base_fieldnames = [
        "corner_id", "turn_direction", "turn_sign",
        "entrance_index", "apex_index", "exit_index",
        "num_points", "approx_length_m",
        "apex_curvature", "apex_curvature_abs",
        "entrance_x", "entrance_y", "apex_x", "apex_y", "exit_x", "exit_y",
    ]
    fieldnames = base_fieldnames + ["selected_direction", "keypoint_reference"]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for c in corners:
            row = {k: c.get(k, "") for k in base_fieldnames}
            row["selected_direction"] = direction
            row["keypoint_reference"] = "world_xy_plus_direction_specific_indices"
            writer.writerow(row)

    metadata_path = os.path.join(output_dir, EDITED_CORNERS_METADATA_YAML_NAME)
    metadata = {
        "selected_direction": direction,
        "corner_count": int(len(corners)),
        "source": "manual_editor",
        "keypoint_reference": "world_xy_plus_direction_specific_indices",
        "resume_rule": "restore_direction_then_snap_saved_world_xy_to_active_centerline",
    }
    with open(metadata_path, "w") as f:
        yaml.safe_dump(metadata, f, sort_keys=False)

    direction_txt_path = os.path.join(output_dir, "raceline_selected_direction.txt")
    with open(direction_txt_path, "w") as f:
        f.write(direction + "\n")

    print(f"Phase editor UI: saved edited corner CSV: {path}")
    print(f"Phase editor UI: saved edited corner metadata: {metadata_path}")
    print(f"Phase editor UI: saved selected direction: {direction_txt_path} -> {direction}")
    return path

def load_corner_keypoints_csv(csv_path):
    corners = []
    embedded_direction = None
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if embedded_direction is None and "selected_direction" in row:
                d = str(row.get("selected_direction", "normal")).lower()
                if d in ("normal", "reverse"):
                    embedded_direction = d
            corners.append({
                "corner_id": int(row["corner_id"]),
                "turn_direction": str(row["turn_direction"]),
                "turn_sign": int(row["turn_sign"]),
                "entrance_index": int(row["entrance_index"]),
                "apex_index": int(row["apex_index"]),
                "exit_index": int(row["exit_index"]),
                "num_points": int(row["num_points"]),
                "approx_length_m": float(row["approx_length_m"]),
                "apex_curvature": float(row["apex_curvature"]),
                "apex_curvature_abs": float(row["apex_curvature_abs"]),
                "entrance_x": float(row["entrance_x"]),
                "entrance_y": float(row["entrance_y"]),
                "apex_x": float(row["apex_x"]),
                "apex_y": float(row["apex_y"]),
                "exit_x": float(row["exit_x"]),
                "exit_y": float(row["exit_y"]),
            })
    return refresh_corner_ids(sort_corners_by_entrance_index(corners))

def load_edited_corner_metadata(output_dir):
    metadata_path = os.path.join(output_dir, EDITED_CORNERS_METADATA_YAML_NAME)
    if not os.path.exists(metadata_path):
        return {}
    try:
        with open(metadata_path, "r") as f:
            data = yaml.safe_load(f) or {}
        return data if isinstance(data, dict) else {}
    except Exception as exc:
        print(f"Phase editor UI: warning: failed to read edited corner metadata: {exc}")
        return {}


def validate_loaded_corners_against_centerline(corners, centerline_rows):
    n = len(centerline_rows)
    if n <= 0 or len(corners) == 0:
        return False
    required = ("entrance_index", "apex_index", "exit_index")
    for c in corners:
        for key in required:
            idx = int(c.get(key, -1))
            if idx < 0 or idx >= n:
                return False
    return True


def update_corner_coordinates_from_indices(corners, centerline_rows):
    for c in corners:
        update_corner_point_from_index(c, "E", int(c["entrance_index"]), centerline_rows)
        update_corner_point_from_index(c, "A", int(c["apex_index"]), centerline_rows)
        update_corner_point_from_index(c, "X", int(c["exit_index"]), centerline_rows)
    return refresh_corner_ids(sort_corners_by_entrance_index(corners))


def update_corner_indices_from_saved_world_xy(corners, centerline_rows):
    """
    Rebuild corner indices by snapping saved world-coordinate E/A/X points to
    the currently active centerline direction.

    This is more robust than trusting saved indices directly, because if the
    previous editor session accepted a reversed driving direction, those indices
    are meaningful only after the in-memory centerline has also been reversed.
    The saved x/y coordinates are direction-independent, so they are the safest
    resume anchor.
    """
    fixed = []
    for c in corners:
        cc = dict(c)
        e_idx = nearest_centerline_index(centerline_rows, cc["entrance_x"], cc["entrance_y"])
        a_idx = nearest_centerline_index(centerline_rows, cc["apex_x"], cc["apex_y"])
        x_idx = nearest_centerline_index(centerline_rows, cc["exit_x"], cc["exit_y"])
        update_corner_point_from_index(cc, "E", e_idx, centerline_rows)
        update_corner_point_from_index(cc, "A", a_idx, centerline_rows)
        update_corner_point_from_index(cc, "X", x_idx, centerline_rows)
        fixed.append(cc)
    return refresh_corner_ids(sort_corners_by_entrance_index(fixed))


def load_recent_keypoints_if_available(output_dir, centerline_csv_path, centerline_rows):
    """
    Resume manual work when raceline_generator.py is run alone.

    Important fix:
      Prefer corner_key_points_edited.csv over corner_key_points.csv.
      main() overwrites corner_key_points.csv after the editor accepts, so the
      detected/active CSV can be newer than the edited CSV. If we simply choose
      the newest file, the saved selected_direction is lost and reverse resume
      falls back to normal.
    """
    if not RESUME_KEYPOINTS_IF_NEWER_THAN_CENTERLINE:
        return None
    if not os.path.exists(centerline_csv_path):
        return None

    centerline_mtime = os.path.getmtime(centerline_csv_path)
    edited_csv = os.path.join(output_dir, EDITED_CORNERS_CSV_NAME)
    detected_csv = os.path.join(output_dir, CORNERS_CSV_NAME)

    source = None
    csv_path = None
    if os.path.exists(edited_csv) and os.path.getmtime(edited_csv) > centerline_mtime:
        source = "edited"
        csv_path = edited_csv
    elif os.path.exists(detected_csv) and os.path.getmtime(detected_csv) > centerline_mtime:
        source = "detected"
        csv_path = detected_csv
    else:
        return None

    selected_direction = "normal"
    crows = centerline_rows

    try:
        corners = load_corner_keypoints_csv(csv_path)

        if source == "edited":
            meta = load_edited_corner_metadata(output_dir)
            selected_direction = str(meta.get("selected_direction", "normal")).lower()
            if selected_direction not in ("normal", "reverse"):
                selected_direction = "normal"

            if selected_direction == "reverse":
                crows = reverse_centerline_rows_for_raceline_editor(centerline_rows)

            # Do not trust saved indices. They are direction-specific. Saved XY
            # is direction-independent, so restore direction first, then snap XY
            # to the active centerline order.
            corners = update_corner_indices_from_saved_world_xy(corners, crows)
        else:
            if not validate_loaded_corners_against_centerline(corners, crows):
                print(f"Phase editor UI: ignored stale/invalid keypoint CSV: {csv_path}")
                return None
            corners = update_corner_coordinates_from_indices(corners, crows)

        print(f"Phase editor UI: resumed from recent {source} keypoints: {csv_path}")
        print(f"Phase editor UI: resumed selected direction: {selected_direction}")
        if source == "edited":
            print("Phase editor UI: resume restored direction first, then snapped saved XY to active centerline.")
        return crows, corners, selected_direction
    except Exception as exc:
        print(f"Phase editor UI: failed to load recent keypoint CSV {csv_path}: {exc}")
        return None

def _corner_role_index(corner, role_short):
    if role_short == "E":
        return int(corner["entrance_index"])
    if role_short == "A":
        return int(corner["apex_index"])
    if role_short == "X":
        return int(corner["exit_index"])
    raise ValueError(f"Unknown role_short: {role_short}")


def draw_keypoint_table(info_ax, corners, selected, direction_text, add_text):
    """Draw an organized E/A/X index table away from the map."""
    info_ax.axis("off")
    lines = [
        "Keypoint index table",
        f"Direction: {direction_text}",
        f"Add mode: {add_text}",
        "",
        "Corner   E idx   A idx   X idx   turn",
        "------   -----   -----   -----   ----",
    ]
    for ci, c in enumerate(corners):
        prefix = ">" if selected is not None and selected[0] == ci else " "
        lines.append(
            f"{prefix}C{int(c['corner_id']):02d}     "
            f"{int(c['entrance_index']):5d}   "
            f"{int(c['apex_index']):5d}   "
            f"{int(c['exit_index']):5d}   "
            f"{str(c.get('turn_direction', '?')):>5s}"
        )
    lines.extend([
        "",
        "Mouse:",
        "  drag selected point",
        "  right-click: deselect",
        "Keyboard:",
        "  ↑ : index +1",
        "  ↓ : index -1",
        "  d/Delete: delete corner",
        "  n: add-corner mode",
        "  f: flip direction",
        "  Enter: accept",
    ])
    info_ax.text(
        0.0, 1.0, "\n".join(lines),
        transform=info_ax.transAxes,
        va="top", ha="left",
        fontsize=9, family="monospace",
        bbox=dict(facecolor="white", alpha=0.92, edgecolor="black", pad=6)
    )


def show_corner_keypoint_edit_ui(img, yaml_data, centerline_rows, corners, output_dir, initial_direction="normal"):
    """
    Phase 1 UI: edit existing detected E/A/X points and select direction.

    Editing model:
      - User edits keypoint positions by snapping them to nearest centerline index.
      - This is intentional: Phase 11 still owns the lateral inside/outside move.
      - Direction flip reverses the in-memory centerline and reruns detection.

    Controls:
      - left-click near E/A/X: select keypoint
      - drag selected keypoint: move along centerline, snapped to nearest index
      - release mouse: keep current snapped point
      - f or Flip button: reverse driving direction and redetect corners
      - n or Add mode button: toggle add-corner mode; next map click creates E/A/X triplet
      - Delete/Backspace or Delete button: delete selected whole corner triplet
      - r or Reset button: redetect corners for current direction
      - Enter or Accept button: save edited_corner_key_points.csv and continue Phase 11/12
      - Escape: cancel UI edits and continue with original input
    """
    if len(corners) == 0:
        print("Phase 1 UI: skipped; no detected corners to edit.")
        return centerline_rows, corners, "normal", False

    interactive_ok = try_enable_interactive_matplotlib_backend()

    # Keep an untouched fallback copy for Escape/cancel.
    original_centerline_rows = [dict(r) for r in centerline_rows]
    original_corners = [dict(c) for c in corners]

    state = {
        "centerline_rows": [dict(r) for r in centerline_rows],
        "corners": [dict(c) for c in corners],
        "direction_label": str(initial_direction).lower() if str(initial_direction).lower() in ("normal", "reverse") else "normal",
        "selected": None,  # (corner_list_index, role_short)
        "dragging": False,
        "add_mode": False,
        "last_mouse_xy": None,
        "view_initialized": False,
        "accepted": False,
        "cancelled": False,
    }

    extent = get_world_extent(img.shape, yaml_data)
    base = img.astype(np.float32) / 255.0

    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_axes([0.055, 0.16, 0.68, 0.78])
    info_ax = fig.add_axes([0.76, 0.16, 0.22, 0.78])
    try:
        fig.canvas.manager.set_window_title("Raceline Keypoint Editor UI - Phase 1")
    except Exception:
        pass

    def toolbar_is_active():
        toolbar = getattr(fig.canvas, "toolbar", None)
        mode = getattr(toolbar, "mode", "") if toolbar is not None else ""
        return bool(mode)

    def apply_scroll_zoom(event, scale_factor):
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
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

    def draw():
        old_xlim = ax.get_xlim() if state.get("view_initialized", False) else None
        old_ylim = ax.get_ylim() if state.get("view_initialized", False) else None
        ax.clear()
        info_ax.clear()
        crows = state["centerline_rows"]
        cs = state["corners"]
        cpts = rows_xy_array(crows)
        ax.imshow(base, cmap="gray", origin="lower", extent=extent)
        ax.plot(cpts[:, 0], cpts[:, 1], linewidth=1.2, color="tab:blue", label="selected-direction centerline")

        # Clear driving-direction arrows directly on the centerline.
        draw_direction_arrows_on_path(
            ax,
            cpts,
            stride=max(12, len(cpts) // 36),
            color="tab:blue",
            label="driving direction",
            zorder=4,
            head_width=0.18,
            head_length=0.26,
            linewidth=1.2,
            alpha=0.95,
        )
        if len(cpts) > 0:
            ax.scatter([cpts[0, 0]], [cpts[0, 1]], s=80, marker="o", color="tab:blue",
                       edgecolors="white", linewidths=1.0, zorder=5, label="index 0/start")
            ax.text(cpts[0, 0], cpts[0, 1], " idx0", fontsize=8, color="tab:blue",
                    bbox=dict(facecolor="white", alpha=0.78, edgecolor="none", pad=1.0), zorder=6)

        n = len(crows)
        for c in cs:
            start = int(c["entrance_index"])
            end = int(c["exit_index"])
            idxs = circular_segment_indices(start, end, n)
            if len(idxs) > 0:
                seg = cpts[idxs]
                ax.plot(seg[:, 0], seg[:, 1], linewidth=3.0, alpha=0.25, color="tab:orange")

        marker_cfg = {
            "E": ("^", "tab:green", "entrance"),
            "A": ("*", "tab:red", "apex"),
            "X": ("s", "tab:purple", "exit"),
        }
        used_label = set()
        selected = state["selected"]
        for ci, role, x, y in build_editor_keypoint_list(cs):
            marker, color, role_name = marker_cfg[role]
            is_sel = selected == (ci, role)
            size = 180 if role == "A" else 110
            if is_sel:
                size *= 1.45
            label = role_name if role_name not in used_label else None
            used_label.add(role_name)
            ax.scatter([x], [y], s=size, marker=marker, color=color,
                       edgecolors="yellow" if is_sel else "black",
                       linewidths=2.0 if is_sel else 0.8,
                       label=label, zorder=5)
            c = cs[ci]
            if role == "E":
                idx = int(c["entrance_index"])
            elif role == "A":
                idx = int(c["apex_index"])
            else:
                idx = int(c["exit_index"])
            # Keep point labels compact so they do not block the map.
            ax.text(x, y, f"C{int(c['corner_id'])}{role}", fontsize=8,
                    bbox=dict(facecolor="white", alpha=0.78, edgecolor="none", pad=1.2),
                    zorder=6)

        direction_text = state["direction_label"].upper()
        selected_text = "none" if selected is None else f"C{cs[selected[0]]['corner_id']} {selected[1]}"
        add_text = "ON" if state["add_mode"] else "OFF"
        ax.set_title(
            "Keypoint Editor: move/add/delete E-A-X triplets and select track direction\n"
            f"Direction: {direction_text} | Selected: {selected_text} | Add mode: {add_text} | Corners: {len(cs)}"
        )
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.axis("equal")
        ax.grid(True, linewidth=0.3, alpha=0.35)
        ax.legend(loc="best")
        ax.text(
            0.01, 0.01,
            "Drag E/A/X = move | right-click = deselect | ↑/↓ = selected idx ±1 | n = add | Delete/d = delete corner | f = flip | r = reset | Enter = accept | Esc = cancel",
            transform=ax.transAxes, fontsize=8.2, va="bottom", ha="left",
            bbox=dict(facecolor="white", alpha=0.88, edgecolor="black", pad=4)
        )

        draw_keypoint_table(info_ax, cs, selected, direction_text, add_text)
        if old_xlim is not None and old_ylim is not None:
            ax.set_xlim(old_xlim)
            ax.set_ylim(old_ylim)
        else:
            state["view_initialized"] = True
        fig.canvas.draw_idle()

    def pick_keypoint(event):
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return None
        best = None
        best_d = None
        for ci, role, x, y in build_editor_keypoint_list(state["corners"]):
            d = float(np.hypot(float(event.xdata) - x, float(event.ydata) - y))
            if best_d is None or d < best_d:
                best_d = d
                best = (ci, role)
        if best_d is not None and best_d <= float(KEYPOINT_PICK_RADIUS_M):
            return best
        return None

    def move_selected_to_event(event):
        if state["selected"] is None or event.xdata is None or event.ydata is None:
            return
        ci, role = state["selected"]
        idx = nearest_centerline_index(state["centerline_rows"], event.xdata, event.ydata)
        update_corner_point_from_index(state["corners"][ci], role, idx, state["centerline_rows"])
        draw()

    def redetect_current_direction():
        new_corners, _, _, _ = detect_corner_keypoints(state["centerline_rows"])
        state["corners"] = [dict(c) for c in new_corners]
        state["selected"] = None
        state["dragging"] = False
        refresh_corner_ids(state["corners"])
        print(f"Phase 1 UI: redetected {len(state['corners'])} corners for current direction.")
        draw()

    def flip_direction():
        state["centerline_rows"] = reverse_centerline_rows_for_raceline_editor(state["centerline_rows"])
        state["direction_label"] = "reverse" if state["direction_label"] == "normal" else "normal"
        new_corners, _, _, _ = detect_corner_keypoints(state["centerline_rows"])
        state["corners"] = [dict(c) for c in new_corners]
        state["selected"] = None
        state["dragging"] = False
        print(f"Phase 1 UI: flipped track direction to {state['direction_label']} and redetected corners.")
        draw()

    def toggle_add_mode():
        state["add_mode"] = not state["add_mode"]
        state["selected"] = None
        state["dragging"] = False
        print(f"Phase editor UI: add-corner mode {'ON' if state['add_mode'] else 'OFF'}.")
        draw()

    def add_corner_at_event(event):
        if event.inaxes != ax or event.xdata is None or event.ydata is None:
            return
        idx = nearest_centerline_index(state["centerline_rows"], event.xdata, event.ydata)
        new_corner = make_corner_from_apex_index(state["centerline_rows"], idx)
        state["corners"].append(new_corner)
        state["corners"] = refresh_corner_ids(sort_corners_by_entrance_index(state["corners"]))
        # Select the new apex after sorting.
        selected_ci = None
        for ci, c in enumerate(state["corners"]):
            if int(c["apex_index"]) == int(idx):
                selected_ci = ci
                break
        state["selected"] = (selected_ci, "A") if selected_ci is not None else None
        state["add_mode"] = False
        print(f"Phase editor UI: added C{selected_ci if selected_ci is not None else '?'} around apex index {idx}.")
        draw()

    def delete_selected_corner():
        if state["selected"] is None:
            print("Phase editor UI: no selected corner to delete.")
            return
        ci, role = state["selected"]
        if ci < 0 or ci >= len(state["corners"]):
            return
        cid = state["corners"][ci].get("corner_id", ci)
        del state["corners"][ci]
        state["corners"] = refresh_corner_ids(sort_corners_by_entrance_index(state["corners"]))
        state["selected"] = None
        state["dragging"] = False
        print(f"Phase editor UI: deleted corner C{cid}.")
        draw()

    def accept():
        state["accepted"] = True
        save_editor_corners_csv(state["corners"], output_dir, selected_direction=state["direction_label"])
        plt.close(fig)

    def cancel():
        state["cancelled"] = True
        print("Phase 1 UI: cancelled; continuing with original detected corners.")
        plt.close(fig)

    def on_press(event):
        # Ignore clicks on buttons/table and while Matplotlib pan/zoom tools are active.
        # This lets the normal toolbar zoom/pan work without accidentally moving points.
        if event.inaxes != ax or toolbar_is_active():
            return

        # Right click: deselect / stop manipulation. This makes fine editing much
        # smoother after dragging or keyboard stepping a point.
        if getattr(event, "button", None) == 3:
            state["selected"] = None
            state["dragging"] = False
            draw()
            return

        if event.xdata is not None and event.ydata is not None:
            state["last_mouse_xy"] = (float(event.xdata), float(event.ydata))

        if state["add_mode"]:
            add_corner_at_event(event)
            return

        picked = pick_keypoint(event)
        if picked is not None:
            state["selected"] = picked
            state["dragging"] = True
            move_selected_to_event(event)
        else:
            state["selected"] = None
            draw()

    def on_motion(event):
        if state["dragging"] and not toolbar_is_active():
            move_selected_to_event(event)

    def on_release(event):
        if state["dragging"] and not toolbar_is_active():
            move_selected_to_event(event)
        state["dragging"] = False

    def on_scroll(event):
        if event.button == "up":
            apply_scroll_zoom(event, 0.80)
        elif event.button == "down":
            apply_scroll_zoom(event, 1.25)

    def step_selected_keypoint(delta):
        if state["selected"] is None:
            print("Phase editor UI: no selected keypoint for keyboard step.")
            return
        ci, role = state["selected"]
        if ci < 0 or ci >= len(state["corners"]):
            return
        n = len(state["centerline_rows"])
        current_idx = _corner_role_index(state["corners"][ci], role)
        new_idx = int((current_idx + int(delta)) % n)
        update_corner_point_from_index(state["corners"][ci], role, new_idx, state["centerline_rows"])
        print(f"Phase editor UI: moved C{state['corners'][ci]['corner_id']} {role} from idx {current_idx} to {new_idx}.")
        draw()

    def on_key(event):
        if event.key in ("enter", "return"):
            accept()
        elif event.key == "escape":
            cancel()
        elif event.key in ("f", "F"):
            flip_direction()
        elif event.key in ("n", "N"):
            toggle_add_mode()
        elif event.key in ("delete", "backspace", "d", "D"):
            delete_selected_corner()
        elif event.key in ("up",):
            step_selected_keypoint(KEYPOINT_KEYBOARD_STEP_POINTS)
        elif event.key in ("down",):
            step_selected_keypoint(-KEYPOINT_KEYBOARD_STEP_POINTS)
        elif event.key in ("r", "R"):
            redetect_current_direction()

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("scroll_event", on_scroll)

    # Buttons.
    ax_flip = plt.axes([0.05, 0.035, 0.14, 0.055])
    ax_add = plt.axes([0.21, 0.035, 0.13, 0.055])
    ax_delete = plt.axes([0.36, 0.035, 0.13, 0.055])
    ax_reset = plt.axes([0.51, 0.035, 0.13, 0.055])
    ax_accept = plt.axes([0.68, 0.035, 0.12, 0.055])
    ax_cancel = plt.axes([0.82, 0.035, 0.12, 0.055])
    b_flip = Button(ax_flip, "Flip direction")
    b_add = Button(ax_add, "Add corner")
    b_delete = Button(ax_delete, "Delete corner")
    b_reset = Button(ax_reset, "Reset detect")
    b_accept = Button(ax_accept, "Accept")
    b_cancel = Button(ax_cancel, "Cancel")
    b_flip.on_clicked(lambda event: flip_direction())
    b_add.on_clicked(lambda event: toggle_add_mode())
    b_delete.on_clicked(lambda event: delete_selected_corner())
    b_reset.on_clicked(lambda event: redetect_current_direction())
    b_accept.on_clicked(lambda event: accept())
    b_cancel.on_clicked(lambda event: cancel())
    # Keep strong references to Button objects for all GUI backends.
    state["buttons"] = [b_flip, b_add, b_delete, b_reset, b_accept, b_cancel]

    draw()

    if interactive_ok:
        print("Phase 1 UI: opening interactive keypoint editor window.")
        print("  Drag E/A/X, or use ↑/↓ to step selected point by 1 centerline index.")
        print("  Press n/Add to create a corner, Delete/d/Delete button to remove selected corner, f to flip, Enter/Accept to continue.")
        try:
            plt.show(block=True)
        except Exception as exc:
            print(f"Phase 1 UI: interactive display failed: {exc}")
            fallback_path = os.path.join(output_dir, KEYPOINT_EDIT_FALLBACK_PNG)
            fig.savefig(fallback_path, dpi=160)
            print(f"Phase 1 UI: saved fallback PNG: {fallback_path}")
            plt.close(fig)
            return centerline_rows, corners, "normal", False
    else:
        fallback_path = os.path.join(output_dir, KEYPOINT_EDIT_FALLBACK_PNG)
        fig.savefig(fallback_path, dpi=160)
        print("Phase 1 UI: no interactive matplotlib backend available.")
        print(f"Phase 1 UI: saved fallback PNG instead: {fallback_path}")
        plt.close(fig)
        return centerline_rows, corners, "normal", False

    if state["accepted"]:
        return state["centerline_rows"], state["corners"], state["direction_label"], True

    # Window closed without Accept: conservative behavior, keep original results.
    if not state["cancelled"]:
        print("Phase editor UI: window closed without Accept; continuing with original detected corners.")
    return original_centerline_rows, original_corners, "normal", False


def draw_direction_arrows_on_path(
    ax,
    pts,
    stride=None,
    color="tab:blue",
    label=None,
    zorder=4,
    head_width=0.12,
    head_length=0.18,
    linewidth=0.8,
    alpha=0.75,
):
    """Draw large, readable arrows along a path.

    The previous arrows were fixed at about 0.55 m, which is almost invisible
    on large maps such as Spielberg. This version scales arrow length/head size
    from the displayed path size, so the direction is visible in saved PNGs.
    """
    pts = np.asarray(pts, dtype=np.float64)
    if len(pts) < 5:
        return

    if len(pts) > 1 and np.allclose(pts[0], pts[-1]):
        pts = pts[:-1]
    if len(pts) < 5:
        return

    min_xy = np.nanmin(pts, axis=0)
    max_xy = np.nanmax(pts, axis=0)
    diag = float(np.hypot(*(max_xy - min_xy)))
    if not np.isfinite(diag) or diag <= 1e-9:
        diag = 10.0

    if stride is None:
        stride = max(8, len(pts) // 32)
    stride = max(1, int(stride))
    step = max(2, stride // 3)

    # Adaptive size: visible on large maps, not ridiculous on small maps.
    arrow_len_target = max(0.75, min(4.0, diag * 0.030))
    hw = max(float(head_width), arrow_len_target * 0.22)
    hl = max(float(head_length), arrow_len_target * 0.32)
    lw = max(float(linewidth), 1.6)

    first = True
    for i in range(0, len(pts), stride):
        j = (i + step) % len(pts)
        dx = pts[j, 0] - pts[i, 0]
        dy = pts[j, 1] - pts[i, 1]
        norm = float(np.hypot(dx, dy))
        if norm < 1e-9:
            continue
        dx = dx / norm * arrow_len_target
        dy = dy / norm * arrow_len_target
        ax.arrow(
            pts[i, 0], pts[i, 1], dx, dy,
            length_includes_head=True,
            head_width=hw,
            head_length=hl,
            linewidth=lw,
            alpha=alpha,
            color=color,
            label=label if first else None,
            zorder=zorder,
        )
        first = False

def overlay_edited_corner_keypoints(img, yaml_data, centerline_rows, corners, path, selected_direction="normal"):
    if not DEBUG:
        return

    extent = get_world_extent(img.shape, yaml_data)
    base = img.astype(np.float32) / 255.0
    cpts = rows_xy_array(centerline_rows)

    plt.figure(figsize=(10, 8))
    ax = plt.gca()
    ax.imshow(base, cmap="gray", origin="lower", extent=extent)
    ax.plot(cpts[:, 0], cpts[:, 1], linewidth=1.2, color="tab:blue", label="edited-direction centerline")
    draw_direction_arrows_on_path(ax, cpts, color="tab:blue", label="centerline direction", zorder=4)

    n = len(centerline_rows)
    for c in corners:
        idxs = circular_segment_indices(int(c["entrance_index"]), int(c["exit_index"]), n)
        if len(idxs) > 0:
            seg = cpts[idxs]
            ax.plot(seg[:, 0], seg[:, 1], linewidth=3.0, alpha=0.25, color="tab:orange")

    marker_cfg = {"E": ("^", "tab:green", "entrance"), "A": ("*", "tab:red", "apex"), "X": ("s", "tab:purple", "exit")}
    used = set()
    for ci, role, x, y in build_editor_keypoint_list(corners):
        marker, color, role_name = marker_cfg[role]
        label = role_name if role_name not in used else None
        used.add(role_name)
        ax.scatter([x], [y], s=140 if role == "A" else 90, marker=marker,
                   color=color, edgecolors="black", linewidths=0.9, label=label, zorder=5)
        c = corners[ci]
        # No idx numbers here; they overlap too much. Use the CSV/UI table for idx.
        ax.text(x, y, f"C{int(c['corner_id'])} {role}", fontsize=8,
                bbox=dict(facecolor="white", alpha=0.85, edgecolor="none", pad=1.5), zorder=6)

    fig = plt.gcf()
    fig.text(0.02, 0.985,
             f"CENTERLINE REFERENCE DIRECTION: {str(selected_direction).upper()} | blue arrows = active centerline index order",
             ha="left", va="top", fontsize=10, weight="bold",
             bbox=dict(facecolor="white", alpha=0.92, edgecolor="black", pad=4))
    ax.set_title("Edited Corner Keypoints Before Phase 11")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.axis("equal")
    ax.grid(True, linewidth=0.3, alpha=0.35)
    ax.legend(loc="best")
    plt.tight_layout(rect=[0, 0, 1, 0.94])
    plt.savefig(path, dpi=150)
    plt.close()

def overlay_raceline_with_direction(img, yaml_data, centerline_rows, moved_keypoints, raceline_points, safety_mask, path, selected_direction="normal"):
    if not DEBUG:
        return

    extent = get_world_extent(img.shape, yaml_data)
    base = img.astype(np.float32) / 255.0
    cpts = rows_xy_array(centerline_rows)

    plt.figure(figsize=(10, 8))
    ax = plt.gca()
    ax.imshow(base, cmap="gray", origin="lower", extent=extent)
    safety_overlay = np.ma.masked_where(safety_mask <= 0, safety_mask)
    ax.imshow(safety_overlay, cmap="Greens", origin="lower", extent=extent, alpha=0.16)

    ax.plot(cpts[:, 0], cpts[:, 1], linewidth=1.0, color="tab:blue", label="centerline reference")
    draw_direction_arrows_on_path(ax, cpts, color="tab:blue", label="centerline direction", zorder=4)

    if len(moved_keypoints) > 0:
        ref = np.array([[kp["x"], kp["y"]] for kp in order_moved_keypoints_for_raceline(moved_keypoints)], dtype=np.float64)
        if len(ref) > 0:
            ref_closed = np.vstack([ref, ref[0]])
            ax.plot(ref_closed[:, 0], ref_closed[:, 1], linewidth=1.0, linestyle="--", color="tab:gray", label="moved keypoint polygon")
            ax.scatter(ref[:, 0], ref[:, 1], s=35, color="tab:gray", label="moved keypoints", zorder=5)

    if len(raceline_points) > 0:
        rp = np.array(raceline_points, dtype=np.float64)
        ax.plot(rp[:, 0], rp[:, 1], linewidth=2.2, color="tab:red", label="final raceline")
        draw_direction_arrows_on_path(ax, rp[:-1] if len(rp) > 1 and np.allclose(rp[0], rp[-1]) else rp,
                                      stride=max(25, len(rp) // 28), color="tab:red", label="raceline CSV order", zorder=6)

    fig = plt.gcf()
    fig.text(0.02, 0.985,
             f"CENTERLINE REF: {str(selected_direction).upper()} | RACELINE CSV ORDER: red arrows",
             ha="left", va="top", fontsize=10, weight="bold",
             bbox=dict(facecolor="white", alpha=0.92, edgecolor="black", pad=4))
    ax.set_title("Final Raceline with Direction")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.axis("equal")
    ax.grid(True, linewidth=0.3, alpha=0.35)
    ax.legend(loc="best")
    plt.tight_layout(rect=[0, 0, 1, 0.94])
    plt.savefig(path, dpi=150)
    plt.close()

def load_centerline_geometry_csv(csv_path):
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "index": int(row["index"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "yaw": float(row["yaw"]),
                "curvature": float(row["curvature"]),
                "curvature_abs": float(row["curvature_abs"]),
            })
    return rows


def resolve_centerline_output_paths(output_dir):
    return {
        "centerline_csv": os.path.join(output_dir, SMOOTH_CSV_NAME),
        "metadata_yaml": os.path.join(output_dir, METADATA_YAML_NAME),
        "drivable_mask": os.path.join(output_dir, DRIVABLE_MASK_NPY_NAME),
    }


def ensure_required_phase9_outputs(paths):
    missing = [p for p in paths.values() if not os.path.exists(p)]
    if missing:
        raise RuntimeError(
            "Missing Phase 9 output(s):\n  " + "\n  ".join(missing) +
            "\nRun centerline_reference_generator.py first, or use run_centerline_and_raceline.py."
        )

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
        # extrasaction='ignore' is intentional. Runtime-only metadata such as
        # selected direction must never break the fixed downstream CSV schema.
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for c in corners:
            writer.writerow({k: c.get(k, "") for k in fieldnames})


def overlay_corner_keypoints(img, yaml_data, centerline_rows, corners, path, selected_direction="normal"):
    if not DEBUG:
        return

    extent = get_world_extent(img.shape, yaml_data)
    base = img.astype(np.float32) / 255.0
    cpts = np.array([[r["x"], r["y"]] for r in centerline_rows], dtype=np.float64)

    plt.figure(figsize=(9, 7))
    ax = plt.gca()
    ax.imshow(base, cmap="gray", origin="lower", extent=extent)
    ax.plot(cpts[:, 0], cpts[:, 1], linewidth=1.0, color="tab:blue", label="centerline reference")
    draw_direction_arrows_on_path(ax, cpts, color="tab:blue", label="centerline direction", zorder=4)

    for c in corners:
        ex, ey = c["entrance_x"], c["entrance_y"]
        axp, ayp = c["apex_x"], c["apex_y"]
        xx, xy = c["exit_x"], c["exit_y"]
        cid = c["corner_id"]
        direction = c["turn_direction"]
        ax.scatter([ex], [ey], s=40, marker="^", color="tab:green", edgecolors="black", label="entrance" if cid == 0 else None, zorder=5)
        ax.scatter([axp], [ayp], s=60, marker="*", color="tab:red", edgecolors="black", label="apex" if cid == 0 else None, zorder=6)
        ax.scatter([xx], [xy], s=40, marker="s", color="tab:purple", edgecolors="black", label="exit" if cid == 0 else None, zorder=5)
        ax.text(axp, ayp, f"C{cid} {direction}", fontsize=8,
                bbox=dict(facecolor="white", alpha=0.8, edgecolor="none", pad=1.2), zorder=7)

    fig = plt.gcf()
    fig.text(0.02, 0.985,
             f"CENTERLINE REFERENCE DIRECTION: {str(selected_direction).upper()} | blue arrows = active centerline index order",
             ha="left", va="top", fontsize=10, weight="bold",
             bbox=dict(facecolor="white", alpha=0.92, edgecolor="black", pad=4))
    ax.set_title("Detected/Active Corner Key Points")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.axis("equal")
    ax.legend()
    plt.tight_layout(rect=[0, 0, 1, 0.94])
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
    raceline_label,
    selected_direction="normal",
):
    if not DEBUG:
        return

    extent = get_world_extent(img.shape, yaml_data)
    base = img.astype(np.float32) / 255.0
    cpts = np.array([[r["x"], r["y"]] for r in centerline_rows], dtype=np.float64)

    plt.figure(figsize=(9, 7))
    ax = plt.gca()
    ax.imshow(base, cmap="gray", origin="lower", extent=extent)

    safety_overlay = np.ma.masked_where(safety_mask <= 0, safety_mask)
    ax.imshow(safety_overlay, cmap="Greens", origin="lower", extent=extent, alpha=0.18)

    ax.plot(cpts[:, 0], cpts[:, 1], linewidth=1.0, color="tab:blue", label="centerline reference")
    draw_direction_arrows_on_path(ax, cpts, color="tab:blue", label="centerline direction", zorder=4)

    if len(moved_keypoints) > 0:
        ref = np.array([[kp["x"], kp["y"]] for kp in order_moved_keypoints_for_raceline(moved_keypoints)], dtype=np.float64)
        ref_closed = np.vstack([ref, ref[0]])
        ax.plot(ref_closed[:, 0], ref_closed[:, 1], linewidth=1.0, linestyle="--", color="tab:gray", label="moved keypoint polygon")
        ax.scatter(ref[:, 0], ref[:, 1], s=35, color="tab:gray", label="moved keypoints", zorder=5)

    if len(raceline_points) > 0:
        rp = np.array(raceline_points, dtype=np.float64)
        ax.plot(rp[:, 0], rp[:, 1], linewidth=2.0, color="tab:red", label=raceline_label)
        draw_direction_arrows_on_path(ax, rp[:-1] if len(rp) > 1 and np.allclose(rp[0], rp[-1]) else rp,
                                      stride=max(25, len(rp) // 28), color="tab:red", label="raceline CSV order", zorder=6)

    fig = plt.gcf()
    fig.text(0.02, 0.985,
             f"CENTERLINE REF: {str(selected_direction).upper()} | blue=centerline order | red=raceline CSV order",
             ha="left", va="top", fontsize=10, weight="bold",
             bbox=dict(facecolor="white", alpha=0.92, edgecolor="black", pad=4))
    ax.set_title(title)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.axis("equal")
    ax.legend()
    plt.tight_layout(rect=[0, 0, 1, 0.94])
    plt.savefig(path, dpi=140)
    plt.close()

def overlay_piecewise_raceline(img, yaml_data, centerline_rows, moved_keypoints, raceline_points, safety_mask, path, selected_direction="normal"):
    overlay_raceline_common(
        img,
        yaml_data,
        centerline_rows,
        moved_keypoints,
        raceline_points,
        safety_mask,
        path,
        title="Offset-field Raceline from Centerline Backbone",
        raceline_label="offset-field raceline",
        selected_direction=selected_direction,
    )

def overlay_global_spline_raceline(img, yaml_data, centerline_rows, moved_keypoints, raceline_points, safety_mask, path, selected_direction="normal"):
    overlay_raceline_common(
        img,
        yaml_data,
        centerline_rows,
        moved_keypoints,
        raceline_points,
        safety_mask,
        path,
        title="Global Soft B-spline Raceline from Moved Keypoints",
        raceline_label="global soft spline",
        selected_direction=selected_direction,
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

# =========================
# MAIN PIPELINE: PHASE 10-12 ONLY
# =========================
def main():
    if len(sys.argv) < 3:
        print("Usage: python3 raceline_generator.py <map_image> <map.yaml> [--review-keypoints] [--no-edit-keypoints]")
        print("Note: Phase 9 outputs must already exist in OUTPUT_DIR.")
        return

    map_path, yaml_path = sys.argv[1], sys.argv[2]
    review_keypoints_ui = REVIEW_CORNER_KEYPOINTS_UI
    edit_keypoints_ui = EDIT_CORNER_KEYPOINTS_UI
    resume_existing_keypoints = True

    for arg in sys.argv[3:]:
        if arg == "--review-keypoints":
            review_keypoints_ui = True
        elif arg == "--edit-keypoints":
            edit_keypoints_ui = True
        elif arg == "--no-edit-keypoints":
            edit_keypoints_ui = False
        elif arg in ("--fresh-detect", "--no-resume-keypoints"):
            resume_existing_keypoints = False
        else:
            raise RuntimeError(f"Unknown optional argument: {arg}")

    if edit_keypoints_ui:
        # The edit UI already includes review visualization.
        review_keypoints_ui = False

    ensure_output_dir(OUTPUT_DIR)

    paths = resolve_centerline_output_paths(OUTPUT_DIR)
    ensure_required_phase9_outputs(paths)

    img, meta = load_map(map_path, yaml_path)
    res = float(meta["resolution"])
    centerline_rows = load_centerline_geometry_csv(paths["centerline_csv"])
    free = np.load(paths["drivable_mask"]).astype(np.uint8)

    print("Loaded Phase 9 reference data")
    print(f"  Centerline CSV:       {paths['centerline_csv']}")
    print(f"  Centerline points:    {len(centerline_rows)}")
    print(f"  Drivable mask:        {paths['drivable_mask']}")
    print(f"  Map shape:            {img.shape}, resolution={res:.3f} m/pixel")

    selected_direction = "normal"
    edited_accepted = False

    # Phase 10: detect corner entrance/apex/exit points only
    corner_debug = None
    if DETECT_CORNERS:
        print("Phase 10: corner key-point detection")

        resumed = None
        if edit_keypoints_ui and resume_existing_keypoints:
            resumed = load_recent_keypoints_if_available(
                OUTPUT_DIR,
                paths["centerline_csv"],
                centerline_rows,
            )

        if resumed is not None:
            centerline_rows, corners, selected_direction = resumed
            curv_raw = np.array([float(r["curvature"]) for r in centerline_rows], dtype=np.float64)
            curv_smooth = smooth_curvature_for_corner_detection(curv_raw)
            corner_mask = np.abs(curv_smooth) >= CORNER_ENTRY_EXIT_THRESHOLD
            print(f"  Starting editor from resumed keypoints: {len(corners)} corners")
        else:
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

        if review_keypoints_ui:
            show_corner_keypoint_review_ui(
                img,
                meta,
                centerline_rows,
                corners,
                OUTPUT_DIR
            )

        if edit_keypoints_ui:
            centerline_rows, corners, selected_direction, edited_accepted = show_corner_keypoint_edit_ui(
                img,
                meta,
                centerline_rows,
                corners,
                OUTPUT_DIR,
                initial_direction=selected_direction,
            )
            # Recompute curvature debug data for the possibly edited/flipped centerline.
            curv_raw = np.array([float(r["curvature"]) for r in centerline_rows], dtype=np.float64)
            curv_smooth = smooth_curvature_for_corner_detection(curv_raw)
            corner_mask = np.abs(curv_smooth) >= CORNER_ENTRY_EXIT_THRESHOLD
            corners_csv_path = os.path.join(OUTPUT_DIR, CORNERS_CSV_NAME)
            save_corner_keypoints_csv(corners, corners_csv_path)
            print(f"Phase 1 UI: selected direction: {selected_direction}")
            print(f"Phase 1 UI: active corner CSV overwritten for Phase 11: {corners_csv_path}")

        corner_debug = {
            "corners": corners,
            "curv_raw": curv_raw,
            "curv_smooth": curv_smooth,
            "corner_mask": corner_mask,
        }

    moved_keypoints = []

    if MOVE_CORNER_KEYPOINTS:
        if corner_debug is None:
            raise RuntimeError("MOVE_CORNER_KEYPOINTS=True requires DETECT_CORNERS=True.")

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
        if DETECT_CORNERS and corner_debug is not None:
            overlay_corner_keypoints(
                img,
                meta,
                centerline_rows,
                corner_debug["corners"],
                os.path.join(OUTPUT_DIR, DEBUG_CORNER_KEYPOINTS),
                selected_direction=selected_direction,
            )
            overlay_edited_corner_keypoints(
                img,
                meta,
                centerline_rows,
                corner_debug["corners"],
                os.path.join(OUTPUT_DIR, DEBUG_EDITED_CORNER_KEYPOINTS),
                selected_direction=selected_direction,
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
            # Keep the legacy debug image name for compatibility, and also save a clearer
            # direction-aware raceline debug image for the manual editor workflow.
            overlay_piecewise_raceline(
                img,
                meta,
                centerline_rows,
                moved_keypoints,
                raceline_points,
                raceline_safety_mask,
                os.path.join(OUTPUT_DIR, DEBUG_RACELINE_PIECEWISE),
                selected_direction=selected_direction,
            )
            overlay_raceline_with_direction(
                img,
                meta,
                centerline_rows,
                moved_keypoints,
                raceline_points,
                raceline_safety_mask,
                os.path.join(OUTPUT_DIR, DEBUG_RACELINE_WITH_DIRECTION),
                selected_direction=selected_direction,
            )
            plot_raceline_curvature(
                raceline_points,
                os.path.join(OUTPUT_DIR, DEBUG_RACELINE_CURVATURE)
            )

    print("Phase 12 completed successfully." if GENERATE_PIECEWISE_RACELINE else ("Phase 11 completed successfully." if MOVE_CORNER_KEYPOINTS else ("Phase 10 completed successfully." if DETECT_CORNERS else "No raceline phase enabled.")))


if __name__ == "__main__":
    main()