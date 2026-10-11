"""Small Matplotlib view; all decisions and promotion stay in Workbench."""

import os

import numpy as np

from . import global_raceline_optimizer as opt


def draw_candidate(bench, candidate, map_axis, curvature_axis):
    """Map/corridor/unsafe regions and actual exported-polyline curvature."""
    map_axis.clear()
    curvature_axis.clear()
    meta = bench.metadata
    height, width = bench.mask.shape
    extent = [meta.origin_x_m, meta.origin_x_m + width * meta.resolution_m,
              meta.origin_y_m, meta.origin_y_m + height * meta.resolution_m]
    import matplotlib.image as mpimg
    # PNG/PGM original pixels, transformed in the same ROS convention as Phase 9.
    image = mpimg.imread(bench.identity["map_image"])
    map_axis.imshow(np.flipud(image), origin="lower", extent=extent, cmap="gray")
    map_axis.imshow(np.ma.masked_where(~bench.safe, bench.safe), origin="lower",
                    extent=extent, cmap="Greens", alpha=0.25, vmin=0, vmax=1)
    ref = np.vstack((bench.centerline, bench.centerline[0]))
    map_axis.plot(ref[:, 0], ref[:, 1], "--", color="tab:blue", lw=0.9, label="Centerline")
    if candidate and candidate.get("path"):
        points = opt.load_path_csv(candidate["path"])
    else:
        points = bench.centerline
    closed = np.vstack((points, points[0]))
    label = "Reference preview (no optimizer result)" if candidate and candidate.get("candidate_kind") == "reference_preview" else "Candidate"
    map_axis.plot(closed[:, 0], closed[:, 1], color="tab:orange", lw=1.0, label=label)
    try:
        dense = opt.densify_closed_path(points, bench.baseline.validation_step_m)
        unsafe = ~opt.points_in_mask(dense, bench.safe, meta)
        if np.any(unsafe):
            map_axis.scatter(dense[unsafe, 0], dense[unsafe, 1], s=7, c="red", label="Unsafe dense samples")
        _, curvature = opt.path_geometry(points)
        excess = np.abs(curvature) > bench.baseline.maximum_curvature_inv_m
        if np.any(excess):
            map_axis.scatter(points[excess, 0], points[excess, 1], s=12, c="magenta", label="Above hard curvature")
        segments = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
        station = np.concatenate(([0.0], np.cumsum(segments[:-1])))
        curvature_axis.plot(station, curvature, label="Candidate curvature")
        curvature_axis.set_title(f"Maximum |curvature|: {np.max(np.abs(curvature)):.3f} 1/m")
    except ValueError as error:
        curvature_axis.set_title("Invalid geometry: " + str(error))
    for sign in (-1, 1):
        curvature_axis.axhline(sign * bench.baseline.maximum_curvature_inv_m, c="red", ls="--",
                              label="Hard limit" if sign == 1 else None)
    map_axis.set_aspect("equal", adjustable="box")
    map_axis.set_xlabel("Map x [m]")
    map_axis.set_ylabel("Map y [m]")
    map_axis.set_title("Map, eroded safe corridor (green), and candidate")
    map_axis.legend(fontsize=8, loc="best")
    curvature_axis.set_xlabel("Candidate path station [m]")
    curvature_axis.set_ylabel("Curvature [1/m]")
    curvature_axis.legend(fontsize=8)


def candidate_status(bench, candidate):
    if not candidate:
        return "No candidate"
    v = candidate["validation"]
    minimum = v["minimum_clearance_m"]
    peak = v["curvature_abs_max_inv_m"]
    return (f"{candidate['id']}: {'PASS' if v['valid'] else 'FAIL'}\n"
            f"Clearance: {minimum if minimum is not None else 'unknown'} m / required {bench.required:.3f} m\n"
            f"Max curvature: {peak if peak is not None else 'unknown'} / hard {bench.baseline.maximum_curvature_inv_m:.3f} 1/m\n"
            + ("; ".join(v["failure_reasons"]) or "All production gates satisfied"))


def save_preview(bench, candidate):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={"width_ratios": [1.4, 1]})
    draw_candidate(bench, candidate, *axes)
    fig.suptitle("Raceline workbench — " + bench.state)
    fig.text(0.02, 0.01, candidate_status(bench, candidate), fontsize=8)
    fig.tight_layout(rect=(0, 0.12, 1, 0.94))
    fig.savefig(bench.output / "preview.png", dpi=120)
    if candidate:
        fig.savefig(bench.output / "candidate" / candidate["id"] / "preview.png", dpi=120)
    plt.close(fig)


def create_workbench_figure(bench):
    """GUI construction can be smoke-tested with Agg; no implicit solving here."""
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, Slider, TextBox
    fig = plt.figure(figsize=(16, 10))
    map_axis = fig.add_axes([0.05, 0.38, 0.48, 0.56])
    curvature_axis = fig.add_axes([0.59, 0.57, 0.38, 0.35])
    info = fig.text(0.58, 0.38, "", fontsize=9, va="bottom", wrap=True)
    sliders = {}
    for i, key in enumerate(bench.settings["search"]["ranges"]):
        low, high = bench.settings["search"]["ranges"][key]
        ax = fig.add_axes([0.25, 0.30 - i * 0.032, 0.28, 0.018])
        sliders[key] = Slider(ax, key, low, high, valinit=getattr(bench.baseline, key))
    hard = ", ".join(f"{k}={getattr(bench.baseline, k)}" for k in
                     ("safety_margin_m", "maximum_curvature_inv_m", "validation_step_m"))
    fig.text(0.58, 0.28, "HARD vehicle envelope (not search controls):\n" + hard +
             f"\nEffective erosion: {bench.required:.3f} m\nSearch controls: sliders affect Run tuned candidate only.",
             fontsize=8, wrap=True)
    error_text = fig.text(0.58, 0.16, "", color="red", fontsize=9, wrap=True)
    buttons = {}
    labels = ("Run Baseline", "Auto Search", "Preview Candidate", "Manual E/A/X", "Revalidate", "Save Validated", "Run tuned candidate")
    for i, label in enumerate(labels):
        buttons[label] = Button(fig.add_axes([0.025 + i * 0.138, 0.05, 0.13, 0.045]), label)

    def refresh():
        draw_candidate(bench, bench.selected, map_axis, curvature_axis)
        info.set_text(bench.state + "\n" + candidate_status(bench, bench.selected))
        valid = bool(bench.selected and bench.selected["validation"]["valid"])
        buttons["Save Validated"].set_active(valid)
        buttons["Save Validated"].ax.set_facecolor("lightgreen" if valid else "lightgray")
        fig.canvas.draw_idle()

    def action(fn):
        def invoke(event):
            error_text.set_text("Working; each solve is bounded. See candidate reports for details.")
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            try:
                fn()
                error_text.set_text("")
            except (ValueError, OSError, RuntimeError) as error:
                error_text.set_text(str(error))
            refresh()
        return invoke

    selector = TextBox(fig.add_axes([0.67, 0.10, 0.25, 0.028]), "Candidate ID: ", initial="best")

    def preview_candidate():
        name = selector.text.strip()
        candidate = bench.best_candidate() if name == "best" else next(
            (c for c in bench.candidates if c["id"] == name), None)
        if candidate is None:
            raise ValueError("Unknown candidate ID; use best or an ID from the YAML report")
        bench.selected = candidate
        bench.preview()

    def revalidate():
        c = bench.selected
        if c and c.get("path"):
            c["validation"] = bench.validate(opt.load_path_csv(c["path"]),
                                             opt.OptimizerConfig(**c["parameters"]),
                                             c["validation"]["solver_converged"], c["direction"])
            bench.save_report()

    def tuned():
        changes = {key: slider.val for key, slider in sliders.items()}
        from raceline_workbench import search_profile
        config = search_profile(bench.baseline, changes, bench.settings["search"]["ranges"])
        candidate = bench.attempt(config, "gui_tuned")
        if candidate["validation"]["valid"]:
            bench.promote(candidate)
        bench.preview()

    for label, fn in zip(labels, (bench.run_baseline, bench.auto_search, preview_candidate,
                                 bench.manual_editor, revalidate, bench.promote, tuned)):
        buttons[label].on_clicked(action(fn))
    refresh()
    # Retain Matplotlib widget references for their callback lifetime.
    fig.workbench_controls = {"buttons": buttons, "sliders": sliders, "refresh": refresh, "candidate_selector": selector}
    return fig


def show_workbench(bench):
    import matplotlib.pyplot as plt
    if "agg" == str(plt.get_backend()).lower() or not os.environ.get("DISPLAY"):
        raise RuntimeError("GUI requires a working interactive display; use --headless or saved preview.png")
    create_workbench_figure(bench)
    plt.show()
