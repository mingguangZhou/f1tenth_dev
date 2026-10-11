"""Small Matplotlib view; all decisions and promotion stay in Workbench."""

import textwrap
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from . import global_raceline_optimizer as opt


def draw_candidate(bench, candidate, map_axis, curvature_axis):
    """Map/corridor/unsafe regions and actual exported-polyline curvature."""
    map_axis.clear()
    curvature_axis.clear()
    meta = bench.metadata
    import matplotlib.image as mpimg
    image = mpimg.imread(bench.identity["map_image"])
    if bench.centerline is None:
        height, width = image.shape[:2]
        extent = [meta.origin_x_m, meta.origin_x_m + width * meta.resolution_m,
                  meta.origin_y_m, meta.origin_y_m + height * meta.resolution_m]
        map_axis.imshow(np.flipud(image), origin="lower", extent=extent, cmap="gray")
        map_axis.set_aspect("equal")
        map_axis.set_title("Map — Generate Raceline to compute the corridor/path")
        map_axis.set_xlabel("Map x [m]")
        map_axis.set_ylabel("Map y [m]")
        curvature_axis.set_title("No candidate generated in this session")
        return
    height, width = bench.mask.shape
    extent = [meta.origin_x_m, meta.origin_x_m + width * meta.resolution_m,
              meta.origin_y_m, meta.origin_y_m + height * meta.resolution_m]
    # PNG/PGM original pixels, transformed in the same ROS convention as Phase 9.
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
            map_axis.scatter(points[excess, 0], points[excess, 1], s=18, c="magenta", label="Beyond steering limit")
        segments = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
        station = np.concatenate(([0.0], np.cumsum(segments[:-1])))
        curvature_axis.plot(station, curvature, label="Candidate curvature")
        curvature_axis.scatter(station[excess], curvature[excess], s=14, c="magenta", label="Beyond limit")
        peak = np.max(np.abs(curvature))
        radius = 1 / peak if peak > 0 else float("inf")
        curvature_axis.set_title(f"Tightest turn: {radius:.2f} m radius | peak curvature {peak:.3f} 1/m")
    except ValueError as error:
        curvature_axis.set_title("Invalid geometry: " + str(error))
    for sign in (-1, 1):
        curvature_axis.axhline(sign * bench.baseline.maximum_curvature_inv_m, c="red", ls="--",
                              label="Hard limit" if sign == 1 else None)
    geometry = np.vstack((bench.centerline, points))
    geometry = geometry[np.all(np.isfinite(geometry), axis=1)]
    if len(geometry):
        margin = max(0.6, bench.baseline.safety_margin_m * 2)
        map_axis.set_xlim(geometry[:, 0].min() - margin, geometry[:, 0].max() + margin)
        map_axis.set_ylim(geometry[:, 1].min() - margin, geometry[:, 1].max() + margin)
    map_axis.set_aspect("equal", adjustable="box")
    map_axis.set_xlabel("Map x [m]")
    map_axis.set_ylabel("Map y [m]")
    map_axis.set_title("Map, eroded safe corridor (green), and candidate")
    map_axis.legend(fontsize=8, loc="best")
    curvature_axis.set_xlabel("Candidate path station [m]")
    curvature_axis.set_ylabel("Curvature [1/m]")
    curvature_axis.legend(fontsize=8)


def previous_export_context(candidate):
    """Read-only view of qualified historical exports, not current validation."""
    folder = Path(candidate["source_session"])
    if opt.file_sha256(candidate["path"]) != candidate["path_sha256"]:
        raise ValueError("Previous export changed since startup")
    import yaml
    identity = yaml.safe_load((folder / "workbench_inputs.yaml").read_text())
    from raceline_workbench import check_phase9
    check_phase9(folder, identity)
    provenance = yaml.safe_load((folder / "phase9_provenance.yaml").read_text())
    for asset, digest in provenance["outputs"].items():
        if opt.file_sha256(folder / asset) != digest:
            raise ValueError("Previous centerline evidence changed")
    config = opt.OptimizerConfig(**candidate["baseline_profile"])
    meta = opt.MapMetadata.from_yaml(identity["map_yaml"])
    mask = np.load(folder / "drivable_region.npy", allow_pickle=False)
    safe, clearance, required = opt.build_safe_mask(mask, meta, config.safety_margin_m)
    return SimpleNamespace(identity=identity, metadata=meta, mask=mask, safe=safe,
                           clearance=clearance, required=required, baseline=config,
                           centerline=opt.load_path_csv(folder / "centerline_points_smooth.csv"))


def zoom_at_cursor(axis, event):
    """Same wheel factors as the manual editor; toolbar pan/zoom remains available."""
    if event.inaxes != axis or event.xdata is None or event.ydata is None:
        return
    factor = 0.8 if event.button == "up" else 1.25 if event.button == "down" else 1.0
    xmin, xmax = axis.get_xlim()
    ymin, ymax = axis.get_ylim()
    axis.set_xlim(event.xdata + (xmin - event.xdata) * factor, event.xdata + (xmax - event.xdata) * factor)
    axis.set_ylim(event.ydata + (ymin - event.ydata) * factor, event.ydata + (ymax - event.ydata) * factor)
    axis.figure.canvas.draw_idle()


def candidate_status(bench, candidate):
    if not candidate:
        return "No candidate yet. Next: Generate Raceline."
    if candidate.get("candidate_kind") == "previous_export":
        profile = candidate["baseline_profile"]
        return ("Latest matching-map export preview: " + candidate["status"].upper() +
                f"\nPrevious margin: {profile['safety_margin_m']:.3f} m; curvature limit: {profile['maximum_curvature_inv_m']:.3f} 1/m"
                "\nNot selected in this new session.\nOpen Previous Session to resume/export that path,\n"
                "or Generate Raceline to start a new attempt.")
    v = candidate["validation"]
    minimum, peak = v["minimum_clearance_m"], v["curvature_abs_max_inv_m"]
    clearance = "unknown" if minimum is None else f"{minimum:.3f}"
    curvature = "unknown" if peak is None else f"{peak:.3f}"
    if v["valid"]:
        reason = "All production checks pass. Next: Select / Export Validated."
    elif v["corridor_valid"] and not v["curvature_valid"]:
        reason = ("The path stays within the permitted corridor, but its tightest turn exceeds "
                  "the vehicle steering limit. Try more settings or edit the path.")
    elif not v["corridor_valid"]:
        reason = "The path enters the required wall-clearance region. Inspect red samples; try settings or edit."
    else:
        reason = "Production checks failed: " + "; ".join(candidate["failure_classification"])
    if v["solver_converged"] is False:
        reason += " The solver did not converge; geometry alone cannot certify this result."
    return (f"Candidate {candidate['id']}: {'VALIDATED CHECKS PASS' if v['valid'] else 'NOT VALIDATED'}\n"
            f"Clearance: {clearance} m achieved / {bench.required:.3f} m required\n"
            f"Curvature: {curvature} / {bench.baseline.maximum_curvature_inv_m:.3f} 1/m limit\n"
            + "\n".join(textwrap.wrap(reason, 65)))


def action_states(bench, busy=False):
    """Observable GUI enablement; production eligibility remains in Workbench."""
    c = bench.selected
    ready = bench.centerline is not None and not busy
    actual = bool(c and c.get("path") and c.get("candidate_kind") != "reference_preview")
    valid = bool(actual and c["validation"]["valid"])
    searched = any(c["phase"] in ("geometry", "objective") for c in bench.candidates)
    return {
        "Generate Raceline": not busy,
        "Try More Settings": ready and bool(bench.candidates) and not searched and not any(c["validation"]["valid"] for c in bench.candidates),
        "Try My Settings": ready and bool(bench.candidates),
        "Edit Path Manually": ready and bool(bench.candidates),
        "Select / Export Validated": ready and valid,
        "Export Experimental": ready and actual and not valid,
        "Previous": ready and len(bench.candidates) > 1,
        "Next": ready and len(bench.candidates) > 1,
        "Best Candidate": ready and bool(bench.candidates),
        "Advanced Tuning / Diagnostics": not busy,
        "Preset Values": not busy,
        "Fit Map": not busy,
        "Open Previous Session": not busy and bool(bench.latest_export and not bench.latest_export.get("error")),
        "Finish / Exit": not busy,
    }


def preset_changes(bench, choices):
    changes = {}
    for group, selected in choices.items():
        options = bench.settings["tuning_presets"][group]
        # Reset every field owned by this group before applying a new choice.
        for key in {k for values in options.values() for k in values}:
            changes[key] = getattr(bench.baseline, key)
        changes.update(options[selected])
    return changes


TUNING_HELP = {
    "optimization_spacing_m": ("Optimizer spacing [m]", "Smaller: finer geometric freedom, longer solve. Larger: coarser/faster."),
    "maximum_lateral_offset_m": ("Lateral search cap [m]", "Larger: search farther from centerline; always clipped to safe corridor."),
    "path_length_weight": ("Corner-cutting preference", "Larger: favors shorter paths; can increase peak curvature."),
    "curvature_rate_weight": ("Curvature-change penalty", "Larger: encourages slower changes in steering demand."),
    "offset_slope_weight": ("Lateral-transition penalty", "Larger: discourages abrupt outside/inside transitions."),
    "comfortable_curvature_inv_m": ("Soft curvature target [1/m]", "Smaller: encourages gentler turns. This is NOT the hard vehicle limit."),
}


def save_preview(bench, candidate):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(15, 7), gridspec_kw={"width_ratios": [1.4, 1]})
    draw_candidate(bench, candidate, *axes)
    fig.suptitle("Raceline workbench — " + bench.state)
    fig.text(0.02, 0.015, candidate_status(bench, candidate), fontsize=10)
    fig.tight_layout(rect=(0, 0.23, 1, 0.94))
    fig.savefig(bench.output / "preview.png", dpi=120)
    if candidate:
        fig.savefig(bench.output / "candidate" / candidate["id"] / "preview.png", dpi=120)
    plt.close(fig)


def numeric_control(fig, slider_rect, entry_rect, value, bounds, changed, feedback):
    """Linked drag/text controls; invalid numeric entry cannot change draft values."""
    from matplotlib.widgets import Slider, TextBox
    slider = Slider(fig.add_axes(slider_rect), "", *bounds, valinit=value)
    slider.valtext.set_visible(False)
    entry = TextBox(fig.add_axes(entry_rect), "", initial=f"{value:g}")
    syncing = False

    def moved(number):
        nonlocal syncing
        if syncing:
            return
        syncing = True
        entry.set_val(f"{number:.6g}")
        syncing = False
        changed(float(number))

    def entered(text):
        nonlocal syncing
        if syncing:
            return
        try:
            number = float(text)
            if not np.isfinite(number) or not bounds[0] <= number <= bounds[1]:
                raise ValueError("out of range")
        except ValueError:
            syncing = True
            entry.set_val(f"{slider.val:.6g}")
            syncing = False
            feedback(f"Enter a finite number in [{bounds[0]:g}, {bounds[1]:g}]. Previous value retained.")
            return
        slider.set_val(number)

    slider.on_changed(moved)
    entry.on_submit(entered)
    return slider, entry


def create_workbench_figure(bench):
    """Three steps, readable settings, deliberate profile edits and finish summary."""
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, RadioButtons, CheckButtons, TextBox
    fig = plt.figure(figsize=(17, 11))
    title = fig.suptitle("", fontsize=13)
    map_axis = fig.add_axes([0.05, 0.51, 0.44, 0.36])
    curvature_axis = fig.add_axes([0.56, 0.72, 0.40, 0.15])
    info = fig.text(0.54, 0.67, "", fontsize=10, va="top")
    selection_text = fig.text(0.04, 0.942, "", fontsize=10)
    path_box = TextBox(fig.add_axes([0.11, 0.898, 0.85, 0.026]), "Export CSV: ", initial="None")
    path_box.set_active(False)  # Display only; editing a label must not impersonate selection.
    feedback = fig.text(0.04, 0.018, "Ready. No simulation or vehicle stack is launched.", fontsize=9)
    vehicle_text = fig.text(0.70, 0.25, "", fontsize=10, va="top")
    buttons, radios, popups = {}, {}, []
    tuning = {key: getattr(bench.baseline, key) for key in TUNING_HELP}
    busy = False
    viewport = {"initialized": False, "session": bench.output}

    def say(message):
        feedback.set_text("\n".join(textwrap.wrap(message, 155)))
        fig.canvas.draw_idle()

    def add_button(label, x, y, width=0.27, primary=False):
        button = Button(fig.add_axes([x, y, width, 0.036]), label,
                        color="#cee9df" if primary else "#eeeeee")
        buttons[label] = button
        return button

    for x, heading in ((0.04, "1 — Generate"), (0.37, "2 — Review / adjust"), (0.70, "3 — Export / finish")):
        fig.text(x, 0.43, heading, fontsize=13, weight="bold")
    add_button("Generate Raceline", 0.04, 0.372, primary=True)
    add_button("Try More Settings", 0.04, 0.322)
    add_button("Try My Settings", 0.37, 0.372)
    add_button("Edit Path Manually", 0.37, 0.322)
    add_button("Select / Export Validated", 0.70, 0.372, primary=True)
    add_button("Export Experimental", 0.70, 0.322)
    add_button("Previous", 0.54, 0.50, width=0.105)
    add_button("Next", 0.655, 0.50, width=0.105)
    add_button("Best Candidate", 0.77, 0.50, width=0.19)
    compare = CheckButtons(fig.add_axes([0.54, 0.456, 0.20, 0.027]), ["Compare baseline"], [False])
    add_button("Fit Map", 0.765, 0.456, width=0.09)
    add_button("Open Previous Session", 0.04, 0.08)
    add_button("Advanced Tuning / Diagnostics", 0.37, 0.08)
    add_button("Finish / Exit", 0.70, 0.08, primary=True)
    add_button("Preset Values", 0.37, 0.272)
    fig.text(0.04, 0.275, "Wheel: zoom at cursor | Toolbar: pan/zoom | Fit Map: reset", fontsize=9)
    for i, (group, options) in enumerate(bench.settings["tuning_presets"].items()):
        ax = fig.add_axes([0.04 + i * 0.215, 0.135, 0.195, 0.112])
        ax.set_title(group, fontsize=10, pad=8)
        radio = RadioButtons(ax, list(options), active=list(options).index("Balanced"))
        def choose(label, name=group):
            if not busy:
                tuning.update(preset_changes(bench, {name: label}))
                say("Preset staged. Preset Values shows exact numbers; Try My Settings runs them. Effects interact.")
        radio.on_clicked(choose)
        radios[group] = radio
    for x, hint in ((0.04, "More: shorter corners"), (0.255, "More: gentler steering changes"),
                    (0.47, "Fine: more freedom / longer solve")):
        fig.text(x, 0.121, hint, fontsize=8.5)

    def refresh():
        preserve = viewport["initialized"] and viewport["session"] == bench.output
        old_limits = (map_axis.get_xlim(), map_axis.get_ylim()) if preserve else None
        previous = bench.latest_export
        shown = bench.selected
        context = bench
        if not shown and previous and not previous.get("error"):
            try:
                context = previous_export_context(previous)
                shown = previous
            except (ValueError, OSError) as error:
                previous["error"] = str(error)
        draw_candidate(context, shown, map_axis, curvature_axis)
        if old_limits:
            map_axis.set_xlim(old_limits[0])
            map_axis.set_ylim(old_limits[1])
        viewport.update(initialized=True, session=bench.output)
        if compare.get_status()[0]:
            baseline = next((c for c in bench.candidates if c["phase"] == "baseline" and c.get("path")), None)
            if baseline:
                points = opt.load_path_csv(baseline["path"])
                closed = np.vstack((points, points[0]))
                map_axis.plot(closed[:, 0], closed[:, 1], color="gray", alpha=0.8, label="Baseline comparison")
                _, curvature = opt.path_geometry(points)
                lengths = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
                curvature_axis.plot(np.r_[0, np.cumsum(lengths[:-1])], curvature, color="gray", alpha=0.7, label="Baseline")
                map_axis.legend(fontsize=8)
                curvature_axis.legend(fontsize=8)
        description = candidate_status(bench, shown)
        if not bench.selected and previous and previous.get("error"):
            description = "Latest export cannot be previewed:\n" + "\n".join(textwrap.wrap(previous["error"], 62))
        info.set_text(bench.state + "\n" + description + "\nValidated CSV in this session: " + ("YES" if bench.outputs else "NONE"))
        selection = bench.active_selection
        state = "Active selection: " + selection["status"].upper() if selection else "No active selection in this session"
        path = selection["raceline_csv"] if selection else shown["path"] if shown and shown.get("path") else "None"
        if not selection and shown and shown.get("candidate_kind") == "previous_export":
            state = "PREVIOUS EXPORT PREVIEW — " + shown["status"].upper() + " — not automatically selected"
        selection_text.set_text(state)
        path_box.set_val(path)
        title.set_text("Raceline Workbench | " + textwrap.shorten(bench.output.name, 95, placeholder="…"))
        original = opt.load_optimizer_config(bench.settings["baseline_optimizer_config"])
        custom = any(getattr(original, k) != getattr(bench.baseline, k) for k in ("safety_margin_m", "maximum_curvature_inv_m", "validation_step_m"))
        vehicle_text.set_text(("CUSTOM offline profile" if custom else "Vehicle / Safety Limits") +
                              f"\nWall margin: {bench.baseline.safety_margin_m:.3f} m"
                              f"\nEffective erosion: {bench.required:.3f} m"
                              f"\nCurvature limit: {bench.baseline.maximum_curvature_inv_m:.3f} 1/m"
                              f"\nValidation step: {bench.baseline.validation_step_m:.3f} m"
                              "\nLimit edits require confirmation + new session.")
        for label, enabled in action_states(bench, busy).items():
            buttons[label].set_active(enabled)
            buttons[label].label.set_alpha(1.0 if enabled else 0.4)
            primary = label in ("Generate Raceline", "Select / Export Validated", "Finish / Exit")
            buttons[label].ax.set_facecolor("#cee9df" if enabled and primary else "#eeeeee" if enabled else "#cccccc")
        fig.canvas.draw_idle()

    def action(fn):
        def invoke(event=None):
            nonlocal busy
            if busy:
                return
            busy = True
            say("RUNNING — please wait. Controls disabled until this bounded operation finishes.")
            refresh()
            fig.canvas.flush_events()
            try:
                fn()
                if plt.fignum_exists(fig.number):
                    say("Done. Review status/export path above. Finish / Exit shows what has been saved.")
            except (ValueError, OSError, RuntimeError) as error:
                say("Action failed: " + str(error))
            finally:
                busy = False
                if plt.fignum_exists(fig.number):
                    refresh()
        return invoke

    def tuned():
        from raceline_workbench import search_profile
        bench.attempt(search_profile(bench.baseline, tuning, bench.settings["search"]["ranges"]), "user_settings")
        bench.preview()

    def browse(step):
        index = bench.candidates.index(bench.selected) if bench.selected in bench.candidates else 0
        bench.selected = bench.candidates[(index + step) % len(bench.candidates)]
        bench.save_report()

    def popup(title_text, size=(11, 7)):
        dialog = plt.figure(figsize=size)
        dialog.suptitle(title_text, fontsize=13)
        popups.append(dialog)
        return dialog

    def display(dialog):
        if str(plt.get_backend()).lower() != "agg":
            dialog.show()

    def close_main():
        for dialog in popups:
            plt.close(dialog)
        plt.close(fig)

    def finish():
        selection = bench.active_selection
        if selection:
            from .raceline_selection import resolve_selection
            selection = resolve_selection(selection["manifest"], True)
        dialog = popup("Finish — saved output and selection", (12, 7))
        if selection:
            message = ("EXPORTED / SELECTED: " + selection["status"].upper() +
                       "\nCSV: " + selection["raceline_csv"] + "\nManifest: " + selection["manifest"])
            if selection["status"] == "experimental":
                message += "\nNOT production-validated. Deliberate simulation experimentation only."
        else:
            message = "No raceline selected in this session."
            if bench.outputs:
                message += "\nGenerated validated CSV: " + bench.outputs["raceline"]
            elif bench.latest_export and not bench.latest_export.get("error"):
                message += "\nPrevious export remains saved: " + bench.latest_export["path"]
                message += "\nPrevious status: " + bench.latest_export["status"].upper()
            message += "\nSave Validated & Exit selects the current passing candidate."
        message += "\nSession / diagnostics: " + str(bench.output)
        dialog.text(0.06, 0.84, "\n".join(textwrap.fill(line, 110) for line in message.splitlines()), va="top", fontsize=11)
        save = Button(dialog.add_axes([0.06, 0.15, 0.30, 0.065]), "Save Validated & Exit")
        save.set_active(bool(bench.selected and bench.selected["validation"]["valid"]))
        save.label.set_alpha(1 if save.active else 0.4)
        def export_finish():
            bench.select_export()
            close_main()
        save.on_clicked(action(export_finish))
        exit_button = Button(dialog.add_axes([0.39, 0.15, 0.25, 0.065]), "Exit — keep saved files")
        exit_button.on_clicked(lambda event: close_main())
        review = Button(dialog.add_axes([0.68, 0.15, 0.26, 0.065]), "Continue Reviewing")
        review.on_clicked(lambda event: plt.close(dialog))
        dialog.workbench_controls = {"save": save, "exit": exit_button, "review": review}
        display(dialog)

    def confirm_experimental():
        candidate = bench.selected
        dialog = popup("Confirm EXPERIMENTAL export — NOT validated", (11, 7))
        reasons = "\n".join("• " + reason for reason in candidate["validation"]["failure_reasons"])
        dialog.text(0.06, 0.86, candidate["id"] + "\n" + reasons +
                    "\n\nExact candidate CSV will be saved with failed checks.\n"
                    "Deliberate simulation experimentation only; not onboard validated.\n"
                    "Nothing will be launched or enabled. Finish / Exit closes the workbench.", va="top", fontsize=11, wrap=True)
        confirm = Button(dialog.add_axes([0.06, 0.10, 0.55, 0.07]), "Confirm Experimental Export")
        cancel = Button(dialog.add_axes([0.69, 0.10, 0.25, 0.07]), "Cancel")
        def commit():
            bench.select_export(candidate, experimental=True, confirmed=True)
            plt.close(dialog)
        confirm.on_clicked(action(commit))
        cancel.on_clicked(lambda event: plt.close(dialog))
        dialog.workbench_controls = {"confirm": confirm, "cancel": cancel}
        display(dialog)

    def preset_values():
        dialog = popup("Preset Values — exact backend configuration", (14, 8))
        rows = []
        for group, choices in bench.settings["tuning_presets"].items():
            for name in choices:
                values = preset_changes(bench, {group: name})
                rows.append([group, name, "; ".join(f"{key} = {value:g}" for key, value in values.items())])
        axis = dialog.add_axes([0.04, 0.22, 0.92, 0.68])
        axis.axis("off")
        table = axis.table(cellText=rows, colLabels=["Control", "Choice", "Resolved parameter values"],
                           colWidths=[0.23, 0.12, 0.65], cellLoc="left", bbox=[0, 0, 1, 1])
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        current = "; ".join(f"{key}={value:g}" for key, value in tuning.items())
        dialog.text(0.05, 0.16, "Current trial settings (including advanced edits):\n" + "\n".join(textwrap.wrap(current, 130)), fontsize=9)
        dialog.text(0.05, 0.065, "Balanced uses this session's baseline; vehicle limits never participate in presets.\nConfiguration: " + str(bench.settings_path), fontsize=9)
        close = Button(dialog.add_axes([0.80, 0.035, 0.15, 0.045]), "Close Values")
        close.on_clicked(lambda event: plt.close(dialog))
        dialog.workbench_controls = {"rows": rows, "close": close}
        display(dialog)

    def advanced():
        dialog = popup("Advanced Settings — drag OR type, then Apply Settings", (16, 11))
        pending = dict(tuning)
        limits = {key: getattr(bench.baseline, key) for key in bench.settings["advanced_vehicle_limits"]}
        sliders, entries, vehicle_sliders = {}, {}, {}
        status = dialog.text(0.05, 0.035, "Draft only. Apply Settings stages a trial; it does not solve or export.", fontsize=10)
        def hint(message):
            status.set_text("\n".join(textwrap.wrap(message, 145)))
            dialog.canvas.draw_idle()
        dialog.text(0.05, 0.93, "Path / optimization trial settings", fontsize=12, weight="bold")
        dialog.text(0.68, 0.93, "Vehicle / clearance profile", fontsize=12, weight="bold")
        for i, (key, (label, explanation)) in enumerate(TUNING_HELP.items()):
            y = 0.855 - i * 0.098
            bounds = bench.settings["search"]["ranges"][key]
            dialog.text(0.05, y + 0.018, label + f"  [{bounds[0]:g} – {bounds[1]:g}]", fontsize=10)
            dialog.text(0.05, y - 0.008, explanation, fontsize=9)
            slider, entry = numeric_control(dialog, [0.06, y - 0.046, 0.45, 0.02], [0.535, y - 0.051, 0.09, 0.032],
                                             pending[key], bounds, lambda value, name=key: pending.update({name: value}), hint)
            sliders[key], entries[key] = slider, entry
        for i, (key, bounds) in enumerate(bench.settings["advanced_vehicle_limits"].items()):
            y = 0.84 - i * 0.145
            label = "Wall clearance margin [m]" if key == "safety_margin_m" else "Hard curvature limit [1/m]"
            dialog.text(0.68, y + 0.035, label, fontsize=10)
            dialog.text(0.68, y + 0.01, f"Editable profile range: {bounds[0]:g}–{bounds[1]:g}", fontsize=9)
            slider, entry = numeric_control(dialog, [0.68, y - 0.035, 0.20, 0.02], [0.90, y - 0.043, 0.065, 0.034],
                                             limits[key], bounds, lambda value, name=key: limits.update({name: value}), hint)
            vehicle_sliders[key], entries[key] = slider, entry
        dialog.text(0.68, 0.60, "Limit edits require confirmation and a NEW session.\nPrevious candidates/exports keep their original checks.\nAuto-search never changes limits.\nA custom profile does not prove physical vehicle capability.\nDense validation step remains fixed at " + f"{bench.baseline.validation_step_m:g} m.", fontsize=9, va="top")
        dialog.text(0.68, 0.39, "Candidate diagnostics (not export)", fontsize=11, weight="bold")
        selector = TextBox(dialog.add_axes([0.78, 0.33, 0.18, 0.035]), "Candidate ID: ", initial=bench.selected["id"] if bench.selected else "best")
        show = Button(dialog.add_axes([0.68, 0.265, 0.13, 0.04]), "Show Candidate")
        check = Button(dialog.add_axes([0.83, 0.265, 0.13, 0.04]), "Check File")
        def show_candidate():
            c = bench.best_candidate() if selector.text.strip() == "best" else next((c for c in bench.candidates if c["id"] == selector.text.strip()), None)
            if c is None:
                raise ValueError("Unknown candidate ID")
            bench.selected = c
            hint("Candidate shown on the main page. No CSV was exported or selected downstream.")
        show.on_clicked(action(show_candidate))
        check.on_clicked(action(lambda: bench.check_candidate(bench.selected)))
        show.set_active(bool(bench.candidates))
        check.set_active(bool(bench.selected and bench.selected.get("path")))
        show.label.set_alpha(1 if show.active else 0.4)
        check.label.set_alpha(1 if check.active else 0.4)
        dialog.text(0.68, 0.24, "Show: preview the named candidate on the main page.\nCheck: verify its CSV with this session's validator.\nNeither action solves, exports or selects downstream.", fontsize=9, va="top")
        reset = Button(dialog.add_axes([0.05, 0.17, 0.24, 0.045]), "Reset Trial Values")
        reset.on_clicked(lambda event: [slider.set_val(getattr(bench.baseline, key)) for key, slider in sliders.items()])
        apply = Button(dialog.add_axes([0.05, 0.09, 0.25, 0.055]), "Apply Settings & Close")
        cancel = Button(dialog.add_axes([0.33, 0.09, 0.20, 0.055]), "Cancel — discard draft")
        cancel.on_clicked(lambda event: plt.close(dialog))
        def commit_settings():
            tuning.update(pending)
            plt.close(dialog)
            say("Settings staged. Generate Raceline first if needed, then Try My Settings; no solve/export occurred.")
        def apply_settings(event):
            if busy:
                return
            changed = {key: value for key, value in limits.items() if value != getattr(bench.baseline, key)}
            if not changed:
                commit_settings()
                return
            confirmation = popup("Confirm custom offline vehicle / safety profile", (11, 7))
            lines = [f"{key}: {getattr(bench.baseline, key):g} → {value:g}" for key, value in changed.items()]
            confirmation.text(0.06, 0.84, "\n".join(lines) +
                              "\n\nThis creates a NEW session with no inherited validation/selection.\n"
                              "The original exports remain unchanged. Generate and validate again.\n"
                              "These are offline acceptance limits, not new hardware steering capability.\n"
                              "Auto-search cannot alter them.", fontsize=11, va="top")
            confirm = Button(confirmation.add_axes([0.06, 0.12, 0.56, 0.065]), "Confirm New Profile / Session")
            back = Button(confirmation.add_axes([0.68, 0.12, 0.26, 0.065]), "Keep Existing Limits")
            def new_profile():
                nonlocal bench
                bench = bench.with_vehicle_limits(changed, confirmed=True)
                viewport["initialized"] = False
                tuning.update(pending)
                plt.close(confirmation)
                plt.close(dialog)
            confirm.on_clicked(action(new_profile))
            back.on_clicked(lambda event: plt.close(confirmation))
            confirmation.workbench_controls = {"confirm": confirm, "cancel": back}
            display(confirmation)
        apply.on_clicked(apply_settings)
        dialog.workbench_controls = {"sliders": sliders, "vehicle_sliders": vehicle_sliders, "entries": entries,
                                     "reset": reset, "selector": selector, "preview": show, "revalidate": check,
                                     "apply": apply, "cancel": cancel, "pending": pending, "limits": limits}
        display(dialog)

    def open_previous():
        nonlocal bench
        if not bench.latest_export or bench.latest_export.get("error"):
            raise ValueError("No usable previous export for these map hashes")
        from raceline_workbench import Workbench
        bench = Workbench.reopen(bench.latest_export["source_session"])
        tuning.update({key: getattr(bench.baseline, key) for key in TUNING_HELP})
        viewport["initialized"] = False

    def fit_map():
        viewport["initialized"] = False
        refresh()

    callbacks = {"Generate Raceline": lambda: bench.run_baseline(), "Try More Settings": lambda: bench.auto_search(),
                 "Try My Settings": tuned, "Edit Path Manually": lambda: bench.manual_editor(),
                 "Select / Export Validated": lambda: bench.select_export(),
                 "Previous": lambda: browse(-1), "Next": lambda: browse(1),
                 "Best Candidate": lambda: setattr(bench, "selected", bench.best_candidate()),
                 "Fit Map": fit_map, "Open Previous Session": open_previous}
    for label, fn in callbacks.items():
        buttons[label].on_clicked(action(fn))
    for label, fn in (("Export Experimental", confirm_experimental), ("Advanced Tuning / Diagnostics", advanced),
                      ("Preset Values", preset_values), ("Finish / Exit", finish)):
        def open_control(event, callback=fn):
            if not busy:
                try:
                    callback()
                except (ValueError, OSError, RuntimeError) as error:
                    say("Cannot open action: " + str(error))
        buttons[label].on_clicked(open_control)
    compare.on_clicked(lambda label: refresh())
    fig.canvas.mpl_connect("scroll_event", lambda event: zoom_at_cursor(map_axis, event))
    fig.canvas.mpl_connect("close_event", lambda event: [plt.close(dialog) for dialog in popups])
    refresh()
    fig.workbench_controls = {"buttons": buttons, "radios": radios, "refresh": refresh,
                              "action": action, "popups": popups, "tuning": tuning, "compare": compare,
                              "confirm_experimental": confirm_experimental, "advanced": advanced,
                              "preset_values": preset_values, "finish": finish, "current_bench": lambda: bench,
                              "map_axis": map_axis, "path_box": path_box}
    return fig


def show_workbench(bench):
    import matplotlib.pyplot as plt
    if str(plt.get_backend()).lower() == "agg":
        raise RuntimeError("GUI requires a working interactive display; use --headless or saved preview.png")
    figure = create_workbench_figure(bench)
    plt.show()
    return figure.workbench_controls["current_bench"]()
