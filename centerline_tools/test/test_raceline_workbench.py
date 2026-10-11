"""Shared workbench contract tests, independent of GUI clicks and real maps."""

from dataclasses import asdict, replace
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import raceline_workbench as wb  # noqa: E402
from centerline_tools import global_raceline_optimizer as opt  # noqa: E402


def circle(radius=3.0, count=120):
    angle = np.arange(count) * 2 * np.pi / count
    return radius * np.column_stack((np.cos(angle), np.sin(angle)))


def fixture_bench(tmp_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    image = tmp_path / "track.png"
    plt.imsave(image, np.full((160, 160), 255, dtype=np.uint8), cmap="gray")
    map_yaml = tmp_path / "track.yaml"
    map_yaml.write_text(yaml.safe_dump({"image": image.name, "resolution": 0.05, "origin": [-4, -4, 0]}))
    bench = wb.Workbench(image, map_yaml, tmp_path / "session")
    write_phase9(bench, circle())
    bench.prepare()
    return bench


def write_phase9(bench, points):
    opt.save_path_csv(points, bench.output / "centerline_points_smooth.csv")
    np.save(bench.output / "drivable_region.npy", np.ones((160, 160), dtype=np.uint8))
    opt.save_report({"source_files": bench.identity}, bench.output / "centerline_metadata.yaml")
    opt.save_report({"generation": {
        "inputs": bench.identity,
        "generator_sha256": bench.source_hashes["centerline_reference_generator.py"],
        "minimum_loop_coverage_ratio": bench.settings["centerline"]["minimum_loop_coverage_ratio"]},
        "outputs": {name: opt.file_sha256(bench.output / name) for name in
                    ("centerline_points_smooth.csv", "drivable_region.npy", "centerline_metadata.yaml")}},
        bench.output / "phase9_provenance.yaml")


def fake_solver(reference, lower, upper, config, retain_failed_candidate=False):
    return opt.OptimizationResult(circle(), np.zeros(len(reference)),
                                  [{"success": True, "name": str(i)} for i in range(3)])


def test_deterministic_baseline_and_ifac_explicit_profile():
    settings, config = wb.load_settings(ROOT / "config/raceline_workbench.yaml")
    assert config == opt.load_optimizer_config(ROOT / "config/global_raceline_optimizer.yaml")
    assert settings["centerline"]["minimum_loop_coverage_ratio"] == 0.98
    ifac, fixture = wb.load_settings(ROOT / "config/raceline_workbench_ifac.yaml")
    assert fixture == opt.load_optimizer_config(ROOT / "config/global_raceline_optimizer_ifac_roboracer.yaml")
    assert ifac["centerline"]["minimum_loop_coverage_ratio"] == 0.94
    assert fixture.safety_margin_m >= config.safety_margin_m
    assert wb.profile_hash(asdict(config)) == wb.profile_hash(asdict(config))


def test_search_order_budget_and_hard_constraints():
    settings, config = wb.load_settings(ROOT / "config/raceline_workbench.yaml")
    first = wb.search_schedule(config, settings)
    assert first == wb.search_schedule(config, settings)
    assert len(first) == 16
    assert [phase for phase, _ in first] == ["geometry"] * 8 + ["objective"] * 8
    for _, candidate in first:
        assert all(getattr(candidate, key) == getattr(config, key) for key in wb.HARD_KEYS)
        assert all(getattr(candidate, key) == getattr(config, key) for key in asdict(config) if key not in wb.SEARCH_KEYS)
    settings["search"]["maximum_candidates"] = 3
    assert len(wb.search_schedule(config, settings)) == 3
    with pytest.raises(ValueError, match="never HARD"):
        wb.search_profile(config, {"maximum_curvature_inv_m": 9}, settings["search"]["ranges"])
    with pytest.raises(ValueError, match="outside declared"):
        wb.search_profile(config, {"optimization_spacing_m": 0.001}, settings["search"]["ranges"])


def test_baseline_pass_and_gui_creation(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    monkeypatch.setattr(opt, "optimize_global_raceline", fake_solver)
    bench.run_baseline()
    assert bench.state == "AUTO_PASS"
    assert (bench.output / "raceline_points_optimized.csv").exists()
    bench.auto_search()
    assert len(bench.candidates) == 1
    from centerline_tools.raceline_workbench_view import create_workbench_figure
    import matplotlib.pyplot as plt
    fig = create_workbench_figure(bench)
    assert fig.workbench_controls["buttons"]["Select / Export Validated"].active
    plt.close(fig)


def test_failure_tuning_required_diagnostics_and_budget(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    def failed(reference, lower, upper, config, retain_failed_candidate=False):
        return opt.OptimizationResult(reference, np.zeros(len(reference)), [{"success": False}])
    monkeypatch.setattr(opt, "optimize_global_raceline", failed)
    bench.run_baseline()
    assert bench.state == "AUTO_TUNING_REQUIRED"
    v = bench.selected["validation"]
    assert not v["valid"] and not v["solver_converged"]
    assert np.isfinite(v["minimum_clearance_m"])
    assert np.isfinite(v["curvature_abs_max_inv_m"])
    assert v["curvature_rms_before"] is not None
    assert not (bench.output / "raceline_points_optimized.csv").exists()
    bench.settings["search"]["maximum_candidates"] = 2
    bench.auto_search()
    bench.auto_search()
    assert bench.state == "AUTO_SEARCH_EXHAUSTED"
    assert len(bench.candidates) == 3
    with pytest.raises(ValueError, match="promotion is disabled"):
        bench.promote()


def test_search_stop_on_first_pass(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    counter = []
    def solver(reference, lower, upper, config, retain_failed_candidate=False):
        counter.append(config)
        result = fake_solver(reference, lower, upper, config)
        if len(counter) == 1:
            result.stages = [{"success": False}]
        return result
    monkeypatch.setattr(opt, "optimize_global_raceline", solver)
    bench.run_baseline()
    bench.auto_search()
    assert bench.state == "AUTO_TUNED_PASS"
    assert len(counter) == 2


def test_manual_common_validator_rejects_legacy_accepted_curvature(tmp_path):
    bench = fixture_bench(tmp_path)
    # Old 24-degree p99 gate could accept this circle (~1.2 /m); hard 1.082 cannot.
    manual = tmp_path / "manual.csv"
    opt.save_path_csv(circle(radius=1 / 1.2), manual)
    result = bench.import_manual(manual)
    assert bench.state == "NO_VALIDATED_RACELINE"
    assert not result["validation"]["curvature_valid"]
    assert result["validation"]["curvature_excess_percent"] > 0
    with pytest.raises(ValueError):
        bench.promote()
    assert not (bench.output / "raceline_points_optimized.csv").exists()


def test_manual_pass_reverse_explicit_and_revalidation(tmp_path):
    bench = fixture_bench(tmp_path)
    manual = tmp_path / "manual.csv"
    opt.save_path_csv(bench.centerline, manual)
    assert bench.import_manual(manual)["validation"]["valid"]
    assert bench.state == "MANUAL_PASS"
    opt.save_path_csv(bench.centerline[::-1], manual)
    assert not bench.import_manual(manual)["validation"]["direction_preserved"]
    assert not (bench.output / "raceline_points_optimized.csv").exists()
    assert bench.import_manual(manual, "reverse")["validation"]["valid"]
    # Cached PASS cannot mask a file changed after validation.
    opt.save_path_csv(circle(radius=0.8), bench.selected["path"])
    with pytest.raises(ValueError, match="revalidation"):
        bench.promote()


def test_common_validator_corridor_curvature_topology_and_geometry(tmp_path):
    bench = fixture_bench(tmp_path)
    unsafe = circle(radius=5)
    assert not bench.validate(unsafe, bench.baseline)["corridor_valid"]
    assert not bench.validate(circle(0.8), bench.baseline)["curvature_valid"]
    backwards = bench.centerline[::-1]
    assert not bench.validate(backwards, bench.baseline)["direction_preserved"]
    crossed = np.array([[0,0],[2,0],[2,2],[0,2],[0,0],[-2,0],[-2,-2],[0,-2]], dtype=float)
    assert bench.validate(crossed, bench.baseline)["self_intersection"] is not None
    invalid = bench.centerline.copy(); invalid[1, 0] = np.nan
    assert not bench.validate(invalid, bench.baseline)["finite_nondegenerate"]


def test_common_validator_rejects_smoothness_degradation(tmp_path):
    bench = fixture_bench(tmp_path)
    angles = np.arange(120) * 2 * np.pi / 120
    radius = 3 + 0.04 * np.sin(5 * angles)
    noisy = radius[:,None] * np.column_stack((np.cos(angles), np.sin(angles)))
    v = bench.validate(noisy, bench.baseline)
    assert v["curvature_valid"] and v["corridor_valid"]
    assert not v["curvature_rms_not_degraded"]
    assert not v["curvature_rate_rms_not_degraded"]


def test_phase9_hashes_and_output_isolation(tmp_path):
    bench = fixture_bench(tmp_path)
    other_identity = dict(bench.identity, map_image_sha256="wrong")
    with pytest.raises(ValueError, match="stale/mismatched"):
        wb.check_phase9(bench.output, other_identity)
    with pytest.raises(ValueError, match="contain/overwrite"):
        wb.Workbench(bench.identity["map_image"], bench.identity["map_yaml"], bench.output.parent)
    assert wb.default_output("track.png", bench.identity) != wb.default_output("track.png", other_identity)
    # Cross-map ownership is rejected even if filenames happen to match.
    owner = bench.output / "workbench_inputs.yaml"
    opt.save_report(other_identity, owner)
    with pytest.raises(ValueError, match="different/unverified"):
        wb.Workbench(bench.identity["map_image"], bench.identity["map_yaml"], bench.output)


def test_reuse_rejects_changed_phase9_artifacts(tmp_path):
    bench = fixture_bench(tmp_path)
    (bench.output / "centerline_points_smooth.csv").write_text("index,x,y\n0,1,2\n")
    with pytest.raises(ValueError, match="artifact changed"):
        bench.prepare()


def test_machine_report_is_deterministic(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    monkeypatch.setattr(opt, "optimize_global_raceline", fake_solver)
    bench.run_baseline()
    before = (bench.output / "raceline_workbench_report.yaml").read_bytes()
    bench.save_report()
    assert (bench.output / "raceline_workbench_report.yaml").read_bytes() == before
    again = wb.Workbench(bench.identity["map_image"], bench.identity["map_yaml"], bench.output)
    again.run_baseline()
    assert (bench.output / "raceline_workbench_report.yaml").read_bytes() == before


def test_ifac_reviewed_artifact_passes_common_production_validator():
    config = opt.load_optimizer_config(ROOT / "config/global_raceline_optimizer_ifac_roboracer.yaml")
    folder = ROOT / "output_backup/ifac_roboracer"
    meta = opt.MapMetadata.from_yaml(ROOT.parent / "f1tenth_gym_ros/maps/ifac_roboracer.yaml")
    safe, clearance, _ = opt.build_safe_mask(np.load(folder / "drivable_region.npy"), meta, config.safety_margin_m)
    reference = opt.load_path_csv(folder / "centerline_points_smooth.csv")
    points = opt.load_path_csv(folder / "raceline_points_optimized.csv")
    v = opt.diagnose_production_path(points, safe, clearance, meta, config,
                                     opt.signed_area(reference), centerline=reference)
    assert v["valid"], v["failure_reasons"]


def test_manual_legacy_preview_cannot_define_production_pass(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    import manual_raceline_generator as manual
    monkeypatch.setattr(manual.rg, "raceline_curvature_is_ok", lambda points: (True, {}, 100))
    context = {"config": bench.baseline, "metadata": bench.metadata, "mask": bench.mask,
               "clearance": bench.clearance, "reference": bench.centerline}
    assert not manual.production_validation(context, circle(0.8))["valid"]
    assert manual.production_validation(context, bench.centerline)["valid"]


def test_solver_failure_retains_last_candidate_without_claiming_convergence():
    angles = np.arange(80) * 2 * np.pi / 80
    radius = 3 + 0.2 * np.sin(5 * angles)
    reference = radius[:,None] * np.column_stack((np.cos(angles), np.sin(angles)))
    config = replace(opt.OptimizerConfig(), maximum_iterations_per_stage=1)
    result = opt.optimize_global_raceline(reference, np.full(80, -0.5), np.full(80, 0.5),
                                         config, retain_failed_candidate=True)
    assert not all(stage["success"] for stage in result.stages)
    assert np.all(np.isfinite(result.points))
    with pytest.raises(RuntimeError, match="optimizer stage"):
        opt.optimize_global_raceline(reference, np.full(80, -0.5), np.full(80, 0.5), config)


def test_search_config_rejects_hard_override(tmp_path):
    settings_path = ROOT / "config/raceline_workbench.yaml"
    raw = yaml.safe_load(settings_path.read_text())
    raw["baseline_optimizer_config"] = str(ROOT / "config/global_raceline_optimizer.yaml")
    raw["search"]["geometry_candidates"].append({"safety_margin_m": 0.1})
    config_path = tmp_path / "invalid.yaml"
    config_path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match="never HARD"):
        wb.load_settings(config_path)


def test_map_image_mismatch_rejected_and_inputs_preserved(tmp_path):
    bench = fixture_bench(tmp_path)
    image = Path(bench.identity["map_image"])
    map_yaml = Path(bench.identity["map_yaml"])
    image_before, yaml_before = image.read_bytes(), map_yaml.read_bytes()
    other = tmp_path / "wrong.png"
    other.write_bytes(b"wrong image")
    with pytest.raises(ValueError, match="disagrees"):
        wb.input_identity(other, map_yaml)
    assert image.read_bytes() == image_before and map_yaml.read_bytes() == yaml_before


def test_invalid_candidate_promotion_does_not_trust_cached_pass(tmp_path):
    bench = fixture_bench(tmp_path)
    manual = tmp_path / "manual.csv"
    opt.save_path_csv(circle(0.8), manual)
    candidate = bench.import_manual(manual)
    candidate["validation"]["valid"] = True  # hypothetical stale/legacy UI claim
    with pytest.raises(ValueError, match="revalidation"):
        bench.promote(candidate)
    assert not (bench.output / "raceline_points_optimized.csv").exists()


def test_manual_editor_figure_creation_uses_production_context(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    import manual_raceline_generator as manual
    import matplotlib.pyplot as plt
    monkeypatch.setattr(manual.rg, "try_enable_interactive_matplotlib_backend", lambda: True)
    monkeypatch.setattr(plt, "show", lambda **kwargs: plt.close("all"))
    rows = [{"x": float(x), "y": float(y), "curvature": 1 / 3} for x, y in bench.centerline]
    context = {"config": bench.baseline, "metadata": bench.metadata, "mask": bench.mask,
               "clearance": bench.clearance, "reference": bench.centerline}
    saved = manual.show_manual_raceline_editor(
        np.ones((160, 160), dtype=np.uint8) * 255,
        {"resolution": 0.05, "origin": [-4, -4, 0]}, rows, bench.mask,
        str(bench.output / "manual"), production_context=context)
    assert not saved
    assert not (bench.output / "raceline_points_optimized.csv").exists()


def test_changed_session_profile_requires_new_output_directory(tmp_path):
    bench = fixture_bench(tmp_path)
    bench.save_report()
    cfg = yaml.safe_load((ROOT / "config/raceline_workbench.yaml").read_text())
    cfg["baseline_optimizer_config"] = str(ROOT / "config/global_raceline_optimizer.yaml")
    cfg["search"]["maximum_candidates"] = 1
    path = tmp_path / "changed.yaml"
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError, match="session profile changed"):
        wb.Workbench(bench.identity["map_image"], bench.identity["map_yaml"], bench.output, path)


def test_promotion_rechecks_phase9_hashes(tmp_path):
    bench = fixture_bench(tmp_path)
    manual = tmp_path / "manual.csv"
    opt.save_path_csv(bench.centerline, manual)
    bench.import_manual(manual)
    np.save(bench.output / "drivable_region.npy", np.zeros((160, 160), dtype=np.uint8))
    with pytest.raises(ValueError, match="qualified Phase-9"):
        bench.promote()
    assert not (bench.output / "raceline_points_optimized.csv").exists()


def test_malformed_workbench_config_is_clear_preflight_error(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text("centerline: []\nsearch: []\n")
    with pytest.raises(ValueError, match="invalid workbench configuration"):
        wb.load_settings(path)


def test_legacy_manual_cancel_keeps_zero_exit_status(tmp_path, monkeypatch):
    bench = fixture_bench(tmp_path)
    import manual_raceline_generator as manual
    monkeypatch.setattr(manual.rg, "OUTPUT_DIR", str(bench.output))
    monkeypatch.setattr(manual, "show_manual_raceline_editor", lambda *args, **kwargs: False)
    monkeypatch.setattr(sys, "argv", ["manual_raceline_generator.py",
                                      bench.identity["map_image"], bench.identity["map_yaml"], "--fresh"])
    assert manual.main() == 0


def test_production_validation_requires_centerline_evidence(tmp_path):
    bench = fixture_bench(tmp_path)
    report = opt.diagnose_production_path(bench.centerline, bench.safe, bench.clearance,
                                         bench.metadata, bench.baseline,
                                         opt.signed_area(bench.centerline))
    assert report["corridor_valid"] and report["curvature_valid"]
    assert not report["valid"]
    assert "centerline required" in "; ".join(report["failure_reasons"])


def test_ifac_automatic_workbench_cli(tmp_path):
    """Preserve the actual no-display generation contract, not just a stored CSV."""
    import os
    import subprocess
    map_root = ROOT.parent / "f1tenth_gym_ros/maps"
    output = tmp_path / "ifac_generated"
    command = [sys.executable, str(ROOT / "raceline_workbench.py"),
               str(map_root / "ifac_roboracer.png"), str(map_root / "ifac_roboracer.yaml"),
               "--config", str(ROOT / "config/raceline_workbench_ifac.yaml"),
               "--headless", "--output-dir", str(output)]
    environment = dict(os.environ, DISPLAY="", PYTHONDONTWRITEBYTECODE="1")
    result = subprocess.run(command, env=environment, capture_output=True,
                            text=True, timeout=180)
    assert result.returncode == 0, result.stdout + result.stderr
    report = yaml.safe_load((output / "raceline_workbench_report.yaml").read_text())
    assert report["result"] == "AUTO_PASS"
    assert report["final_validation"]["valid"]
    assert report["final_validation"]["curvature_rms_not_degraded"]
    assert report["final_validation"]["curvature_rate_rms_not_degraded"]
    assert report["outputs"]["raceline_sha256"] == opt.file_sha256(
        output / "raceline_points_optimized.csv")
