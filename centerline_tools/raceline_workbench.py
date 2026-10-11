#!/usr/bin/env python3
"""Offline raceline generation with strict validation and explicit session selection."""

import argparse
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import uuid

import numpy as np
import scipy
import yaml

from centerline_tools import global_raceline_optimizer as opt
from centerline_tools.raceline_selection import resolve_selection

ROOT = Path(__file__).resolve().parent
SEARCH_KEYS = (
    "optimization_spacing_m", "maximum_lateral_offset_m", "path_length_weight",
    "curvature_rate_weight", "offset_slope_weight", "comfortable_curvature_inv_m",
)
HARD_KEYS = ("safety_margin_m", "maximum_curvature_inv_m", "validation_step_m")
SUCCESS_STATES = ("AUTO_PASS", "AUTO_TUNED_PASS", "MANUAL_PASS")


def save_yaml(report, path):
    """Generated diagnostics must be readable from the host even after root exec."""
    opt.save_report(report, path)
    Path(path).chmod(0o644)


def save_csv(points, path):
    opt.save_path_csv(points, path)
    Path(path).chmod(0o644)


def failure_classification(validation):
    if validation["valid"]:
        return ["NONE"]
    labels = []
    for label, key in (("SOLVER", "solver_converged"), ("GEOMETRY", "finite_nondegenerate"),
                       ("CLEARANCE", "corridor_valid"), ("CURVATURE", "curvature_valid"),
                       ("DIRECTION", "direction_preserved")):
        if validation[key] is False:
            labels.append(label)
    if validation["self_intersection"] is not None:
        labels.append("TOPOLOGY")
    if validation["finite_nondegenerate"] and (not validation["curvature_rms_not_degraded"] or
                                               not validation["curvature_rate_rms_not_degraded"]):
        labels.append("SMOOTHNESS")
    return labels


def profile_hash(mapping):
    return hashlib.sha256(json.dumps(mapping, sort_keys=True).encode()).hexdigest()


def load_settings(path):
    """Turn malformed editable YAML into a concise preflight error."""
    try:
        return _load_settings(path)
    except (KeyError, TypeError, AttributeError, yaml.YAMLError) as error:
        raise ValueError("invalid workbench configuration: " + str(error)) from error


def _load_settings(path):
    """Resolve a single optional parent file; paths belong to their defining YAML."""
    path = Path(path).resolve()
    raw = yaml.safe_load(path.read_text())
    if not isinstance(raw, dict):
        raise ValueError("workbench config must be a mapping")
    if "extends" in raw:
        parent = (path.parent / raw.pop("extends")).resolve()
        if parent == path or "extends" in yaml.safe_load(parent.read_text()):
            raise ValueError("only one workbench config inheritance level is supported")
        settings, _ = load_settings(parent)
        settings.update(raw)
    else:
        settings = raw
    if "baseline_optimizer_config" in raw:
        settings["baseline_optimizer_config"] = str(
            (path.parent / raw["baseline_optimizer_config"]).resolve())
    unknown = set(settings) - {"baseline_optimizer_config", "centerline", "vehicle_profile", "search", "tuning_presets", "advanced_vehicle_limits"}
    if unknown:
        raise ValueError(f"unknown workbench settings: {sorted(unknown)}")
    advanced = settings.get("vehicle_profile", {})
    if set(advanced) - set(HARD_KEYS):
        raise ValueError("vehicle_profile may override only HARD constraints")
    config = replace(opt.load_optimizer_config(settings["baseline_optimizer_config"]), **advanced)
    config.validate()
    coverage = settings["centerline"]["minimum_loop_coverage_ratio"]
    if not 0 < float(coverage) <= 1:
        raise ValueError("centerline coverage must be in (0, 1]")
    search = settings["search"]
    if set(search["ranges"]) != set(SEARCH_KEYS):
        raise ValueError("search ranges must declare exactly the six supported SEARCH keys")
    if not isinstance(search["maximum_candidates"], int) or not 0 <= search["maximum_candidates"] <= 100:
        raise ValueError("maximum_candidates must be an integer in [0, 100]")
    for key, bounds in search["ranges"].items():
        if len(bounds) != 2 or not all(np.isfinite(bounds)) or bounds[0] <= 0 or bounds[0] > bounds[1]:
            raise ValueError(f"invalid range for {key}")
    # Validate the whole schedule even when the budget truncates it.
    for changes in search["geometry_candidates"] + [search["objective_geometry"]] + search["objective_candidates"]:
        search_profile(config, changes, search["ranges"])
    for choices in settings.get("tuning_presets", {}).values():
        for changes in choices.values():
            search_profile(config, changes, search["ranges"])
    for key, bounds in settings.get("advanced_vehicle_limits", {}).items():
        if (key not in ("safety_margin_m", "maximum_curvature_inv_m") or len(bounds) != 2 or
                not all(np.isfinite(bounds)) or not 0 < bounds[0] <= bounds[1]):
            raise ValueError("invalid advanced vehicle-limit range: " + key)
    return settings, config


def search_profile(baseline, changes, ranges):
    if set(changes) - set(SEARCH_KEYS):
        raise ValueError("routine search may change only SEARCH parameters, never HARD constraints")
    for key, value in changes.items():
        if not np.isfinite(value) or not ranges[key][0] <= value <= ranges[key][1]:
            raise ValueError(f"{key}={value} outside declared range {ranges[key]}")
    config = replace(baseline, **changes)
    config.validate()
    return config


def search_schedule(baseline, settings):
    """Baseline-relative Phase A then fixed-geometry Phase B, budgeted/deduplicated."""
    search = settings["search"]
    changes = [("geometry", v) for v in search["geometry_candidates"]]
    changes += [("objective", dict(search["objective_geometry"], **v))
                for v in search["objective_candidates"]]
    seen = {profile_hash(asdict(baseline))}
    candidates = []
    for phase, values in changes:
        config = search_profile(baseline, values, search["ranges"])
        digest = profile_hash(asdict(config))
        if digest not in seen:
            seen.add(digest)
            candidates.append((phase, config))
    return candidates[:search["maximum_candidates"]]


def input_identity(image, map_yaml):
    image, map_yaml = Path(image).resolve(), Path(map_yaml).resolve()
    raw = yaml.safe_load(map_yaml.read_text())
    if not isinstance(raw, dict) or "image" not in raw:
        raise ValueError("map YAML must be a mapping containing image")
    metadata = opt.MapMetadata.from_yaml(map_yaml)
    linked = (map_yaml.parent / raw["image"]).resolve()
    if opt.file_sha256(image) != opt.file_sha256(linked):
        raise ValueError("supplied map image disagrees with map YAML image")
    if abs(metadata.origin_yaw_rad) > 1e-12:
        raise ValueError("Phase-9 generator does not support rotated map origins")
    if raw.get("negate", 0) != 0:
        raise ValueError("Phase-9 generator requires negate: 0")
    return {"map_image": str(image), "map_image_sha256": opt.file_sha256(image),
            "map_yaml": str(map_yaml), "map_yaml_sha256": opt.file_sha256(map_yaml)}


def default_output(image, identity):
    digest = profile_hash({k: v for k, v in identity.items() if k.endswith("sha256")})[:12]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return ROOT / "centerline_output" / "workbench" / f"{Path(image).stem}-{stamp}-{digest}-{uuid.uuid4().hex[:6]}"


def check_phase9(directory, identity):
    directory = Path(directory)
    metadata = yaml.safe_load((directory / "centerline_metadata.yaml").read_text())
    sources = metadata.get("source_files", {})
    for key in ("map_image_sha256", "map_yaml_sha256"):
        if sources.get(key) != identity[key]:
            raise ValueError("stale/mismatched Phase-9 inputs: " + key)
    for name in ("centerline_points_smooth.csv", "drivable_region.npy"):
        if not (directory / name).is_file():
            raise ValueError("missing Phase-9 artifact: " + name)


def latest_export(identity, roots, exclude=None):
    """Discover by map hashes, read-only. Never silently activate/fallback selection."""
    matches = []
    for root in set(Path(p).resolve() for p in roots):
        if not root.is_dir():
            continue
        for folder in root.iterdir():
            if not folder.is_dir() or folder == exclude:
                continue
            try:
                owner = yaml.safe_load((folder / "workbench_inputs.yaml").read_text())
                if any(owner.get(k) != identity[k] for k in ("map_yaml_sha256", "map_image_sha256")):
                    continue
                manifest = folder / "selected_raceline.yaml"
                path = manifest if manifest.exists() else folder / "raceline_points_optimized.csv"
                if path.exists():
                    matches.append((path.stat().st_mtime_ns, str(folder), path))
            except (OSError, TypeError, AttributeError, yaml.YAMLError):
                continue
    if not matches:
        return None
    _, name, path = max(matches)
    result = {"source_session": name, "candidate_kind": "previous_export"}
    try:
        report = yaml.safe_load((Path(name) / "raceline_workbench_report.yaml").read_text())
        if path.name == "selected_raceline.yaml":
            selected = resolve_selection(path, True, identity["map_yaml"])
            csv_path = Path(selected["raceline_csv"])
            evidence = yaml.safe_load(Path(selected["validation_report"]).read_text())
            result.update(status=selected["status"], id=selected["candidate_id"], manifest=str(path))
        else:
            evidence = yaml.safe_load((Path(name) / "raceline_validation.yaml").read_text())
            csv_path = path
            if not evidence["valid"] or evidence["raceline_sha256"] != opt.file_sha256(path):
                raise ValueError("previous validated export changed or failed its declared checks")
            result.update(status="validated", id=evidence["candidate_id"], manifest=None)
        check_phase9(name, identity)
        provenance = yaml.safe_load((Path(name) / "phase9_provenance.yaml").read_text())
        for asset, digest in provenance["outputs"].items():
            if opt.file_sha256(Path(name) / asset) != digest:
                raise ValueError("previous centerline artifact changed: " + asset)
        result.update(path=str(csv_path), path_sha256=opt.file_sha256(csv_path),
                      validation=evidence, baseline_profile=report["baseline_profile"])
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        result["error"] = str(error)  # Most recent broken export is shown, not replaced by an older one.
    return result


def candidate_rank(candidate):
    """Diagnostic preference only; never changes production acceptance."""
    v = candidate["validation"]
    if v["valid"]:
        return (0, 0.0, 0.0, candidate["id"])
    if not v["finite_nondegenerate"] or v["solver_converged"] is False or v["self_intersection"] is not None or not v["direction_preserved"]:
        group = 4
    elif v["corridor_valid"] and not v["curvature_valid"]:
        group = 1
    elif v["curvature_valid"] and not v["corridor_valid"]:
        group = 2
    else:
        group = 3
    return (group, v.get("curvature_excess_inv_m") or 0.0,
            v.get("clearance_deficit_m") or 0.0, candidate["id"])


class Workbench:
    """Shared non-GUI state/actions; GUI and CLI call these same methods."""

    def __init__(self, image, map_yaml, output_dir=None, config_path=None):
        self.identity = input_identity(image, map_yaml)
        self.settings_path = Path(config_path or ROOT / "config/raceline_workbench.yaml").resolve()
        self.settings, self.baseline = load_settings(self.settings_path)
        self.output = Path(output_dir or default_output(image, self.identity)).resolve()
        # Do not claim arbitrary shared legacy directories or place inputs under outputs.
        guarded = [Path(self.identity["map_image"]), Path(self.identity["map_yaml"]),
                   Path(self.settings["baseline_optimizer_config"])]
        if self.settings_path != self.output / "session_profile.yaml":
            guarded.append(self.settings_path)
        if any(self.output == p or self.output in p.parents for p in guarded):
            raise ValueError("output directory must not contain/overwrite any supplied input")
        owner = self.output / "workbench_inputs.yaml"
        if self.output.exists() and any(self.output.iterdir()):
            if not owner.is_file() or yaml.safe_load(owner.read_text()) != self.identity:
                raise ValueError("output directory belongs to different/unverified inputs; use a new --output-dir")
        previous_report = self.output / "raceline_workbench_report.yaml"
        if previous_report.exists():
            previous = yaml.safe_load(previous_report.read_text())
            # UI-only preset additions do not invalidate V1 solver/Phase-9 sessions.
            ui_keys = ("tuning_presets", "advanced_vehicle_limits")
            old_settings = {k: v for k, v in previous["configuration"]["resolved"].items() if k not in ui_keys}
            new_settings = {k: v for k, v in self.settings.items() if k not in ui_keys}
            if (profile_hash(old_settings) != profile_hash(new_settings) or
                    previous["baseline_profile_sha256"] != profile_hash(asdict(self.baseline))):
                raise ValueError("session profile changed; use a new --output-dir")
            archive = self.output / "diagnostics" / ("previous_report_" + opt.file_sha256(previous_report)[:16] + ".yaml")
            archive.parent.mkdir(parents=True, exist_ok=True)
            archive.write_bytes(previous_report.read_bytes())
        self.output.mkdir(parents=True, exist_ok=True)
        save_yaml(self.identity, owner)
        self.metadata = opt.MapMetadata.from_yaml(self.identity["map_yaml"])
        self.required = self.baseline.safety_margin_m + self.metadata.resolution_m / np.sqrt(2)
        self.candidates = []
        self.state = "NO_VALIDATED_RACELINE"
        self.centerline_status = "NOT_RUN"
        self.error = None
        self.centerline = None
        self.selected = None
        self.outputs = {}
        self.active_selection = None
        self.latest_export = latest_export(self.identity, (self.output.parent, ROOT / "centerline_output/workbench"), self.output)
        self.source_hashes = {str(p.relative_to(ROOT)): opt.file_sha256(p) for p in
                              (ROOT / "raceline_workbench.py", ROOT / "centerline_reference_generator.py",
                               ROOT / "centerline_tools/global_raceline_optimizer.py",
                               ROOT / "manual_raceline_generator.py",
                               ROOT / "centerline_tools/raceline_workbench_view.py")}

    def with_vehicle_limits(self, changes, confirmed=False):
        """Explicit profile change creates a new session; previous validation stays historical."""
        if not confirmed:
            raise ValueError("Vehicle/safety limit changes require explicit confirmation")
        ranges = self.settings["advanced_vehicle_limits"]
        if set(changes) - set(ranges):
            raise ValueError("Only declared advanced vehicle limits may be edited")
        for key, value in changes.items():
            if not np.isfinite(value) or not ranges[key][0] <= value <= ranges[key][1]:
                raise ValueError(key + " outside advanced profile range")
        config = replace(self.baseline, **changes)
        config.validate()
        if config == self.baseline:
            return self
        folder = default_output(self.identity["map_image"], self.identity)
        folder.mkdir(parents=True, exist_ok=False)
        settings = dict(self.settings)
        settings["vehicle_profile"] = {k: getattr(config, k) for k in HARD_KEYS}
        profile = folder / "session_profile.yaml"
        save_yaml(settings, profile)
        save_yaml(self.identity, folder / "workbench_inputs.yaml")
        new = Workbench(self.identity["map_image"], self.identity["map_yaml"], folder, profile)
        new.save_report()
        return new

    @classmethod
    def reopen(cls, directory):
        """Explicitly restore a session, qualifying inputs/Phase 9 before reuse."""
        directory = Path(directory).expanduser().resolve()
        previous = yaml.safe_load((directory / "raceline_workbench_report.yaml").read_text())
        identity = previous["inputs"]
        bench = cls(identity["map_image"], identity["map_yaml"], directory,
                    previous["configuration"]["path"])
        bench.candidates = previous["attempted_candidates"]
        for candidate in bench.candidates:
            candidate.setdefault("tool_source_sha256", previous["source_sha256"])
        bench.selected = next((c for c in bench.candidates if c["id"] == previous["selected_candidate_id"]), None)
        bench.state = previous["result"]
        bench.outputs = previous["outputs"]
        if bench.outputs and opt.file_sha256(bench.outputs["raceline"]) != bench.outputs["raceline_sha256"]:
            raise ValueError("Session validated output changed")
        for candidate in bench.candidates:
            if candidate.get("path"):
                path = Path(candidate["path"])
                if not path.is_file() or (candidate.get("path_sha256") and opt.file_sha256(path) != candidate["path_sha256"]):
                    raise ValueError("Session candidate changed/missing: " + candidate["id"])
        manifest = directory / "selected_raceline.yaml"
        if manifest.exists():
            bench.active_selection = resolve_selection(manifest, allow_experimental=True)
        bench.prepare()
        bench.save_report()
        return bench

    def prepare(self):
        """Reuse only hash-qualified own exports; otherwise invoke existing Phase 9."""
        metadata_path = self.output / "centerline_metadata.yaml"
        provenance_path = self.output / "phase9_provenance.yaml"
        expected = {
            "inputs": self.identity,
            "generator_sha256": self.source_hashes["centerline_reference_generator.py"],
            "minimum_loop_coverage_ratio": self.settings["centerline"]["minimum_loop_coverage_ratio"],
        }
        if metadata_path.exists():
            check_phase9(self.output, self.identity)
            provenance = yaml.safe_load(provenance_path.read_text()) if provenance_path.exists() else {}
            if provenance.get("generation") != expected:
                raise ValueError("stale Phase-9 generator/settings; use a new output directory")
            for name, digest in provenance.get("outputs", {}).items():
                if opt.file_sha256(self.output / name) != digest:
                    raise ValueError("Phase-9 artifact changed: " + name)
            if set(provenance.get("outputs", {})) != {"centerline_points_smooth.csv", "drivable_region.npy", "centerline_metadata.yaml"}:
                raise ValueError("Phase-9 provenance is incomplete")
        else:
            env = dict(os.environ, CENTERLINE_HEADLESS="1")
            command = [sys.executable, str(ROOT / "centerline_reference_generator.py"),
                       self.identity["map_image"], self.identity["map_yaml"],
                       "--output-dir", str(self.output), "--minimum-loop-coverage",
                       str(expected["minimum_loop_coverage_ratio"])]
            with (self.output / "centerline_generation.log").open("w") as log:
                subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
            check_phase9(self.output, self.identity)
            save_yaml({"generation": expected, "outputs": {
                name: opt.file_sha256(self.output / name) for name in
                ("centerline_points_smooth.csv", "drivable_region.npy", "centerline_metadata.yaml")}}, provenance_path)
        self.phase9_hashes = {name: opt.file_sha256(self.output / name) for name in
                              ("centerline_points_smooth.csv", "drivable_region.npy", "centerline_metadata.yaml")}
        self.centerline = opt.load_path_csv(self.output / "centerline_points_smooth.csv")
        self.mask = np.load(self.output / "drivable_region.npy", allow_pickle=False)
        self.centerline_status = "PASS"
        self.safe, self.clearance, self.required = opt.build_safe_mask(
            self.mask, self.metadata, self.baseline.safety_margin_m)
        self.centerline_status = "PASS"
        self.save_report()

    def invalidate_promotion(self):
        """Retain a prior valid output as history, never leave it as current output."""
        # Trying another candidate must never silently change an operator selection.
        if (self.output / "selected_raceline.yaml").exists():
            self.active_selection = resolve_selection(self.output / "selected_raceline.yaml", True)
            return
        csv_path = self.output / "raceline_points_optimized.csv"
        if csv_path.exists():
            history = self.output / "candidate" / ("previous_validated_" + opt.file_sha256(csv_path)[:12])
            history.mkdir(parents=True, exist_ok=True)
            for name in ("raceline_points_optimized.csv", "raceline_validation.yaml"):
                path = self.output / name
                if path.exists():
                    os.replace(path, history / name)
        self.outputs = {}

    def validate(self, points, config, solver_converged=True, direction="normal"):
        reference = self.centerline if direction == "normal" else self.centerline[::-1]
        return opt.diagnose_production_path(
            points, self.safe, self.clearance, self.metadata, config,
            opt.signed_area(reference), centerline=reference,
            solver_converged=solver_converged)

    def attempt(self, config, phase="baseline"):
        self.invalidate_promotion()
        self.state = "AUTO_TUNING_REQUIRED"
        candidate_id = f"{len(self.candidates):03d}_{phase}"
        folder = self.output / "candidate" / candidate_id
        folder.mkdir(parents=True, exist_ok=True)
        points = None
        stages = []
        error = None
        try:
            reference, _ = opt.resample_closed_loop(self.centerline, config.optimization_spacing_m)
            lower, upper = opt.lateral_offset_bounds(
                reference, opt.path_normals(reference), self.safe, self.metadata,
                config.maximum_lateral_offset_m, config.ray_step_fraction)
            result = opt.optimize_global_raceline(reference, lower, upper, config,
                                                 retain_failed_candidate=True)
            stages = result.stages
            points, _ = opt.resample_closed_loop(result.points, config.output_spacing_m)
            # Validate the serialized geometry the downstream consumer will load.
            save_csv(points, folder / "candidate.csv")
            points = opt.load_path_csv(folder / "candidate.csv")
        except (ValueError, RuntimeError, FloatingPointError) as exc:
            error = str(exc)
            # A geometric search failure still gets reference diagnostics, clearly
            # labelled as a failed solver/reference preview, never a solved line.
            points = self.centerline.copy()
        converged = len(stages) == 3 and all(v["success"] for v in stages) and error is None
        validation = self.validate(points, config, solver_converged=converged)
        candidate = {"id": candidate_id, "phase": phase, "parameters": asdict(config),
                     "tool_source_sha256": self.source_hashes,
                     "parameter_sha256": profile_hash(asdict(config)), "solver_stages": stages,
                     "error": error, "candidate_kind": "optimizer_output" if error is None else "reference_preview",
                     "path": str(folder / "candidate.csv") if error is None else None,
                     "path_sha256": opt.file_sha256(folder / "candidate.csv") if error is None else None,
                     "direction": "normal", "validation": validation,
                     "failure_classification": failure_classification(validation),
                     "result_state": ("AUTO_PASS" if phase == "baseline" else "AUTO_TUNED_PASS")
                         if validation["valid"] else "AUTO_TUNING_REQUIRED"}
        self.candidates.append(candidate)
        self.selected = candidate
        self.state = ("AUTO_PASS" if phase == "baseline" else "AUTO_TUNED_PASS") if validation["valid"] else "AUTO_TUNING_REQUIRED"
        save_yaml(candidate, folder / "diagnostics.yaml")
        self.save_report()
        return candidate

    def run_baseline(self):
        if self.centerline is None:
            self.prepare()
        candidate = self.attempt(self.baseline)
        if candidate["validation"]["valid"] and not self.active_selection:
            self.promote(candidate)
        self.preview(candidate)
        return candidate

    def auto_search(self):
        if not self.candidates:
            self.run_baseline()
        if self.state in SUCCESS_STATES:
            return
        # At most one schedule per session. Repeated button presses do not exceed budget.
        if any(c["phase"] in ("geometry", "objective") for c in self.candidates):
            return
        for phase, config in search_schedule(self.baseline, self.settings):
            candidate = self.attempt(config, phase)
            self.preview(candidate)
            print(f"SEARCH {candidate['id']}: " + ("PASS" if candidate["validation"]["valid"] else "FAIL"), flush=True)
            if candidate["validation"]["valid"]:
                if not self.active_selection:
                    self.promote(candidate)
                return
        self.state = "AUTO_SEARCH_EXHAUSTED"
        self.selected = self.best_candidate()
        self.save_report()
        self.preview(self.selected)

    def best_candidate(self):
        return min(self.candidates, key=candidate_rank) if self.candidates else None

    def import_manual(self, path, direction="normal"):
        if direction not in ("normal", "reverse"):
            raise ValueError("manual direction must be normal or reverse")
        if self.centerline is None:
            self.prepare()
        self.invalidate_promotion()
        points = opt.load_path_csv(path)
        candidate_id = f"{len(self.candidates):03d}_manual"
        folder = self.output / "candidate" / candidate_id
        folder.mkdir(parents=True, exist_ok=True)
        if Path(path).resolve() == (folder / "candidate.csv").resolve():
            raise ValueError("manual candidate source must not be overwritten; use a separate output session")
        save_csv(points, folder / "candidate.csv")
        points = opt.load_path_csv(folder / "candidate.csv")
        validation = self.validate(points, self.baseline, solver_converged=None, direction=direction)
        candidate = {"id": candidate_id, "phase": "manual", "parameters": asdict(self.baseline),
                     "tool_source_sha256": self.source_hashes,
                     "parameter_sha256": profile_hash(asdict(self.baseline)), "solver_stages": [],
                     "error": None, "candidate_kind": "manual_candidate", "direction": direction,
                     "source": str(Path(path).resolve()), "source_sha256": opt.file_sha256(path),
                     "path": str(folder / "candidate.csv"), "validation": validation,
                     "path_sha256": opt.file_sha256(folder / "candidate.csv"),
                     "failure_classification": failure_classification(validation),
                     "result_state": "MANUAL_PASS" if validation["valid"] else "NO_VALIDATED_RACELINE"}
        self.candidates.append(candidate)
        self.selected = candidate
        self.state = "MANUAL_PASS" if validation["valid"] else "NO_VALIDATED_RACELINE"
        save_yaml(candidate, folder / "diagnostics.yaml")
        if validation["valid"] and not self.active_selection:
            self.promote(candidate)
        self.save_report()
        self.preview(candidate)
        return candidate

    def check_candidate(self, candidate):
        if not candidate or candidate.get("candidate_kind") == "reference_preview" or not candidate.get("path"):
            raise ValueError("Reference-only preview is not an optimized/manual path and cannot be exported")
        if input_identity(self.identity["map_image"], self.identity["map_yaml"]) != self.identity:
            raise ValueError("map inputs changed since preflight")
        for name, digest in self.phase9_hashes.items():
            if opt.file_sha256(self.output / name) != digest:
                raise ValueError("qualified Phase-9 artifact changed: " + name)
        config = opt.OptimizerConfig(**candidate["parameters"])
        if any(getattr(config, k) != getattr(self.baseline, k) for k in HARD_KEYS):
            raise ValueError("candidate HARD constraints differ from this session")
        points = opt.load_path_csv(candidate["path"])
        if candidate.get("path_sha256") and opt.file_sha256(candidate["path"]) != candidate["path_sha256"]:
            raise ValueError("candidate changed; revalidation/export requires a newly imported candidate")
        validation = self.validate(points, config, candidate["validation"]["solver_converged"], candidate["direction"])
        if not validation["finite_nondegenerate"]:
            raise ValueError("Cannot export nonfinite/degenerate candidate geometry")
        candidate["validation"] = validation
        return points, config, validation

    def promote(self, candidate=None, selection_action=False):
        if (self.output / "selected_raceline.yaml").exists() and not selection_action:
            raise ValueError("An active selection exists; use Select / Export Validated to change it")
        candidate = candidate or self.selected
        if not candidate or not candidate.get("path") or not candidate["validation"]["valid"]:
            raise ValueError("candidate failed common validation; promotion is disabled")
        try:
            if input_identity(self.identity["map_image"], self.identity["map_yaml"]) != self.identity:
                raise ValueError("map inputs changed since preflight")
            for name, digest in self.phase9_hashes.items():
                if opt.file_sha256(self.output / name) != digest:
                    raise ValueError("qualified Phase-9 artifact changed: " + name)
        except (ValueError, OSError):
            self.invalidate_promotion()
            self.state = "NO_VALIDATED_RACELINE"
            raise
        config = opt.OptimizerConfig(**candidate["parameters"])
        if any(getattr(config, k) != getattr(self.baseline, k) for k in HARD_KEYS):
            raise ValueError("candidate HARD constraints differ from this session")
        points = opt.load_path_csv(candidate["path"])
        # Revalidate immediately before publishing; cached UI/legacy acceptance cannot bypass gates.
        validation = self.validate(points, config,
                                   solver_converged=candidate["validation"]["solver_converged"],
                                   direction=candidate["direction"])
        if not validation["valid"]:
            raise ValueError("candidate changed or failed revalidation; promotion is disabled")
        text = opt.path_csv_text(points)
        output = self.output / "raceline_points_optimized.csv"
        report = dict(validation, candidate_id=candidate["id"], direction=candidate["direction"],
                      parameters=asdict(config), inputs=self.identity,
                      status="validated", vehicle_constraints={k: getattr(config, k) for k in HARD_KEYS},
                      raceline_sha256=hashlib.sha256(text.encode()).hexdigest())
        opt._write_output_pair(output, text, self.output / "raceline_validation.yaml",
                               yaml.safe_dump(report, sort_keys=False))
        output.chmod(0o644)
        (self.output / "raceline_validation.yaml").chmod(0o644)
        self.state = ("MANUAL_PASS" if candidate["phase"] == "manual" else
                      "AUTO_PASS" if candidate["phase"] == "baseline" else "AUTO_TUNED_PASS")
        self.selected = candidate
        self.outputs = {"candidate_id": candidate["id"], "raceline": str(output), "raceline_sha256": report["raceline_sha256"],
                        "validation": str(self.output / "raceline_validation.yaml")}
        self.save_report()

    def select_export(self, candidate=None, experimental=False, confirmed=False):
        """One intentional selection action for CLI and GUI; no launch/control."""
        candidate = candidate or self.selected
        if experimental and not confirmed:
            raise ValueError("Experimental export requires explicit confirmation of failed checks")
        points, config, validation = self.check_candidate(candidate)
        if experimental and validation["valid"]:
            raise ValueError("This candidate passes; use Select / Export Validated")
        if not experimental and not validation["valid"]:
            raise ValueError("candidate failed common validation; promotion is disabled")
        if not experimental:
            self.promote(candidate, selection_action=True)
            output = self.output / "raceline_points_optimized.csv"
            evidence_path = self.output / "raceline_validation.yaml"
        else:
            output = self.output / "raceline_points_experimental.csv"
            evidence_path = self.output / "experimental_validation.yaml"
            # Copy exact candidate bytes: no smoothing/resampling during export.
            text = Path(candidate["path"]).read_text()
            evidence = dict(validation, status="experimental", candidate_id=candidate["id"],
                            source_candidate=candidate["path"], source_sha256=opt.file_sha256(candidate["path"]),
                            parameters=asdict(config), inputs=self.identity, direction="csv",
                            vehicle_constraints={k: getattr(config, k) for k in HARD_KEYS},
                            raceline_sha256=hashlib.sha256(text.encode()).hexdigest(),
                            limitations="Failed production validation. Explicit simulation experimentation only; not onboard validated.")
            opt._write_output_pair(output, text, evidence_path, yaml.safe_dump(evidence, sort_keys=False))
            output.chmod(0o644)
            evidence_path.chmod(0o644)
        # Candidate geometry already contains its intended direction (including manual flip).
        evidence = yaml.safe_load(evidence_path.read_text())
        evidence.update(direction="csv", source_candidate=candidate["path"],
                        source_sha256=opt.file_sha256(candidate["path"]))
        save_yaml(evidence, evidence_path)
        selection = {"candidate_id": candidate["id"], "status": "experimental" if experimental else "validated",
                     "generator": "manual_editor" if candidate["phase"] == "manual" else "global_optimizer",
                     "direction": "csv", "candidate_direction": candidate["direction"],
                     "raceline_csv": str(output), "raceline_csv_sha256": opt.file_sha256(output),
                     "validation_report": str(evidence_path), "validation_report_sha256": opt.file_sha256(evidence_path),
                     "map_yaml": self.identity["map_yaml"], "map_yaml_sha256": self.identity["map_yaml_sha256"],
                     "map_image": self.identity["map_image"], "map_image_sha256": self.identity["map_image_sha256"],
                     "vehicle_constraints": {k: getattr(config, k) for k in HARD_KEYS},
                     "custom_vehicle_profile": any(getattr(config, k) != getattr(opt.load_optimizer_config(
                         self.settings["baseline_optimizer_config"]), k) for k in HARD_KEYS),
                     "vehicle_profile_sha256": profile_hash({k: getattr(config, k) for k in HARD_KEYS})}
        manifest = self.output / "selected_raceline.yaml"
        save_yaml({"format_version": 1, "selection": selection}, manifest)
        self.active_selection = resolve_selection(manifest, True)
        self.selected = candidate
        self.save_report()
        self.preview(candidate)
        return self.active_selection

    def save_report(self):
        report = {
            "format_version": 1, "result": self.state, "centerline": self.centerline_status,
            "error": self.error, "inputs": self.identity,
            "configuration": {"path": str(self.settings_path), "sha256": opt.file_sha256(self.settings_path),
                              "resolved": self.settings, "resolved_sha256": profile_hash(self.settings),
                              "baseline_config_sha256": opt.file_sha256(self.settings["baseline_optimizer_config"])},
            "baseline_profile": asdict(self.baseline), "baseline_profile_sha256": profile_hash(asdict(self.baseline)),
            "vehicle_constraints": {k: getattr(self.baseline, k) for k in HARD_KEYS},
            "map_erosion": {"resolution_m": self.metadata.resolution_m,
                            "pixel_uncertainty_m": self.metadata.resolution_m / np.sqrt(2),
                            "effective_margin_m": self.baseline.safety_margin_m + self.metadata.resolution_m / np.sqrt(2)},
            "source_sha256": self.source_hashes,
            "environment": {"python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__},
            "attempted_candidates": self.candidates, "best_candidate": self.best_candidate(),
            "selected_candidate_id": self.selected["id"] if self.selected else None,
            "failure_classification": failure_classification(self.selected["validation"]) if self.selected else ["CENTERLINE"],
            "final_validation": next((c["validation"] for c in self.candidates
                                      if c["id"] == self.outputs.get("candidate_id")), None),
            "outputs": self.outputs,
            "active_selection": self.active_selection,
            "latest_export_preview": self.latest_export,
            "next_available_action": "use_validated_raceline" if self.outputs else
                ("inspect_centerline_generation_log" if self.centerline_status != "PASS" else
                 "auto_search" if self.state == "AUTO_TUNING_REQUIRED" else "manual_editor"),
        }
        if self.centerline is not None:
            report["centerline_artifacts"] = self.phase9_hashes
        save_yaml(report, self.output / "raceline_workbench_report.yaml")

    def preview(self, candidate=None):
        from centerline_tools.raceline_workbench_view import save_preview
        save_preview(self, candidate or self.selected)

    def manual_editor(self):
        """Reuse E/A/X editor in a subprocess; its exports are candidates only."""
        if self.centerline is None:
            self.prepare()
        folder = self.output / "manual"
        folder.mkdir(exist_ok=True)
        candidate_file = folder / "raceline_points_smooth.csv"
        command = [sys.executable, str(ROOT / "manual_raceline_generator.py"),
                   self.identity["map_image"], self.identity["map_yaml"],
                   "--workbench-dir", str(self.output)]
        print("MANUAL: E/A/X editor; Save exports a candidate, not a validated raceline.", flush=True)
        with (self.output / "manual_editor.log").open("w") as log:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        if result.returncode == 3:
            return None
        if result.returncode != 0:
            raise RuntimeError("manual editor unavailable/failed; use --manual-candidate to validate a saved CSV")
        if candidate_file.exists():
            direction_file = folder / "raceline_selected_direction.txt"
            direction = direction_file.read_text().strip() if direction_file.exists() else "normal"
            return self.import_manual(candidate_file, direction)
        return None  # Cancel never silently promotes a saved/centerline fallback.

    def summary(self):
        baseline = next((c for c in self.candidates if c["phase"] == "baseline"), None)
        best = self.best_candidate()
        lines = ["CENTERLINE: " + self.centerline_status]
        if baseline:
            v = baseline["validation"]
            lines.append("BASELINE: " + ("PASS" if v["valid"] else "FAIL — " + "; ".join(v["failure_reasons"])))
        lines.append("RESULT: " + self.state)
        lines.append("AUTO SEARCH: " + str(sum(c["phase"] in ("geometry", "objective") for c in self.candidates)) + " candidates")
        if best:
            v = best["validation"]
            excess = v['curvature_excess_percent']
            excess_text = f"{excess:.1f}%" if excess is not None else "unknown"
            lines.append(f"BEST: {best['id']}; corridor {'PASS' if v['corridor_valid'] else 'FAIL'}, "
                         f"curvature excess {excess_text}")
        lines.append("VALIDATED RACELINE: " + self.outputs.get("raceline", "NONE"))
        lines.append("REPORT: " + str(self.output / "raceline_workbench_report.yaml"))
        if self.latest_export and not self.active_selection:
            previous = self.latest_export
            lines.append("PREVIOUS EXPORT PREVIEW: " + (previous.get("error") or
                         previous["status"].upper() + " — " + previous["path"]))
            lines.append("Previous export is not automatically selected in this session.")
        if self.active_selection:
            s = self.active_selection
            lines.extend(["SELECTED RACELINE", "Track: " + Path(self.identity["map_image"]).stem,
                          "Status: " + s["status"].upper(), "CSV: " + s["raceline_csv"],
                          "Direction: " + s["direction"], "Manifest: " + s["manifest"],
                          "Ready for explicit simulation selection." if s["status"] == "validated" else
                          "Experimental simulation only; not validated for onboard use."])
            if s["status"] == "experimental":
                evidence = yaml.safe_load(Path(s["validation_report"]).read_text())
                lines.append("Failed checks: " + "; ".join(evidence["failure_reasons"]))
        if not self.outputs:
            next_action = "--manual (E/A/X editor) or --manual-candidate <CSV> (saved edit)"
            if self.state == "AUTO_TUNING_REQUIRED":
                next_action = "--auto-search; if exhausted, " + next_action
            lines.append("NEXT: Generate Raceline or Open Previous Session in the GUI" if self.centerline_status == "NOT_RUN" else
                         "NEXT: inspect centerline_generation.log" if self.centerline_status != "PASS" else
                         "NEXT: " + next_action)
        if self.error:
            lines.append("ERROR: " + self.error)
        return "\n".join(lines)


def interactive_display_available():
    """Probe the existing Matplotlib/Tk display, not merely the DISPLAY string."""
    if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return False
    try:
        import tkinter
    except ImportError:
        return False
    try:
        root = tkinter.Tk()
        root.withdraw()
        root.update_idletasks()
        root.destroy()
        return True
    except (RuntimeError, tkinter.TclError):
        return False


def use_gui(headless=False, explicit_gui=False):
    if headless:
        return False
    available = interactive_display_available()
    if explicit_gui and not available:
        raise RuntimeError("No working display; use --headless and preview.png")
    return available


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_image", nargs="?")
    parser.add_argument("map_yaml", nargs="?")
    parser.add_argument("--config", help="workbench YAML; default config/raceline_workbench.yaml")
    parser.add_argument("--output-dir")
    parser.add_argument("--open-session", nargs="?", const="__choose_session__",
                        help="Reopen a directory; omit its value to choose a saved report in a GUI dialog")
    parser.add_argument("--auto-search", action="store_true", help="bounded deterministic search after baseline failure")
    parser.add_argument("--gui", action="store_true", help="Require GUI (normally selected automatically with a working display)")
    parser.add_argument("--headless", action="store_true", help="Explicitly suppress GUI")
    parser.add_argument("--manual", action="store_true", help="open existing E/A/X editor after baseline/search")
    parser.add_argument("--manual-candidate", help="validate a saved manual CSV after baseline")
    parser.add_argument("--manual-direction", choices=("normal", "reverse"), default="normal")
    export = parser.add_mutually_exclusive_group()
    export.add_argument("--select-validated", nargs="?", const="best", help="Intentionally select/export best or candidate ID")
    export.add_argument("--export-experimental", metavar="CANDIDATE_ID", help="Explicit acknowledgment: export this failed candidate for experimentation")
    args = parser.parse_args(argv)
    if args.headless and (args.gui or args.manual):
        parser.error("--headless cannot open --gui/--manual; use --manual-candidate for saved edits")
    if args.open_session == "__choose_session__":
        if args.headless or not interactive_display_available():
            parser.error("Without a display, --open-session requires an explicit directory")
        import tkinter
        from tkinter import filedialog
        root = tkinter.Tk()
        root.withdraw()
        chosen = filedialog.askopenfilename(parent=root, title="Open workbench session report",
                                            initialdir=str(ROOT / "centerline_output/workbench"),
                                            filetypes=[("Workbench report", "raceline_workbench_report.yaml")])
        root.destroy()
        if not chosen:
            print("Open session cancelled.")
            return 0
        args.open_session = str(Path(chosen).parent)
    if args.open_session:
        if args.map_image or args.map_yaml or args.config or args.output_dir:
            parser.error("--open-session owns maps/config/output; do not combine these arguments")
    elif not args.map_image or not args.map_yaml:
        parser.error("Supply a map image and YAML, or --open-session DIRECTORY")
    bench = None
    try:
        gui = use_gui(args.headless, args.gui or args.manual)
        import matplotlib
        matplotlib.use("TkAgg" if gui else "Agg")
        if not gui and not args.headless:
            print("No working interactive display; using headless mode and preview.png.", flush=True)
        if args.open_session:
            bench = Workbench.reopen(args.open_session)
        else:
            if args.output_dir and Path(args.output_dir).exists() and any(Path(args.output_dir).iterdir()):
                raise ValueError("Existing session: use --open-session DIRECTORY; no implicit reuse")
            bench = Workbench(args.map_image, args.map_yaml, args.output_dir, args.config)
        print("SESSION: " + str(bench.output), flush=True)
        if not bench.candidates and (not gui or args.auto_search or args.manual or args.manual_candidate or
                                     args.select_validated or args.export_experimental):
            print("Generating centerline and baseline; please wait.", flush=True)
            bench.run_baseline()
        elif bench.centerline is None and not gui:
            print("Preparing centerline; please wait.", flush=True)
            bench.prepare()
        if args.auto_search:
            bench.auto_search()
        if args.manual_candidate:
            bench.import_manual(args.manual_candidate, args.manual_direction)
        elif args.manual:
            bench.manual_editor()
        if args.select_validated or args.export_experimental:
            name = args.select_validated or args.export_experimental
            candidate = bench.best_candidate() if name == "best" else next((c for c in bench.candidates if c["id"] == name), None)
            if candidate is None:
                raise ValueError("Unknown candidate ID: " + name)
            bench.select_export(candidate, experimental=bool(args.export_experimental), confirmed=bool(args.export_experimental))
        if gui:
            from centerline_tools.raceline_workbench_view import show_workbench
            bench.save_report()
            bench = show_workbench(bench)
    except (ValueError, OSError, RuntimeError, KeyError, TypeError, yaml.YAMLError, subprocess.CalledProcessError) as error:
        if bench is None:
            print("NO_VALIDATED_RACELINE: " + str(error), file=sys.stderr)
            return 2
        bench.error = str(error)
        if bench.centerline is None:
            bench.centerline_status = "FAIL"
        bench.save_report()
    print(bench.summary())
    if bench.error:
        return 2
    return 0 if bench.active_selection or (bench.state in SUCCESS_STATES and bench.outputs) else 1


if __name__ == "__main__":
    raise SystemExit(main())
