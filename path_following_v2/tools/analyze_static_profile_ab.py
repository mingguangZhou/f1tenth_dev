#!/usr/bin/env python3
"""Compare two obstacle-trial logger result directories.

The logger intentionally stores two views of a run:

* the CSV is a regularly sampled vehicle/control time series;
* ``.events.jsonl`` records planner and arbitration state changes with finer timing.

This tool keeps those two views separate.  In particular, plan diagnostics are
deduplicated per planning attempt instead of being weighted by how long a plan
remained active in the sampled CSV.  The PLANNING_HOLD-to-outcome interval is an
end-to-end latency proxy; it includes waiting and callback scheduling and must
not be interpreted as pure planner CPU time.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


OBSTACLE_TRAJECTORY_MODES = {
    "AVOIDANCE_DEPARTING",
    "AVOIDANCE_PASSING",
    "AVOIDANCE_RETURNING",
    "FOLLOWING_OBSTACLE",
    "PLANNING_HOLD",
    "RECOVERING_TO_RACELINE",
    "REPLAN_PENDING",
}

PLANNER_FAILURE_STATES = {
    "CRITICAL_OBSTACLE",
    "INPUT_INVALID",
    "NO_SAFE_PATH_CONFIRMED",
    "PLAN_INVALIDATED",
    "SCAN_INVALID",
    "TF_UNAVAILABLE",
}

ACCEPTED_REASON_MARKERS = (
    "persistent local trajectory accepted",
    "materially updated",
    "recovery accepted",
)

COMPARISON_METRICS = {
    "plan_initial_latency_median_ms": "Initial plan latency median (ms)",
    "plan_latency_median_ms": "All accepted plan latency median (ms)",
    "plan_latency_p95_ms": "All accepted plan latency p95 (ms)",
    "plan_grid_median_ms": "Clearance-grid median (ms)",
    "plan_dp_median_ms": "Lattice DP median (ms)",
    "minimum_plan_clearance_m": "Minimum accepted clearance (m)",
    "maximum_plan_curvature_inv_m": "Maximum accepted curvature (1/m)",
    "straight_cte_p95_m": "Straight-raceline CTE p95 (m)",
    "straight_steering_p95_rad": "Straight steering p95 (rad)",
    "obstacle_steering_p95_rad": "Avoidance steering p95 (rad)",
    "obstacle_steering_rate_p95_radps": "Avoidance steering-rate p95 (rad/s)",
    "obstacle_steering_jerk_p95_radps2": "Avoidance steering jerk p95 (rad/s^2)",
    "obstacle_steering_variation_per_m": "Avoidance steering variation (rad/m)",
    "obstacle_steering_reversals_per_m": "Avoidance steering reversals (/m)",
    "reactive_entries": "Reactive entries",
    "guard_blocked_entries": "Guard BLOCKED entries",
    "lower_estop_entries": "Lower emergency-stop entries",
    "planner_failure_entries": "Planner failure-state entries",
    "first_lap_sec": "First lap (s)",
    "steady_lap_median_sec": "Steady lap median (s)",
}


def finite_float(value: Any) -> float | None:
    """Return a finite float, accepting numeric strings and rejecting NaN/inf."""

    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def finite_int(value: Any) -> int | None:
    number = finite_float(value)
    if number is None:
        return None
    return int(number)


def percentile(values: Sequence[float], quantile: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), quantile))


def describe(values: Iterable[Any], include_rms: bool = False) -> dict[str, Any]:
    clean = [value for value in (finite_float(item) for item in values) if value is not None]
    if not clean:
        result: dict[str, Any] = {
            "count": 0,
            "minimum": None,
            "p50": None,
            "median": None,
            "p95": None,
            "p99": None,
            "maximum": None,
            "mean": None,
        }
        if include_rms:
            result["rms"] = None
        return result

    data = np.asarray(clean, dtype=np.float64)
    median = float(np.median(data))
    result = {
        "count": int(data.size),
        "minimum": float(np.min(data)),
        "p50": median,
        "median": median,
        "p95": float(np.percentile(data, 95.0)),
        "p99": float(np.percentile(data, 99.0)),
        "maximum": float(np.max(data)),
        "mean": float(np.mean(data)),
    }
    if include_rms:
        result["rms"] = float(np.sqrt(np.mean(np.square(data))))
    return result


def read_csv_rows(path: Path, warnings: list[str]) -> list[dict[str, str]]:
    try:
        with path.open("r", newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None:
                warnings.append(f"{path}: CSV has no header")
                return []
            return list(reader)
    except (OSError, csv.Error) as error:
        warnings.append(f"{path}: could not read CSV: {error}")
        return []


def read_json(path: Path, warnings: list[str]) -> dict[str, Any]:
    if not path.exists():
        warnings.append(f"{path}: summary sidecar is missing")
        return {}
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
        if not isinstance(value, dict):
            warnings.append(f"{path}: summary root is not an object")
            return {}
        return value
    except (OSError, json.JSONDecodeError) as error:
        warnings.append(f"{path}: could not read summary: {error}")
        return {}


def read_events(path: Path, warnings: list[str]) -> list[dict[str, Any]]:
    if not path.exists():
        warnings.append(f"{path}: event sidecar is missing; latency proxy unavailable")
        return []
    events: list[dict[str, Any]] = []
    try:
        with path.open("r", encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, start=1):
                if not line.strip():
                    continue
                try:
                    event = json.loads(line)
                except json.JSONDecodeError as error:
                    warnings.append(f"{path}:{line_number}: malformed event JSON: {error}")
                    continue
                if not isinstance(event, dict):
                    warnings.append(f"{path}:{line_number}: event is not an object")
                    continue
                if finite_float(event.get("elapsed_sec")) is None:
                    warnings.append(f"{path}:{line_number}: event has no finite elapsed_sec")
                    continue
                events.append(event)
    except OSError as error:
        warnings.append(f"{path}: could not read events: {error}")
        return []
    events.sort(key=lambda event: float(event["elapsed_sec"]))
    return events


def load_raceline_curvature(path: Path | None, warnings: list[str]) -> list[float] | None:
    if path is None:
        return None
    try:
        with path.open("r", newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            fields = set(reader.fieldnames or ())
            if "curvature_abs" in fields:
                column = "curvature_abs"
            elif "curvature" in fields:
                column = "curvature"
            else:
                warnings.append(
                    f"{path}: no curvature_abs or curvature column; straight-track metrics unavailable"
                )
                return None
            values: list[float] = []
            for row_number, row in enumerate(reader, start=2):
                value = finite_float(row.get(column))
                if value is None:
                    warnings.append(f"{path}:{row_number}: invalid {column}; using infinity")
                    values.append(math.inf)
                else:
                    values.append(abs(value))
    except (OSError, csv.Error) as error:
        warnings.append(f"{path}: could not read raceline curvature: {error}")
        return None
    if not values:
        warnings.append(f"{path}: raceline is empty")
        return None
    return values


def planner_dict(event: Mapping[str, Any]) -> Mapping[str, Any]:
    value = event.get("planner", {})
    return value if isinstance(value, Mapping) else {}


def planning_kind(hold_reason: str) -> str:
    lowered = hold_reason.lower()
    if "recover" in lowered or "return" in lowered:
        return "recovery"
    if "updat" in lowered or "active path" in lowered:
        return "update"
    return "initial"


def planning_outcome(planner: Mapping[str, Any]) -> str:
    reason = str(planner.get("reason", ""))
    lowered = reason.lower()
    if any(marker in lowered for marker in ACCEPTED_REASON_MARKERS):
        return "accepted"
    state = str(planner.get("state", ""))
    if state in PLANNER_FAILURE_STATES:
        return "failure"
    mode = str(planner.get("trajectory_mode", ""))
    if mode == "REPLAN_PENDING":
        return "pending"
    if "yield" in lowered or "waiting" in lowered or "pending" in lowered:
        return "yield_or_pending"
    return "other"


def extract_plan_attempts(events: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """Pair each entry into PLANNING_HOLD with the first subsequent exit."""

    attempts: list[dict[str, Any]] = []
    active: dict[str, Any] | None = None
    for event in events:
        elapsed = finite_float(event.get("elapsed_sec"))
        if elapsed is None:
            continue
        planner = planner_dict(event)
        mode = str(planner.get("trajectory_mode", ""))
        if mode == "PLANNING_HOLD":
            if active is None:
                hold_reason = str(planner.get("reason", ""))
                active = {
                    "attempt_index": len(attempts) + 1,
                    "kind": planning_kind(hold_reason),
                    "hold_elapsed_sec": elapsed,
                    "hold_reason": hold_reason,
                    "hold_plan_id": str(planner.get("plan_id", "")),
                }
            continue
        if active is None:
            continue

        latency_ms = max(0.0, (elapsed - active["hold_elapsed_sec"]) * 1000.0)
        grid_ms = finite_float(planner.get("lattice_clearance_grid_time_ms"))
        dp_ms = finite_float(planner.get("lattice_compute_time_ms"))
        measured_parts = [value for value in (grid_ms, dp_ms) if value is not None]
        residual_ms = max(0.0, latency_ms - sum(measured_parts)) if measured_parts else None
        active.update(
            {
                "outcome": planning_outcome(planner),
                "outcome_elapsed_sec": elapsed,
                "hold_to_outcome_ms": latency_ms,
                "outcome_state": str(planner.get("state", "")),
                "outcome_mode": mode,
                "outcome_reason": str(planner.get("reason", "")),
                "plan_id": str(planner.get("plan_id", "")),
                "detour_side": str(
                    planner.get(
                        "candidate_selected_side", planner.get("detour_side", "")
                    )
                ),
                "lattice_clearance_grid_time_ms": grid_ms,
                "lattice_compute_time_ms": dp_ms,
                "unattributed_latency_ms": residual_ms,
                "minimum_clearance_m": finite_float(
                    planner.get("minimum_clearance_m")
                ),
                "maximum_curvature_inv_m": finite_float(
                    planner.get("maximum_curvature_inv_m")
                ),
            }
        )
        attempts.append(active)
        active = None

    if active is not None:
        active.update(
            {
                "outcome": "incomplete",
                "outcome_elapsed_sec": None,
                "hold_to_outcome_ms": None,
                "outcome_state": None,
                "outcome_mode": None,
                "outcome_reason": None,
                "plan_id": None,
                "detour_side": None,
                "lattice_clearance_grid_time_ms": None,
                "lattice_compute_time_ms": None,
                "unattributed_latency_ms": None,
                "minimum_clearance_m": None,
                "maximum_curvature_inv_m": None,
            }
        )
        attempts.append(active)
    return attempts


def accepted_plan_rows(rows: Sequence[Mapping[str, str]]) -> list[dict[str, Any]]:
    """Best-effort plan diagnostics when an older run lacks event sidecars."""

    plans: list[dict[str, Any]] = []
    seen: set[str] = set()
    for row in rows:
        plan_id = str(row.get("planner_plan_id", "")).strip()
        if not plan_id or plan_id == "0" or plan_id in seen:
            continue
        mode = str(row.get("planner_trajectory_mode", ""))
        if mode in ("", "NONE", "PLANNING_HOLD"):
            continue
        seen.add(plan_id)
        plans.append(
            {
                "plan_id": plan_id,
                "detour_side": str(
                    row.get("planner_candidate_selected_side")
                    or row.get("planner_detour_side", "")
                ),
                "lattice_clearance_grid_time_ms": finite_float(
                    row.get("planner_lattice_clearance_grid_time_ms")
                ),
                "lattice_compute_time_ms": finite_float(
                    row.get("planner_lattice_compute_time_ms")
                ),
                "minimum_clearance_m": finite_float(
                    row.get("planner_minimum_clearance_m")
                ),
                "maximum_curvature_inv_m": finite_float(
                    row.get("planner_maximum_curvature_inv_m")
                ),
            }
        )
    return plans


def plan_metrics(
    attempts: Sequence[Mapping[str, Any]], rows: Sequence[Mapping[str, str]]
) -> dict[str, Any]:
    accepted = [attempt for attempt in attempts if attempt.get("outcome") == "accepted"]
    completed = [attempt for attempt in attempts if attempt.get("outcome") != "incomplete"]
    plans: list[Mapping[str, Any]]
    diagnostic_source: str
    if accepted:
        plans = accepted
        diagnostic_source = "events"
    else:
        plans = accepted_plan_rows(rows)
        diagnostic_source = "csv_fallback" if plans else "unavailable"

    by_kind: dict[str, Any] = {}
    for kind in ("initial", "update", "recovery"):
        selected = [
            attempt
            for attempt in accepted
            if attempt.get("kind") == kind
            and finite_float(attempt.get("hold_to_outcome_ms")) is not None
        ]
        by_kind[kind] = {
            "accepted_count": len(selected),
            "hold_to_accepted_ms": describe(
                attempt.get("hold_to_outcome_ms") for attempt in selected
            ),
        }

    return {
        "attempt_count": len(attempts),
        "completed_attempt_count": len(completed),
        "accepted_attempt_count": len(accepted),
        "pairing_coverage": (
            len(completed) / len(attempts) if attempts else None
        ),
        "outcomes": dict(Counter(str(item.get("outcome", "")) for item in attempts)),
        "diagnostic_source": diagnostic_source,
        "accepted_plan_ids": sorted(
            {
                str(item.get("plan_id"))
                for item in plans
                if item.get("plan_id") not in (None, "", "0")
            }
        ),
        "hold_to_accepted_ms": describe(
            item.get("hold_to_outcome_ms") for item in accepted
        ),
        "lattice_clearance_grid_time_ms": describe(
            item.get("lattice_clearance_grid_time_ms") for item in plans
        ),
        "lattice_compute_time_ms": describe(
            item.get("lattice_compute_time_ms") for item in plans
        ),
        "unattributed_latency_ms": describe(
            item.get("unattributed_latency_ms") for item in accepted
        ),
        "minimum_clearance_m": describe(
            item.get("minimum_clearance_m") for item in plans
        ),
        "maximum_curvature_inv_m": describe(
            item.get("maximum_curvature_inv_m") for item in plans
        ),
        "by_kind": by_kind,
        "attempts": list(attempts),
    }


def float_column(rows: Sequence[Mapping[str, str]], field: str) -> np.ndarray:
    return np.asarray(
        [
            value if value is not None else np.nan
            for value in (finite_float(row.get(field)) for row in rows)
        ],
        dtype=np.float64,
    )


def steering_metrics(
    rows: Sequence[Mapping[str, str]],
    mask: np.ndarray,
    minimum_dt_sec: float,
    maximum_dt_sec: float,
    steering_deadband_rad: float,
) -> dict[str, Any]:
    if not rows:
        return {
            "sample_count": 0,
            "steering_abs_rad": describe([], include_rms=True),
            "steering_rate_abs_radps": describe([], include_rms=True),
            "steering_jerk_abs_radps2": describe([], include_rms=True),
            "distance_m": None,
            "total_variation_rad": None,
            "variation_per_m": None,
            "sign_reversals": 0,
            "sign_reversals_per_m": None,
        }

    times = float_column(rows, "elapsed_sec")
    steering = float_column(rows, "drive_steering_rad")
    x_values = float_column(rows, "x_m")
    y_values = float_column(rows, "y_m")
    valid_samples = mask & np.isfinite(times) & np.isfinite(steering)
    magnitudes = np.abs(steering[valid_samples]).tolist()

    rates: list[float] = []
    rate_intervals: list[tuple[int, int, float, float]] = []
    distance_m = 0.0
    variation_rad = 0.0
    has_distance = False
    sign_reversals = 0
    previous_sign: int | None = None
    previous_sign_index: int | None = None

    for index in range(len(rows)):
        if not valid_samples[index]:
            previous_sign = None
            previous_sign_index = None
            continue
        if steering[index] > steering_deadband_rad:
            sign = 1
        elif steering[index] < -steering_deadband_rad:
            sign = -1
        else:
            sign = 0

        if index > 0 and valid_samples[index - 1]:
            dt = times[index] - times[index - 1]
            if minimum_dt_sec <= dt <= maximum_dt_sec:
                delta = steering[index] - steering[index - 1]
                rate = delta / dt
                rates.append(abs(float(rate)))
                rate_intervals.append((index - 1, index, float(rate), float(dt)))
                variation_rad += abs(float(delta))
                if all(
                    math.isfinite(value)
                    for value in (
                        x_values[index - 1],
                        y_values[index - 1],
                        x_values[index],
                        y_values[index],
                    )
                ):
                    distance_m += math.hypot(
                        x_values[index] - x_values[index - 1],
                        y_values[index] - y_values[index - 1],
                    )
                    has_distance = True

                if sign != 0:
                    if (
                        previous_sign is not None
                        and previous_sign_index is not None
                        and previous_sign != sign
                    ):
                        sign_reversals += 1
                    previous_sign = sign
                    previous_sign_index = index
            else:
                previous_sign = sign if sign != 0 else None
                previous_sign_index = index if sign != 0 else None
        elif sign != 0:
            previous_sign = sign
            previous_sign_index = index

    jerks: list[float] = []
    for previous, current in zip(rate_intervals, rate_intervals[1:]):
        if previous[1] != current[0]:
            continue
        midpoint_dt = 0.5 * (previous[3] + current[3])
        if midpoint_dt > 0.0:
            jerks.append(abs((current[2] - previous[2]) / midpoint_dt))

    usable_distance = distance_m if has_distance and distance_m > 1e-9 else None
    return {
        "sample_count": int(np.count_nonzero(valid_samples)),
        "steering_abs_rad": describe(magnitudes, include_rms=True),
        "steering_rate_abs_radps": describe(rates, include_rms=True),
        "steering_jerk_abs_radps2": describe(jerks, include_rms=True),
        "distance_m": usable_distance,
        "total_variation_rad": variation_rad if rates else None,
        "variation_per_m": (
            variation_rad / usable_distance if usable_distance is not None else None
        ),
        "sign_reversals": sign_reversals,
        "sign_reversals_per_m": (
            sign_reversals / usable_distance if usable_distance is not None else None
        ),
    }


def tracking_metrics(
    rows: Sequence[Mapping[str, str]],
    raceline_curvature: Sequence[float] | None,
    straight_curvature_max: float,
    minimum_dt_sec: float,
    maximum_dt_sec: float,
    steering_deadband_rad: float,
) -> dict[str, Any]:
    count = len(rows)
    modes = np.asarray(
        [str(row.get("planner_trajectory_mode", "")) for row in rows], dtype=object
    )
    cte = float_column(rows, "cross_track_error_m")
    raceline_mask = modes == "RACELINE"
    obstacle_mask = np.asarray(
        [mode in OBSTACLE_TRAJECTORY_MODES for mode in modes], dtype=bool
    )
    straight_mask = np.zeros(count, dtype=bool)
    if raceline_curvature is not None:
        for index, row in enumerate(rows):
            nearest = finite_int(row.get("nearest_raceline_index"))
            if (
                raceline_mask[index]
                and nearest is not None
                and 0 <= nearest < len(raceline_curvature)
                and raceline_curvature[nearest] <= straight_curvature_max
            ):
                straight_mask[index] = True

    def cte_for(mask: np.ndarray) -> dict[str, Any]:
        selected = cte[mask & np.isfinite(cte)]
        return describe(np.abs(selected).tolist(), include_rms=True)

    all_mask = np.ones(count, dtype=bool)
    return {
        "cross_track_error_m": {
            "all_samples": cte_for(all_mask),
            "raceline_mode": cte_for(raceline_mask),
            "straight_raceline": cte_for(straight_mask),
        },
        "steering": {
            "all_samples": steering_metrics(
                rows,
                all_mask,
                minimum_dt_sec,
                maximum_dt_sec,
                steering_deadband_rad,
            ),
            "raceline_mode": steering_metrics(
                rows,
                raceline_mask,
                minimum_dt_sec,
                maximum_dt_sec,
                steering_deadband_rad,
            ),
            "straight_raceline": steering_metrics(
                rows,
                straight_mask,
                minimum_dt_sec,
                maximum_dt_sec,
                steering_deadband_rad,
            ),
            "obstacle_maneuver": steering_metrics(
                rows,
                obstacle_mask,
                minimum_dt_sec,
                maximum_dt_sec,
                steering_deadband_rad,
            ),
        },
        "straight_curvature_max_inv_m": straight_curvature_max,
        "straight_classification_available": raceline_curvature is not None,
    }


def categorical_timeline(
    rows: Sequence[Mapping[str, str]], field: str, maximum_sample_gap_sec: float
) -> dict[str, Any]:
    entries: Counter[str] = Counter()
    durations: defaultdict[str, float] = defaultdict(float)
    previous = ""
    for index, row in enumerate(rows):
        value = str(row.get(field, "")).strip()
        if value and value != previous:
            entries[value] += 1
        if value:
            previous = value
        if index + 1 >= len(rows) or not value:
            continue
        start = finite_float(row.get("elapsed_sec"))
        end = finite_float(rows[index + 1].get("elapsed_sec"))
        if start is None or end is None:
            continue
        dt = end - start
        if 0.0 < dt <= maximum_sample_gap_sec:
            durations[value] += dt
    return {
        "entries": dict(sorted(entries.items())),
        "duration_sec": {
            key: float(value) for key, value in sorted(durations.items())
        },
    }


def mode_and_failure_metrics(
    rows: Sequence[Mapping[str, str]], maximum_sample_gap_sec: float
) -> dict[str, Any]:
    fields = {
        "planner_state": "planner_state",
        "planner_trajectory_mode": "planner_trajectory_mode",
        "arbitrator_mode": "arbitrator_mode",
        "guard_state": "guard_state",
        "lower_mode": "lower_mode",
    }
    timelines = {
        name: categorical_timeline(rows, field, maximum_sample_gap_sec)
        for name, field in fields.items()
    }
    planner_entries = timelines["planner_state"]["entries"]
    primary_failures = categorical_timeline(
        rows, "arbitrator_primary_failure", maximum_sample_gap_sec
    )
    primary_failures["entries"] = {
        key: value
        for key, value in primary_failures["entries"].items()
        if key.lower() not in ("none", "")
    }
    primary_failures["duration_sec"] = {
        key: value
        for key, value in primary_failures["duration_sec"].items()
        if key.lower() not in ("none", "")
    }
    failure_entries = {
        state: int(planner_entries.get(state, 0))
        for state in sorted(PLANNER_FAILURE_STATES)
        if planner_entries.get(state, 0)
    }
    return {
        "timelines": timelines,
        "primary_failures": primary_failures,
        "planner_failure_entries": failure_entries,
        "planner_failure_entry_total": sum(failure_entries.values()),
        "reactive_entries": int(
            timelines["arbitrator_mode"]["entries"].get("REACTIVE", 0)
        ),
        "guard_blocked_entries": int(
            timelines["guard_state"]["entries"].get("BLOCKED", 0)
        ),
        "lower_estop_entries": int(
            timelines["lower_mode"]["entries"].get("EMERGENCY_STOP", 0)
        ),
    }


def derive_lap_metrics(
    rows: Sequence[Mapping[str, str]], summary: Mapping[str, Any]
) -> dict[str, Any]:
    completion_times = summary.get("lap_completion_times_sec")
    durations = summary.get("lap_durations_sec")
    if not isinstance(completion_times, list):
        completion_times = []
    if not isinstance(durations, list):
        durations = []
    completion_times = [
        value
        for value in (finite_float(item) for item in completion_times)
        if value is not None
    ]
    durations = [
        value for value in (finite_float(item) for item in durations) if value is not None
    ]

    if not completion_times and rows:
        previous_laps = 0
        for row in rows:
            completed = finite_int(row.get("completed_laps")) or 0
            elapsed = finite_float(row.get("elapsed_sec"))
            while elapsed is not None and completed > previous_laps:
                completion_times.append(elapsed)
                previous_laps += 1
        durations = [
            value - (completion_times[index - 1] if index else 0.0)
            for index, value in enumerate(completion_times)
        ]

    completed_laps = finite_int(summary.get("completed_laps"))
    if completed_laps is None:
        completed_laps = len(completion_times)
    target_laps = finite_int(summary.get("target_laps"))
    target_reached = summary.get("target_laps_reached")
    if not isinstance(target_reached, bool):
        target_reached = (
            target_laps is not None
            and target_laps > 0
            and completed_laps >= target_laps
        )
    return {
        "completed_laps": completed_laps,
        "target_laps": target_laps,
        "target_laps_reached": target_reached,
        "lap_completion_times_sec": completion_times,
        "lap_durations_sec": durations,
        "first_lap_sec": durations[0] if durations else None,
        "steady_lap_median_sec": (
            float(np.median(durations[1:])) if len(durations) > 1 else None
        ),
        "all_laps": describe(durations),
    }


def classify_scenario(
    csv_path: Path,
    summary: Mapping[str, Any],
    plans: Mapping[str, Any],
    rows: Sequence[Mapping[str, str]],
) -> str:
    label = f"{csv_path.stem} {summary.get('trial', '')}".lower()
    if "moving" in label or "agent" in label:
        return "moving_obstacle"
    if any(marker in label for marker in ("clean", "clear", "no_obstacle", "obstacle_free")):
        return "clear"
    if any(marker in label for marker in ("static", "obstacle", "uturn", "u_turn", "west", "east")):
        return "static_obstacle"
    if plans.get("accepted_attempt_count", 0) or plans.get("accepted_plan_ids"):
        return "static_obstacle"
    modes = {str(row.get("planner_trajectory_mode", "")) for row in rows}
    if modes & OBSTACLE_TRAJECTORY_MODES:
        return "static_obstacle"
    if modes == {"RACELINE"} or ("RACELINE" in modes and len(modes - {"", "RACELINE"}) == 0):
        return "clear"
    return "unknown"


def nested_value(value: Mapping[str, Any], path: str) -> Any:
    current: Any = value
    for component in path.split("."):
        if not isinstance(current, Mapping):
            return None
        current = current.get(component)
    return current


def make_scorecard(trial: Mapping[str, Any]) -> dict[str, Any]:
    plan = trial["planning"]
    tracking = trial["tracking"]
    modes = trial["modes_and_failures"]
    laps = trial["laps"]
    obstacle_steering = tracking["steering"]["obstacle_maneuver"]
    straight_steering = tracking["steering"]["straight_raceline"]
    return {
        "plan_initial_latency_median_ms": nested_value(
            plan, "by_kind.initial.hold_to_accepted_ms.median"
        ),
        "plan_latency_median_ms": nested_value(plan, "hold_to_accepted_ms.median"),
        "plan_latency_p95_ms": nested_value(plan, "hold_to_accepted_ms.p95"),
        "plan_grid_median_ms": nested_value(
            plan, "lattice_clearance_grid_time_ms.median"
        ),
        "plan_dp_median_ms": nested_value(plan, "lattice_compute_time_ms.median"),
        "minimum_plan_clearance_m": nested_value(plan, "minimum_clearance_m.minimum"),
        "maximum_plan_curvature_inv_m": nested_value(
            plan, "maximum_curvature_inv_m.maximum"
        ),
        "straight_cte_p95_m": nested_value(
            tracking, "cross_track_error_m.straight_raceline.p95"
        ),
        "straight_steering_p95_rad": nested_value(
            straight_steering, "steering_abs_rad.p95"
        ),
        "obstacle_steering_p95_rad": nested_value(
            obstacle_steering, "steering_abs_rad.p95"
        ),
        "obstacle_steering_rate_p95_radps": nested_value(
            obstacle_steering, "steering_rate_abs_radps.p95"
        ),
        "obstacle_steering_jerk_p95_radps2": nested_value(
            obstacle_steering, "steering_jerk_abs_radps2.p95"
        ),
        "obstacle_steering_variation_per_m": obstacle_steering.get(
            "variation_per_m"
        ),
        "obstacle_steering_reversals_per_m": obstacle_steering.get(
            "sign_reversals_per_m"
        ),
        "reactive_entries": modes["reactive_entries"],
        "guard_blocked_entries": modes["guard_blocked_entries"],
        "lower_estop_entries": modes["lower_estop_entries"],
        "planner_failure_entries": modes["planner_failure_entry_total"],
        "accepted_plan_count": len(plan["accepted_plan_ids"]),
        "first_lap_sec": laps["first_lap_sec"],
        "steady_lap_median_sec": laps["steady_lap_median_sec"],
    }


def analyze_trial(
    csv_path: Path,
    root: Path,
    raceline_curvature: Sequence[float] | None,
    args: argparse.Namespace,
) -> dict[str, Any]:
    warnings: list[str] = []
    rows = read_csv_rows(csv_path, warnings)
    sidecar_base = csv_path.with_suffix("")
    events_path = sidecar_base.with_suffix(".events.jsonl")
    summary_path = sidecar_base.with_suffix(".summary.json")
    events = read_events(events_path, warnings)
    summary = read_json(summary_path, warnings)
    attempts = extract_plan_attempts(events)
    planning = plan_metrics(attempts, rows)
    tracking = tracking_metrics(
        rows,
        raceline_curvature,
        args.straight_curvature_max,
        args.minimum_derivative_dt,
        args.maximum_derivative_dt,
        args.steering_deadband,
    )
    modes = mode_and_failure_metrics(rows, args.maximum_sample_gap)
    laps = derive_lap_metrics(rows, summary)
    scenario = classify_scenario(csv_path, summary, planning, rows)
    try:
        relative = csv_path.relative_to(root)
    except ValueError:
        relative = csv_path.name
    trial: dict[str, Any] = {
        "id": str(Path(relative).with_suffix("")),
        "trial_label": str(summary.get("trial") or csv_path.stem),
        "scenario": scenario,
        "files": {
            "csv": str(csv_path),
            "events": str(events_path) if events_path.exists() else None,
            "summary": str(summary_path) if summary_path.exists() else None,
        },
        "sample_count": len(rows),
        "event_count": len(events),
        "duration_sec": (
            finite_float(summary.get("duration_sec"))
            or (finite_float(rows[-1].get("elapsed_sec")) if rows else None)
        ),
        "planning": planning,
        "tracking": tracking,
        "modes_and_failures": modes,
        "laps": laps,
        "warnings": warnings,
    }
    trial["scorecard"] = make_scorecard(trial)
    return trial


def aggregate_profile(name: str, root: Path, trials: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    by_scenario: defaultdict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for trial in trials:
        by_scenario[str(trial["scenario"])].append(trial)

    def metric_stats(selected: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
        return {
            metric: describe(trial["scorecard"].get(metric) for trial in selected)
            for metric in COMPARISON_METRICS
        }

    groups: dict[str, Any] = {"all": metric_stats(trials)}
    for scenario, selected in sorted(by_scenario.items()):
        groups[scenario] = metric_stats(selected)

    accepted_attempts = [
        attempt
        for trial in trials
        for attempt in trial["planning"]["attempts"]
        if attempt.get("outcome") == "accepted"
    ]
    all_lap_durations = [
        duration
        for trial in trials
        for duration in trial["laps"]["lap_durations_sec"]
    ]
    return {
        "name": name,
        "directory": str(root),
        "trial_count": len(trials),
        "scenario_counts": dict(Counter(str(trial["scenario"]) for trial in trials)),
        "totals": {
            "samples": sum(int(trial["sample_count"]) for trial in trials),
            "events": sum(int(trial["event_count"]) for trial in trials),
            "accepted_attempts": len(accepted_attempts),
            "reactive_entries": sum(
                int(trial["modes_and_failures"]["reactive_entries"]) for trial in trials
            ),
            "guard_blocked_entries": sum(
                int(trial["modes_and_failures"]["guard_blocked_entries"])
                for trial in trials
            ),
            "lower_estop_entries": sum(
                int(trial["modes_and_failures"]["lower_estop_entries"])
                for trial in trials
            ),
            "planner_failure_entries": sum(
                int(trial["modes_and_failures"]["planner_failure_entry_total"])
                for trial in trials
            ),
            "completed_laps": sum(int(trial["laps"]["completed_laps"]) for trial in trials),
        },
        "plan_level": {
            "hold_to_accepted_ms": describe(
                attempt.get("hold_to_outcome_ms") for attempt in accepted_attempts
            ),
            "lattice_clearance_grid_time_ms": describe(
                attempt.get("lattice_clearance_grid_time_ms")
                for attempt in accepted_attempts
            ),
            "lattice_compute_time_ms": describe(
                attempt.get("lattice_compute_time_ms") for attempt in accepted_attempts
            ),
            "unattributed_latency_ms": describe(
                attempt.get("unattributed_latency_ms") for attempt in accepted_attempts
            ),
            "minimum_clearance_m": describe(
                attempt.get("minimum_clearance_m") for attempt in accepted_attempts
            ),
            "maximum_curvature_inv_m": describe(
                attempt.get("maximum_curvature_inv_m")
                for attempt in accepted_attempts
            ),
        },
        "lap_level": describe(all_lap_durations),
        "trial_metric_stats": groups,
    }


def pair_trials(
    baseline: Sequence[Mapping[str, Any]], candidate: Sequence[Mapping[str, Any]]
) -> tuple[list[tuple[Mapping[str, Any], Mapping[str, Any], str]], list[str], list[str]]:
    candidate_by_id = {str(trial["id"]): trial for trial in candidate}
    pairs: list[tuple[Mapping[str, Any], Mapping[str, Any], str]] = []
    used_baseline: set[str] = set()
    used_candidate: set[str] = set()
    for trial in baseline:
        trial_id = str(trial["id"])
        match = candidate_by_id.get(trial_id)
        if match is not None:
            pairs.append((trial, match, "exact_id"))
            used_baseline.add(trial_id)
            used_candidate.add(str(match["id"]))

    baseline_remaining: defaultdict[str, list[Mapping[str, Any]]] = defaultdict(list)
    candidate_remaining: defaultdict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for trial in baseline:
        if str(trial["id"]) not in used_baseline:
            baseline_remaining[str(trial["scenario"])].append(trial)
    for trial in candidate:
        if str(trial["id"]) not in used_candidate:
            candidate_remaining[str(trial["scenario"])].append(trial)
    for scenario in sorted(set(baseline_remaining) | set(candidate_remaining)):
        left = sorted(baseline_remaining[scenario], key=lambda trial: str(trial["id"]))
        right = sorted(candidate_remaining[scenario], key=lambda trial: str(trial["id"]))
        for baseline_trial, candidate_trial in zip(left, right):
            pairs.append((baseline_trial, candidate_trial, "scenario_ordinal"))
            used_baseline.add(str(baseline_trial["id"]))
            used_candidate.add(str(candidate_trial["id"]))

    unmatched_baseline = sorted(
        str(trial["id"])
        for trial in baseline
        if str(trial["id"]) not in used_baseline
    )
    unmatched_candidate = sorted(
        str(trial["id"])
        for trial in candidate
        if str(trial["id"]) not in used_candidate
    )
    return pairs, unmatched_baseline, unmatched_candidate


def bootstrap_median_delta_ci(
    deltas: Sequence[float], samples: int, seed: int
) -> list[float] | None:
    if not deltas or samples <= 0:
        return None
    data = np.asarray(deltas, dtype=np.float64)
    rng = np.random.default_rng(seed)
    indices = rng.integers(0, data.size, size=(samples, data.size))
    medians = np.median(data[indices], axis=1)
    return [
        float(np.percentile(medians, 2.5)),
        float(np.percentile(medians, 97.5)),
    ]


def compare_profiles(
    baseline: Sequence[Mapping[str, Any]],
    candidate: Sequence[Mapping[str, Any]],
    bootstrap_samples: int,
    seed: int,
) -> dict[str, Any]:
    pairs, unmatched_baseline, unmatched_candidate = pair_trials(baseline, candidate)
    metric_results: dict[str, Any] = {}
    for metric, label in COMPARISON_METRICS.items():
        observations: list[dict[str, Any]] = []
        absolute_deltas: list[float] = []
        percent_deltas: list[float] = []
        baseline_values: list[float] = []
        candidate_values: list[float] = []
        for baseline_trial, candidate_trial, pairing_method in pairs:
            baseline_value = finite_float(baseline_trial["scorecard"].get(metric))
            candidate_value = finite_float(candidate_trial["scorecard"].get(metric))
            if baseline_value is None or candidate_value is None:
                continue
            delta = candidate_value - baseline_value
            percent = (
                100.0 * delta / abs(baseline_value)
                if abs(baseline_value) > 1e-12
                else None
            )
            baseline_values.append(baseline_value)
            candidate_values.append(candidate_value)
            absolute_deltas.append(delta)
            if percent is not None:
                percent_deltas.append(percent)
            observations.append(
                {
                    "baseline_trial": baseline_trial["id"],
                    "candidate_trial": candidate_trial["id"],
                    "pairing_method": pairing_method,
                    "baseline": baseline_value,
                    "candidate": candidate_value,
                    "delta": delta,
                    "percent_change": percent,
                }
            )
        baseline_median = percentile(baseline_values, 50.0)
        candidate_median = percentile(candidate_values, 50.0)
        metric_results[metric] = {
            "label": label,
            "pair_count": len(observations),
            "baseline": describe(baseline_values),
            "candidate": describe(candidate_values),
            "candidate_to_baseline_median_ratio": (
                candidate_median / baseline_median
                if baseline_median is not None
                and candidate_median is not None
                and abs(baseline_median) > 1e-12
                else None
            ),
            "paired_delta": describe(absolute_deltas),
            "paired_percent_change": describe(percent_deltas),
            "paired_median_delta_95pct_bootstrap_ci": bootstrap_median_delta_ci(
                absolute_deltas, bootstrap_samples, seed
            ),
            "observations": observations,
        }
    return {
        "pair_count": len(pairs),
        "pairs": [
            {
                "baseline_trial": baseline_trial["id"],
                "candidate_trial": candidate_trial["id"],
                "scenario": baseline_trial["scenario"],
                "method": method,
            }
            for baseline_trial, candidate_trial, method in pairs
        ],
        "unmatched_baseline_trials": unmatched_baseline,
        "unmatched_candidate_trials": unmatched_candidate,
        "metrics": metric_results,
        "bootstrap_samples": bootstrap_samples,
        "bootstrap_seed": seed,
    }


def discover_csvs(root: Path) -> list[Path]:
    return sorted(path for path in root.rglob("*.csv") if path.is_file())


def analyze_directory(
    name: str,
    root: Path,
    raceline_curvature: Sequence[float] | None,
    args: argparse.Namespace,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    trials = [
        analyze_trial(path, root, raceline_curvature, args)
        for path in discover_csvs(root)
    ]
    return trials, aggregate_profile(name, root, trials)


def format_number(value: Any, digits: int = 4) -> str:
    number = finite_float(value)
    if number is None:
        return "—"
    return f"{number:.{digits}f}"


def markdown_table(headers: Sequence[str], rows: Sequence[Sequence[Any]]) -> list[str]:
    def escape(value: Any) -> str:
        return str(value).replace("|", "\\|").replace("\n", " ")

    result = [
        "| " + " | ".join(escape(header) for header in headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    result.extend(
        "| " + " | ".join(escape(value) for value in row) + " |" for row in rows
    )
    return result


def render_markdown(report: Mapping[str, Any]) -> str:
    baseline = report["profiles"]["baseline"]
    candidate = report["profiles"]["candidate"]
    lines = [
        "# Static planner profile A/B report",
        "",
        f"Baseline: `{baseline['directory']}`  ",
        f"Candidate: `{candidate['directory']}`",
        "",
        "The hold-to-outcome measurement is an end-to-end proxy. It includes planner work, "
        "input waiting, callback scheduling, and event delivery; grid and DP timings are the "
        "planner-reported submodule measurements.",
        "",
        "## Profile overview",
        "",
    ]
    overview_rows = []
    for profile in (baseline, candidate):
        plan = profile["plan_level"]
        totals = profile["totals"]
        overview_rows.append(
            [
                profile["name"],
                profile["trial_count"],
                totals["accepted_attempts"],
                format_number(plan["hold_to_accepted_ms"]["median"], 3),
                format_number(plan["hold_to_accepted_ms"]["p95"], 3),
                format_number(plan["lattice_clearance_grid_time_ms"]["median"], 3),
                format_number(plan["lattice_compute_time_ms"]["median"], 3),
                format_number(plan["minimum_clearance_m"]["minimum"], 4),
                format_number(plan["maximum_curvature_inv_m"]["maximum"], 4),
                totals["reactive_entries"],
                totals["planner_failure_entries"],
                totals["completed_laps"],
            ]
        )
    lines.extend(
        markdown_table(
            [
                "Profile",
                "Trials",
                "Accepted",
                "Latency p50 ms",
                "Latency p95 ms",
                "Grid p50 ms",
                "DP p50 ms",
                "Min clearance m",
                "Max curvature 1/m",
                "Reactive",
                "Planner failures",
                "Laps",
            ],
            overview_rows,
        )
    )
    lines.extend(["", "## Paired comparison", ""])
    comparison_rows = []
    for metric in COMPARISON_METRICS:
        result = report["comparison"]["metrics"][metric]
        if result["pair_count"] == 0:
            continue
        comparison_rows.append(
            [
                result["label"],
                result["pair_count"],
                format_number(result["baseline"]["median"]),
                format_number(result["candidate"]["median"]),
                format_number(result["paired_delta"]["median"]),
                (
                    format_number(result["paired_percent_change"]["median"], 2) + "%"
                    if result["paired_percent_change"]["median"] is not None
                    else "—"
                ),
            ]
        )
    if comparison_rows:
        lines.extend(
            markdown_table(
                ["Metric", "Pairs", "Baseline", "Candidate", "Paired delta", "Change"],
                comparison_rows,
            )
        )
    else:
        lines.append("No paired trials contained comparable numeric metrics.")

    lines.extend(["", "## Per-trial scorecard", ""])
    scorecard_rows = []
    for profile_name in ("baseline", "candidate"):
        for trial in report["trials"][profile_name]:
            score = trial["scorecard"]
            scorecard_rows.append(
                [
                    profile_name,
                    trial["id"],
                    trial["scenario"],
                    format_number(score["plan_latency_median_ms"], 3),
                    format_number(score["minimum_plan_clearance_m"], 4),
                    format_number(score["maximum_plan_curvature_inv_m"], 4),
                    format_number(score["straight_cte_p95_m"], 4),
                    format_number(score["obstacle_steering_p95_rad"], 4),
                    score["reactive_entries"],
                    score["planner_failure_entries"],
                    trial["laps"]["completed_laps"],
                ]
            )
    lines.extend(
        markdown_table(
            [
                "Profile",
                "Trial",
                "Scenario",
                "Plan p50 ms",
                "Min clearance m",
                "Max curvature 1/m",
                "Straight CTE p95 m",
                "Avoid steer p95 rad",
                "Reactive",
                "Failures",
                "Laps",
            ],
            scorecard_rows,
        )
    )

    attempts = []
    for profile_name in ("baseline", "candidate"):
        for trial in report["trials"][profile_name]:
            for attempt in trial["planning"]["attempts"]:
                attempts.append(
                    [
                        profile_name,
                        trial["id"],
                        attempt.get("attempt_index"),
                        attempt.get("kind"),
                        attempt.get("outcome"),
                        attempt.get("plan_id") or "—",
                        format_number(attempt.get("hold_to_outcome_ms"), 3),
                        format_number(attempt.get("lattice_clearance_grid_time_ms"), 3),
                        format_number(attempt.get("lattice_compute_time_ms"), 3),
                        format_number(attempt.get("unattributed_latency_ms"), 3),
                        format_number(attempt.get("minimum_clearance_m"), 4),
                        format_number(attempt.get("maximum_curvature_inv_m"), 4),
                    ]
                )
    lines.extend(["", "## Planning attempts", ""])
    if attempts:
        lines.extend(
            markdown_table(
                [
                    "Profile",
                    "Trial",
                    "#",
                    "Kind",
                    "Outcome",
                    "Plan",
                    "Total ms",
                    "Grid ms",
                    "DP ms",
                    "Other ms",
                    "Clearance m",
                    "Curvature 1/m",
                ],
                attempts,
            )
        )
    else:
        lines.append("No PLANNING_HOLD attempts were present in event logs.")

    warning_rows = []
    for profile_name in ("baseline", "candidate"):
        for trial in report["trials"][profile_name]:
            for warning in trial["warnings"]:
                warning_rows.append([profile_name, trial["id"], warning])
    if report["comparison"]["unmatched_baseline_trials"]:
        warning_rows.append(
            [
                "comparison",
                "baseline",
                "Unmatched: "
                + ", ".join(report["comparison"]["unmatched_baseline_trials"]),
            ]
        )
    if report["comparison"]["unmatched_candidate_trials"]:
        warning_rows.append(
            [
                "comparison",
                "candidate",
                "Unmatched: "
                + ", ".join(report["comparison"]["unmatched_candidate_trials"]),
            ]
        )
    if warning_rows:
        lines.extend(["", "## Data-quality warnings", ""])
        lines.extend(markdown_table(["Profile", "Trial", "Warning"], warning_rows))
    lines.append("")
    return "\n".join(lines)


def build_report(args: argparse.Namespace) -> dict[str, Any]:
    global_warnings: list[str] = []
    raceline_path = Path(args.raceline_csv).resolve() if args.raceline_csv else None
    raceline_curvature = load_raceline_curvature(raceline_path, global_warnings)
    baseline_root = Path(args.baseline_dir).resolve()
    candidate_root = Path(args.candidate_dir).resolve()
    baseline_trials, baseline_profile = analyze_directory(
        "baseline", baseline_root, raceline_curvature, args
    )
    candidate_trials, candidate_profile = analyze_directory(
        "candidate", candidate_root, raceline_curvature, args
    )
    if not baseline_trials:
        global_warnings.append(f"{baseline_root}: no CSV trials discovered")
    if not candidate_trials:
        global_warnings.append(f"{candidate_root}: no CSV trials discovered")
    return {
        "schema_version": 1,
        "inputs": {
            "baseline_dir": str(baseline_root),
            "candidate_dir": str(candidate_root),
            "raceline_csv": str(raceline_path) if raceline_path else None,
            "straight_curvature_max_inv_m": args.straight_curvature_max,
            "derivative_dt_range_sec": [
                args.minimum_derivative_dt,
                args.maximum_derivative_dt,
            ],
            "maximum_sample_gap_sec": args.maximum_sample_gap,
            "steering_deadband_rad": args.steering_deadband,
        },
        "profiles": {
            "baseline": baseline_profile,
            "candidate": candidate_profile,
        },
        "trials": {
            "baseline": baseline_trials,
            "candidate": candidate_trials,
        },
        "comparison": compare_profiles(
            baseline_trials,
            candidate_trials,
            args.bootstrap_samples,
            args.bootstrap_seed,
        ),
        "warnings": global_warnings,
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare obstacle_trial_logger CSV/event/summary directories without "
            "pooling repeated samples as independent planner decisions."
        )
    )
    parser.add_argument("--baseline-dir", required=True, help="Baseline log directory")
    parser.add_argument("--candidate-dir", required=True, help="Candidate log directory")
    parser.add_argument(
        "--raceline-csv",
        help=(
            "Raceline CSV containing curvature_abs or curvature; enables "
            "low-curvature clear-track metrics"
        ),
    )
    parser.add_argument(
        "--straight-curvature-max",
        type=float,
        default=0.05,
        help="Maximum absolute raceline curvature for straight samples (default: 0.05 1/m)",
    )
    parser.add_argument(
        "--minimum-derivative-dt",
        type=float,
        default=0.02,
        help="Minimum sample interval used for steering derivatives (default: 0.02 s)",
    )
    parser.add_argument(
        "--maximum-derivative-dt",
        type=float,
        default=0.10,
        help="Maximum sample interval used for steering derivatives (default: 0.10 s)",
    )
    parser.add_argument(
        "--maximum-sample-gap",
        type=float,
        default=0.25,
        help="Maximum sample-and-hold gap counted in mode durations (default: 0.25 s)",
    )
    parser.add_argument(
        "--steering-deadband",
        type=float,
        default=0.01,
        help="Deadband used when counting steering sign reversals (default: 0.01 rad)",
    )
    parser.add_argument(
        "--bootstrap-samples",
        type=int,
        default=10000,
        help="Bootstrap resamples for paired median-delta CI; 0 disables it",
    )
    parser.add_argument(
        "--bootstrap-seed", type=int, default=20260826, help="Deterministic bootstrap seed"
    )
    parser.add_argument("--output", required=True, help="Output JSON report path")
    parser.add_argument(
        "--markdown-output",
        help="Output Markdown path (default: JSON output path with .md suffix)",
    )
    args = parser.parse_args(argv)
    if args.straight_curvature_max < 0.0:
        parser.error("--straight-curvature-max must be >= 0")
    if args.minimum_derivative_dt <= 0.0:
        parser.error("--minimum-derivative-dt must be > 0")
    if args.maximum_derivative_dt < args.minimum_derivative_dt:
        parser.error("--maximum-derivative-dt must be >= --minimum-derivative-dt")
    if args.maximum_sample_gap <= 0.0:
        parser.error("--maximum-sample-gap must be > 0")
    if args.steering_deadband < 0.0:
        parser.error("--steering-deadband must be >= 0")
    if args.bootstrap_samples < 0:
        parser.error("--bootstrap-samples must be >= 0")
    for attribute in ("baseline_dir", "candidate_dir"):
        path = Path(getattr(args, attribute))
        if not path.is_dir():
            parser.error(f"--{attribute.replace('_', '-')} is not a directory: {path}")
    if args.raceline_csv and not Path(args.raceline_csv).is_file():
        parser.error(f"--raceline-csv is not a file: {args.raceline_csv}")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    report = build_report(args)
    output_path = Path(args.output).resolve()
    markdown_path = (
        Path(args.markdown_output).resolve()
        if args.markdown_output
        else output_path.with_suffix(".md")
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    markdown_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with markdown_path.open("w", encoding="utf-8") as stream:
        stream.write(render_markdown(report))
    print(f"Wrote JSON report: {output_path}")
    print(f"Wrote Markdown report: {markdown_path}")
    if report["warnings"]:
        for warning in report["warnings"]:
            print(f"warning: {warning}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
