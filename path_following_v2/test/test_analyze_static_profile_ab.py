#!/usr/bin/env python3
"""Focused tests for the offline obstacle-trial A/B analyzer."""

import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "tools" / "analyze_static_profile_ab.py"
)
SPEC = importlib.util.spec_from_file_location("analyze_static_profile_ab", MODULE_PATH)
ANALYZER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(ANALYZER)


class StaticProfileAnalyzerTest(unittest.TestCase):
    def write_trial(self, root, latency_sec, steering_scale=1.0):
        csv_path = root / "west_uturn_1.csv"
        rows = []
        modes = [
            "RACELINE",
            "RACELINE",
            "AVOIDANCE_DEPARTING",
            "AVOIDANCE_PASSING",
            "AVOIDANCE_RETURNING",
        ]
        steering = [0.01, -0.02, 0.10, -0.10, 0.02]
        for index, mode in enumerate(modes):
            rows.append(
                {
                    "elapsed_sec": 0.05 * index,
                    "x_m": 0.1 * index,
                    "y_m": 0.0,
                    "drive_steering_rad": steering[index] * steering_scale,
                    "cross_track_error_m": 0.01 * (index + 1),
                    "nearest_raceline_index": index,
                    "planner_trajectory_mode": mode,
                    "planner_state": "READY",
                    "planner_plan_id": "1" if index >= 2 else "0",
                    "planner_minimum_clearance_m": "0.31" if index >= 2 else "",
                    "planner_maximum_curvature_inv_m": "1.0" if index >= 2 else "",
                    "planner_lattice_clearance_grid_time_ms": "2.0",
                    "planner_lattice_compute_time_ms": "1.0",
                    "arbitrator_mode": "RACELINE",
                    "arbitrator_primary_failure": "none",
                    "guard_state": "CLEAR",
                    "lower_mode": "NOMINAL",
                    "completed_laps": 0,
                }
            )
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

        events = [
            {
                "elapsed_sec": 0.01,
                "planner": {
                    "state": "READY",
                    "trajectory_mode": "PLANNING_HOLD",
                    "reason": "computing avoidance while following a collision-free raceline prefix",
                    "plan_id": "0",
                },
            },
            {
                "elapsed_sec": 0.01 + latency_sec,
                "planner": {
                    "state": "READY",
                    "trajectory_mode": "AVOIDANCE_DEPARTING",
                    "reason": "persistent local trajectory accepted",
                    "plan_id": "1",
                    "candidate_selected_side": "LEFT",
                    "lattice_clearance_grid_time_ms": "2.0",
                    "lattice_compute_time_ms": "1.0",
                    "minimum_clearance_m": "0.31",
                    "maximum_curvature_inv_m": "1.0",
                },
            },
        ]
        with (root / "west_uturn_1.events.jsonl").open(
            "w", encoding="utf-8"
        ) as stream:
            for event in events:
                stream.write(json.dumps(event) + "\n")
        with (root / "west_uturn_1.summary.json").open(
            "w", encoding="utf-8"
        ) as stream:
            json.dump(
                {
                    "trial": "west_uturn_1",
                    "duration_sec": 20.0,
                    "completed_laps": 2,
                    "target_laps": 2,
                    "target_laps_reached": True,
                    "lap_completion_times_sec": [10.0, 19.0],
                    "lap_durations_sec": [10.0, 9.0],
                },
                stream,
            )

    @staticmethod
    def write_raceline(path):
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["x", "y", "curvature_abs"])
            writer.writeheader()
            for index in range(5):
                writer.writerow({"x": index, "y": 0, "curvature_abs": 0.01})

    def test_hold_entry_is_paired_with_first_exit(self):
        attempts = ANALYZER.extract_plan_attempts(
            [
                {
                    "elapsed_sec": 1.0,
                    "planner": {
                        "trajectory_mode": "PLANNING_HOLD",
                        "reason": "computing avoidance",
                    },
                },
                {
                    "elapsed_sec": 1.002,
                    "planner": {
                        "trajectory_mode": "PLANNING_HOLD",
                        "reason": "heartbeat changed a diagnostic counter",
                    },
                },
                {
                    "elapsed_sec": 1.008,
                    "planner": {
                        "state": "READY",
                        "trajectory_mode": "AVOIDANCE_DEPARTING",
                        "reason": "persistent local trajectory accepted",
                        "plan_id": "7",
                    },
                },
            ]
        )
        self.assertEqual(len(attempts), 1)
        self.assertEqual(attempts[0]["outcome"], "accepted")
        self.assertAlmostEqual(attempts[0]["hold_to_outcome_ms"], 8.0)

    def test_cli_builds_json_and_markdown_report(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline = root / "baseline"
            candidate = root / "candidate"
            baseline.mkdir()
            candidate.mkdir()
            self.write_trial(baseline, 0.012)
            self.write_trial(candidate, 0.006, steering_scale=0.9)
            raceline = root / "raceline.csv"
            self.write_raceline(raceline)
            output = root / "report.json"

            return_code = ANALYZER.main(
                [
                    "--baseline-dir",
                    str(baseline),
                    "--candidate-dir",
                    str(candidate),
                    "--raceline-csv",
                    str(raceline),
                    "--bootstrap-samples",
                    "50",
                    "--output",
                    str(output),
                ]
            )

            self.assertEqual(return_code, 0)
            self.assertTrue(output.exists())
            self.assertTrue(output.with_suffix(".md").exists())
            report = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(report["comparison"]["pair_count"], 1)
            latency = report["comparison"]["metrics"]["plan_latency_median_ms"]
            self.assertAlmostEqual(latency["baseline"]["median"], 12.0)
            self.assertAlmostEqual(latency["candidate"]["median"], 6.0)
            baseline_trial = report["trials"]["baseline"][0]
            self.assertAlmostEqual(
                baseline_trial["scorecard"]["minimum_plan_clearance_m"], 0.31
            )
            self.assertGreater(
                baseline_trial["tracking"]["steering"]["obstacle_maneuver"]
                ["steering_rate_abs_radps"]["count"],
                0,
            )
            self.assertIn(
                "Planning attempts", output.with_suffix(".md").read_text(encoding="utf-8")
            )

    def test_missing_sidecars_are_reported_without_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            csv_path = root / "clean.csv"
            csv_path.write_text(
                "elapsed_sec,drive_steering_rad,planner_trajectory_mode\n"
                "0.0,0.01,RACELINE\n",
                encoding="utf-8",
            )
            args = ANALYZER.parse_args(
                [
                    "--baseline-dir",
                    str(root),
                    "--candidate-dir",
                    str(root),
                    "--output",
                    str(root / "report.json"),
                ]
            )
            report = ANALYZER.build_report(args)
            warnings = report["trials"]["baseline"][0]["warnings"]
            self.assertTrue(any("event sidecar is missing" in item for item in warnings))
            self.assertTrue(any("summary sidecar is missing" in item for item in warnings))
            self.assertEqual(
                report["trials"]["baseline"][0]["planning"]["attempt_count"], 0
            )


if __name__ == "__main__":
    unittest.main()
