#!/usr/bin/env python3
"""Focused geometry checks for the generated Spielberg traffic route."""

import csv
import math
from pathlib import Path
import sys
import unittest

import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
CENTERLINE_TOOLS = REPO_ROOT / "centerline_tools"
sys.path.insert(0, str(CENTERLINE_TOOLS))

import generate_moving_agent_path as route_generator  # noqa: E402


class MovingAgentRouteTest(unittest.TestCase):
    """Verify raceline coverage, safe nudges, and equal-arc starts."""

    @classmethod
    def setUpClass(cls):
        cls.output_dir = (
            CENTERLINE_TOOLS
            / "moving_agent_output"
            / "spielberg_slow_agent"
        )
        cls.route_csv = cls.output_dir / "spielberg_slow_agent_path.csv"
        cls.report_yaml = (
            cls.output_dir / "spielberg_slow_agent_path_validation.yaml"
        )
        with cls.report_yaml.open("r", encoding="utf-8") as stream:
            cls.report = yaml.safe_load(stream)
        with cls.route_csv.open("r", newline="", encoding="utf-8") as stream:
            cls.rows = list(csv.DictReader(stream))
        cls.route = np.asarray(
            [[float(row["x"]), float(row["y"])] for row in cls.rows],
            dtype=np.float64,
        )
        cls.weight = np.asarray(
            [float(row["nudge_weight"]) for row in cls.rows],
            dtype=np.float64,
        )
        cls.curvature = np.asarray(
            [float(row["curvature"]) for row in cls.rows],
            dtype=np.float64,
        )
        cls.raceline, _ = route_generator.load_reference_path(
            REPO_ROOT / cls.report["inputs"]["raceline"],
            "Raceline",
        )

    def test_route_is_exact_raceline_away_from_nudge_windows(self):
        self.assertEqual(len(self.route), len(self.raceline))
        exact = self.weight <= 1e-12
        self.assertGreaterEqual(float(np.mean(exact)), 0.70)
        np.testing.assert_array_equal(self.route[exact], self.raceline[exact])
        self.assertAlmostEqual(
            float(np.mean(exact)),
            self.report["metrics"]["exact_raceline_point_fraction"],
            places=12,
        )

    def test_dense_route_and_every_obstacle_hold_are_safe(self):
        parameters = self.report["parameters"]
        grid = route_generator.load_clearance_grid(
            REPO_ROOT / self.report["inputs"]["map_image"],
            REPO_ROOT / self.report["inputs"]["map_yaml"],
        )
        dense = route_generator.densify_closed_path(
            self.route,
            parameters["continuous_clearance_step_m"],
        )
        dense_minimum = float(
            np.min(route_generator.sample_map_clearance(dense, grid))
        )
        self.assertGreaterEqual(
            dense_minimum,
            parameters["minimum_map_clearance_m"],
        )
        self.assertAlmostEqual(
            dense_minimum,
            self.report["metrics"]["minimum_continuous_map_clearance_m"],
            places=9,
        )
        self.assertLessEqual(
            float(np.max(np.abs(self.curvature))),
            parameters["maximum_curvature_inv_m"],
        )

        nudges = self.report["nudges"]
        self.assertEqual(len(nudges), self.report["metrics"]["nudge_count"])
        self.assertEqual(len(nudges), 9)
        for nudge in nudges:
            self.assertGreaterEqual(
                nudge["minimum_hold_map_clearance_m"],
                parameters["minimum_nudge_clearance_m"],
                msg=f"unsafe hold at obstacle {nudge['obstacle_id']}",
            )
            self.assertLessEqual(
                nudge["maximum_hold_curvature_inv_m"],
                parameters["maximum_curvature_inv_m"],
                msg=f"excess curvature at obstacle {nudge['obstacle_id']}",
            )
            expected_sign = -1.0 if nudge["obstacle_side"] == "left" else 1.0
            actual_sign = math.copysign(1.0, nudge["guide_offset_m"])
            self.assertEqual(actual_sign, expected_sign)

    def test_ten_spawn_poses_have_exact_equal_arc_phase(self):
        spawn = self.report["spawn"]
        poses = spawn["poses"]
        self.assertEqual(spawn["count"], 10)
        self.assertEqual(len(poses), 10)
        _, _, lap_length = route_generator.closed_geometry(self.route)
        self.assertAlmostEqual(
            spawn["spacing_m"], lap_length / 10.0, places=12
        )

        for pose in poses:
            progress, error, _ = route_generator.project_point_to_closed_path(
                self.route,
                [pose["x"], pose["y"]],
            )
            self.assertLessEqual(error, 1e-9)
            progress_error = abs(
                route_generator.wrapped_signed_distance(
                    progress,
                    pose["progress_m"],
                    lap_length,
                )
            )
            self.assertLessEqual(progress_error, 1e-9)

        for current, following in zip(poses, poses[1:] + poses[:1]):
            gap = (
                following["progress_m"] - current["progress_m"]
            ) % lap_length
            self.assertAlmostEqual(gap, spawn["spacing_m"], places=10)


if __name__ == "__main__":
    unittest.main()
