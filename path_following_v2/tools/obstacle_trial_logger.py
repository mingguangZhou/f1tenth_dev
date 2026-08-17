#!/usr/bin/env python3
"""Record synchronized obstacle-avoidance diagnostics for one simulator trial."""

import argparse
import csv
import json
import math
import os
import statistics
import time
from collections import Counter

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, UInt8


PLANNER_FIELDS = (
    "state",
    "reason",
    "trajectory_mode",
    "detour_side",
    "plan_id",
    "maneuver_phase",
    "planning_reference",
    "obstacle_distance_m",
    "obstacle_s_min_m",
    "obstacle_s_max_m",
    "obstacle_d_min_m",
    "obstacle_d_max_m",
    "interfering_points",
    "minimum_clearance_m",
    "maximum_curvature_inv_m",
    "peak_offset_m",
    "planner_objective_cost",
    "lattice_result",
    "lattice_compute_time_ms",
    "active_blocked_cycles",
    "active_margin_blocked_cycles",
    "active_physical_blocked_cycles",
    "no_safe_path_cycles",
    "speed_cap_mps",
    "remaining_maximum_curvature_inv_m",
    "maneuver_lateral_acceleration_limit_mps2",
    "rolling_pass_active",
    "obstacle_track_valid",
    "obstacle_track_moving",
    "obstacle_track_longitudinal_speed_mps",
    "obstacle_track_observations",
    "obstacle_track_missing_scans",
    "rolling_pass_clear_cycles",
    "candidate_decision_id",
    "candidate_decision_context",
    "candidate_selected_side",
    "left_candidate_evaluated",
    "left_candidate_valid",
    "left_candidate_reason",
    "left_candidate_minimum_clearance_m",
    "left_candidate_objective_cost",
    "left_candidate_objective_domain",
    "left_candidate_maximum_curvature_inv_m",
    "left_candidate_peak_offset_m",
    "left_candidate_maximum_connected_interference",
    "right_candidate_evaluated",
    "right_candidate_valid",
    "right_candidate_reason",
    "right_candidate_minimum_clearance_m",
    "right_candidate_objective_cost",
    "right_candidate_objective_domain",
    "right_candidate_maximum_curvature_inv_m",
    "right_candidate_peak_offset_m",
    "right_candidate_maximum_connected_interference",
)

ARBITRATOR_FIELDS = (
    "mode",
    "reason",
    "primary_ready",
    "primary_failure",
    "reactive_ready",
    "reactive_latched",
    "latched_trigger_classes",
    "path_generator_state",
    "follower_state",
    "guard_state",
    "reactive_upper_state",
    "lower_status_state",
)

GUARD_FIELDS = (
    "state",
    "reason",
    "interfering_points",
    "blocked_confirmation_count",
    "checked_path_reach_m",
)

REACTIVE_FIELDS = (
    "state",
    "reason",
    "selected_side",
    "corridor_reach_m",
    "usable_branches",
    "swept_path_failure_cycles",
    "smoothed_failure_code",
)

LOWER_FIELDS = (
    "mode",
    "reason",
    "arbitration_mode",
    "front_emergency_active",
    "fallback_available",
    "current_speed_mps",
)


def diagnostic_values(message):
    if not message.status:
        return {}
    status = message.status[0]
    values = {item.key: item.value for item in status.values}
    values["diagnostic_name"] = status.name
    values["diagnostic_message"] = status.message
    return values


def finite_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def load_closed_path_geometry(path):
    with open(path, "r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or not {"x", "y"}.issubset(reader.fieldnames):
            raise RuntimeError(f"Raceline CSV must contain x and y columns: {path}")
        points = np.asarray(
            [[float(row["x"]), float(row["y"])] for row in reader],
            dtype=np.float64,
        )
    if len(points) < 3:
        raise RuntimeError(f"Raceline CSV has too few points: {path}")
    segment_lengths = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths[:-1])))
    lap_length = float(np.sum(segment_lengths))
    if not math.isfinite(lap_length) or lap_length <= 0.0:
        raise RuntimeError(f"Raceline CSV has invalid closed-loop length: {path}")
    return points, cumulative, lap_length


class TrialLogger(Node):
    def __init__(
        self,
        output_path,
        trial_name,
        sample_hz,
        reset_pose=None,
        raceline_csv=None,
        target_laps=0,
    ):
        super().__init__("obstacle_trial_logger")
        self.output_path = output_path
        self.event_path = os.path.splitext(output_path)[0] + ".events.jsonl"
        self.summary_path = os.path.splitext(output_path)[0] + ".summary.json"
        self.trial_name = trial_name
        self.started = time.monotonic()
        self.latest = {
            "planner": {},
            "arbitrator": {},
            "guard": {},
            "reactive": {},
            "lower": {},
        }
        self.pose = {"x": "", "y": "", "yaw": "", "speed": ""}
        self.drive = {"speed": "", "steering": ""}
        self.scan = {"front_min": "", "valid_ratio": ""}
        self.speed_cap = ""
        self.selected_mode = ""
        self.rows = []
        self.events = []
        self.last_event_signature = None
        self.reset_pose = reset_pose
        self.target_laps = max(0, int(target_laps))
        self.raceline_points = None
        self.raceline_cumulative_m = None
        self.raceline_lap_length_m = 0.0
        self.nearest_raceline_index = None
        self.cross_track_error_m = None
        self.last_raceline_progress_m = None
        self.accumulated_lap_distance_m = 0.0
        self.completed_laps = 0
        self.lap_completion_times_sec = []
        if raceline_csv:
            (
                self.raceline_points,
                self.raceline_cumulative_m,
                self.raceline_lap_length_m,
            ) = load_closed_path_geometry(raceline_csv)

        self.create_subscription(
            Odometry, "/ego_racecar/odom", self.odom_callback, 10
        )
        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            AckermannDriveStamped, "/drive", self.drive_callback, 10
        )
        self.create_subscription(
            Float64,
            "/path_following_v2/trajectory_speed_cap_mps",
            self.speed_cap_callback,
            10,
        )
        self.create_subscription(
            UInt8,
            "/drive_arbitration_v2/selected_mode",
            self.selected_mode_callback,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/path_following_v2/path_status",
            lambda message: self.diagnostic_callback("planner", message),
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/drive_arbitration_v2/status",
            lambda message: self.diagnostic_callback("arbitrator", message),
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/drive_arbitration_v2/raceline_guard_status",
            lambda message: self.diagnostic_callback("guard", message),
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/reactive_control_v2/status",
            lambda message: self.diagnostic_callback("reactive", message),
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/reactive_control_v2/lower_safety_status",
            lambda message: self.diagnostic_callback("lower", message),
            10,
        )
        self.create_timer(1.0 / max(sample_hz, 1.0), self.sample)
        self.pose_reset_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.planner_reset_pub = self.create_publisher(
            Bool, "/path_following_v2/reset", 10
        )
        self.arbitration_reset_pub = self.create_publisher(
            Bool, "/drive_arbitration_v2/reset", 10
        )
        self.drive_reset_pub = self.create_publisher(
            AckermannDriveStamped, "/drive", 10
        )

    def elapsed(self):
        return time.monotonic() - self.started

    def publish_trial_reset(self, reset_state=False):
        if self.reset_pose is None:
            return
        x, y, yaw = self.reset_pose
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_reset_pub.publish(pose)

        stop = AckermannDriveStamped()
        stop.drive.speed = 0.0
        stop.drive.steering_angle = 0.0
        self.drive_reset_pub.publish(stop)

        if reset_state:
            reset = Bool()
            reset.data = True
            self.planner_reset_pub.publish(reset)
            self.arbitration_reset_pub.publish(reset)

    def prepare_trial(self):
        if self.reset_pose is None:
            return

        # Let DDS discover all reset subscribers before publishing. Hold the car
        # at the requested pose long enough for the raw local raceline to update,
        # then reset planner state once more immediately before recording.
        discovery_deadline = time.monotonic() + 0.75
        while rclpy.ok() and time.monotonic() < discovery_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        hold_deadline = time.monotonic() + 0.50
        first = True
        while rclpy.ok() and time.monotonic() < hold_deadline:
            self.publish_trial_reset(reset_state=first)
            first = False
            rclpy.spin_once(self, timeout_sec=0.04)

        self.publish_trial_reset(reset_state=True)
        self.rows.clear()
        self.events.clear()
        self.last_event_signature = None
        self.reset_lap_progress()
        self.started = time.monotonic()

    def reset_lap_progress(self):
        self.nearest_raceline_index = None
        self.cross_track_error_m = None
        self.last_raceline_progress_m = None
        self.accumulated_lap_distance_m = 0.0
        self.completed_laps = 0
        self.lap_completion_times_sec = []

    def update_lap_progress(self, x, y):
        if self.raceline_points is None:
            return
        position = np.asarray([x, y], dtype=np.float64)
        squared_distances = np.sum(
            (self.raceline_points - position) ** 2, axis=1
        )
        nearest = int(np.argmin(squared_distances))
        progress = float(self.raceline_cumulative_m[nearest])
        self.nearest_raceline_index = nearest
        self.cross_track_error_m = math.sqrt(float(squared_distances[nearest]))
        if self.last_raceline_progress_m is not None:
            delta = progress - self.last_raceline_progress_m
            half_lap = 0.5 * self.raceline_lap_length_m
            if delta < -half_lap:
                delta += self.raceline_lap_length_m
            elif delta > half_lap:
                delta -= self.raceline_lap_length_m
            # Reject localization teleports while accepting any physically
            # plausible update interval in this simulator.
            if abs(delta) <= 20.0:
                self.accumulated_lap_distance_m += delta
        self.last_raceline_progress_m = progress

        completed = max(
            0,
            int(self.accumulated_lap_distance_m // self.raceline_lap_length_m),
        )
        while completed > self.completed_laps:
            self.completed_laps += 1
            self.lap_completion_times_sec.append(self.elapsed())

    def odom_callback(self, message):
        orientation = message.pose.pose.orientation
        siny = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        self.pose = {
            "x": message.pose.pose.position.x,
            "y": message.pose.pose.position.y,
            "yaw": math.atan2(siny, cosy),
            "speed": math.hypot(
                message.twist.twist.linear.x, message.twist.twist.linear.y
            ),
        }
        self.update_lap_progress(self.pose["x"], self.pose["y"])

    def scan_callback(self, message):
        valid = [
            value
            for value in message.ranges
            if math.isfinite(value) and message.range_min <= value <= message.range_max
        ]
        front = []
        for index, value in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if abs(angle) <= math.radians(20.0) and math.isfinite(value):
                front.append(value)
        self.scan = {
            "front_min": min(front) if front else "",
            "valid_ratio": len(valid) / len(message.ranges) if message.ranges else 0.0,
        }

    def drive_callback(self, message):
        self.drive = {
            "speed": message.drive.speed,
            "steering": message.drive.steering_angle,
        }

    def speed_cap_callback(self, message):
        self.speed_cap = message.data

    def selected_mode_callback(self, message):
        self.selected_mode = message.data

    def diagnostic_callback(self, source, message):
        self.latest[source] = diagnostic_values(message)
        self.record_event_if_changed()

    def record_event_if_changed(self):
        planner = self.latest["planner"]
        arbitrator = self.latest["arbitrator"]
        guard = self.latest["guard"]
        lower = self.latest["lower"]
        signature = (
            planner.get("state", ""),
            planner.get("trajectory_mode", ""),
            planner.get("detour_side", ""),
            planner.get("plan_id", ""),
            planner.get("candidate_decision_id", ""),
            planner.get("reason", ""),
            planner.get("active_blocked_cycles", ""),
            planner.get("no_safe_path_cycles", ""),
            arbitrator.get("mode", ""),
            arbitrator.get("reason", ""),
            arbitrator.get("primary_failure", ""),
            arbitrator.get("reactive_latched", ""),
            guard.get("state", ""),
            guard.get("reason", ""),
            lower.get("mode", ""),
            lower.get("reason", ""),
        )
        if signature == self.last_event_signature:
            return
        self.last_event_signature = signature
        event = {
            "elapsed_sec": round(self.elapsed(), 6),
            "pose": dict(self.pose),
            "planner": dict(planner),
            "arbitrator": dict(arbitrator),
            "guard": dict(guard),
            "reactive": dict(self.latest["reactive"]),
            "lower": dict(lower),
            "drive": dict(self.drive),
            "scan": dict(self.scan),
        }
        self.events.append(event)

    def sample(self):
        row = {
            "trial": self.trial_name,
            "elapsed_sec": round(self.elapsed(), 6),
            "x_m": self.pose["x"],
            "y_m": self.pose["y"],
            "yaw_rad": self.pose["yaw"],
            "odom_speed_mps": self.pose["speed"],
            "drive_speed_mps": self.drive["speed"],
            "drive_steering_rad": self.drive["steering"],
            "front_scan_min_m": self.scan["front_min"],
            "scan_valid_ratio": self.scan["valid_ratio"],
            "speed_cap_topic_mps": self.speed_cap,
            "selected_mode_code": self.selected_mode,
            "nearest_raceline_index": (
                "" if self.nearest_raceline_index is None else self.nearest_raceline_index
            ),
            "cross_track_error_m": (
                "" if self.cross_track_error_m is None else self.cross_track_error_m
            ),
            "accumulated_lap_distance_m": self.accumulated_lap_distance_m,
            "completed_laps": self.completed_laps,
        }
        groups = (
            ("planner", PLANNER_FIELDS),
            ("arbitrator", ARBITRATOR_FIELDS),
            ("guard", GUARD_FIELDS),
            ("reactive", REACTIVE_FIELDS),
            ("lower", LOWER_FIELDS),
        )
        for prefix, fields in groups:
            values = self.latest[prefix]
            for field in fields:
                row[f"{prefix}_{field}"] = values.get(field, "")
        self.rows.append(row)

    def finish(self):
        os.makedirs(os.path.dirname(os.path.abspath(self.output_path)), exist_ok=True)
        if self.rows:
            with open(self.output_path, "w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(self.rows[0]))
                writer.writeheader()
                writer.writerows(self.rows)
        with open(self.event_path, "w", encoding="utf-8") as stream:
            for event in self.events:
                stream.write(json.dumps(event, sort_keys=True) + "\n")

        planner_states = Counter(
            row["planner_state"] for row in self.rows if row["planner_state"]
        )
        planner_modes = Counter(
            row["planner_trajectory_mode"]
            for row in self.rows
            if row["planner_trajectory_mode"]
        )
        sides = Counter(
            row["planner_detour_side"]
            for row in self.rows
            if row["planner_detour_side"] not in ("", "NONE")
        )
        arbitrator_modes = Counter(
            row["arbitrator_mode"]
            for row in self.rows
            if row["arbitrator_mode"]
        )
        primary_failures = Counter(
            row["arbitrator_primary_failure"]
            for row in self.rows
            if row["arbitrator_primary_failure"] not in ("", "none")
        )
        clearances = [
            value
            for value in (
                finite_float(row["planner_minimum_clearance_m"])
                for row in self.rows
            )
            if value is not None
        ]
        speed_by_mode = {}
        for mode in sorted(planner_modes):
            speeds = [
                value
                for value in (
                    finite_float(row["drive_speed_mps"])
                    for row in self.rows
                    if row["planner_trajectory_mode"] == mode
                )
                if value is not None
            ]
            caps = [
                value
                for value in (
                    finite_float(row["speed_cap_topic_mps"])
                    for row in self.rows
                    if row["planner_trajectory_mode"] == mode
                )
                if value is not None
            ]
            if speeds:
                speed_by_mode[mode] = {
                    "samples": len(speeds),
                    "mean_drive_speed_mps": statistics.fmean(speeds),
                    "median_drive_speed_mps": statistics.median(speeds),
                    "maximum_drive_speed_mps": max(speeds),
                    "mean_speed_cap_mps": statistics.fmean(caps) if caps else None,
                }

        def maximum_counter(field):
            values = [
                value
                for value in (
                    finite_float(row[field]) for row in self.rows
                )
                if value is not None
            ]
            return int(max(values)) if values else 0

        summary = {
            "trial": self.trial_name,
            "duration_sec": self.elapsed(),
            "samples": len(self.rows),
            "events": len(self.events),
            "target_laps": self.target_laps,
            "completed_laps": self.completed_laps,
            "target_laps_reached": (
                self.target_laps > 0 and self.completed_laps >= self.target_laps
            ),
            "raceline_lap_length_m": (
                self.raceline_lap_length_m if self.raceline_points is not None else None
            ),
            "accumulated_lap_distance_m": self.accumulated_lap_distance_m,
            "lap_completion_times_sec": self.lap_completion_times_sec,
            "lap_durations_sec": [
                completion - (
                    self.lap_completion_times_sec[index - 1] if index > 0 else 0.0
                )
                for index, completion in enumerate(self.lap_completion_times_sec)
            ],
            "planner_states": planner_states,
            "planner_modes": planner_modes,
            "chosen_sides": sides,
            "arbitrator_modes": arbitrator_modes,
            "primary_failures": primary_failures,
            "reactive_triggered": arbitrator_modes.get("REACTIVE", 0) > 0,
            "no_safe_path_seen": planner_states.get("NO_SAFE_PATH_CONFIRMED", 0) > 0,
            "minimum_reported_clearance_m": min(clearances) if clearances else None,
            "maximum_planning_margin_blocked_cycles": maximum_counter(
                "planner_active_margin_blocked_cycles"
            ),
            "maximum_physical_blocked_cycles": maximum_counter(
                "planner_active_physical_blocked_cycles"
            ),
            "maximum_no_safe_path_cycles": maximum_counter(
                "planner_no_safe_path_cycles"
            ),
            "speed_by_planner_mode": speed_by_mode,
            "candidate_decisions": sorted(
                {
                    (
                        row["planner_candidate_decision_id"],
                        row["planner_candidate_selected_side"],
                        row["planner_left_candidate_valid"],
                        row["planner_left_candidate_minimum_clearance_m"],
                        row["planner_left_candidate_objective_cost"],
                        row["planner_left_candidate_objective_domain"],
                        row["planner_right_candidate_valid"],
                        row["planner_right_candidate_minimum_clearance_m"],
                        row["planner_right_candidate_objective_cost"],
                        row["planner_right_candidate_objective_domain"],
                    )
                    for row in self.rows
                    if row["planner_candidate_decision_id"] not in ("", "0")
                }
            ),
            "final_pose": self.pose,
        }
        with open(self.summary_path, "w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
            stream.write("\n")
        return summary


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, help="Output CSV path")
    parser.add_argument("--trial", default="trial", help="Trial label")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument(
        "--target-laps",
        type=int,
        default=0,
        help="Stop after this many complete laps; --duration remains the timeout",
    )
    parser.add_argument(
        "--raceline-csv",
        help="Closed raceline x/y CSV used for lap progress and cross-track error",
    )
    parser.add_argument("--reset-x", type=float)
    parser.add_argument("--reset-y", type=float)
    parser.add_argument("--reset-yaw", type=float)
    return parser.parse_args()


def main():
    args = parse_args()
    reset_values = (args.reset_x, args.reset_y, args.reset_yaw)
    if any(value is not None for value in reset_values) and not all(
        value is not None for value in reset_values
    ):
        raise SystemExit("--reset-x, --reset-y, and --reset-yaw must be used together")
    reset_pose = reset_values if all(value is not None for value in reset_values) else None
    if args.target_laps < 0:
        raise SystemExit("--target-laps must be >= 0")
    if args.target_laps > 0 and not args.raceline_csv:
        raise SystemExit("--target-laps requires --raceline-csv")
    rclpy.init()
    node = TrialLogger(
        args.output,
        args.trial,
        args.sample_hz,
        reset_pose,
        args.raceline_csv,
        args.target_laps,
    )
    try:
        node.prepare_trial()
        deadline = time.monotonic() + args.duration
        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and not (
                args.target_laps > 0 and node.completed_laps >= args.target_laps
            )
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        summary = node.finish()
        print(json.dumps(summary, indent=2, sort_keys=True))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
