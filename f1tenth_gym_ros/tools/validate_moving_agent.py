#!/usr/bin/env python3
# Copyright 2026 F1TENTH Development Contributors
# SPDX-License-Identifier: MIT

"""Validate one slow autonomous simulator agent against the ego planner."""

import argparse
import csv
import json
import math
import os
import time
from collections import Counter

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, UInt8


def values_from_status(message, name):
    for status in message.status:
        if status.name == name:
            values = {value.key: value.value for value in status.values}
            values["message"] = status.message
            return values
    return None


def finite_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def quaternion_yaw(orientation):
    sine = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosine = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(sine, cosine)


class MovingAgentValidator(Node):
    def __init__(self, reference_csv):
        super().__init__("moving_agent_validator")
        self.reference, self.cumulative, self.lap_length = self.load_reference(
            reference_csv
        )
        self.ego = None
        self.opponent = None
        self.scan = None
        self.planner = {}
        self.arbitrator = {}
        self.agent = {}
        self.simulator = {}
        self.selected_mode = None
        self.create_subscription(
            Odometry, "/ego_racecar/odom", self.ego_callback, 10
        )
        self.create_subscription(
            Odometry, "/opp_racecar/odom", self.opponent_callback, 10
        )
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(
            DiagnosticArray,
            "/path_following_v2/path_status",
            self.planner_callback,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/drive_arbitration_v2/status",
            self.arbitrator_callback,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/moving_agent/status",
            self.agent_callback,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/simulator/agent_status",
            self.simulator_callback,
            10,
        )
        self.create_subscription(
            UInt8,
            "/drive_arbitration_v2/selected_mode",
            self.mode_callback,
            10,
        )
        self.ego_reset = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.opponent_reset = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.planner_reset = self.create_publisher(
            Bool, "/path_following_v2/reset", 10
        )
        self.arbitrator_reset = self.create_publisher(
            Bool, "/drive_arbitration_v2/reset", 10
        )

    @staticmethod
    def load_reference(path):
        with open(path, "r", newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        if not rows or not {"x", "y"}.issubset(rows[0]):
            raise RuntimeError(f"Reference CSV must contain x and y: {path}")
        points = np.asarray(
            [[float(row["x"]), float(row["y"])] for row in rows],
            dtype=np.float64,
        )
        lengths = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(lengths[:-1])))
        return points, cumulative, float(np.sum(lengths))

    @staticmethod
    def odom_value(message):
        return {
            "x": float(message.pose.pose.position.x),
            "y": float(message.pose.pose.position.y),
            "yaw": quaternion_yaw(message.pose.pose.orientation),
            "speed": math.hypot(
                message.twist.twist.linear.x, message.twist.twist.linear.y
            ),
        }

    def ego_callback(self, message):
        self.ego = self.odom_value(message)

    def opponent_callback(self, message):
        self.opponent = self.odom_value(message)

    def scan_callback(self, message):
        self.scan = message

    def planner_callback(self, message):
        values = values_from_status(message, "path_following_v2/path_generator")
        if values is not None:
            self.planner = values

    def arbitrator_callback(self, message):
        values = values_from_status(message, "drive_arbitration_v2/drive_arbitrator")
        if values is not None:
            self.arbitrator = values

    def agent_callback(self, message):
        values = values_from_status(message, "moving_agent/slow_path_follower")
        if values is not None:
            self.agent = values

    def simulator_callback(self, message):
        ego = values_from_status(message, "simulator/ego")
        opponent = values_from_status(message, "simulator/slow_agent")
        if ego is not None and opponent is not None:
            self.simulator = {"ego": ego, "opponent": opponent}

    def mode_callback(self, message):
        self.selected_mode = int(message.data)

    def publish_resets(self):
        ego_target = (17.0, 4.5)
        opponent_target = (20.810496388313787, 20.67235055941259)
        ego = PoseWithCovarianceStamped()
        ego.header.frame_id = "map"
        ego.pose.pose.position.x = ego_target[0]
        ego.pose.pose.position.y = ego_target[1]
        ego.pose.pose.orientation.z = math.sin(0.5 * 0.7)
        ego.pose.pose.orientation.w = math.cos(0.5 * 0.7)

        opponent = PoseStamped()
        opponent.header.frame_id = "map"
        opponent.pose.position.x = opponent_target[0]
        opponent.pose.position.y = opponent_target[1]
        opponent.pose.orientation.z = math.sin(0.5 * 1.850140460509907)
        opponent.pose.orientation.w = math.cos(0.5 * 1.850140460509907)

        discovery_deadline = time.monotonic() + 2.0
        while time.monotonic() < discovery_deadline and (
            self.ego_reset.get_subscription_count() < 1
            or self.opponent_reset.get_subscription_count() < 1
        ):
            rclpy.spin_once(self, timeout_sec=0.05)
        if (
            self.ego_reset.get_subscription_count() < 1
            or self.opponent_reset.get_subscription_count() < 1
        ):
            raise RuntimeError("simulator reset subscribers were not discovered")

        reset = Bool(data=True)
        reset_deadline = time.monotonic() + 3.0
        reset_confirmed = False
        while time.monotonic() < reset_deadline:
            stamp = self.get_clock().now().to_msg()
            ego.header.stamp = stamp
            opponent.header.stamp = stamp
            self.ego_reset.publish(ego)
            self.opponent_reset.publish(opponent)
            self.planner_reset.publish(reset)
            self.arbitrator_reset.publish(reset)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.ego is None or self.opponent is None:
                continue
            ego_error = math.hypot(
                self.ego["x"] - ego_target[0], self.ego["y"] - ego_target[1]
            )
            opponent_error = math.hypot(
                self.opponent["x"] - opponent_target[0],
                self.opponent["y"] - opponent_target[1],
            )
            if ego_error <= 0.25 and opponent_error <= 0.25:
                reset_confirmed = True
                break
        if not reset_confirmed:
            raise RuntimeError("simulator did not confirm the requested two-agent reset")

    def progress(self, pose):
        position = np.asarray([pose["x"], pose["y"]], dtype=np.float64)
        distances_squared = np.sum((self.reference - position) ** 2, axis=1)
        index = int(np.argmin(distances_squared))
        return float(self.cumulative[index]), math.sqrt(
            float(distances_squared[index])
        )

    def scan_matches_opponent(self):
        if self.scan is None or self.ego is None or self.opponent is None:
            return False, None, None
        laser_x = self.ego["x"] + 0.27 * math.cos(self.ego["yaw"])
        laser_y = self.ego["y"] + 0.27 * math.sin(self.ego["yaw"])
        dx = self.opponent["x"] - laser_x
        dy = self.opponent["y"] - laser_y
        expected_range = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) - self.ego["yaw"]
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        if (
            expected_range > self.scan.range_max
            or bearing < self.scan.angle_min
            or bearing > self.scan.angle_max
        ):
            return False, expected_range, None
        center = int(round((bearing - self.scan.angle_min) / self.scan.angle_increment))
        half_width = max(
            2,
            int(
                math.ceil(
                    math.atan2(0.20, max(expected_range, 0.20))
                    / self.scan.angle_increment
                )
            ),
        )
        start = max(0, center - half_width)
        end = min(len(self.scan.ranges), center + half_width + 1)
        valid = [
            float(value)
            for value in self.scan.ranges[start:end]
            if math.isfinite(value) and value >= self.scan.range_min
        ]
        observed = min(valid) if valid else None
        matched = observed is not None and expected_range - 0.75 <= observed <= expected_range
        return matched, expected_range, observed


def unwrap_progress(current, previous, lap_length):
    delta = current - previous
    if delta < -0.5 * lap_length:
        delta += lap_length
    elif delta > 0.5 * lap_length:
        delta -= lap_length
    return delta if abs(delta) <= 10.0 else 0.0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument(
        "--reference-csv",
        default=(
            "/sim_ws/src/centerline_tools/output_backup/"
            "V0_reward_ppo_speed_spielberg_1000k_20260612/"
            "centerline_points_smooth.csv"
        ),
    )
    parser.add_argument(
        "--output",
        default=(
            "/sim_ws/src/path_following_v2/trial_logs/moving_agent/"
            "spielberg_moving_agent_summary.json"
        ),
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.duration <= 0.0 or args.sample_hz <= 0.0:
        raise SystemExit("duration and sample rate must be positive")
    rclpy.init()
    node = MovingAgentValidator(args.reference_csv)
    try:
        ready_deadline = time.monotonic() + 5.0
        while time.monotonic() < ready_deadline and (
            node.ego is None or node.opponent is None or node.scan is None
        ):
            rclpy.spin_once(node, timeout_sec=0.10)
        if node.ego is None or node.opponent is None or node.scan is None:
            raise RuntimeError("two-agent simulator topics did not become ready")

        node.publish_resets()
        settle_deadline = time.monotonic() + 1.0
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        initial_ego_s, _ = node.progress(node.ego)
        initial_opponent_s, _ = node.progress(node.opponent)
        initial_forward_gap = (
            initial_opponent_s - initial_ego_s
        ) % node.lap_length
        previous_ego_s = initial_ego_s
        previous_opponent_s = initial_opponent_s
        ego_travel = 0.0
        opponent_travel = 0.0
        overtakes = 0
        next_overtake_gain = initial_forward_gap
        last_overtake_ego_travel = None
        minimum_separation = math.inf
        maximum_agent_cross_track = 0.0
        collision_seen = False
        scan_match_samples = 0
        close_agent_samples = 0
        dynamic_response_samples = 0
        planner_states = Counter()
        planner_modes = Counter()
        arbitration_modes = Counter()
        agent_states = Counter()
        samples = 0

        started = time.monotonic()
        next_sample = started
        deadline = started + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.monotonic()
            if now < next_sample or node.ego is None or node.opponent is None:
                continue
            next_sample += 1.0 / args.sample_hz
            ego_s, _ = node.progress(node.ego)
            opponent_s, _ = node.progress(node.opponent)
            ego_travel += unwrap_progress(ego_s, previous_ego_s, node.lap_length)
            opponent_travel += unwrap_progress(
                opponent_s, previous_opponent_s, node.lap_length
            )
            previous_ego_s = ego_s
            previous_opponent_s = opponent_s

            separation = math.hypot(
                node.ego["x"] - node.opponent["x"],
                node.ego["y"] - node.opponent["y"],
            )
            minimum_separation = min(minimum_separation, separation)
            relative_gain = ego_travel - opponent_travel
            while relative_gain >= next_overtake_gain:
                overtakes += 1
                last_overtake_ego_travel = ego_travel
                next_overtake_gain += node.lap_length

            mode = node.planner.get("trajectory_mode", "UNKNOWN")
            planner_state = node.planner.get("state", "UNKNOWN")
            arbitration = node.arbitrator.get("mode", "UNKNOWN")
            agent_state = node.agent.get("state", "UNKNOWN")
            planner_states[planner_state] += 1
            planner_modes[mode] += 1
            arbitration_modes[arbitration] += 1
            agent_states[agent_state] += 1

            cross_track = finite_float(node.agent.get("cross_track_error_m"))
            if cross_track is not None:
                maximum_agent_cross_track = max(
                    maximum_agent_cross_track, cross_track
                )
            if node.simulator:
                collision_seen = collision_seen or (
                    node.simulator["ego"].get("collision") == "True"
                    or node.simulator["opponent"].get("collision") == "True"
                )

            scan_match, expected_range, _ = node.scan_matches_opponent()
            if scan_match:
                scan_match_samples += 1
            signed_gap = (
                opponent_s - ego_s + 0.5 * node.lap_length
            ) % node.lap_length - 0.5 * node.lap_length
            close_agent = (
                -1.5 <= signed_gap <= 8.0
                and separation <= 8.0
                and expected_range is not None
                and expected_range <= 8.0
                and scan_match
            )
            if close_agent:
                close_agent_samples += 1
                if (
                    mode.startswith("AVOIDANCE")
                    or mode in ("FOLLOWING_OBSTACLE", "REPLAN_PENDING")
                    or arbitration == "REACTIVE"
                ):
                    dynamic_response_samples += 1
            samples += 1

        post_overtake_distance = (
            0.0
            if last_overtake_ego_travel is None
            else ego_travel - last_overtake_ego_travel
        )
        checks = {
            "opponent_moved": opponent_travel >= 25.0,
            "ego_moved": ego_travel >= 25.0,
            "no_collision": not collision_seen,
            "agent_path_tracking": maximum_agent_cross_track <= 0.50,
            "opponent_visible_in_ego_scan": scan_match_samples >= 3,
            "close_dynamic_encounter": close_agent_samples >= 3,
            "avoidance_response_near_agent": dynamic_response_samples >= 3,
            "successful_overtake": overtakes >= 1,
            "ego_continued_after_overtake": post_overtake_distance >= 5.0,
            "no_primary_planner_failure": not any(
                planner_states[state] > 0
                for state in ("CRITICAL_OBSTACLE", "NO_SAFE_PATH_CONFIRMED")
            ),
            "no_reactive_handoff": arbitration_modes["REACTIVE"] == 0,
        }
        summary = {
            "duration_sec": args.duration,
            "samples": samples,
            "lap_length_m": node.lap_length,
            "initial_forward_gap_m": initial_forward_gap,
            "ego_travel_m": ego_travel,
            "opponent_travel_m": opponent_travel,
            "overtakes": overtakes,
            "post_overtake_ego_distance_m": post_overtake_distance,
            "minimum_agent_separation_m": minimum_separation,
            "maximum_opponent_cross_track_error_m": maximum_agent_cross_track,
            "opponent_scan_match_samples": scan_match_samples,
            "close_dynamic_encounter_samples": close_agent_samples,
            "dynamic_response_samples": dynamic_response_samples,
            "planner_states": dict(planner_states),
            "planner_modes": dict(planner_modes),
            "arbitration_modes": dict(arbitration_modes),
            "opponent_states": dict(agent_states),
            "collision_seen": collision_seen,
            "checks": checks,
            "passed": all(checks.values()),
        }
        os.makedirs(os.path.dirname(args.output), exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
            stream.write("\n")
        print(json.dumps(summary, indent=2, sort_keys=True))
        if not summary["passed"]:
            raise SystemExit("moving-agent validation failed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
