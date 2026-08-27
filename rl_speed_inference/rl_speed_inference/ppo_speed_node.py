#!/usr/bin/env python3
"""ROS 2 inference node for RoboRacer PPO speed residual policies.

Two model interfaces are supported:

1. legacy physical_mps
   observation = [current_speed_mps, rule_speed_mps, cte, heading,
                  curv_short, curv_mid, curv_long, previous_delta_mps]
   action -> residual_mps = action * max_delta_speed_mps

2. speed_ratio, recommended for highspeed-train -> reserved-apply workflow
   observation = [current_speed_ratio, rule_speed_ratio, cte, heading,
                  curv_short, curv_mid, curv_long, previous_assist_ratio]
   action -> assist_ratio -> residual_mps = assist_ratio * rule_speed_mps

The node always publishes std_msgs/Float64 residual speed in m/s. The path
follower composes final speed as rule_speed + fresh residual, or falls back to
rule speed if the residual is stale/unavailable.
"""

import csv
import math
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from stable_baselines3 import PPO
from std_msgs.msg import Float64


@dataclass
class CenterlinePoint:
    index: int
    x: float
    y: float
    yaw: float
    curvature: float
    curvature_abs: float


def load_centerline_csv(csv_path: str) -> List[CenterlinePoint]:
    points: List[CenterlinePoint] = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        required_columns = ["index", "x", "y", "yaw", "curvature", "curvature_abs"]
        for col in required_columns:
            if col not in reader.fieldnames:
                raise ValueError(f"Missing required column '{col}'. Found columns: {reader.fieldnames}")
        for row in reader:
            points.append(CenterlinePoint(
                index=int(row["index"]),
                x=float(row["x"]),
                y=float(row["y"]),
                yaw=float(row["yaw"]),
                curvature=float(row["curvature"]),
                curvature_abs=float(row["curvature_abs"]),
            ))
    if len(points) < 10:
        raise ValueError(f"Raceline too short: only {len(points)} points loaded.")
    return points


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return float(angle)


def get_loop_index(index: int, size: int) -> int:
    return int(index) % int(size)


def find_nearest_centerline_index(x: float, y: float, centerline: List[CenterlinePoint]) -> int:
    best_idx = 0
    best_dist_sq = float("inf")
    for i, p in enumerate(centerline):
        dx = x - p.x
        dy = y - p.y
        dist_sq = dx * dx + dy * dy
        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best_idx = i
    return int(best_idx)


def compute_cross_track_error(car_x: float, car_y: float, nearest_point: CenterlinePoint) -> float:
    dx = car_x - nearest_point.x
    dy = car_y - nearest_point.y
    left_normal_x = -math.sin(nearest_point.yaw)
    left_normal_y = math.cos(nearest_point.yaw)
    return float(dx * left_normal_x + dy * left_normal_y)


def compute_heading_error(car_yaw: float, nearest_point: CenterlinePoint) -> float:
    return wrap_angle(car_yaw - nearest_point.yaw)


def _segment_distance(centerline: List[CenterlinePoint], from_idx: int, to_idx: int) -> float:
    n = len(centerline)
    p0 = centerline[get_loop_index(from_idx, n)]
    p1 = centerline[get_loop_index(to_idx, n)]
    return float(math.hypot(p1.x - p0.x, p1.y - p0.y))


def get_preview_end_offset_by_distance(nearest_idx: int, centerline: List[CenterlinePoint], preview_distance_m: float) -> int:
    n = len(centerline)
    if n <= 1 or preview_distance_m <= 0.0:
        return 0

    preview_distance_m = float(preview_distance_m)
    accumulated = 0.0
    end_offset = 0
    for offset in range(1, n):
        previous_idx = get_loop_index(nearest_idx + offset - 1, n)
        idx = get_loop_index(nearest_idx + offset, n)
        next_accumulated = accumulated + _segment_distance(centerline, previous_idx, idx)
        if next_accumulated >= preview_distance_m:
            if abs(next_accumulated - preview_distance_m) < abs(preview_distance_m - accumulated):
                end_offset = offset
            break
        accumulated = next_accumulated
        end_offset = offset
    return int(end_offset)


def get_upcoming_curvature_abs(nearest_idx: int, centerline: List[CenterlinePoint], lookahead_points: int = 20) -> float:
    n = len(centerline)
    max_curv = 0.0
    for offset in range(max(0, int(lookahead_points)) + 1):
        idx = get_loop_index(nearest_idx + offset, n)
        max_curv = max(max_curv, max(0.0, float(centerline[idx].curvature_abs)))
    return float(max_curv)


def get_upcoming_curvature_abs_by_distance(nearest_idx: int, centerline: List[CenterlinePoint], preview_distance_m: float) -> float:
    n = len(centerline)
    end_offset = get_preview_end_offset_by_distance(nearest_idx, centerline, preview_distance_m)
    max_curv = 0.0
    for offset in range(end_offset + 1):
        idx = get_loop_index(nearest_idx + offset, n)
        max_curv = max(max_curv, max(0.0, float(centerline[idx].curvature_abs)))
    return float(max_curv)


def curvature_based_speed(upcoming_curvature_abs: float, min_speed: float, max_speed: float, curvature_gain: float) -> float:
    raw_speed = max_speed / (1.0 + curvature_gain * max(0.0, float(upcoming_curvature_abs)))
    return float(np.clip(raw_speed, min_speed, max_speed))


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(math.atan2(siny_cosp, cosy_cosp))


def get_optional_float_parameter(node: Node, name: str) -> Optional[float]:
    """Return a non-negative float parameter, or None when disabled.

    ROS parameters do not have a portable YAML null type across all usage, so
    -1.0 is used as the explicit legacy/disabled value for metre previews.
    """
    value = node.get_parameter(name).value
    if value is None:
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return None if out < 0.0 else out


class PPOSpeedNode(Node):
    """PPO speed residual inference node.

    The node is intentionally narrow: it only publishes a speed residual. It does
    not own path-following, obstacle handling, arbitration, or emergency stop.
    """

    def __init__(self):
        super().__init__("ppo_speed_node")

        # Core paths/topics.
        self.declare_parameter("model_path", "")
        self.declare_parameter("centerline_csv", "")
        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("speed_residual_topic", "/rl_speed_inference/speed_residual_mps")

        # Model interface.
        self.declare_parameter("residual_output_mode", "physical_mps")
        self.declare_parameter("positive_assist_ratio", 0.667)
        self.declare_parameter("negative_assist_ratio", 0.50)
        self.declare_parameter("assist_gain", 1.0)
        self.declare_parameter("max_delta_speed_mps", 4.0)

        # Speed profile. New path_following_v2-aligned names are preferred;
        # legacy aliases remain available.
        self.declare_parameter("command_speed_min_mps", 1.0)
        self.declare_parameter("command_speed_max_mps", 10.0)
        self.declare_parameter("speed_min", 1.0)
        self.declare_parameter("speed_max", 10.0)
        self.declare_parameter("rule_curve_min_speed_mps", 1.0)
        self.declare_parameter("rule_straight_speed_mps", 6.0)
        self.declare_parameter("rule_min_speed_mps", 1.0)
        self.declare_parameter("rule_max_speed_mps", 6.0)
        self.declare_parameter("rule_speed_curvature_gain", 2.0)
        self.declare_parameter("curvature_gain", 2.0)

        # Curvature previews. Meter-based values match the newest rl_training and
        # path_following_v2 style. Point values are legacy fallback.
        self.declare_parameter("rule_speed_curvature_preview_m", -1.0)
        self.declare_parameter("model_curvature_short_preview_m", -1.0)
        self.declare_parameter("model_curvature_mid_preview_m", -1.0)
        self.declare_parameter("model_curvature_long_preview_m", -1.0)
        self.declare_parameter("rule_curvature_lookahead_points", 3)
        self.declare_parameter("model_curvature_short_points", 10)
        self.declare_parameter("model_curvature_mid_points", 40)
        self.declare_parameter("model_curvature_long_points", 80)

        # Runtime residual gate.
        self.declare_parameter("enable_rl_gate", True)
        self.declare_parameter("rl_gate_enable_cte", 0.25)
        self.declare_parameter("rl_gate_enable_heading", 0.20)
        self.declare_parameter("rl_gate_disable_cte", 0.45)
        self.declare_parameter("rl_gate_disable_heading", 0.35)
        self.declare_parameter("rl_gate_enable_count", 10)
        self.declare_parameter("rl_gate_disable_count", 3)
        self.declare_parameter("rl_gate_fade_in_step", 0.05)
        self.declare_parameter("rl_gate_fade_out_step", 0.10)

        self.model_path = str(self.get_parameter("model_path").value)
        self.centerline_csv = str(self.get_parameter("centerline_csv").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.speed_residual_topic = str(self.get_parameter("speed_residual_topic").value)

        self.residual_output_mode = str(self.get_parameter("residual_output_mode").value).strip().lower()
        if self.residual_output_mode not in {"physical_mps", "speed_ratio"}:
            raise ValueError("residual_output_mode must be 'physical_mps' or 'speed_ratio'")

        self.min_speed = float(self.get_parameter("command_speed_min_mps").value)
        self.max_speed = float(self.get_parameter("command_speed_max_mps").value)
        # Fallback for older configs that only set speed_min/speed_max.
        if self.min_speed is None:
            self.min_speed = float(self.get_parameter("speed_min").value)
        if self.max_speed is None:
            self.max_speed = float(self.get_parameter("speed_max").value)

        self.rule_min_speed_mps = float(self.get_parameter("rule_curve_min_speed_mps").value)
        self.rule_max_speed_mps = float(self.get_parameter("rule_straight_speed_mps").value)
        self.curvature_gain = float(self.get_parameter("rule_speed_curvature_gain").value)
        self.max_delta_speed_mps = max(0.0, float(self.get_parameter("max_delta_speed_mps").value))
        self.positive_assist_ratio = max(0.0, float(self.get_parameter("positive_assist_ratio").value))
        self.negative_assist_ratio = max(0.0, float(self.get_parameter("negative_assist_ratio").value))
        self.assist_gain = max(0.0, float(self.get_parameter("assist_gain").value))

        self.rule_speed_curvature_preview_m = get_optional_float_parameter(self, "rule_speed_curvature_preview_m")
        self.model_curvature_short_preview_m = get_optional_float_parameter(self, "model_curvature_short_preview_m")
        self.model_curvature_mid_preview_m = get_optional_float_parameter(self, "model_curvature_mid_preview_m")
        self.model_curvature_long_preview_m = get_optional_float_parameter(self, "model_curvature_long_preview_m")
        self.rule_curvature_lookahead_points = int(self.get_parameter("rule_curvature_lookahead_points").value)
        self.model_curvature_short_points = int(self.get_parameter("model_curvature_short_points").value)
        self.model_curvature_mid_points = int(self.get_parameter("model_curvature_mid_points").value)
        self.model_curvature_long_points = int(self.get_parameter("model_curvature_long_points").value)

        self.enable_rl_gate = bool(self.get_parameter("enable_rl_gate").value)
        self.rl_gate_enable_cte = abs(float(self.get_parameter("rl_gate_enable_cte").value))
        self.rl_gate_enable_heading = abs(float(self.get_parameter("rl_gate_enable_heading").value))
        self.rl_gate_disable_cte = abs(float(self.get_parameter("rl_gate_disable_cte").value))
        self.rl_gate_disable_heading = abs(float(self.get_parameter("rl_gate_disable_heading").value))
        self.rl_gate_enable_count = max(1, int(self.get_parameter("rl_gate_enable_count").value))
        self.rl_gate_disable_count = max(1, int(self.get_parameter("rl_gate_disable_count").value))
        self.rl_gate_fade_in_step = float(np.clip(float(self.get_parameter("rl_gate_fade_in_step").value), 0.0, 1.0))
        self.rl_gate_fade_out_step = float(np.clip(float(self.get_parameter("rl_gate_fade_out_step").value), 0.0, 1.0))

        if self.max_speed <= self.min_speed:
            raise ValueError(f"command_speed_max_mps must be larger than command_speed_min_mps. Got {self.min_speed}, {self.max_speed}")
        self.rule_min_speed_mps = float(np.clip(self.rule_min_speed_mps, self.min_speed, self.max_speed))
        self.rule_max_speed_mps = float(np.clip(self.rule_max_speed_mps, self.rule_min_speed_mps, self.max_speed))

        if not self.model_path or not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model path does not exist: {self.model_path}")
        if not self.centerline_csv or not os.path.exists(self.centerline_csv):
            raise FileNotFoundError(f"Raceline CSV path does not exist: {self.centerline_csv}")

        self.get_logger().info(f"Loading PPO model: {self.model_path}")
        self.model = PPO.load(self.model_path)
        self.get_logger().info(f"Loading raceline CSV: {self.centerline_csv}")
        self.centerline = load_centerline_csv(self.centerline_csv)

        self.previous_delta_speed_mps = 0.0
        self.previous_assist_ratio = 0.0
        self._reset_rl_gate_state()

        self.speed_residual_pub = self.create_publisher(Float64, self.speed_residual_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.get_logger().info("PPO speed inference node started.")
        self.get_logger().info(f"  residual_output_mode: {self.residual_output_mode}")
        self.get_logger().info(f"  odom_topic: {self.odom_topic}")
        self.get_logger().info(f"  speed_residual_topic: {self.speed_residual_topic}")
        self.get_logger().info(f"  command speed range: [{self.min_speed:.2f}, {self.max_speed:.2f}] m/s")
        self.get_logger().info(f"  rule speed range: [{self.rule_min_speed_mps:.2f}, {self.rule_max_speed_mps:.2f}] m/s")
        self.get_logger().info(f"  assist_gain: {self.assist_gain:.3f}")
        self.get_logger().info(f"  enable_rl_gate: {self.enable_rl_gate}")

    def odom_callback(self, msg: Odometry) -> None:
        car_x, car_y, car_yaw, car_speed = self._state_from_odom(msg)
        obs, nearest_idx, rule_speed_mps, cross_track_error, heading_error = self._build_observation(
            car_x=car_x,
            car_y=car_y,
            car_yaw=car_yaw,
            car_speed=car_speed,
        )

        action, _ = self.model.predict(obs, deterministic=True)
        correction_action = float(np.clip(np.asarray(action).reshape(-1)[0], -1.0, 1.0))
        raw_delta_speed_mps, raw_assist_ratio = self._action_to_raw_residual(correction_action, rule_speed_mps)

        gate_enabled, gate_scale = self._update_rl_gate(
            cross_track_error=cross_track_error,
            heading_error=heading_error,
        )

        delta_speed_mps = gate_scale * raw_delta_speed_mps
        assist_ratio = gate_scale * raw_assist_ratio

        out = Float64()
        out.data = float(delta_speed_mps)
        self.speed_residual_pub.publish(out)

        self.previous_delta_speed_mps = float(delta_speed_mps)
        self.previous_assist_ratio = float(assist_ratio)

        self.get_logger().debug(
            "idx=%d obs=%s action=%.3f rule=%.3f assist=%.3f residual=%.3f gate=%s/%.2f" % (
                nearest_idx,
                np.array2string(obs, precision=3),
                correction_action,
                rule_speed_mps,
                assist_ratio,
                delta_speed_mps,
                gate_enabled,
                gate_scale,
            )
        )

    def _state_from_odom(self, msg: Odometry) -> Tuple[float, float, float, float]:
        car_x = float(msg.pose.pose.position.x)
        car_y = float(msg.pose.pose.position.y)
        car_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        car_speed = math.sqrt(vx * vx + vy * vy)
        return car_x, car_y, car_yaw, float(car_speed)

    def _build_observation(self, car_x: float, car_y: float, car_yaw: float, car_speed: float):
        nearest_idx = find_nearest_centerline_index(car_x, car_y, self.centerline)
        nearest_point = self.centerline[nearest_idx]
        cross_track_error = compute_cross_track_error(car_x, car_y, nearest_point)
        heading_error = compute_heading_error(car_yaw, nearest_point)

        rule_curvature_abs = self._get_preview_curvature_abs(
            nearest_idx, self.rule_speed_curvature_preview_m, self.rule_curvature_lookahead_points
        )
        curv_short_abs = self._get_preview_curvature_abs(
            nearest_idx, self.model_curvature_short_preview_m, self.model_curvature_short_points
        )
        curv_mid_abs = self._get_preview_curvature_abs(
            nearest_idx, self.model_curvature_mid_preview_m, self.model_curvature_mid_points
        )
        curv_long_abs = self._get_preview_curvature_abs(
            nearest_idx, self.model_curvature_long_preview_m, self.model_curvature_long_points
        )

        rule_speed_mps = curvature_based_speed(
            upcoming_curvature_abs=rule_curvature_abs,
            min_speed=self.rule_min_speed_mps,
            max_speed=self.rule_max_speed_mps,
            curvature_gain=self.curvature_gain,
        )

        if self.residual_output_mode == "speed_ratio":
            obs = np.array([
                self._normalize_command_speed(car_speed),
                self._normalize_rule_speed(rule_speed_mps),
                float(cross_track_error),
                float(heading_error),
                float(curv_short_abs),
                float(curv_mid_abs),
                float(curv_long_abs),
                float(self.previous_assist_ratio),
            ], dtype=np.float32)
        else:
            obs = np.array([
                float(car_speed),
                float(rule_speed_mps),
                float(cross_track_error),
                float(heading_error),
                float(curv_short_abs),
                float(curv_mid_abs),
                float(curv_long_abs),
                float(self.previous_delta_speed_mps),
            ], dtype=np.float32)

        return obs, int(nearest_idx), float(rule_speed_mps), float(cross_track_error), float(heading_error)

    def _action_to_raw_residual(self, correction_action: float, rule_speed_mps: float) -> Tuple[float, float]:
        action = float(np.clip(correction_action, -1.0, 1.0))
        rule_speed = max(float(rule_speed_mps), 1e-6)
        if self.residual_output_mode == "speed_ratio":
            if action >= 0.0:
                raw_assist_ratio = action * self.positive_assist_ratio * self.assist_gain
            else:
                raw_assist_ratio = action * self.negative_assist_ratio * self.assist_gain
            return float(raw_assist_ratio * rule_speed), float(raw_assist_ratio)
        raw_delta = self.max_delta_speed_mps * action
        return float(raw_delta), float(raw_delta / rule_speed)

    def _get_preview_curvature_abs(self, nearest_idx: int, preview_m: Optional[float], fallback_points: int) -> float:
        if preview_m is not None and preview_m >= 0.0:
            return get_upcoming_curvature_abs_by_distance(nearest_idx, self.centerline, float(preview_m))
        return get_upcoming_curvature_abs(nearest_idx, self.centerline, int(fallback_points))

    def _normalize_command_speed(self, speed_mps: float) -> float:
        denom = max(self.max_speed - self.min_speed, 1e-6)
        return float(np.clip((float(speed_mps) - self.min_speed) / denom, -0.5, 1.5))

    def _normalize_rule_speed(self, rule_speed_mps: float) -> float:
        denom = max(self.rule_max_speed_mps - self.rule_min_speed_mps, 1e-6)
        return float(np.clip((float(rule_speed_mps) - self.rule_min_speed_mps) / denom, -0.5, 1.5))

    def _reset_rl_gate_state(self) -> None:
        if self.enable_rl_gate:
            self.rl_gate_enabled = False
            self.rl_gate_scale = 0.0
        else:
            self.rl_gate_enabled = True
            self.rl_gate_scale = 1.0
        self.rl_gate_good_count = 0
        self.rl_gate_bad_count = 0

    def _update_rl_gate(self, cross_track_error: float, heading_error: float):
        if not self.enable_rl_gate:
            self.rl_gate_enabled = True
            self.rl_gate_scale = 1.0
            self.rl_gate_good_count = 0
            self.rl_gate_bad_count = 0
            return self.rl_gate_enabled, self.rl_gate_scale

        abs_cte = abs(float(cross_track_error))
        abs_heading = abs(float(heading_error))
        good = abs_cte < self.rl_gate_enable_cte and abs_heading < self.rl_gate_enable_heading
        bad = abs_cte > self.rl_gate_disable_cte or abs_heading > self.rl_gate_disable_heading

        self.rl_gate_good_count = self.rl_gate_good_count + 1 if good else 0
        self.rl_gate_bad_count = self.rl_gate_bad_count + 1 if bad else 0

        if (not self.rl_gate_enabled) and self.rl_gate_good_count >= self.rl_gate_enable_count:
            self.rl_gate_enabled = True
        if self.rl_gate_enabled and self.rl_gate_bad_count >= self.rl_gate_disable_count:
            self.rl_gate_enabled = False

        if self.rl_gate_enabled:
            self.rl_gate_scale = min(1.0, self.rl_gate_scale + self.rl_gate_fade_in_step)
        else:
            self.rl_gate_scale = max(0.0, self.rl_gate_scale - self.rl_gate_fade_out_step)

        return self.rl_gate_enabled, self.rl_gate_scale


def main(args=None):
    rclpy.init(args=args)
    node = PPOSpeedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
