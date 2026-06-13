#!/usr/bin/env python3

import math
import os
from typing import Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from stable_baselines3 import PPO

import csv
from dataclasses import dataclass
from typing import List


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
    return angle


def get_loop_index(index: int, size: int) -> int:
    return index % size


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
    return best_idx


def compute_cross_track_error(car_x: float, car_y: float, nearest_point: CenterlinePoint) -> float:
    dx = car_x - nearest_point.x
    dy = car_y - nearest_point.y
    left_normal_x = -math.sin(nearest_point.yaw)
    left_normal_y = math.cos(nearest_point.yaw)
    return dx * left_normal_x + dy * left_normal_y


def compute_heading_error(car_yaw: float, nearest_point: CenterlinePoint) -> float:
    return wrap_angle(car_yaw - nearest_point.yaw)


def get_upcoming_curvature_abs(nearest_idx: int, centerline: List[CenterlinePoint], lookahead_points: int = 20) -> float:
    n = len(centerline)
    max_curv = 0.0
    for offset in range(lookahead_points + 1):
        idx = get_loop_index(nearest_idx + offset, n)
        max_curv = max(max_curv, centerline[idx].curvature_abs)
    return float(max_curv)


def get_centerline_state_features(car_x: float, car_y: float, car_yaw: float, car_speed: float,
                                  centerline: List[CenterlinePoint], curvature_lookahead_points: int = 20):
    nearest_idx = find_nearest_centerline_index(car_x, car_y, centerline)
    nearest_point = centerline[nearest_idx]
    cross_track_error = compute_cross_track_error(car_x, car_y, nearest_point)
    heading_error = compute_heading_error(car_yaw, nearest_point)
    upcoming_curvature_abs = get_upcoming_curvature_abs(
        nearest_idx, centerline, lookahead_points=curvature_lookahead_points)
    features = [float(car_speed), float(cross_track_error), float(heading_error), float(upcoming_curvature_abs)]
    return features, nearest_idx


def curvature_based_speed(upcoming_curvature_abs: float, min_speed: float = 0.5,
                          max_speed: float = 4.0, curvature_gain: float = 2.0) -> float:
    raw_speed = max_speed / (1.0 + curvature_gain * upcoming_curvature_abs)
    return float(np.clip(raw_speed, min_speed, max_speed))


def yaw_from_quaternion(q) -> float:
    """Convert quaternion to yaw angle."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PPOSpeedNode(Node):
    """
    ROS 2 Foxy inference node for the residual PPO speed policy.

    This node mirrors the current rl_training F110SpeedEnv speed interface:
      observation = [
          current_speed_mps,
          rule_speed_mps,
          cross_track_error,
          heading_error,
          curv_short_abs,
          curv_mid_abs,
          curv_long_abs,
          previous_delta_speed_mps,
      ]

      model action in [-1, 1]
      raw_delta_speed_mps = max_delta_speed_mps * action
      published_residual_mps = gate_scale * raw_delta_speed_mps

    The node publishes only the gated residual. The path follower owns final speed
    composition: rule_speed + fresh_residual, or rule_speed only if residual is stale.
    The node intentionally does not implement fallback arbitration or health checks.
    The optional residual gate is the same operating-domain gate used by training.
    """

    def __init__(self):
        super().__init__("ppo_speed_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("centerline_csv", "")
        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("speed_residual_topic", "/rl_speed_inference/speed_residual_mps")

        self.declare_parameter("speed_min", 1.0)
        self.declare_parameter("speed_max", 10.0)
        self.declare_parameter("rule_min_speed_mps", 1.0)
        self.declare_parameter("rule_max_speed_mps", 6.0)
        self.declare_parameter("max_delta_speed_mps", 4.0)
        self.declare_parameter("curvature_gain", 2.0)
        self.declare_parameter("rule_curvature_lookahead_points", 3)
        self.declare_parameter("model_curvature_short_points", 10)
        self.declare_parameter("model_curvature_mid_points", 40)
        self.declare_parameter("model_curvature_long_points", 80)

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

        self.speed_min = float(self.get_parameter("speed_min").value)
        self.speed_max = float(self.get_parameter("speed_max").value)
        self.rule_min_speed_mps = float(self.get_parameter("rule_min_speed_mps").value)
        self.rule_max_speed_mps = float(self.get_parameter("rule_max_speed_mps").value)
        self.max_delta_speed_mps = float(self.get_parameter("max_delta_speed_mps").value)
        self.curvature_gain = float(self.get_parameter("curvature_gain").value)
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

        if self.speed_max <= self.speed_min:
            raise ValueError(f"speed_max must be larger than speed_min. Got {self.speed_min}, {self.speed_max}")

        self.rule_min_speed_mps = float(np.clip(self.rule_min_speed_mps, self.speed_min, self.speed_max))
        self.rule_max_speed_mps = float(np.clip(self.rule_max_speed_mps, self.rule_min_speed_mps, self.speed_max))
        self.max_delta_speed_mps = max(0.0, self.max_delta_speed_mps)

        if not self.model_path or not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model path does not exist: {self.model_path}")
        if not self.centerline_csv or not os.path.exists(self.centerline_csv):
            raise FileNotFoundError(f"Raceline CSV path does not exist: {self.centerline_csv}")

        self.get_logger().info(f"Loading PPO model: {self.model_path}")
        self.model = PPO.load(self.model_path)

        self.get_logger().info(f"Loading raceline CSV: {self.centerline_csv}")
        self.centerline = load_centerline_csv(self.centerline_csv)

        self.previous_delta_speed_mps = 0.0
        self._reset_rl_gate_state()

        self.speed_residual_pub = self.create_publisher(Float64, self.speed_residual_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.get_logger().info("PPO residual speed inference node started.")
        self.get_logger().info(f"  odom_topic: {self.odom_topic}")
        self.get_logger().info(f"  speed_residual_topic: {self.speed_residual_topic}")
        self.get_logger().info(f"  speed range: [{self.speed_min:.2f}, {self.speed_max:.2f}] m/s")
        self.get_logger().info(f"  rule speed range: [{self.rule_min_speed_mps:.2f}, {self.rule_max_speed_mps:.2f}] m/s")
        self.get_logger().info(f"  max_delta_speed_mps: {self.max_delta_speed_mps:.2f}")
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
        raw_delta_speed_mps = self.max_delta_speed_mps * correction_action

        gate_enabled, gate_scale = self._update_rl_gate(
            cross_track_error=cross_track_error,
            heading_error=heading_error,
        )

        delta_speed_mps = gate_scale * raw_delta_speed_mps
        out = Float64()
        out.data = float(delta_speed_mps)
        self.speed_residual_pub.publish(out)

        self.previous_delta_speed_mps = float(delta_speed_mps)

        self.get_logger().debug(
            "idx=%d obs=%s action=%.3f rule=%.3f residual=%.3f gate=%s/%.2f" % (
                nearest_idx,
                np.array2string(obs, precision=3),
                correction_action,
                rule_speed_mps,
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
        return car_x, car_y, car_yaw, car_speed

    def _build_observation(self, car_x: float, car_y: float, car_yaw: float, car_speed: float):
        features, nearest_idx = get_centerline_state_features(
            car_x=car_x,
            car_y=car_y,
            car_yaw=car_yaw,
            car_speed=car_speed,
            centerline=self.centerline,
            curvature_lookahead_points=self.rule_curvature_lookahead_points,
        )

        current_speed_mps = float(features[0])
        cross_track_error = float(features[1])
        heading_error = float(features[2])
        rule_curvature_abs = float(features[3])

        curv_short_abs = get_upcoming_curvature_abs(nearest_idx, self.centerline, self.model_curvature_short_points)
        curv_mid_abs = get_upcoming_curvature_abs(nearest_idx, self.centerline, self.model_curvature_mid_points)
        curv_long_abs = get_upcoming_curvature_abs(nearest_idx, self.centerline, self.model_curvature_long_points)

        rule_speed_mps = curvature_based_speed(
            upcoming_curvature_abs=rule_curvature_abs,
            min_speed=self.rule_min_speed_mps,
            max_speed=self.rule_max_speed_mps,
            curvature_gain=self.curvature_gain,
        )

        obs = np.array([
            current_speed_mps,
            float(rule_speed_mps),
            cross_track_error,
            heading_error,
            float(curv_short_abs),
            float(curv_mid_abs),
            float(curv_long_abs),
            float(self.previous_delta_speed_mps),
        ], dtype=np.float32)

        return obs, int(nearest_idx), float(rule_speed_mps), cross_track_error, heading_error

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
