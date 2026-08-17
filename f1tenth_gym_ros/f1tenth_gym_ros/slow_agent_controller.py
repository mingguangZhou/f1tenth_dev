#!/usr/bin/env python3
# Copyright 2026 F1TENTH Development Contributors
# SPDX-License-Identifier: MIT

"""Follow a validated closed path as a slower autonomous simulator agent."""

import csv
import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def angle_difference(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


class SlowAgentController(Node):
    def __init__(self):
        super().__init__("slow_agent_controller")
        self.declare_parameter("path_csv", "")
        self.declare_parameter("odom_topic", "/opp_racecar/odom")
        self.declare_parameter("scan_topic", "/opp_scan")
        self.declare_parameter("drive_topic", "/opp_drive")
        self.declare_parameter("path_topic", "/moving_agent/reference_path")
        self.declare_parameter("status_topic", "/moving_agent/status")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("lookahead_m", 1.20)
        self.declare_parameter("wheelbase_m", 0.33)
        self.declare_parameter("maximum_speed_mps", 1.40)
        self.declare_parameter("minimum_speed_mps", 0.75)
        self.declare_parameter("maximum_steering_rad", 0.36)
        self.declare_parameter("front_slow_distance_m", 1.20)
        self.declare_parameter("front_stop_distance_m", 0.55)
        self.declare_parameter("front_cone_half_angle_deg", 18.0)
        self.declare_parameter("input_timeout_sec", 0.30)
        self.declare_parameter("maximum_cross_track_error_m", 1.00)

        path_csv = self.get_parameter("path_csv").value
        if not path_csv:
            raise RuntimeError("path_csv must name a validated closed path.")
        self.path, self.path_yaw = self.load_path(path_csv)
        self.segment_lengths = np.linalg.norm(
            np.roll(self.path, -1, axis=0) - self.path, axis=1
        )
        self.lookahead_m = max(0.20, float(self.get_parameter("lookahead_m").value))
        self.wheelbase_m = max(0.05, float(self.get_parameter("wheelbase_m").value))
        self.maximum_speed_mps = max(
            0.0, float(self.get_parameter("maximum_speed_mps").value)
        )
        self.minimum_speed_mps = min(
            self.maximum_speed_mps,
            max(0.0, float(self.get_parameter("minimum_speed_mps").value)),
        )
        self.maximum_steering_rad = max(
            0.01, abs(float(self.get_parameter("maximum_steering_rad").value))
        )
        self.front_slow_distance_m = max(
            0.10, float(self.get_parameter("front_slow_distance_m").value)
        )
        self.front_stop_distance_m = max(
            0.0, float(self.get_parameter("front_stop_distance_m").value)
        )
        if self.front_slow_distance_m <= self.front_stop_distance_m:
            raise RuntimeError("front_slow_distance_m must exceed front_stop_distance_m.")
        self.front_cone_half_angle_rad = math.radians(
            max(1.0, float(self.get_parameter("front_cone_half_angle_deg").value))
        )
        self.input_timeout = Duration(
            seconds=max(0.05, float(self.get_parameter("input_timeout_sec").value))
        )
        self.maximum_cross_track_error_m = max(
            0.10, float(self.get_parameter("maximum_cross_track_error_m").value)
        )

        self.pose = None
        self.pose_time = None
        self.front_distance_m = math.inf
        self.scan_time = None
        self.progress_index = None
        self.cross_track_error_m = math.inf
        self.last_command = (0.0, 0.0)
        self.state = "WAITING"
        self.reason = "waiting for odometry and scan"

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, self.get_parameter("drive_topic").value, 10
        )
        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_publisher = self.create_publisher(
            Path, self.get_parameter("path_topic").value, path_qos
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray, self.get_parameter("status_topic").value, 10
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.odom_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.scan_callback,
            10,
        )
        control_rate_hz = max(
            1.0, float(self.get_parameter("control_rate_hz").value)
        )
        self.create_timer(1.0 / control_rate_hz, self.control_callback)
        self.create_timer(1.0, self.publish_reference_path)
        self.publish_reference_path()
        self.get_logger().info(
            f"slow agent ready: points={len(self.path)} "
            f"speed={self.maximum_speed_mps:.2f} m/s lookahead={self.lookahead_m:.2f} m"
        )

    @staticmethod
    def load_path(path):
        with open(path, "r", newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        required = {"x", "y", "yaw"}
        if len(rows) < 3 or not required.issubset(rows[0]):
            raise RuntimeError(f"Path CSV must contain {sorted(required)}: {path}")
        points = np.asarray(
            [[float(row["x"]), float(row["y"])] for row in rows],
            dtype=np.float64,
        )
        yaw = np.asarray([float(row["yaw"]) for row in rows], dtype=np.float64)
        if not np.all(np.isfinite(points)) or not np.all(np.isfinite(yaw)):
            raise RuntimeError(f"Path CSV contains non-finite values: {path}")
        return points, yaw

    def odom_callback(self, message):
        orientation = message.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        self.pose = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            math.atan2(sin_yaw, cos_yaw),
        )
        self.pose_time = self.get_clock().now()

    def scan_callback(self, message):
        front_ranges = []
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if (
                abs(angle) <= self.front_cone_half_angle_rad
                and math.isfinite(distance)
                and distance >= message.range_min
            ):
                front_ranges.append(float(distance))
        self.front_distance_m = min(front_ranges) if front_ranges else math.inf
        self.scan_time = self.get_clock().now()

    def nearest_path_index(self, position):
        if self.progress_index is None:
            candidates = np.arange(len(self.path), dtype=np.int64)
        else:
            offsets = np.arange(-25, 251, dtype=np.int64)
            candidates = (self.progress_index + offsets) % len(self.path)
        differences = self.path[candidates] - position
        distances_squared = np.sum(differences * differences, axis=1)
        selected = int(candidates[int(np.argmin(distances_squared))])
        distance = math.sqrt(float(np.min(distances_squared)))
        if distance > 2.0 and self.progress_index is not None:
            differences = self.path - position
            distances_squared = np.sum(differences * differences, axis=1)
            selected = int(np.argmin(distances_squared))
            distance = math.sqrt(float(distances_squared[selected]))
        return selected, distance

    def target_index(self, start_index):
        distance = 0.0
        index = start_index
        for _ in range(len(self.path)):
            if distance >= self.lookahead_m:
                break
            distance += float(self.segment_lengths[index])
            index = (index + 1) % len(self.path)
        return index

    def speed_for_steering(self, steering):
        ratio = min(1.0, abs(steering) / self.maximum_steering_rad)
        return self.maximum_speed_mps - ratio * (
            self.maximum_speed_mps - self.minimum_speed_mps
        )

    def obstacle_speed_scale(self):
        if self.front_distance_m <= self.front_stop_distance_m:
            return 0.0
        if self.front_distance_m >= self.front_slow_distance_m:
            return 1.0
        return (self.front_distance_m - self.front_stop_distance_m) / (
            self.front_slow_distance_m - self.front_stop_distance_m
        )

    def publish_stop(self, state, reason):
        self.state = state
        self.reason = reason
        self.publish_command(0.0, 0.0)
        self.publish_status()

    def control_callback(self):
        now = self.get_clock().now()
        if self.pose is None or self.pose_time is None:
            self.publish_stop("WAITING", "odometry has not arrived")
            return
        if self.scan_time is None:
            self.publish_stop("WAITING", "scan has not arrived")
            return
        if now - self.pose_time > self.input_timeout:
            self.publish_stop("INPUT_STALE", "odometry is stale")
            return
        if now - self.scan_time > self.input_timeout:
            self.publish_stop("INPUT_STALE", "scan is stale")
            return

        x, y, vehicle_yaw = self.pose
        position = np.asarray([x, y], dtype=np.float64)
        nearest, cross_track_error = self.nearest_path_index(position)
        self.progress_index = nearest
        self.cross_track_error_m = cross_track_error
        if cross_track_error > self.maximum_cross_track_error_m:
            self.publish_stop(
                "PATH_DEVIATION",
                f"cross-track error {cross_track_error:.2f} m exceeds limit",
            )
            return

        target = self.path[self.target_index(nearest)]
        dx = float(target[0] - x)
        dy = float(target[1] - y)
        local_x = math.cos(vehicle_yaw) * dx + math.sin(vehicle_yaw) * dy
        local_y = -math.sin(vehicle_yaw) * dx + math.cos(vehicle_yaw) * dy
        target_distance_squared = local_x * local_x + local_y * local_y
        if local_x <= 0.05 or target_distance_squared <= 0.01:
            self.publish_stop("TARGET_INVALID", "lookahead target is not forward")
            return

        curvature = 2.0 * local_y / target_distance_squared
        steering = math.atan(self.wheelbase_m * curvature)
        steering = float(
            np.clip(steering, -self.maximum_steering_rad, self.maximum_steering_rad)
        )
        obstacle_scale = self.obstacle_speed_scale()
        speed = self.speed_for_steering(steering) * obstacle_scale
        if obstacle_scale <= 0.0:
            self.state = "BLOCKED"
            self.reason = f"forward obstacle at {self.front_distance_m:.2f} m"
        elif obstacle_scale < 1.0:
            self.state = "SLOWING"
            self.reason = f"forward obstacle at {self.front_distance_m:.2f} m"
        else:
            self.state = "DRIVING"
            self.reason = "validated path and inputs are healthy"
        self.publish_command(speed, steering)
        self.publish_status()

    def publish_command(self, speed, steering):
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "opp_racecar/base_link"
        message.drive.speed = float(speed)
        message.drive.steering_angle = float(steering)
        self.drive_publisher.publish(message)
        self.last_command = (float(speed), float(steering))

    def publish_reference_path(self):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.poses = []
        for point, yaw in zip(self.path, self.path_yaw):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.orientation.z = math.sin(0.5 * float(yaw))
            pose.pose.orientation.w = math.cos(0.5 * float(yaw))
            message.poses.append(pose)
        self.path_publisher.publish(message)

    def publish_status(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "moving_agent/slow_path_follower"
        status.hardware_id = "simulator"
        status.level = (
            DiagnosticStatus.OK
            if self.state in {"DRIVING", "SLOWING"}
            else DiagnosticStatus.WARN
        )
        status.message = f"{self.state}: {self.reason}"
        values = {
            "state": self.state,
            "reason": self.reason,
            "progress_index": -1 if self.progress_index is None else self.progress_index,
            "cross_track_error_m": self.cross_track_error_m,
            "front_distance_m": self.front_distance_m,
            "command_speed_mps": self.last_command[0],
            "command_steering_rad": self.last_command[1],
        }
        status.values = [
            KeyValue(key=str(key), value=str(value)) for key, value in values.items()
        ]
        array.status = [status]
        self.status_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = SlowAgentController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
