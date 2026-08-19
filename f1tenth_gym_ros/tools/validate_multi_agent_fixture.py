#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2026 F1TENTH Development Contributors
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Smoke-check that every car in the multi-agent fixture exists and moves."""

import argparse
import math
import time

from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import yaml

from f1tenth_gym_ros.multi_agent_fixture import traffic_agent_specs


class FixtureValidator(Node):
    """Collect per-agent motion and collision evidence during a short run."""

    def __init__(self, parameters):
        """Create subscriptions for every configured simulator identity."""
        super().__init__("multi_agent_fixture_validator")
        self.specs = traffic_agent_specs(parameters)
        self.initial_positions = {}
        self.latest_positions = {}
        self.collisions = set()
        self.seen_statuses = set()
        self.odom_subscriptions = []
        ego_namespace = str(parameters["ego_namespace"]).strip("/")
        ego_odom_leaf = str(parameters["ego_odom_topic"]).strip("/")
        self.odom_subscriptions.append(
            self.create_subscription(
                Odometry,
                f"/{ego_namespace}/{ego_odom_leaf}",
                lambda message: self.odom_callback("ego", message),
                10,
            )
        )
        odom_leaf = str(parameters["opp_odom_topic"]).strip("/")
        for spec in self.specs:
            topic = f"/{spec.namespace}/{odom_leaf}"
            self.odom_subscriptions.append(
                self.create_subscription(
                    Odometry,
                    topic,
                    lambda message, name=spec.namespace: self.odom_callback(
                        name, message
                    ),
                    10,
                )
            )
        self.status_subscription = self.create_subscription(
            DiagnosticArray,
            "/simulator/agent_status",
            self.status_callback,
            10,
        )
        self.ego_drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            str(parameters["ego_drive_topic"]),
            10,
        )
        self.create_timer(0.05, self.publish_stationary_ego_command)

    def publish_stationary_ego_command(self):
        """Allow the synchronized simulator to step without moving ego."""
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.drive.speed = 0.0
        message.drive.steering_angle = 0.0
        self.ego_drive_publisher.publish(message)

    def odom_callback(self, namespace, message):
        """Record first and latest map position for one traffic car."""
        position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        self.initial_positions.setdefault(namespace, position)
        self.latest_positions[namespace] = position

    def status_callback(self, message):
        """Remember every identity that reports a simulator collision."""
        for status in message.status:
            self.seen_statuses.add(status.name)
            values = {item.key: item.value for item in status.values}
            if values.get("collision", "false").lower() == "true":
                self.collisions.add(status.name)

    def failures(self, minimum_motion_m):
        """Return human-readable fixture failures after collection."""
        failures = []
        if "ego" not in self.latest_positions:
            failures.append("ego: no odometry")
        for spec in self.specs:
            initial = self.initial_positions.get(spec.namespace)
            latest = self.latest_positions.get(spec.namespace)
            if initial is None or latest is None:
                failures.append(f"{spec.namespace}: no odometry")
                continue
            motion = math.hypot(
                latest[0] - initial[0], latest[1] - initial[1]
            )
            if motion < minimum_motion_m:
                failures.append(
                    f"{spec.namespace}: moved only {motion:.3f} m"
                )
        if self.collisions:
            failures.append(
                "simulator collisions: " + ", ".join(sorted(self.collisions))
            )
        expected_statuses = {"simulator/ego"}
        expected_statuses.update(
            f"simulator/{spec.namespace}" for spec in self.specs
        )
        missing_statuses = expected_statuses - self.seen_statuses
        if missing_statuses:
            failures.append(
                "missing statuses: " + ", ".join(sorted(missing_statuses))
            )
        return failures


def parse_args():
    """Parse command-line fixture and duration settings."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=(
            "/sim_ws/src/f1tenth_gym_ros/config/sim_multi_agent.yaml"
        ),
    )
    parser.add_argument("--duration-sec", type=float, default=12.0)
    parser.add_argument("--minimum-motion-m", type=float, default=2.0)
    return parser.parse_args()


def main():
    """Run the ROS smoke check and exit nonzero on structural failure."""
    args = parse_args()
    with open(args.config, "r", encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)["bridge"]["ros__parameters"]

    rclpy.init()
    node = FixtureValidator(parameters)
    deadline = time.monotonic() + max(0.1, args.duration_sec)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        failures = node.failures(max(0.0, args.minimum_motion_m))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if failures:
        raise RuntimeError("; ".join(failures))
    print(
        f"PASS: {len(node.specs) + 1} vehicles present; "
        f"all {len(node.specs)} traffic cars moved without collision"
    )


if __name__ == "__main__":
    main()
