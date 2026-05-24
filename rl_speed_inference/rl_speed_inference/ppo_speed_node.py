#!/usr/bin/env python3

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from stable_baselines3 import PPO

from rl_training.centerline_utils import (
    load_centerline_csv,
    get_centerline_state_features,
)


def yaw_from_quaternion(q):
    """
    Convert quaternion to yaw angle.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PPOSpeedNode(Node):
    """
    ROS2 inference node for PPO speed policy.

    Subscribes:
        odometry

    Publishes:
        target speed from trained PPO policy
    """

    def __init__(self):
        super().__init__("ppo_speed_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("centerline_csv", "")
        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("speed_topic", "/rl_target_speed")
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("max_speed", 4.0)
        self.declare_parameter("curvature_lookahead_points", 20)

        self.model_path = self.get_parameter("model_path").value
        self.centerline_csv = self.get_parameter("centerline_csv").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.speed_topic = self.get_parameter("speed_topic").value

        self.min_speed = float(self.get_parameter("min_speed").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.curvature_lookahead_points = int(
            self.get_parameter("curvature_lookahead_points").value
        )

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model path does not exist: {self.model_path}")

        if not os.path.exists(self.centerline_csv):
            raise FileNotFoundError(
                f"Centerline CSV path does not exist: {self.centerline_csv}"
            )

        self.get_logger().info(f"Loading PPO model: {self.model_path}")
        self.model = PPO.load(self.model_path)

        self.get_logger().info(f"Loading centerline CSV: {self.centerline_csv}")
        self.centerline = load_centerline_csv(self.centerline_csv)

        self.speed_pub = self.create_publisher(Float32, self.speed_topic, 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.get_logger().info("PPO speed inference node started.")
        self.get_logger().info(f"Subscribing odom: {self.odom_topic}")
        self.get_logger().info(f"Publishing speed: {self.speed_topic}")

    def odom_callback(self, msg: Odometry):
        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y
        car_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        car_speed = math.sqrt(vx * vx + vy * vy)

        features, nearest_idx = get_centerline_state_features(
            car_x=car_x,
            car_y=car_y,
            car_yaw=car_yaw,
            car_speed=car_speed,
            centerline=self.centerline,
            curvature_lookahead_points=self.curvature_lookahead_points,
        )

        obs = np.array(features, dtype=np.float32)

        action, _ = self.model.predict(obs, deterministic=True)

        target_speed = float(action[0])
        target_speed = float(np.clip(target_speed, self.min_speed, self.max_speed))

        speed_msg = Float32()
        speed_msg.data = target_speed
        self.speed_pub.publish(speed_msg)

        self.get_logger().debug(
            f"obs={obs}, nearest_idx={nearest_idx}, speed_cmd={target_speed:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PPOSpeedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
