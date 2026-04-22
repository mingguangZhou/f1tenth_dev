#!/usr/bin/env python3
"""ROS 2 node that publishes an offline-generated centerline CSV.

Foxy-safe behavior:
- Uses transient-local QoS for static topics.
- Uses a dedicated wall-time thread for republishing, so publishing keeps going
  even when use_sim_time=true and /clock is paused or not advancing.
- Keeps CSV loading forward-compatible with future extra columns.
"""

from __future__ import annotations

import os
import threading
from typing import List

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .csv_loader import CenterlinePoint, close_loop, is_closed_loop, load_centerline_csv


class CenterlinePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('centerline_publisher')

        self.declare_parameter('csv_path', 'centerline_output/centerline_points_smooth.csv')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('path_topic', '/centerline_path')
        self.declare_parameter('marker_topic', '/centerline_markers')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('publish_start_marker', True)
        self.declare_parameter('publish_point_markers', False)
        self.declare_parameter('point_marker_stride', 10)
        self.declare_parameter('line_width', 0.03)
        self.declare_parameter('point_scale', 0.06)
        self.declare_parameter('start_marker_scale', 0.12)
        self.declare_parameter('close_loop_if_needed', True)
        self.declare_parameter('loop_closure_tolerance_m', 0.15)
        self.declare_parameter('publish_debug_every_n', 0)

        raw_csv_path = str(self.get_parameter('csv_path').value)
        self.csv_path = self._resolve_csv_path(raw_csv_path)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.publish_start_marker = bool(self.get_parameter('publish_start_marker').value)
        self.publish_point_markers = bool(self.get_parameter('publish_point_markers').value)
        self.point_marker_stride = int(self.get_parameter('point_marker_stride').value)
        self.line_width = float(self.get_parameter('line_width').value)
        self.point_scale = float(self.get_parameter('point_scale').value)
        self.start_marker_scale = float(self.get_parameter('start_marker_scale').value)
        self.close_loop_if_needed = bool(self.get_parameter('close_loop_if_needed').value)
        self.loop_closure_tolerance_m = float(self.get_parameter('loop_closure_tolerance_m').value)
        self.publish_debug_every_n = int(self.get_parameter('publish_debug_every_n').value)

        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be > 0.0')
        if self.point_marker_stride <= 0:
            raise ValueError('point_marker_stride must be > 0')

        self.publish_count = 0
        self._stop_event = threading.Event()
        self._publish_lock = threading.Lock()

        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.path_pub = self.create_publisher(Path, self.path_topic, static_qos)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, static_qos)

        self.points, self.extra_columns = self._load_points()
        self.path_msg = self._build_path_msg(self.points)
        self.marker_msg = self._build_marker_array(self.points)

        self._log_startup_summary(raw_csv_path)
        self._publish_messages()

        self._publish_thread = threading.Thread(
            target=self._wall_timer_publish_loop,
            name='centerline_publish_loop',
            daemon=True,
        )
        self._publish_thread.start()
        self.get_logger().info('Started wall-time republish thread.')

    def destroy_node(self):
        self._stop_event.set()
        thread = getattr(self, '_publish_thread', None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        return super().destroy_node()

    def _wall_timer_publish_loop(self) -> None:
        period = 1.0 / self.publish_rate_hz
        while rclpy.ok() and not self._stop_event.wait(period):
            try:
                self._publish_messages()
            except Exception as exc:  # pragma: no cover
                self.get_logger().error(f'Publish loop error: {exc}')

    def _resolve_csv_path(self, csv_path_param: str) -> str:
        if not csv_path_param:
            raise ValueError('csv_path parameter must not be empty')

        if os.path.isabs(csv_path_param):
            return csv_path_param

        package_share_dir = get_package_share_directory('centerline_tools')
        return os.path.normpath(os.path.join(package_share_dir, csv_path_param))

    def _load_points(self) -> tuple[List[CenterlinePoint], List[str]]:
        points, extra_columns = load_centerline_csv(self.csv_path)

        if is_closed_loop(points, self.loop_closure_tolerance_m):
            self.get_logger().info(
                f'Centerline appears closed within tolerance {self.loop_closure_tolerance_m:.3f} m.'
            )
        elif self.close_loop_if_needed:
            self.get_logger().warn(
                f'Centerline is not closed within {self.loop_closure_tolerance_m:.3f} m; appending first point to end.'
            )
            points = close_loop(points)
        else:
            self.get_logger().warn(
                f'Centerline is not closed within {self.loop_closure_tolerance_m:.3f} m and auto-closure is disabled.'
            )

        return points, extra_columns

    def _build_path_msg(self, points: List[CenterlinePoint]) -> Path:
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id

        for centerline_point in points:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = centerline_point.x
            pose.pose.position.y = centerline_point.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        return path_msg

    def _build_marker_array(self, points: List[CenterlinePoint]) -> MarkerArray:
        marker_array = MarkerArray()

        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.ns = 'centerline'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = self.line_width
        line_marker.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=1.0)
        line_marker.pose.orientation.w = 1.0
        line_marker.points = [self._make_point_msg(p.x, p.y) for p in points]
        marker_array.markers.append(line_marker)

        next_marker_id = 1

        if self.publish_start_marker and points:
            start_marker = Marker()
            start_marker.header.frame_id = self.frame_id
            start_marker.ns = 'centerline'
            start_marker.id = next_marker_id
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.scale.x = self.start_marker_scale
            start_marker.scale.y = self.start_marker_scale
            start_marker.scale.z = self.start_marker_scale
            start_marker.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
            start_marker.pose.position.x = points[0].x
            start_marker.pose.position.y = points[0].y
            start_marker.pose.position.z = 0.0
            start_marker.pose.orientation.w = 1.0
            marker_array.markers.append(start_marker)
            next_marker_id += 1

        if self.publish_point_markers:
            for point_index, point in enumerate(points[::self.point_marker_stride]):
                point_marker = Marker()
                point_marker.header.frame_id = self.frame_id
                point_marker.ns = 'centerline_points'
                point_marker.id = next_marker_id + point_index
                point_marker.type = Marker.SPHERE
                point_marker.action = Marker.ADD
                point_marker.scale.x = self.point_scale
                point_marker.scale.y = self.point_scale
                point_marker.scale.z = self.point_scale
                point_marker.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.85)
                point_marker.pose.position.x = point.x
                point_marker.pose.position.y = point.y
                point_marker.pose.position.z = 0.0
                point_marker.pose.orientation.w = 1.0
                marker_array.markers.append(point_marker)

        return marker_array

    @staticmethod
    def _make_point_msg(x: float, y: float) -> Point:
        point_msg = Point()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = 0.0
        return point_msg

    def _refresh_headers(self) -> None:
        now = self.get_clock().now().to_msg()

        self.path_msg.header.stamp = now
        for pose in self.path_msg.poses:
            pose.header.stamp = now

        for marker in self.marker_msg.markers:
            marker.header.stamp = now

    def _publish_messages(self) -> None:
        with self._publish_lock:
            self._refresh_headers()
            self.path_pub.publish(self.path_msg)
            self.marker_pub.publish(self.marker_msg)
            self.publish_count += 1

            if self.publish_count <= 3:
                self.get_logger().info(
                    f'Published centerline set #{self.publish_count}: '
                    f'{len(self.path_msg.poses)} poses, {len(self.marker_msg.markers)} markers.'
                )
            elif self.publish_debug_every_n > 0 and self.publish_count % self.publish_debug_every_n == 0:
                self.get_logger().info(
                    f'Published centerline set #{self.publish_count}: '
                    f'{len(self.path_msg.poses)} poses, {len(self.marker_msg.markers)} markers.'
                )

    def _log_startup_summary(self, raw_csv_path: str) -> None:
        extra_text = ', '.join(self.extra_columns) if self.extra_columns else 'none'
        self.get_logger().info(f'CSV path parameter: {raw_csv_path}')
        self.get_logger().info(f'Resolved centerline CSV: {self.csv_path}')
        self.get_logger().info(f'Frame id: {self.frame_id}')
        self.get_logger().info(f'Point count: {len(self.points)}')
        self.get_logger().info(f'Extra CSV columns detected: {extra_text}')
        self.get_logger().info(f'Publishing Path on: {self.path_topic}')
        self.get_logger().info(f'Publishing markers on: {self.marker_topic}')
        self.get_logger().info(f'Publish rate: {self.publish_rate_hz:.2f} Hz')
        self.get_logger().info(f'publish_point_markers parameter: {self.publish_point_markers}')
        self.get_logger().info(
            'MarkerArray contains 1 LINE_STRIP marker plus optional point markers.'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CenterlinePublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # pragma: no cover
        if node is not None:
            node.get_logger().fatal(f'Centerline publisher failed: {exc}')
        else:
            print(f'[centerline_publisher] Fatal startup error: {exc}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
