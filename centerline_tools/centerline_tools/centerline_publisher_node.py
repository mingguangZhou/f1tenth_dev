#!/usr/bin/env python3
"""ROS 2 node that publishes an offline-generated centerline CSV.

Foxy-safe behavior:
- Uses transient-local QoS for static topics.
- Uses a dedicated wall-time thread for republishing, so publishing keeps going
  even when use_sim_time=true and /clock is paused or not advancing.
- Publishes both a nav_msgs/Path and a rich waypoint array.
"""

from __future__ import annotations

import math
import os
import threading
from typing import List

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import ColorRGBA, Float64MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker, MarkerArray

from .csv_loader import CenterlinePoint, close_loop, is_closed_loop, load_centerline_csv


class CenterlinePublisherNode(Node):
    """Publish offline centerline artifacts as ROS 2 static topics."""

    WAYPOINT_FIELDS = ('index', 'x', 'y', 'yaw', 'curvature', 'curvature_abs')

    def __init__(self) -> None:
        super().__init__('centerline_publisher')

        self.declare_parameter('csv_path', 'centerline_output/centerline_points_smooth.csv')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('path_topic', '/centerline_path')
        self.declare_parameter('marker_topic', '/centerline_markers')
        self.declare_parameter('waypoints_topic', '/centerline_waypoints')
        self.declare_parameter('direction', 'csv')  # csv/normal or reverse
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('publish_start_marker', True)
        self.declare_parameter('publish_point_markers', False)
        self.declare_parameter('point_marker_stride', 10)
        self.declare_parameter('publish_direction_arrows', True)
        self.declare_parameter('direction_arrow_stride', 40)
        self.declare_parameter('direction_arrow_length', 0.35)
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
        self.waypoints_topic = str(self.get_parameter('waypoints_topic').value)
        self.direction = str(self.get_parameter('direction').value).lower()
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.publish_start_marker = self._get_bool_param('publish_start_marker')
        self.publish_point_markers = self._get_bool_param('publish_point_markers')
        self.point_marker_stride = int(self.get_parameter('point_marker_stride').value)
        self.publish_direction_arrows = self._get_bool_param('publish_direction_arrows')
        self.direction_arrow_stride = int(self.get_parameter('direction_arrow_stride').value)
        self.direction_arrow_length = float(self.get_parameter('direction_arrow_length').value)
        self.line_width = float(self.get_parameter('line_width').value)
        self.point_scale = float(self.get_parameter('point_scale').value)
        self.start_marker_scale = float(self.get_parameter('start_marker_scale').value)
        self.close_loop_if_needed = self._get_bool_param('close_loop_if_needed')
        self.loop_closure_tolerance_m = float(self.get_parameter('loop_closure_tolerance_m').value)
        self.publish_debug_every_n = int(self.get_parameter('publish_debug_every_n').value)

        if self.direction == 'normal':
            self.direction = 'csv'
        if self.direction not in ('csv', 'reverse'):
            raise ValueError("direction must be one of: 'csv', 'normal', 'reverse'")
        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be > 0.0')
        if self.point_marker_stride <= 0:
            raise ValueError('point_marker_stride must be > 0')
        if self.direction_arrow_stride <= 0:
            raise ValueError('direction_arrow_stride must be > 0')
        if self.direction_arrow_length <= 0.0:
            raise ValueError('direction_arrow_length must be > 0.0')

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
        self.waypoints_pub = self.create_publisher(Float64MultiArray, self.waypoints_topic, static_qos)

        self.points, self.extra_columns = self._load_points()
        self.path_msg = self._build_path_msg(self.points)
        self.marker_msg = self._build_marker_array(self.points)
        self.waypoints_msg = self._build_waypoints_msg(self.points)

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


    def _get_bool_param(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in ('true', '1', 'yes', 'y', 'on'):
                return True
            if normalized in ('false', '0', 'no', 'n', 'off'):
                return False
        raise ValueError(f"Parameter {name} must be a boolean value, got {value!r}")

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

        if self.direction == 'reverse':
            points = self._reverse_closed_points(points)
            self.get_logger().warn('Publishing centerline in reversed direction.')

        # Recompute yaw and curvature after final direction choice. This keeps
        # CSV direction, Path orientation, and waypoint curvature consistent.
        points = self._recompute_directional_fields(points)
        extra_columns = sorted(set(extra_columns).union({'yaw', 'curvature', 'curvature_abs'}))
        return points, extra_columns

    def _reverse_closed_points(self, points: List[CenterlinePoint]) -> List[CenterlinePoint]:
        closed = is_closed_loop(points, self.loop_closure_tolerance_m)
        core = list(points[:-1] if closed else points)
        core.reverse()

        reversed_points = [
            CenterlinePoint(index=i, x=p.x, y=p.y, extras=dict(p.extras))
            for i, p in enumerate(core)
        ]

        if closed and reversed_points:
            first = reversed_points[0]
            reversed_points.append(
                CenterlinePoint(index=len(reversed_points), x=first.x, y=first.y, extras=dict(first.extras))
            )

        return reversed_points

    def _recompute_directional_fields(self, points: List[CenterlinePoint]) -> List[CenterlinePoint]:
        if len(points) < 3:
            return points

        closed = is_closed_loop(points, self.loop_closure_tolerance_m)
        core = list(points[:-1] if closed else points)
        n = len(core)
        if n < 3:
            return points

        recomputed: List[CenterlinePoint] = []
        for i, p in enumerate(core):
            p_prev = core[(i - 1) % n]
            p_next = core[(i + 1) % n]

            tx = p_next.x - p_prev.x
            ty = p_next.y - p_prev.y
            yaw = math.atan2(ty, tx)

            ax = p.x - p_prev.x
            ay = p.y - p_prev.y
            bx = p_next.x - p.x
            by = p_next.y - p.y
            cx = p_next.x - p_prev.x
            cy = p_next.y - p_prev.y

            la = math.hypot(ax, ay)
            lb = math.hypot(bx, by)
            lc = math.hypot(cx, cy)

            if la < 1e-9 or lb < 1e-9 or lc < 1e-9:
                curvature = 0.0
            else:
                cross = ax * by - ay * bx
                curvature = 2.0 * cross / (la * lb * lc)

            extras = dict(p.extras)
            extras['yaw'] = yaw
            extras['curvature'] = curvature
            extras['curvature_abs'] = abs(curvature)

            recomputed.append(CenterlinePoint(index=i, x=p.x, y=p.y, extras=extras))

        if closed:
            first = recomputed[0]
            recomputed.append(CenterlinePoint(index=len(recomputed), x=first.x, y=first.y, extras=dict(first.extras)))

        return recomputed

    def _build_path_msg(self, points: List[CenterlinePoint]) -> Path:
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id

        for centerline_point in points:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = centerline_point.x
            pose.pose.position.y = centerline_point.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = self._yaw_to_quaternion(centerline_point.get_float('yaw', 0.0))
            path_msg.poses.append(pose)

        return path_msg

    def _build_waypoints_msg(self, points: List[CenterlinePoint]) -> Float64MultiArray:
        msg = Float64MultiArray()
        msg.layout.dim.append(
            MultiArrayDimension(label='points', size=len(points), stride=len(points) * len(self.WAYPOINT_FIELDS))
        )
        msg.layout.dim.append(
            MultiArrayDimension(label='fields:index,x,y,yaw,curvature,curvature_abs', size=len(self.WAYPOINT_FIELDS), stride=len(self.WAYPOINT_FIELDS))
        )
        msg.layout.data_offset = 0

        data = []
        for p in points:
            curvature = p.get_float('curvature', 0.0)
            data.extend([
                float(p.index),
                float(p.x),
                float(p.y),
                float(p.get_float('yaw', 0.0)),
                float(curvature),
                float(p.get_float('curvature_abs', abs(curvature))),
            ])
        msg.data = data
        return msg

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

        if self.publish_direction_arrows:
            # Sampled arrows make the CSV direction visible in RViz.
            # For Marker.ARROW, scale.x is arrow length, scale.y is shaft diameter,
            # and scale.z is head diameter.
            sampled_points = points[:-1] if is_closed_loop(points, self.loop_closure_tolerance_m) else points
            for arrow_index, point in enumerate(sampled_points[::self.direction_arrow_stride]):
                arrow_marker = Marker()
                arrow_marker.header.frame_id = self.frame_id
                arrow_marker.ns = 'centerline_direction'
                arrow_marker.id = next_marker_id + arrow_index
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                arrow_marker.scale.x = self.direction_arrow_length
                arrow_marker.scale.y = max(self.line_width * 2.0, 0.025)
                arrow_marker.scale.z = max(self.line_width * 4.0, 0.06)
                arrow_marker.color = ColorRGBA(r=1.0, g=0.75, b=0.05, a=1.0)
                arrow_marker.pose.position.x = point.x
                arrow_marker.pose.position.y = point.y
                arrow_marker.pose.position.z = 0.03
                arrow_marker.pose.orientation = self._yaw_to_quaternion(point.get_float('yaw', 0.0))
                marker_array.markers.append(arrow_marker)
            next_marker_id += len(sampled_points[::self.direction_arrow_stride])

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

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

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
            self.waypoints_pub.publish(self.waypoints_msg)
            self.publish_count += 1

            if self.publish_count <= 3:
                self.get_logger().info(
                    f'Published centerline set #{self.publish_count}: '
                    f'{len(self.path_msg.poses)} poses, {len(self.marker_msg.markers)} markers, {len(self.points)} waypoint rows.'
                )
            elif self.publish_debug_every_n > 0 and self.publish_count % self.publish_debug_every_n == 0:
                self.get_logger().info(
                    f'Published centerline set #{self.publish_count}: '
                    f'{len(self.path_msg.poses)} poses, {len(self.marker_msg.markers)} markers, {len(self.points)} waypoint rows.'
                )

    def _log_startup_summary(self, raw_csv_path: str) -> None:
        extra_text = ', '.join(self.extra_columns) if self.extra_columns else 'none'
        self.get_logger().info(f'CSV path parameter: {raw_csv_path}')
        self.get_logger().info(f'Resolved centerline CSV: {self.csv_path}')
        self.get_logger().info(f'Frame id: {self.frame_id}')
        self.get_logger().info(f'Direction parameter: {self.direction}')
        self.get_logger().info(f'Point count: {len(self.points)}')
        self.get_logger().info(f'Extra CSV columns detected/available: {extra_text}')
        self.get_logger().info(f'Publishing Path on: {self.path_topic}')
        self.get_logger().info(f'Publishing markers on: {self.marker_topic}')
        self.get_logger().info(f'Publishing waypoint rows on: {self.waypoints_topic}')
        self.get_logger().info('Waypoint row fields: index,x,y,yaw,curvature,curvature_abs')
        self.get_logger().info(f'Publish rate: {self.publish_rate_hz:.2f} Hz')
        self.get_logger().info(f'publish_point_markers parameter: {self.publish_point_markers}')
        self.get_logger().info(f'publish_direction_arrows parameter: {self.publish_direction_arrows}')
        self.get_logger().info('MarkerArray contains 1 LINE_STRIP marker, optional start marker, direction arrows, and optional point markers.')


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
