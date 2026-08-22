# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from tf2_ros import TransformBroadcaster

import gym
import numpy as np
from transforms3d import euler

class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        self.declare_parameter('ego_namespace')
        self.declare_parameter('ego_odom_topic')
        self.declare_parameter('ego_opp_odom_topic')
        self.declare_parameter('ego_scan_topic')
        self.declare_parameter('ego_drive_topic')
        self.declare_parameter('opp_namespace')
        self.declare_parameter('opp_odom_topic')
        self.declare_parameter('opp_ego_odom_topic')
        self.declare_parameter('opp_scan_topic')
        self.declare_parameter('opp_drive_topic')
        self.declare_parameter('scan_distance_to_base_link')
        self.declare_parameter('scan_fov')
        self.declare_parameter('scan_beams')
        self.declare_parameter('map_path')
        self.declare_parameter('map_img_ext')
        self.declare_parameter('num_agent')
        self.declare_parameter('sx')
        self.declare_parameter('sy')
        self.declare_parameter('stheta')
        self.declare_parameter('sx1')
        self.declare_parameter('sy1')
        self.declare_parameter('stheta1')
        self.declare_parameter('kb_teleop')

        # Non-empty defaults give ROS Foxy explicit array element types.
        self.declare_parameter('traffic_namespaces', [''])
        self.declare_parameter('traffic_scan_topics', [''])
        self.declare_parameter('traffic_drive_topics', [''])
        self.declare_parameter('traffic_start_x', [0.0])
        self.declare_parameter('traffic_start_y', [0.0])
        self.declare_parameter('traffic_start_theta', [0.0])

        # check num_agents
        num_agents = self.get_parameter('num_agent').value
        if type(num_agents) != int:
            raise ValueError('num_agent should be an int.')
        if num_agents < 1:
            raise ValueError('num_agent should be at least one.')
        self.num_agents = num_agents
        self.has_opp = num_agents > 1

        self.agent_namespaces = [
            str(self.get_parameter('ego_namespace').value).strip().strip('/')]
        self.scan_topics = [self._normalize_topic(
            self.get_parameter('ego_scan_topic').value)]
        self.drive_topics = [self._normalize_topic(
            self.get_parameter('ego_drive_topic').value)]
        self.odom_topics = [
            self._topic_in_namespace(
                self.agent_namespaces[0],
                self.get_parameter('ego_odom_topic').value)]
        starts = [[
            float(self.get_parameter('sx').value),
            float(self.get_parameter('sy').value),
            float(self.get_parameter('stheta').value),
        ]]

        traffic_count = num_agents - 1
        if traffic_count == 1:
            namespace = str(
                self.get_parameter('opp_namespace').value).strip().strip('/')
            self.agent_namespaces.append(namespace)
            self.scan_topics.append(
                self._normalize_topic(
                    self.get_parameter('opp_scan_topic').value))
            self.drive_topics.append(
                self._normalize_topic(
                    self.get_parameter('opp_drive_topic').value))
            self.odom_topics.append(
                self._topic_in_namespace(
                    namespace, self.get_parameter('opp_odom_topic').value))
            starts.append([
                float(self.get_parameter('sx1').value),
                float(self.get_parameter('sy1').value),
                float(self.get_parameter('stheta1').value),
            ])
        elif traffic_count > 1:
            namespaces = [
                str(value).strip().strip('/') for value in
                self.get_parameter('traffic_namespaces').value]
            scan_topics = [
                self._normalize_topic(value) for value in
                self.get_parameter('traffic_scan_topics').value]
            drive_topics = [
                self._normalize_topic(value) for value in
                self.get_parameter('traffic_drive_topics').value]
            starts_x = list(self.get_parameter('traffic_start_x').value)
            starts_y = list(self.get_parameter('traffic_start_y').value)
            starts_theta = list(
                self.get_parameter('traffic_start_theta').value)
            configured = {
                'traffic_namespaces': namespaces,
                'traffic_scan_topics': scan_topics,
                'traffic_drive_topics': drive_topics,
                'traffic_start_x': starts_x,
                'traffic_start_y': starts_y,
                'traffic_start_theta': starts_theta,
            }
            for name, values in configured.items():
                if len(values) != traffic_count:
                    raise ValueError(
                        f'{name} should contain {traffic_count} values for '
                        f'num_agent={num_agents}.')
            self.agent_namespaces.extend(namespaces)
            self.scan_topics.extend(scan_topics)
            self.drive_topics.extend(drive_topics)
            odom_topic = self.get_parameter('opp_odom_topic').value
            self.odom_topics.extend(
                self._topic_in_namespace(namespace, odom_topic)
                for namespace in namespaces)
            starts.extend(
                [float(x), float(y), float(theta)]
                for x, y, theta in zip(starts_x, starts_y, starts_theta))

        if any(not namespace for namespace in self.agent_namespaces):
            raise ValueError('Agent namespaces cannot be blank.')
        for namespace in self.agent_namespaces[1:]:
            if '/' in namespace:
                raise ValueError(
                    'Traffic namespaces cannot contain internal slashes.')
            if namespace == 'ego':
                raise ValueError(
                    "Traffic namespace 'ego' is reserved for diagnostics.")
        if any(not topic for topic in self.scan_topics):
            raise ValueError('Agent scan topics cannot be blank.')
        if any(not topic for topic in self.drive_topics):
            raise ValueError('Agent drive topics cannot be blank.')
        if any(not topic for topic in self.odom_topics):
            raise ValueError('Agent odom topics cannot be blank.')
        if len(set(self.agent_namespaces)) != num_agents:
            raise ValueError('Agent namespaces should be unique.')
        if len(set(self.scan_topics)) != num_agents:
            raise ValueError('Agent scan topics should be unique.')
        if len(set(self.drive_topics)) != num_agents:
            raise ValueError('Agent drive topics should be unique.')
        if len(set(self.odom_topics)) != num_agents:
            raise ValueError('Agent odom topics should be unique.')

        all_odom_topics = list(self.odom_topics)
        if num_agents == 2:
            all_odom_topics.extend([
                self._topic_in_namespace(
                    self.agent_namespaces[0],
                    self.get_parameter('ego_opp_odom_topic').value),
                self._topic_in_namespace(
                    self.agent_namespaces[1],
                    self.get_parameter('opp_ego_odom_topic').value),
            ])
        if any(not topic for topic in all_odom_topics):
            raise ValueError('Resolved odom topics cannot be blank.')
        if len(set(all_odom_topics)) != len(all_odom_topics):
            raise ValueError('Resolved odom topics should be unique.')

        topic_roles = {}
        for role, topics in (
                ('scan', self.scan_topics),
                ('drive', self.drive_topics),
                ('odom', all_odom_topics)):
            for topic in topics:
                resolved_topic = '/' + topic
                previous_role = topic_roles.get(resolved_topic)
                if previous_role is not None:
                    raise ValueError(
                        f'Resolved topic {resolved_topic} is used for both '
                        f'{previous_role} and {role}.')
                topic_roles[resolved_topic] = role

        # env backend
        self.env = gym.make('f110_gym:f110-v0',
                            map=self.get_parameter('map_path').value,
                            map_ext=self.get_parameter('map_img_ext').value,
                            num_agents=num_agents,
                            lidar_dist=self.get_parameter("scan_distance_to_base_link").value
                            )
        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        # self.angle_inc = scan_fov / scan_beams
        self.angle_inc = scan_fov / (scan_beams - 1)
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value
        self.obs, _ , self.done, _ = self.env.reset(np.array(starts))
        self.poses = [list(start) for start in starts]
        self.speeds = [[0.0, 0.0, 0.0] for _ in range(num_agents)]
        self.scans = [[] for _ in range(num_agents)]
        self.collisions = [False for _ in range(num_agents)]
        self.requested_speeds = [0.0 for _ in range(num_agents)]
        self.steering_angles = [0.0 for _ in range(num_agents)]
        self.drive_published = [False for _ in range(num_agents)]
        self._update_sim_state()

        # sim physical step timer
        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
        # topic publishing timer
        self.timer = self.create_timer(0.004, self.timer_callback)

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        # publishers
        self.scan_publishers = [
            self.create_publisher(LaserScan, topic, 10)
            for topic in self.scan_topics]
        self.odom_publishers = [
            self.create_publisher(Odometry, topic, 10)
            for topic in self.odom_topics]
        self.agent_status_pub = self.create_publisher(
            DiagnosticArray, '/simulator/agent_status', 10)
        self.ego_opp_odom_pub = None
        self.opp_ego_odom_pub = None
        if num_agents == 2:
            ego_opp_odom_topic = self._topic_in_namespace(
                self.agent_namespaces[0],
                self.get_parameter('ego_opp_odom_topic').value)
            opp_ego_odom_topic = self._topic_in_namespace(
                self.agent_namespaces[1],
                self.get_parameter('opp_ego_odom_topic').value)
            self.ego_opp_odom_pub = self.create_publisher(
                Odometry, ego_opp_odom_topic, 10)
            self.opp_ego_odom_pub = self.create_publisher(
                Odometry, opp_ego_odom_topic, 10)

        # subscribers
        self.drive_subscriptions = []
        for index, topic in enumerate(self.drive_topics):
            self.drive_subscriptions.append(self.create_subscription(
                AckermannDriveStamped,
                topic,
                lambda message, agent_index=index:
                    self.drive_callback(message, agent_index),
                10))
        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.ego_reset_callback,
            10)
        if self.has_opp:
            self.opp_reset_sub = self.create_subscription(
                PoseStamped,
                '/goal_pose',
                self.opp_reset_callback,
                10)

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.teleop_callback,
                10)

    @staticmethod
    def _normalize_topic(topic):
        return str(topic).strip().strip('/')

    @staticmethod
    def _topic_in_namespace(namespace, topic):
        normalized_namespace = str(namespace).strip().strip('/')
        normalized_topic = GymBridge._normalize_topic(topic)
        if not normalized_namespace or not normalized_topic:
            return ''
        return normalized_namespace + '/' + normalized_topic

    def drive_callback(self, drive_msg, agent_index=0):
        self.requested_speeds[agent_index] = drive_msg.drive.speed
        self.steering_angles[agent_index] = drive_msg.drive.steering_angle
        self.drive_published[agent_index] = True

    def ego_reset_callback(self, pose_msg):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
        poses = [list(pose) for pose in self.poses]
        poses[0] = [rx, ry, rtheta]
        self.obs, _ , self.done, _ = self.env.reset(np.array(poses))
        self._update_sim_state()

    def opp_reset_callback(self, pose_msg):
        if self.has_opp:
            rx = pose_msg.pose.position.x
            ry = pose_msg.pose.position.y
            rqx = pose_msg.pose.orientation.x
            rqy = pose_msg.pose.orientation.y
            rqz = pose_msg.pose.orientation.z
            rqw = pose_msg.pose.orientation.w
            _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
            poses = [list(pose) for pose in self.poses]
            poses[1] = [rx, ry, rtheta]
            self.obs, _ , self.done, _ = self.env.reset(np.array(poses))
            self._update_sim_state()

    def teleop_callback(self, twist_msg):
        if not self.drive_published[0]:
            self.drive_published[0] = True

        self.requested_speeds[0] = twist_msg.linear.x

        if twist_msg.angular.z > 0.0:
            self.steering_angles[0] = 0.3
        elif twist_msg.angular.z < 0.0:
            self.steering_angles[0] = -0.3
        else:
            self.steering_angles[0] = 0.0

    def drive_timer_callback(self):
        if all(self.drive_published):
            actions = np.array(list(zip(
                self.steering_angles, self.requested_speeds)))
            self.obs, _, self.done, _ = self.env.step(actions)
        self._update_sim_state()

    def timer_callback(self):
        ts = self.get_clock().now().to_msg()

        # pub scans
        for index, publisher in enumerate(self.scan_publishers):
            scan = LaserScan()
            scan.header.stamp = ts
            scan.header.frame_id = self.agent_namespaces[index] + '/laser'
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_inc
            scan.range_min = 0.
            scan.range_max = 10.
            scan.ranges = self.scans[index]
            publisher.publish(scan)

        # pub tf
        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)
        self._publish_agent_status(ts)

    def _update_sim_state(self):
        collisions = self.obs.get('collisions', [])
        for index in range(self.num_agents):
            self.scans[index] = list(self.obs['scans'][index])
            self.poses[index][0] = self.obs['poses_x'][index]
            self.poses[index][1] = self.obs['poses_y'][index]
            self.poses[index][2] = self.obs['poses_theta'][index]
            self.speeds[index][0] = self.obs['linear_vels_x'][index]
            self.speeds[index][1] = self.obs['linear_vels_y'][index]
            self.speeds[index][2] = self.obs['ang_vels_z'][index]
            self.collisions[index] = (
                bool(collisions[index]) if len(collisions) > index else False)

    def _publish_agent_status(self, stamp):
        message = DiagnosticArray()
        message.header.stamp = stamp
        statuses = []
        for index, (namespace, pose, speed, collision) in enumerate(zip(
                self.agent_namespaces, self.poses,
                self.speeds, self.collisions)):
            name = 'ego' if index == 0 else namespace
            if self.num_agents == 2 and index == 1:
                name = 'slow_agent'
            separation = min(
                (math.hypot(
                    pose[0] - other_pose[0],
                    pose[1] - other_pose[1])
                 for other_index, other_pose in enumerate(self.poses)
                 if other_index != index),
                default=math.inf)
            status = DiagnosticStatus()
            status.name = f'simulator/{name}'
            status.hardware_id = 'f1tenth_gym'
            status.level = (
                DiagnosticStatus.ERROR if collision else DiagnosticStatus.OK)
            status.message = 'COLLISION' if collision else 'DRIVING'
            values = {
                'collision': collision,
                'x_m': pose[0],
                'y_m': pose[1],
                'yaw_rad': pose[2],
                'speed_mps': math.hypot(speed[0], speed[1]),
                'body_speed_mps': speed[0],
                'yaw_rate_radps': speed[2],
                'agent_separation_m': separation,
            }
            status.values = [
                KeyValue(key=str(key), value=str(value))
                for key, value in values.items()]
            statuses.append(status)
        message.status = statuses
        self.agent_status_pub.publish(message)

        

    def _publish_odom(self, ts):
        odometry = []
        for index, publisher in enumerate(self.odom_publishers):
            pose = self.poses[index]
            speed = self.speeds[index]
            odom = Odometry()
            odom.header.stamp = ts
            odom.header.frame_id = 'map'
            odom.child_frame_id = (
                self.agent_namespaces[index] + '/base_link')
            odom.pose.pose.position.x = pose[0]
            odom.pose.pose.position.y = pose[1]
            quat = euler.euler2quat(0., 0., pose[2], axes='sxyz')
            odom.pose.pose.orientation.x = quat[1]
            odom.pose.pose.orientation.y = quat[2]
            odom.pose.pose.orientation.z = quat[3]
            odom.pose.pose.orientation.w = quat[0]
            odom.twist.twist.linear.x = speed[0]
            odom.twist.twist.linear.y = speed[1]
            odom.twist.twist.angular.z = speed[2]
            publisher.publish(odom)
            odometry.append(odom)

        if self.num_agents == 2:
            self.opp_ego_odom_pub.publish(odometry[0])
            self.ego_opp_odom_pub.publish(odometry[1])

    def _publish_transforms(self, ts):
        for namespace, pose in zip(self.agent_namespaces, self.poses):
            transform = Transform()
            transform.translation.x = pose[0]
            transform.translation.y = pose[1]
            transform.translation.z = 0.0
            quat = euler.euler2quat(0.0, 0.0, pose[2], axes='sxyz')
            transform.rotation.x = quat[1]
            transform.rotation.y = quat[2]
            transform.rotation.z = quat[3]
            transform.rotation.w = quat[0]

            stamped = TransformStamped()
            stamped.transform = transform
            stamped.header.stamp = ts
            stamped.header.frame_id = 'map'
            stamped.child_frame_id = namespace + '/base_link'
            self.br.sendTransform(stamped)

    def _publish_wheel_transforms(self, ts):
        for namespace, steering in zip(
                self.agent_namespaces, self.steering_angles):
            wheel_quat = euler.euler2quat(0., 0., steering, axes='sxyz')
            for side in ('left', 'right'):
                wheel_ts = TransformStamped()
                wheel_ts.transform.rotation.x = wheel_quat[1]
                wheel_ts.transform.rotation.y = wheel_quat[2]
                wheel_ts.transform.rotation.z = wheel_quat[3]
                wheel_ts.transform.rotation.w = wheel_quat[0]
                wheel_ts.header.stamp = ts
                wheel_ts.header.frame_id = (
                    namespace + f'/front_{side}_hinge')
                wheel_ts.child_frame_id = (
                    namespace + f'/front_{side}_wheel')
                self.br.sendTransform(wheel_ts)

    def _publish_laser_transforms(self, ts):
        for namespace in self.agent_namespaces:
            scan_ts = TransformStamped()
            scan_ts.transform.translation.x = self.scan_distance_to_base_link
            scan_ts.transform.rotation.w = 1.
            scan_ts.header.stamp = ts
            scan_ts.header.frame_id = namespace + '/base_link'
            scan_ts.child_frame_id = namespace + '/laser'
            self.br.sendTransform(scan_ts)

def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)

if __name__ == '__main__':
    main()
