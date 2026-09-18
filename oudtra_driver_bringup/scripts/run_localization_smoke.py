#!/usr/bin/env python3
"""Bounded PF/PnC startup and motion acceptance check; run in a sourced ROS workspace."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def initial_pose(config, offsets):
    values = [float(config[key]) + offset for key, offset in
              zip(('sx', 'sy', 'stheta'), offsets)]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Initial pose and offsets must be finite')
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-config', required=True)
    parser.add_argument('--run-duration-sec', type=float, default=15)
    parser.add_argument('--startup-timeout-sec', type=float, default=60)
    parser.add_argument('--motion-timeout-sec', type=float, default=15)
    parser.add_argument('--x-offset-m', type=float, default=0)
    parser.add_argument('--y-offset-m', type=float, default=0)
    parser.add_argument('--yaw-offset-rad', type=float, default=0)
    parser.add_argument('--ros-domain-id', type=int, default=94)
    parser.add_argument('--result-json', required=True)
    args = parser.parse_args()
    for value in (args.run_duration_sec, args.startup_timeout_sec, args.motion_timeout_sec):
        if not math.isfinite(value) or value <= 0:
            parser.error('Durations must be finite and positive')
    os.environ['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    import yaml
    import rclpy
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_share_directory
    from tf2_ros import Buffer, TransformListener
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from ackermann_msgs.msg import AckermannDriveStamped
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import Float32MultiArray
    from rcl_interfaces.srv import GetParameters

    config = yaml.safe_load(Path(args.sim_config).read_text())['bridge']['ros__parameters']
    pose = initial_pose(config, (args.x_offset_m, args.y_offset_m, args.yaw_offset_rad))
    overlay = str(Path(get_package_share_directory('oudtra_driver_bringup')) /
                  'config/localization_eval.yaml')
    logdir = Path(tempfile.mkdtemp(prefix='localization_smoke_'))
    result = {'result': 'FAIL', 'log_directory': str(logdir), 'states': [], 'commands': []}
    processes = {}
    last, received, counts = {}, {}, {}
    rclpy.init()
    node = rclpy.create_node('localization_smoke')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    subscriptions = []
    def receive(topic, message):
        last[topic] = message
        received[topic] = time.monotonic()
        counts[topic] = counts.get(topic, 0) + 1
    for topic, kind in [('/scan', LaserScan), ('/ego_racecar/odom', Odometry),
                        ('/pf/pose/odom', Odometry), ('/pf/health', Float32MultiArray),
                        ('/drive', AckermannDriveStamped),
                        ('/simulator/agent_status', DiagnosticArray),
                        ('/reactive_control_v2/lower_safety_status', DiagnosticArray)]:
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, t=topic: receive(t, msg), qos_profile_sensor_data))
    initpub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    stoppub = node.create_publisher(AckermannDriveStamped, '/drive', 1)

    def fresh(topic, age=1.0):
        return topic in received and time.monotonic() - received[topic] < age

    def check_processes():
        for name, (process, log, path) in processes.items():
            if process.poll() is not None:
                raise RuntimeError(name + ' launch exited before STOP')
            # Launch parents can survive failed child nodes. RViz is visualization only.
            for line in path.read_text(errors='replace').splitlines():
                if 'process has died' in line and 'rviz' not in line.lower():
                    raise RuntimeError('Critical child exited: ' + line)

    def wait(state, predicate, timeout=None):
        print(state, flush=True)
        result['states'].append(state)
        deadline = time.monotonic() + (timeout or args.startup_timeout_sec)
        while not predicate():
            check_processes()
            if time.monotonic() >= deadline:
                raise RuntimeError('Timeout in ' + state)
            rclpy.spin_once(node, timeout_sec=0.02)

    def launch(name, package, filename, arguments):
        command = ['ros2', 'launch', package, filename] + arguments
        path = logdir / (name + '.log')
        log = path.open('w')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes[name] = (process, log, path)
        result['commands'].append(command)

    def parameters(name, names):
        client = node.create_client(GetParameters, '/' + name + '/get_parameters')
        wait('PARAMETERS ' + name, client.service_is_ready)
        request = GetParameters.Request()
        request.names = names
        future = client.call_async(request)
        wait('PARAMETER_RESPONSE ' + name, future.done)
        values = future.result().values
        node.destroy_client(client)
        return values

    def localized():
        if not fresh('/pf/pose/odom') or not fresh('/pf/health'):
            return False
        msg = last['/pf/pose/odom']
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        return (msg.header.frame_id == 'map' and msg.header.stamp.sec > 0 and
                all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)) and
                abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) < .01 and
                len(last['/pf/health'].data) > 0 and
                last['/pf/health'].data[0] in (1, 2) and
                buffer.can_transform('map', 'ego_racecar/laser', Time()))

    def truth_xy():
        if not fresh('/simulator/agent_status'):
            raise RuntimeError('Simulator state unavailable')
        status = next(s for s in last['/simulator/agent_status'].status
                      if s.name == 'simulator/ego')
        fields = {v.key: v.value for v in status.values}
        xy = (float(fields['x_m']), float(fields['y_m']))
        if not all(math.isfinite(v) for v in xy):
            raise RuntimeError('Nonfinite simulator state')
        return xy

    def moving():
        return (fresh('/drive') and abs(last['/drive'].drive.speed) > .05 and
                fresh('/ego_racecar/odom') and
                abs(last['/ego_racecar/odom'].twist.twist.linear.x) > .05)

    def stop_process(name):
        if name not in processes:
            return
        process = processes[name][0]
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
                raise RuntimeError(name + ' required SIGTERM')

    try:
        launch('sim', 'f1tenth_gym_ros', 'gym_bridge_slam_launch.py',
               ['config_file:=' + str(Path(args.sim_config).resolve())])
        wait('WAIT_SCAN', lambda: fresh('/scan') and fresh('/ego_racecar/odom') and
             fresh('/simulator/agent_status'))
        if node.count_publishers('/scan') != 1 or node.count_publishers('/drive') != 1:
            # This observer owns the sole (zero-only) drive publisher until PnC starts.
            raise RuntimeError('ROS domain is not exclusive; use an unused --ros-domain-id')
        launch('pf', 'particle_filter', 'localize_sim_launch.py',
               ['parameter_overlay:=' + overlay])
        wait('WAIT_PF', lambda: fresh('/pf/pose/odom') and initpub.get_subscription_count() > 0)
        grace = parameters('particle_filter', ['manual_reset_grace_updates'])[0].integer_value
        before = counts.get('/pf/health', 0)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
        msg.pose.pose.orientation.z = math.sin(pose[2]/2)
        msg.pose.pose.orientation.w = math.cos(pose[2]/2)
        result['states'].append('SEND_INITIAL_POSE')
        initpub.publish(msg)
        result['initial_pose'] = pose
        wait('WAIT_LOCALIZATION', lambda: counts.get('/pf/health', 0) > before + grace + 5 and localized())
        start_xy = truth_xy()
        launch('pnc', 'oudtra_driver_bringup', 'full_stack_sim_launch.py',
               ['use_sim_time:=false'] + [key + ':=' + overlay for key in (
                   'path_platform_config', 'reactive_platform_config',
                   'arbitration_platform_config', 'integration_platform_config')])
        values = parameters('lower_safety_controller',
                            ['enable_sim_reverse_swept_gate', 'odom_topic'])
        if values[0].bool_value or values[1].string_value != '/pf/pose/odom':
            raise RuntimeError('Lower safety evaluation profile not applied')
        if not parameters('drive_arbitrator', ['require_pf_health'])[0].bool_value:
            raise RuntimeError('PF health gate not enabled')
        consumers = [e.node_name for e in node.get_subscriptions_info_by_topic('/simulator/agent_status')]
        result['gt_subscribers'] = consumers
        result['effective_parameters'] = {
            'enable_sim_reverse_swept_gate': values[0].bool_value,
            'lower_odom_topic': values[1].string_value, 'require_pf_health': True}
        if any(name != node.get_name() for name in consumers):
            raise RuntimeError('Unexpected GT consumer: ' + str(consumers))
        wait('WAIT_MOTION', lambda: moving() and math.dist(start_xy, truth_xy()) > .2,
             args.motion_timeout_sec)
        result['states'].append('RUNNING')
        start = time.monotonic()
        running_counts = counts.copy()
        last_motion = start
        progress_time, progress_xy = start, truth_xy()
        while time.monotonic() - start < args.run_duration_sec:
            rclpy.spin_once(node, timeout_sec=.02)
            check_processes()
            if not fresh('/pf/pose/odom') or not fresh('/scan'):
                raise RuntimeError('Required message flow lost')
            if moving():
                last_motion = time.monotonic()
            if time.monotonic() - last_motion > 2:
                raise RuntimeError('Motion stopped for more than two seconds')
            if time.monotonic() - progress_time >= 2:
                current_xy = truth_xy()
                if math.dist(progress_xy, current_xy) < .05:
                    raise RuntimeError('No physical progress over two seconds')
                progress_time, progress_xy = time.monotonic(), current_xy
        result['motion_duration_sec'] = time.monotonic() - start
        result['running_rates_hz'] = {topic: (count - running_counts.get(topic, 0)) /
                                      result['motion_duration_sec']
                                      for topic, count in counts.items()}
        result['displacement_m'] = math.dist(start_xy, truth_xy())
        result['result'] = 'PASS'
    except (Exception, KeyboardInterrupt) as error:
        result['reason'] = str(error) or 'Interrupted'
    finally:
        result['states'].append('STOP')
        try:
            stop_process('pnc')
            if 'sim' in processes and processes['sim'][0].poll() is None:
                stop_started = time.monotonic()
                deadline = stop_started + 3
                next_stop_command = stop_started
                stopped = False
                while time.monotonic() < deadline:
                    if time.monotonic() >= next_stop_command:
                        stoppub.publish(AckermannDriveStamped())
                        next_stop_command = time.monotonic() + .05
                    rclpy.spin_once(node, timeout_sec=.02)
                    if (received.get('/ego_racecar/odom', 0) > stop_started and
                            fresh('/ego_racecar/odom', .2) and
                            abs(last['/ego_racecar/odom'].twist.twist.linear.x) < .01):
                        stopped = True
                        break
                if not stopped:
                    raise RuntimeError('Failed to observe stationary shutdown')
        except Exception as error:
            result['result'] = 'FAIL'
            result['cleanup_error'] = str(error)
        for name in ('pnc', 'pf', 'sim'):
            try:
                stop_process(name)
            except Exception as error:
                result['result'] = 'FAIL'
                result['cleanup_error'] = str(error)
        result['message_counts'] = counts
        result['shutdown_return_codes'] = {}
        for name, (process, log, path) in processes.items():
            result['shutdown_return_codes'][name] = process.poll()
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                pass
            else:
                result['result'] = 'FAIL'
                result['cleanup_error'] = 'Process group still exists: ' + name
            log.close()
        node.destroy_node()
        rclpy.shutdown()
        Path(args.result_json).write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
    return 0 if result['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
