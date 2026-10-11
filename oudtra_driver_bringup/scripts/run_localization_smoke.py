#!/usr/bin/env python3
"""Bounded PF/PnC startup and motion acceptance check; run in a sourced ROS workspace."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import tempfile
import time

import yaml


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _map_image(map_yaml):
    try:
        data = yaml.safe_load(Path(map_yaml).read_text()) or {}
    except (OSError, yaml.YAMLError) as error:
        raise ValueError('Invalid map YAML {}: {}'.format(map_yaml, error))
    if not isinstance(data, dict):
        raise ValueError('Map YAML root must be a mapping: ' + str(map_yaml))
    image = data.get('image')
    if not isinstance(image, str) or not image:
        raise ValueError('Map YAML has no image: ' + str(map_yaml))
    path = Path(image)
    return (path if path.is_absolute() else Path(map_yaml).parent / path).resolve(), data


def validate_track_inputs(scenario_id, sim_config, pf_map_yaml, raceline_csv,
                          raceline_direction, localize_config):
    """Validate one explicit scenario before any ROS process is launched."""
    if not scenario_id or not scenario_id.strip():
        raise ValueError('scenario_id must not be empty')
    if raceline_direction not in ('csv', 'normal', 'reverse'):
        raise ValueError('raceline_direction must be csv, normal, or reverse')
    paths = {name: Path(value).expanduser().resolve() for name, value in {
        'simulator_config': sim_config, 'pf_map_yaml': pf_map_yaml,
        'raceline_csv': raceline_csv, 'localize_config': localize_config}.items()}
    for name, path in paths.items():
        if not path.is_file():
            raise ValueError('{} does not exist: {}'.format(name, path))
    try:
        sim_parameters = yaml.safe_load(paths['simulator_config'].read_text())['bridge']['ros__parameters']
        map_path = Path(sim_parameters['map_path'])
        if not map_path.is_absolute():
            map_path = paths['simulator_config'].parent / map_path
        sim_map_yaml = Path(str(map_path) + '.yaml').resolve()
        sim_config_image = Path(str(map_path) + str(sim_parameters['map_img_ext'])).resolve()
        for key in ('sx', 'sy', 'stheta'):
            float(sim_parameters[key])
    except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError) as error:
        raise ValueError('Invalid simulator bridge track configuration: ' + str(error))
    for label, path in [('simulator map YAML', sim_map_yaml),
                        ('simulator map image', sim_config_image)]:
        if not path.is_file():
            raise ValueError('{} does not exist: {}'.format(label, path))
    sim_yaml_image, sim_map = _map_image(sim_map_yaml)
    pf_image, pf_map = _map_image(paths['pf_map_yaml'])
    for label, path in [('image referenced by simulator map YAML', sim_yaml_image),
                        ('image referenced by PF map YAML', pf_image)]:
        if not path.is_file():
            raise ValueError('{} does not exist: {}'.format(label, path))
    if _sha256(sim_config_image) != _sha256(sim_yaml_image):
        raise ValueError('Simulator map_img_ext image disagrees with its map YAML image')
    map_fields = ('resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh')
    if (any(sim_map.get(key) != pf_map.get(key) for key in map_fields) or
            _sha256(sim_yaml_image) != _sha256(pf_image)):
        raise ValueError('Simulator and PF map inputs describe different maps')
    try:
        with paths['raceline_csv'].open(newline='') as stream:
            reader = csv.DictReader(stream)
            if not reader.fieldnames or not {'x', 'y'} <= set(reader.fieldnames):
                raise ValueError('raceline CSV requires x and y columns')
            points = [(float(row['x']), float(row['y'])) for row in reader]
        if len(points) < 2 or not all(math.isfinite(v) for point in points for v in point):
            raise ValueError('raceline CSV requires at least two finite geometry rows')
    except (OSError, ValueError, TypeError, KeyError) as error:
        raise ValueError('Invalid raceline CSV: ' + str(error))
    assets = {
        **{name: {'path': str(path), 'sha256': _sha256(path)} for name, path in paths.items()},
        'simulator_map_yaml': {'path': str(sim_map_yaml), 'sha256': _sha256(sim_map_yaml)},
        'simulator_map_image': {'path': str(sim_config_image), 'sha256': _sha256(sim_config_image)},
        'pf_map_image': {'path': str(pf_image), 'sha256': _sha256(pf_image)},
    }
    return {'scenario_id': scenario_id.strip(), 'raceline_direction': raceline_direction,
            'sim_parameters': sim_parameters, 'assets': assets}


def pf_launch_arguments(localize_config, map_yaml, overlay):
    """Build the explicit PF launch contract for one selected track."""
    return ['localize_config:=' + str(Path(localize_config).expanduser().resolve()),
            'map_yaml:=' + str(Path(map_yaml).expanduser().resolve()),
            'parameter_overlay:=' + str(Path(overlay).expanduser().resolve())]


def pnc_launch_arguments(raceline_csv, raceline_direction, overlay):
    """Build the explicit PnC launch contract for one selected reference."""
    arguments = ['use_sim_time:=false',
                 'raceline_csv_path:=' + str(Path(raceline_csv).expanduser().resolve()),
                 'raceline_direction:=' + raceline_direction]
    arguments += [key + ':=' + str(Path(overlay).expanduser().resolve()) for key in (
        'path_platform_config', 'reactive_platform_config',
        'arbitration_platform_config', 'integration_platform_config')]
    return arguments


def resolve_raceline_input(manifest, csv_path, direction, map_yaml, allow_experimental=False):
    """Selection is resolved at orchestration only; controllers keep their API."""
    if manifest:
        if csv_path:
            raise ValueError('Use either a selected manifest or an explicit CSV, not both')
        from centerline_tools.raceline_selection import resolve_selection
        selection = resolve_selection(manifest, allow_experimental, map_yaml)
        if direction is not None and direction != selection['direction']:
            raise ValueError('Explicit direction disagrees with selected manifest')
        return selection['raceline_csv'], selection['direction'], selection
    if allow_experimental:
        raise ValueError('--allow-experimental requires --selected-raceline')
    if not csv_path or direction is None:
        raise ValueError('Explicit CSV requires --raceline-direction')
    return csv_path, direction, None


def preserve_raceline_selection(recording, selection):
    """Snapshot selected evidence; offline inspection does not require the session."""
    recording.metadata['selected_raceline'] = selection
    (recording.root / 'selected_raceline.yaml').write_bytes(Path(selection['manifest']).read_bytes())
    (recording.root / 'selected_raceline_validation.yaml').write_bytes(Path(selection['validation_report']).read_bytes())
    recording.save()


def speed_profile_overlay(evaluation_overlay, speed_profile):
    """Merge explicit speed limits without changing frames, health or GT isolation."""
    base = yaml.safe_load(Path(evaluation_overlay).read_text())
    profile = yaml.safe_load(Path(speed_profile).read_text())
    expected = {'/**': {'command_speed_min_mps', 'command_speed_max_mps'},
                'lower_safety_controller': {'absolute_speed_limit_mps'}}
    if not isinstance(profile, dict) or set(profile) != set(expected):
        raise ValueError('Speed profile requires shared command limits and lower-safety cap')
    for node, keys in expected.items():
        block = profile[node]
        if not isinstance(block, dict) or set(block) != {'ros__parameters'}:
            raise ValueError('Invalid speed profile node: ' + node)
        params = block['ros__parameters']
        if not isinstance(params, dict) or set(params) != keys:
            raise ValueError('Speed profile may contain only declared speed limits')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) for v in params.values()):
            raise ValueError('Speed limits must be finite numbers')
    limits = profile['/**']['ros__parameters']
    cap = profile['lower_safety_controller']['ros__parameters']['absolute_speed_limit_mps']
    if not 0 < limits['command_speed_min_mps'] <= limits['command_speed_max_mps'] == cap:
        raise ValueError('Require positive min <= max and identical final speed cap')
    for node, block in profile.items():
        base.setdefault(node, {'ros__parameters': {}})['ros__parameters'].update(
            block['ros__parameters'])
    return base


def initial_pose(config, offsets):
    values = [float(config[key]) + offset for key, offset in
              zip(('sx', 'sy', 'stheta'), offsets)]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Initial pose and offsets must be finite')
    return values


def timing_distribution(values):
    if not values:
        return None
    ordered = sorted(values)

    def percentile(p):
        location = (len(ordered) - 1) * p
        low = int(location)
        high = min(low + 1, len(ordered) - 1)
        return ordered[low] + (ordered[high] - ordered[low]) * (location - low)

    return {'p50': percentile(.50), 'p95': percentile(.95), 'max': ordered[-1]}


def summarize_observer_timing(samples, duration_sec):
    arrivals = [row[0] for row in samples]
    gaps = [(right - left) / 1000000 for left, right in zip(arrivals, arrivals[1:])]
    ages = [row[1] for row in samples if row[1] is not None]
    median_gap = statistics.median(gaps) if gaps else None
    return {'count': len(samples), 'rate_hz': len(samples) / duration_sec,
            'interarrival_ms': timing_distribution(gaps),
            'interarrival_jitter_ms': timing_distribution(
                [abs(gap - median_gap) for gap in gaps]),
            'source_to_observer_age_ms': timing_distribution(ages),
            'source_age_samples': len(ages)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario-id', required=True)
    parser.add_argument('--sim-config', required=True)
    parser.add_argument('--pf-map-yaml', required=True)
    raceline = parser.add_mutually_exclusive_group(required=True)
    raceline.add_argument('--raceline-csv')
    raceline.add_argument('--selected-raceline', help='Verified workbench session manifest; map remains explicit')
    parser.add_argument('--allow-experimental', action='store_true', help='Acknowledge failed production gates for explicit simulation experimentation')
    parser.add_argument('--raceline-direction',
                        choices=('csv', 'normal', 'reverse'))
    parser.add_argument('--localize-config', required=True)
    parser.add_argument('--speed-profile', help='Optional explicit shared command range and final safety cap YAML')
    parser.add_argument('--run-duration-sec', type=float, default=15)
    parser.add_argument('--with-rviz', action='store_true',
                        help='Start interactive RViz; automated runs are headless by default')
    parser.add_argument('--startup-timeout-sec', type=float, default=60)
    parser.add_argument('--motion-timeout-sec', type=float, default=15)
    parser.add_argument('--x-offset-m', type=float, default=0)
    parser.add_argument('--y-offset-m', type=float, default=0)
    parser.add_argument('--yaw-offset-rad', type=float, default=0)
    parser.add_argument('--ros-domain-id', type=int, default=94)
    parser.add_argument('--result-json', required=True)
    parser.add_argument('--record-dir', help='New directory for an optional preserved rosbag run')
    parser.add_argument('--source-revision', default='')
    parser.add_argument('--pf-revision', default='')
    parser.add_argument('--source-state', choices=('clean', 'dirty', 'unknown'), default='unknown')
    args = parser.parse_args()
    try:
        args.raceline_csv, args.raceline_direction, selection = resolve_raceline_input(
            args.selected_raceline, args.raceline_csv, args.raceline_direction,
            args.pf_map_yaml, args.allow_experimental)
    except ValueError as error:
        parser.error(str(error))
    for value in (args.run_duration_sec, args.startup_timeout_sec, args.motion_timeout_sec):
        if not math.isfinite(value) or value <= 0:
            parser.error('Durations must be finite and positive')
    os.environ['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
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

    track = validate_track_inputs(
        args.scenario_id, args.sim_config, args.pf_map_yaml, args.raceline_csv,
        args.raceline_direction, args.localize_config)
    config = track['sim_parameters']
    pose = initial_pose(config, (args.x_offset_m, args.y_offset_m, args.yaw_offset_rad))
    overlay = str(Path(get_package_share_directory('oudtra_driver_bringup')) /
                  'config/localization_eval_sim.yaml')
    overlay_workspace = None
    speed_profile = None
    base_overlay = overlay
    if args.speed_profile:
        speed_profile = Path(args.speed_profile).expanduser().resolve()
        merged = speed_profile_overlay(base_overlay, speed_profile)
        overlay_workspace = tempfile.TemporaryDirectory(prefix='localization_speed_profile_')
        overlay = str(Path(overlay_workspace.name) / 'evaluation_with_speed_limits.yaml')
        Path(overlay).write_text(yaml.safe_dump(merged, sort_keys=False))
    recording = None
    if args.record_dir:
        from localization_recording import RunRecording
        share = Path(get_package_share_directory('oudtra_driver_bringup'))
        configs = {'simulator': args.sim_config, 'evaluation_sim': overlay,
                   'recording': share / 'config/localization_recording.yaml',
                   'integration': share / 'config/full_stack.yaml',
                   'particle_filter': args.localize_config,
                   'pf_map_yaml': args.pf_map_yaml,
                   'pf_map_image': track['assets']['pf_map_image']['path'],
                   'map_yaml': track['assets']['simulator_map_yaml']['path'],
                   'map_image': track['assets']['simulator_map_image']['path'],
                   'reference_csv': args.raceline_csv}
        if speed_profile:
            configs['speed_profile'] = speed_profile
            configs['evaluation_sim_base'] = base_overlay
        for package, filename in [('path_following_v2', 'path_following_v2.yaml'),
                                  ('reactive_control_v2', 'reactive_control_v2.yaml'),
                                  ('drive_arbitration_v2', 'drive_arbitration_v2.yaml'),
                                  ('centerline_tools', 'raceline_publisher.yaml')]:
            configs[package] = Path(get_package_share_directory(package)) / 'config' / filename
        recording = RunRecording(args.record_dir, configs['recording'], args, configs)
        recording.metadata['initial_pose'] = pose
        recording.metadata['visualization_mode'] = 'rviz' if args.with_rviz else 'headless'
        recording.metadata['scenario'] = track['scenario_id']
        recording.metadata['track'] = {
            'scenario_id': track['scenario_id'],
            'raceline_direction': track['raceline_direction'],
            'assets': track['assets']}
        recording.metadata['comparison_identity'] = {
            'scenario_id': track['scenario_id'],
            'reference_sha256': track['assets']['raceline_csv']['sha256'],
            'algorithm_id': 'particle_filter',
            'algorithm_config_sha256': track['assets']['localize_config']['sha256']}
        if selection:
            preserve_raceline_selection(recording, selection)
    logdir = recording.root / 'logs' if recording else Path(tempfile.mkdtemp(prefix='localization_smoke_'))
    result = {'result': 'FAIL', 'log_directory': str(logdir), 'states': [], 'commands': [],
              'visualization_mode': 'rviz' if args.with_rviz else 'headless',
              'track': {'scenario_id': track['scenario_id'],
                        'raceline_direction': track['raceline_direction'],
                        'assets': track['assets']}}
    if selection:
        result['selected_raceline'] = selection
    processes = {}
    last, received, counts = {}, {}, {}
    timing_samples = {}
    evaluation_running = False
    evaluation_end_marked = False
    rclpy.init()
    node = rclpy.create_node('localization_smoke')
    buffer = Buffer()
    listener = TransformListener(buffer, node)  # noqa: F841 - retain TF subscriptions for the run
    subscriptions = []
    parameter_clients = []  # Keep service endpoints alive through Foxy response handling.
    def receive(topic, message):
        last[topic] = message
        received[topic] = time.monotonic()
        counts[topic] = counts.get(topic, 0) + 1
        if evaluation_running and hasattr(message, 'header'):
            stamp = message.header.stamp
            source_ns = stamp.sec * 1000000000 + stamp.nanosec
            age_ms = (time.time_ns() - source_ns) / 1000000 if source_ns > 0 else None
            timing_samples.setdefault(topic, []).append((time.monotonic_ns(), age_ms))
        if (recording and topic == '/ego_racecar/odom' and moving() and
                not any(p['state'] == 'MOTION_OBSERVED' for p in recording.metadata['phases'])):
            recording.mark('MOTION_OBSERVED', node)
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

    def mark(state):
        result['states'].append(state)
        if recording:
            recording.mark(state, node)

    def check_processes():
        if recording:
            recording.check()
        for name, (process, log, path) in processes.items():
            if process.poll() is not None:
                raise RuntimeError(name + ' launch exited before STOP')
            # Launch parents can survive failed child nodes. RViz is visualization only.
            for line in path.read_text(errors='replace').splitlines():
                if 'process has died' in line and 'rviz' not in line.lower():
                    raise RuntimeError('Critical child exited: ' + line)

    def wait(state, predicate, timeout=None):
        print(state, flush=True)
        mark(state)
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
        parameter_clients.append(client)
        wait('PARAMETERS ' + name, client.service_is_ready)
        request = GetParameters.Request()
        request.names = names
        future = client.call_async(request)
        wait('PARAMETER_RESPONSE ' + name, future.done)
        values = future.result().values
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
        if recording:
            recording.start()
        launch('sim', 'f1tenth_gym_ros', 'gym_bridge_slam_launch.py',
               ['config_file:=' + str(Path(args.sim_config).resolve()),
                'use_rviz:=' + ('true' if args.with_rviz else 'false')])
        wait('WAIT_SCAN', lambda: fresh('/scan') and fresh('/ego_racecar/odom') and
             fresh('/simulator/agent_status'))
        if node.count_publishers('/scan') != 1 or node.count_publishers('/drive') != 1:
            # This observer owns the sole (zero-only) drive publisher until PnC starts.
            raise RuntimeError('ROS domain is not exclusive; use an unused --ros-domain-id')
        launch('pf', 'particle_filter', 'localize_sim_launch.py',
               pf_launch_arguments(args.localize_config, args.pf_map_yaml, overlay))
        wait('WAIT_PF', lambda: fresh('/pf/pose/odom') and initpub.get_subscription_count() > 0)
        loaded_map = parameters('map_server', ['yaml_filename'])[0].string_value
        if Path(loaded_map).resolve() != Path(args.pf_map_yaml).expanduser().resolve():
            raise RuntimeError('PF map server did not load the explicitly selected map')
        result['runtime_pf_map_yaml'] = loaded_map
        if recording:
            wait('WAIT_RECORDING_INPUTS', lambda: recording.ready(
                ['estimated_pose', 'source_pose', 'health', 'truth', 'raw_odometry']))
        grace = parameters('particle_filter', ['manual_reset_grace_updates'])[0].integer_value
        before = counts.get('/pf/health', 0)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
        msg.pose.pose.orientation.z = math.sin(pose[2]/2)
        msg.pose.pose.orientation.w = math.cos(pose[2]/2)
        mark('SEND_INITIAL_POSE')
        initpub.publish(msg)
        result['initial_pose'] = pose
        wait('WAIT_LOCALIZATION', lambda: counts.get('/pf/health', 0) > before + grace + 5 and localized())
        mark('LOCALIZATION_READY')
        start_xy = truth_xy()
        mark('START_PNC')
        launch('pnc', 'oudtra_driver_bringup', 'full_stack_sim_launch.py',
               pnc_launch_arguments(args.raceline_csv, args.raceline_direction, overlay))
        values = parameters('lower_safety_controller',
                            ['enable_sim_reverse_swept_gate', 'odom_topic'])
        if values[0].bool_value or values[1].string_value != '/pf/pose/odom':
            raise RuntimeError('Lower safety evaluation profile not applied')
        if not parameters('drive_arbitrator', ['require_pf_health'])[0].bool_value:
            raise RuntimeError('PF health gate not enabled')
        if recording:
            wait('WAIT_RECORDING_TOPICS', lambda: recording.ready(recording.contract['roles']))
        consumers = [e.node_name for e in node.get_subscriptions_info_by_topic('/simulator/agent_status')]
        result['gt_subscribers'] = consumers
        result['effective_parameters'] = {
            'enable_sim_reverse_swept_gate': values[0].bool_value,
            'lower_odom_topic': values[1].string_value, 'require_pf_health': True}
        if speed_profile:
            expected_limits = yaml.safe_load(speed_profile.read_text())['/**']['ros__parameters']
            result['effective_parameters']['speed_limits'] = {}
            for target in ('path_generator', 'local_trajectory_planner', 'path_following_v2'):
                actual = parameters(target, ['command_speed_min_mps', 'command_speed_max_mps'])
                values = [value.double_value for value in actual]
                if values != [expected_limits['command_speed_min_mps'], expected_limits['command_speed_max_mps']]:
                    raise RuntimeError('Speed profile not applied to ' + target)
                result['effective_parameters']['speed_limits'][target] = values
            cap = parameters('lower_safety_controller', ['absolute_speed_limit_mps'])[0].double_value
            if cap != expected_limits['command_speed_max_mps']:
                raise RuntimeError('Final command speed cap not applied')
            result['effective_parameters']['absolute_speed_limit_mps'] = cap
        allowed_gt_readers = {node.get_name()}
        if recording:
            recording.check()
            allowed_gt_readers.add('_ros2cli_rosbag2')  # Verified Foxy recorder node name.
        if any(name not in allowed_gt_readers for name in consumers):
            raise RuntimeError('Unexpected GT consumer: ' + str(consumers))
        wait('WAIT_MOTION', lambda: moving() and math.dist(start_xy, truth_xy()) > .2,
             args.motion_timeout_sec)
        mark('RUNNING')
        start = time.monotonic()
        evaluation_running = True
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
        evaluation_running = False
        if recording:
            recording.mark('EVALUATION_END', node)
            evaluation_end_marked = True
        result['observer_timing'] = {
            topic: summarize_observer_timing(timing_samples.get(topic, []),
                                             result['motion_duration_sec'])
            for topic in ('/scan', '/ego_racecar/odom', '/pf/pose/odom', '/drive',
                          '/simulator/agent_status')}
        result['running_rates_hz'] = {topic: (count - running_counts.get(topic, 0)) /
                                      result['motion_duration_sec']
                                      for topic, count in counts.items()}
        result['displacement_m'] = math.dist(start_xy, truth_xy())
        result['result'] = 'PASS'
    except (Exception, KeyboardInterrupt) as error:
        result['reason'] = str(error) or 'Interrupted'
    finally:
        # A recording I/O failure must never bypass vehicle/process shutdown.
        try:
            if (recording and not evaluation_end_marked and
                    any(p['state'] == 'RUNNING' for p in recording.metadata['phases'])):
                recording.mark('EVALUATION_END', node)
            mark('STOP')
        except Exception as error:
            result['recording_error'] = str(error)
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
        if recording:
            try:
                recording.mark('CLEANUP_COMPLETE', node)
            except Exception as error:
                result['recording_error'] = str(error)
            try:
                finalized = recording.stop(dict(result))
                result['recording_passed'] = finalized and 'recording_error' not in result
                (recording.root / 'smoke_result.json').write_text(json.dumps(result, indent=2) + '\n')
            except Exception as error:
                result['recording_passed'] = False
                result['recording_error'] = str(error)
        node.destroy_node()
        rclpy.shutdown()
        Path(args.result_json).write_text(json.dumps(result, indent=2) + '\n')
        if overlay_workspace:
            overlay_workspace.cleanup()
        print(json.dumps(result, indent=2), flush=True)
    return 0 if result['result'] == 'PASS' and result.get('recording_passed', True) else 1


if __name__ == '__main__':
    raise SystemExit(main())
