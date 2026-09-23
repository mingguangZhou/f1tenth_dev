#!/usr/bin/env python3
"""Passively record a bounded onboard localization/PnC evaluation interval."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml

from recording_support import (asset_record, atomic_yaml, classify_preflight, create_run_directory,
                               file_sha256, git_identity, load_contract,
                               stop_rosbag)


def parse_named_path(value):
    if '=' not in value or not value.split('=', 1)[0]:
        raise argparse.ArgumentTypeError('expected LABEL=PATH')
    return value.split('=', 1)


def qos_profile(spec):
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    return QoSProfile(
        depth=int(spec.get('depth', 10)),
        reliability=(QoSReliabilityPolicy.RELIABLE if spec.get('reliability') == 'reliable'
                     else QoSReliabilityPolicy.BEST_EFFORT),
        durability=(QoSDurabilityPolicy.TRANSIENT_LOCAL if spec.get('durability') == 'transient_local'
                    else QoSDurabilityPolicy.VOLATILE),
        history=(QoSHistoryPolicy.KEEP_ALL if spec.get('history') == 'keep_all'
                 else QoSHistoryPolicy.KEEP_LAST))


def observe_message(message):
    result = {}
    header = getattr(message, 'header', None)
    if header is not None and getattr(header, 'frame_id', ''):
        result['frame_id'] = header.frame_id
    child = getattr(message, 'child_frame_id', '')
    if child:
        result['child_frame_id'] = child
    if hasattr(message, 'transforms'):
        result['tf_frames'] = sorted(set(
            '{} -> {}'.format(item.header.frame_id, item.child_frame_id)
            for item in message.transforms))
    if hasattr(message, 'status'):
        result['diagnostic_names'] = sorted(set(item.name for item in message.status))
    return result


def probe_graph(node, expected, qos_overrides, timeout_sec):
    from rosidl_runtime_py.utilities import get_message
    observations, subscriptions = {}, []

    def receive(topic, message):
        row = observations.setdefault(topic, {'message_count': 0})
        row['message_count'] += 1
        row.update(observe_message(message))

    for topic, spec in expected.items():
        subscriptions.append(node.create_subscription(
            get_message(spec['type']), topic,
            lambda message, selected=topic: receive(selected, message),
            qos_profile(qos_overrides.get(topic, {}))))
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        import rclpy
        rclpy.spin_once(node, timeout_sec=.05)
        graph = {}
        for topic in expected:
            publishers = node.get_publishers_info_by_topic(topic)
            graph[topic] = {
                'types': sorted(set(endpoint.topic_type for endpoint in publishers)),
                'publishers': len(publishers)}
        result = classify_preflight(expected, graph, observations)
        if not result['required_failures']:
            return result
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', nargs='?', help='new output directory; omitted with --preflight-only')
    parser.add_argument('--contract', default='')
    parser.add_argument('--preflight-timeout-sec', type=float, default=10.)
    parser.add_argument('--preflight-only', action='store_true')
    parser.add_argument('--preflight-json', default='')
    parser.add_argument('--duration-sec', type=float, help='end automatically after this evaluation duration')
    parser.add_argument('--post-roll-sec', type=float, default=.5,
                        help='continue passive recording after evaluation end (default: 0.5)')
    parser.add_argument('--main-repo', default='/f1tenth_ws/src/f1tenth_dev')
    parser.add_argument('--particle-filter-repo', default='/f1tenth_ws/src/f1tenth_dev/particle_filter')
    parser.add_argument('--system-repo', default='/f1tenth_ws/src/f1tenth_system')
    parser.add_argument('--map-yaml', default='')
    parser.add_argument('--map-image', default='')
    parser.add_argument('--raceline-csv', default='')
    parser.add_argument('--config', action='append', default=[], type=parse_named_path,
                        metavar='LABEL=PATH')
    args = parser.parse_args()
    if (args.preflight_timeout_sec <= 0 or args.post_roll_sec < 0 or
            args.duration_sec is not None and args.duration_sec <= 0):
        parser.error('timeouts/durations must be positive')
    if not args.preflight_only and not args.run_directory:
        parser.error('run_directory is required unless --preflight-only is used')
    if args.preflight_only and args.run_directory:
        parser.error('do not supply run_directory with --preflight-only')

    if args.contract:
        contract_path = Path(args.contract)
    else:
        from ament_index_python.packages import get_package_share_directory
        contract_path = (Path(get_package_share_directory('oudtra_driver_bringup')) /
                         'config/onboard_localization_recording.yaml')
    contract, topics = load_contract(contract_path)

    if args.run_directory:
        try:
            root = create_run_directory(args.run_directory)
        except FileExistsError:
            print('Refusing to overwrite existing run directory: ' +
                  str(Path(args.run_directory).expanduser().resolve()), file=sys.stderr)
            return 2
    else:
        root = None

    import rclpy
    rclpy.init()
    node = rclpy.create_node('onboard_localization_recorder')
    process = log = None
    metadata = None
    exit_code = 1
    try:
        preflight = probe_graph(node, topics, contract.get('qos', {}),
                                args.preflight_timeout_sec)
        print(json.dumps(preflight, indent=2, sort_keys=True), flush=True)
        if args.preflight_json:
            Path(args.preflight_json).write_text(json.dumps(preflight, indent=2, sort_keys=True) + '\n')
        if args.preflight_only:
            return 0 if preflight['status'] == 'PASS' else 2

        now = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        repos = {name: git_identity(path) for name, path in {
            'main': args.main_repo, 'particle_filter': args.particle_filter_repo,
            'f1tenth_system': args.system_repo}.items()}
        assets = {
            'map_yaml': asset_record(args.map_yaml),
            'map_image': asset_record(args.map_image),
            'raceline_csv': asset_record(args.raceline_csv),
        }
        configs = {name: asset_record(path) for name, path in args.config}
        contract_copy = root / 'config/onboard_localization_recording.yaml'
        contract_copy.write_bytes(contract_path.read_bytes())
        analysis_path = contract_path.with_name('evaluation_analysis.yaml')
        if analysis_path.is_file():
            analysis = yaml.safe_load(analysis_path.read_text())
            analysis_copy = root / 'config/evaluation_analysis.yaml'
            analysis_copy.write_bytes(analysis_path.read_bytes())
            analysis_config = {
                'source': str(analysis_path), 'copy': str(analysis_copy.relative_to(root)),
                'sha256': file_sha256(analysis_path)}
        else:
            analysis, analysis_config = {}, {
                'source': str(analysis_path), 'copy': None, 'sha256': None,
                'state': 'unknown', 'reason': 'analysis settings file not found'}
        metadata = {
            'schema_version': 2, 'run_id': root.name, 'platform': 'onboard',
            'created_utc': now,
            'description': 'Passive onboard PF and PnC evidence recording; no ground truth',
            'clock': 'ROS system/wall time; interval duration checked against monotonic time',
            'main_revision': repos['main']['revision'],
            'pf_revision': repos['particle_filter']['revision'],
            'f1tenth_system_revision': repos['f1tenth_system']['revision'],
            'source_repositories': repos,
            'roles': contract['roles'],
            'analysis': analysis,
            'analysis_config': analysis_config,
            'recording_contract': {
                'source': str(contract_path), 'copy': str(contract_copy.relative_to(root)),
                'sha256': file_sha256(contract_path)},
            'preflight': preflight, 'assets': assets, 'configs': configs,
            'software_sha256': {
                'record_onboard_localization.py': file_sha256(__file__),
                'recording_support.py': file_sha256(Path(__file__).with_name('recording_support.py'))},
            'phases': [], 'recording': {'status': 'INCOMPLETE'},
        }
        atomic_yaml(root / 'metadata.yaml', metadata)
        if preflight['status'] != 'PASS':
            metadata['recording'].update(status='PREFLIGHT_FAILED',
                                         reason='required onboard evidence did not pass preflight')
            atomic_yaml(root / 'metadata.yaml', metadata)
            return 2

        qos_path = root / 'config/recorder_qos.yaml'
        qos_path.write_text(yaml.safe_dump(contract.get('qos', {}), sort_keys=False))
        command = ['ros2', 'bag', 'record', '-o', str(root / 'rosbag'),
                   '--qos-profile-overrides-path', str(qos_path)] + sorted(topics)
        metadata['recording']['command'] = command
        log = (root / 'logs/recorder.log').open('w')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        metadata['recording']['started_wall_ns'] = time.time_ns()
        atomic_yaml(root / 'metadata.yaml', metadata)
        time.sleep(.5)
        if process.poll() is not None:
            raise RuntimeError('rosbag exited during startup; inspect logs/recorder.log')

        input('Recording startup evidence. Press Enter to mark EVALUATION_START... ')
        start_wall, start_mono = time.time_ns(), time.monotonic_ns()
        metadata['phases'].append({'state': 'RUNNING', 'wall_ns': start_wall,
                                   'monotonic_ns': start_mono})
        atomic_yaml(root / 'metadata.yaml', metadata)
        print('Evaluation active. Stop safely with Ctrl-C.', flush=True)
        try:
            if args.duration_sec is None:
                while True:
                    time.sleep(.2)
            else:
                deadline = time.monotonic() + args.duration_sec
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError('rosbag exited during evaluation')
                    time.sleep(min(.2, max(0., deadline-time.monotonic())))
        except KeyboardInterrupt:
            pass
        end_wall, end_mono = time.time_ns(), time.monotonic_ns()
        metadata['phases'].append({'state': 'EVALUATION_END', 'wall_ns': end_wall,
                                   'monotonic_ns': end_mono})
        metadata['interval'] = {'start_wall_ns': start_wall, 'end_wall_ns': end_wall,
                                'duration_sec': (end_mono-start_mono)*1e-9}
        atomic_yaml(root / 'metadata.yaml', metadata)
        post_roll_deadline = time.monotonic() + args.post_roll_sec
        while time.monotonic() < post_roll_deadline:
            if process.poll() is not None:
                raise RuntimeError('rosbag exited during post-roll')
            time.sleep(min(.05, max(0., post_roll_deadline-time.monotonic())))
        code, counts = stop_rosbag(process, root / 'rosbag')
        process = None
        missing_required = [topic for topic, spec in topics.items()
                            if spec['required'] and counts.get(topic, 0) < 1]
        metadata['recording'].update(return_code=code, message_counts=counts,
                                     status='PASS' if not missing_required else 'FAIL')
        if missing_required:
            metadata['recording']['reason'] = 'bag has no messages for: ' + ', '.join(missing_required)
        else:
            metadata['outcome'] = {
                'completed': True,
                'meaning': 'operator-defined evaluation interval ended and bag finalized'}
        atomic_yaml(root / 'metadata.yaml', metadata)
        exit_code = 0 if not missing_required else 1
        print('Recording {}: {}'.format(metadata['recording']['status'], root), flush=True)
    except KeyboardInterrupt:
        if metadata is not None:
            metadata['recording'].update(status='INTERRUPTED_BEFORE_EVALUATION_END')
            atomic_yaml(root / 'metadata.yaml', metadata)
    except Exception as error:
        print('Recorder failure: ' + str(error), file=sys.stderr, flush=True)
        if metadata is not None:
            metadata['recording'].update(status='FAIL', reason=str(error))
            atomic_yaml(root / 'metadata.yaml', metadata)
    finally:
        if process is not None:
            try:
                code, counts = stop_rosbag(process, root / 'rosbag')
                if metadata is not None:
                    metadata['recording'].update(return_code=code, message_counts=counts)
                    atomic_yaml(root / 'metadata.yaml', metadata)
            except Exception as error:
                if metadata is not None:
                    metadata['recording'].update(status='FAIL', reason=str(error))
                    atomic_yaml(root / 'metadata.yaml', metadata)
        if log:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
