#!/usr/bin/env python3
"""Characterize wall-time message flow in one completed simulation rosbag."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

from analyze_localization_run import read_bag


TOPICS = {
    'bridge_status': ('/simulator/agent_status', 'diagnostic_msgs/msg/DiagnosticArray'),
    'raw_odometry': ('/ego_racecar/odom', 'nav_msgs/msg/Odometry'),
    'pf_pose': ('/pf/pose/odom', 'nav_msgs/msg/Odometry'),
    'drive': ('/drive', 'ackermann_msgs/msg/AckermannDriveStamped'),
    'tf': ('/tf', 'tf2_msgs/msg/TFMessage'),
}


def distribution(values):
    if not values:
        return None
    numbers = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(numbers)):
        raise ValueError('Nonfinite timing evidence')
    return {'p50': float(np.percentile(numbers, 50)),
            'p95': float(np.percentile(numbers, 95)),
            'max': float(np.max(numbers))}


def stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1000000000 + stamp.nanosec


def summarize_stream(rows, start_ns, end_ns, stamped=True):
    """Bag receive-time rate/jitter plus comparable wall source-stamp age."""
    samples = [(received, message) for received, message in rows
               if start_ns <= received < end_ns]
    arrivals = [received for received, _ in samples]
    gaps = [float(right - left) / 1000000 for left, right in zip(arrivals, arrivals[1:])]
    ages = []
    if stamped:
        for received, message in samples:
            source = stamp_ns(message)
            if source > 0:
                ages.append(float(received - source) / 1000000)
    return {'count': len(samples),
            'rate_hz': len(samples) / ((end_ns - start_ns) / 1000000000),
            'interarrival_ms': distribution(gaps),
            'interarrival_jitter_ms': distribution(
                [abs(gap - float(np.median(gaps))) for gap in gaps]) if gaps else None,
            'source_to_bag_age_ms': distribution(ages) if stamped else None,
            'source_age_samples': len(ages)}


def analyze(root):
    root = Path(root).resolve()
    metadata = yaml.safe_load((root / 'metadata.yaml').read_text())
    if metadata.get('platform') != 'sim' or 'wall' not in metadata.get('clock', ''):
        raise ValueError('Expected simulation metadata with a wall-clock contract')
    phases = {entry['state']: entry for entry in metadata['phases']}
    start, end = (phases[name]['wall_ns'] for name in ('RUNNING', 'EVALUATION_END'))
    if end <= start:
        raise ValueError('Invalid evaluation interval')
    roles = {name: {'topic': topic, 'type': kind} for name, (topic, kind) in TOPICS.items()}
    errors = {}
    records = read_bag(root, roles, errors)
    if errors:
        raise ValueError('Invalid timing topics: ' + str(errors))
    for name in ('bridge_status', 'raw_odometry', 'pf_pose', 'drive'):
        if not records[name]:
            raise ValueError('Missing timing topic: ' + TOPICS[name][0])
    streams = {name: summarize_stream(records[name], start, end)
               for name in ('bridge_status', 'raw_odometry', 'pf_pose', 'drive')}
    smoke_result = json.loads((root / 'smoke_result.json').read_text())
    scan = smoke_result.get('observer_timing', {}).get('/scan')
    if not scan or not scan.get('count'):
        raise ValueError('Missing passive scan observer timing in smoke_result.json')
    streams['scan'] = scan
    tf_rows = []
    for received, message in records['tf']:
        if start <= received < end:
            tf_rows.extend((received, transform) for transform in message.transforms
                           if transform.header.frame_id.lstrip('/') == 'map' and
                           transform.child_frame_id.lstrip('/') == 'ego_racecar/base_link')
    pf_input_age = []
    raw_arrival = {}
    for received, message in records['raw_odometry']:
        if start <= received < end:
            raw_arrival.setdefault(stamp_ns(message), received)
    pf_after_raw_bag = []
    for received, transform in tf_rows:
        source = stamp_ns(transform)
        if source > 0:
            pf_input_age.append((received - source) / 1000000)
            if source in raw_arrival:
                pf_after_raw_bag.append((received - raw_arrival[source]) / 1000000)
    return {
        'method': 'rosbag receive timestamps, plus passive runner /scan callback, within the wall-time evaluation interval',
        'run_id': metadata.get('run_id', root.name),
        'visualization_mode': metadata.get('visualization_mode', 'unknown'),
        'smoke_result': smoke_result.get('result', 'unknown'),
        'recording_status': metadata.get('recording', {}).get('status', 'unknown'),
        'track_hashes': {name: asset['sha256'] for name, asset in
                         metadata.get('track', {}).get('assets', {}).items()},
        'wall_duration_sec': (end - start) / 1000000000,
        'streams': streams,
        'pf_input_stamp_to_tf_bag_age_ms': distribution(pf_input_age),
        'pf_raw_bag_to_tf_bag_delay_ms': distribution(pf_after_raw_bag),
        'pf_raw_bag_to_tf_matched_samples': len(pf_after_raw_bag),
        'real_time_factor': None,
        'real_time_factor_reason': 'No qualified physics-time clock or step counter is recorded',
        'limitations': [
            'Bag receive time includes ROS transport and recorder scheduling, not display latency.',
            'Scan timing uses the runner callback because /scan is not part of the scorecard rosbag contract.',
            'PF TF carries the latest raw-odometry stamp; its age includes input queue, PF work and transport, not isolated PF compute time.',
            'Final drive header may carry an upstream sensor stamp, not command creation time.',
            'Bridge status is a publication heartbeat, not a physics-step counter.',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory')
    args = parser.parse_args()
    root = Path(args.run_directory).resolve()
    result = analyze(root)
    (root / 'timing.json').write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    print('TIMING PASS: {} ({:.3f} s)'.format(result['run_id'], result['wall_duration_sec']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
