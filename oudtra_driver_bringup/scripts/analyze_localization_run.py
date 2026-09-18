#!/usr/bin/env python3
"""Analyze a completed Foxy SQLite/CDR simulation bag without ROS replay."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
import yaml

VERSION = 1


def wrapped(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def quantiles(values):
    if not len(values):
        return None
    return dict(zip(('p50', 'p95', 'max'), map(float, [
        np.percentile(values, 50), np.percentile(values, 95), np.max(values)])))


def coverage(intervals, start, end):
    """Union length, clipped to the evaluation window (seconds)."""
    total, cursor = 0., start
    for left, right in sorted(intervals):
        left, right = max(start, left, cursor), min(end, right)
        if right > left:
            total += right - left
            cursor = right
    return total


def interpolate(truth, times, max_gap):
    """XY plus shortest-arc yaw; exact matches valid, no gap bridging/extrapolation."""
    output = np.full((len(times), 3), np.nan)
    for row, timestamp in enumerate(times):
        index = np.searchsorted(truth[:, 0], timestamp)
        if index < len(truth) and truth[index, 0] == timestamp:
            output[row] = truth[index, 1:4]
        elif 0 < index < len(truth):
            before, after = truth[index-1], truth[index]
            gap = after[0] - before[0]
            if 0 < gap <= max_gap:
                fraction = (timestamp - before[0]) / gap
                output[row, :2] = before[1:3] + fraction * (after[1:3] - before[1:3])
                output[row, 2] = before[3] + fraction * wrapped(after[3] - before[3])
    return output


def segment_distances(points, reference):
    starts, vectors = reference[:-1], np.diff(reference, axis=0)
    lengths = np.sum(vectors*vectors, axis=1)
    keep = lengths > 1e-12
    starts, vectors, lengths = starts[keep], vectors[keep], lengths[keep]
    if not len(starts):
        raise ValueError('Reference has no nondegenerate segments')
    result = []
    for point in points:
        fractions = np.clip(np.sum((point - starts)*vectors, axis=1)/lengths, 0, 1)
        closest = starts + fractions[:, None]*vectors
        result.append(np.min(np.linalg.norm(closest-point, axis=1)))
    return np.asarray(result)


def emergency_events(samples, start, end, max_gap):
    relevant = [(t, mode) for t, mode in samples if start <= t < end]
    predecessors = [(t, mode) for t, mode in samples if t < start]
    previous = predecessors[-1] if predecessors and start-predecessors[-1][0] <= max_gap else None
    active_at_start = previous[1] == 'EMERGENCY_STOP' if previous else None
    count, uncertain = 0, 0
    for timestamp, mode in relevant:
        if mode == 'EMERGENCY_STOP':
            if previous is None or timestamp-previous[0] > max_gap:
                uncertain += 1
            elif previous[1] != 'EMERGENCY_STOP':
                count += 1
        previous = (timestamp, mode)
    return {'observed': any(mode == 'EMERGENCY_STOP' for _, mode in relevant) or active_at_start is True,
            'observed_entries': count, 'active_at_start': active_at_start,
            'uncertain_entries': uncertain}


def stamp(header):
    return header.stamp.sec * 1000000000 + header.stamp.nanosec


def yaw(quaternion):
    q = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    if not np.all(np.isfinite(q)) or abs(float(q @ q)-1) > .01:
        raise ValueError('Invalid pose quaternion')
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def read_bag(root, roles):
    """Decode only declared semantic roles; preserve source and bag timestamps."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    bag = root / 'rosbag'
    info = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    if info['storage_identifier'] != 'sqlite3' or info.get('compression_format'):
        raise ValueError('Expected uncompressed sqlite3/CDR bag')
    by_topic = {role['topic']: (name, role) for name, role in roles.items()}
    records = {name: [] for name in roles}
    for filename in info['relative_file_paths']:
        dbpath = (bag / filename).resolve()
        if bag.resolve() not in dbpath.parents or not dbpath.is_file():
            raise ValueError('Missing or invalid bag database path')
        with sqlite3.connect('file:' + str(dbpath) + '?mode=ro', uri=True) as db:
            if db.execute('PRAGMA integrity_check').fetchone()[0] != 'ok':
                raise ValueError('Invalid SQLite database')
            topics = {}
            for ident, name, typ, encoding in db.execute('SELECT id,name,type,serialization_format FROM topics'):
                if name in by_topic:
                    role, spec = by_topic[name]
                    if typ != spec['type'] or encoding != 'cdr':
                        raise ValueError('Unexpected topic type/encoding: ' + name)
                    topics[ident] = (role, get_message(typ))
            for ident, timestamp, payload in db.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp,id'):
                if ident in topics:
                    role, cls = topics[ident]
                    records[role].append((timestamp, deserialize_message(payload, cls)))
    for name, rows in records.items():
        rows.sort(key=lambda row: row[0])
        if not rows and name != 'reference':
            raise ValueError('Missing recorded role: ' + name)
    return records


def analyze(root, overrides=None):
    metadata = yaml.safe_load((root / 'metadata.yaml').read_text())
    if metadata.get('schema_version') != 1:
        raise ValueError('Unsupported metadata schema')
    phases = {p['state']: p for p in metadata['phases']}
    if 'RUNNING' not in phases or 'EVALUATION_END' not in phases:
        raise ValueError('No evaluation interval: RUNNING/EVALUATION_END missing')
    begin, finish = phases['RUNNING'], phases['EVALUATION_END']
    origin = begin['wall_ns']
    duration = (finish['wall_ns']-origin)*1e-9
    monotonic = (finish['monotonic_ns']-begin['monotonic_ns'])*1e-9
    if duration <= 0 or abs(duration-monotonic) > .05:
        raise ValueError('Invalid interval or wall/monotonic clock mismatch')
    settings = dict(metadata['analysis'])
    settings.update(overrides or {})
    gap, pose_age, status_gap = (float(settings[k]) for k in
                               ('max_gt_gap_sec', 'max_pose_age_sec', 'max_status_gap_sec'))
    if not all(math.isfinite(v) and v > 0 for v in (gap, pose_age, status_gap)):
        raise ValueError('Invalid analysis bounds')
    records = read_bag(root, metadata['roles'])
    relative = lambda ns: (ns-origin)*1e-9
    issues = []
    truth_rows, poses, safety, reference = [], [], [], None
    invalid = {'truth': 0, 'source_pose': 0, 'safety': 0}
    collisions = []
    for receive_ns, msg in records['truth']:
        try:
            status = next(s for s in msg.status if s.name == metadata['roles']['truth']['status'])
            fields = {v.key: v.value for v in status.values}
            row = [relative(stamp(msg.header))] + [float(fields[k]) for k in ('x_m', 'y_m', 'yaw_rad')]
            if not stamp(msg.header) or not np.all(np.isfinite(row)):
                raise ValueError('Invalid truth')
            truth_rows.append(row)
            value = fields['collision'].lower()
            if value not in ('true', 'false'):
                raise ValueError('Invalid collision flag')
            collisions.append((row[0], value == 'true'))
        except (ValueError, KeyError, StopIteration):
            invalid['truth'] += 1
    for receive_ns, msg in records['source_pose']:
        for transform in msg.transforms:
            spec = metadata['roles']['source_pose']
            if transform.header.frame_id != spec['parent'] or transform.child_frame_id != spec['child']:
                continue
            try:
                xyz = transform.transform.translation
                row = [relative(stamp(transform.header)), xyz.x, xyz.y,
                       yaw(transform.transform.rotation), relative(receive_ns)]
                if not stamp(transform.header) or not np.all(np.isfinite(row)):
                    raise ValueError('Invalid TF')
                poses.append(row)
            except ValueError:
                invalid['source_pose'] += 1
    def ordered_unique(rows, name):
        if not rows:
            raise ValueError('No finite ' + name)
        data = np.asarray(rows)
        if np.any(np.diff(data[:, 0]) < 0):
            raise ValueError(name + ' source clock reversed')
        keep = [0]
        for index in range(1, len(data)):
            if data[index, 0] == data[keep[-1], 0]:
                if not np.allclose(data[index, 1:4], data[keep[-1], 1:4], atol=1e-9, rtol=0):
                    raise ValueError(name + ' conflicting duplicate source stamp')
            else:
                keep.append(index)
        return data[keep]
    truth = ordered_unique(truth_rows, 'GT')
    pf = ordered_unique(poses, 'PF')
    # Same inferred pose is published on both interfaces, with different timestamps.
    tf_values = {tuple(np.round(row[1:4], 8)) for row in pf}
    odom_valid, odom_matches = 0, 0
    for _, msg in records['estimated_pose']:
        try:
            position = msg.pose.pose.position
            values = (position.x, position.y, yaw(msg.pose.pose.orientation))
            if msg.header.frame_id != metadata['roles']['estimated_pose']['frame'] or not np.all(np.isfinite(values)):
                continue
            odom_valid += 1
            odom_matches += tuple(np.round(values, 8)) in tf_values
        except ValueError:
            continue
    if not odom_matches:
        raise ValueError('No agreement between public PF odometry and source-stamped PF TF')
    selected = pf[(pf[:, 0] >= 0) & (pf[:, 0] < duration)]
    aligned = interpolate(truth, selected[:, 0], gap)
    valid = np.all(np.isfinite(aligned), axis=1)
    if not np.any(valid):
        raise ValueError('No aligned PF/GT evaluation samples')
    paired, gt = selected[valid], aligned[valid]
    errors = np.linalg.norm(paired[:, 1:3]-gt[:, :2], axis=1)
    heading = np.abs(wrapped(paired[:, 3]-gt[:, 2]))
    intervals = [(row[4], row[0]+pose_age) for row in pf if row[0] <= row[4]]
    availability = coverage(intervals, 0, duration)/duration
    gt_intervals = [(a, b) for a, b in zip(truth[:-1, 0], truth[1:, 0]) if 0 < b-a <= gap]
    gt_coverage = coverage(gt_intervals, 0, duration)/duration
    inside = truth[(truth[:, 0] > 0) & (truth[:, 0] < duration)]
    boundaries = interpolate(truth, np.array([0., duration]), gap)
    vehicle = np.vstack(([0., *boundaries[0]], inside, [duration, *boundaries[1]]))
    usable_segments = (np.diff(vehicle[:, 0]) <= gap) & np.all(np.isfinite(vehicle[:-1, 1:3]), axis=1) & np.all(np.isfinite(vehicle[1:, 1:3]), axis=1)
    partial_distance = float(np.sum(np.linalg.norm(np.diff(vehicle[:, 1:3], axis=0)[usable_segments], axis=1)))
    complete_gt = gt_coverage >= 1-1e-6 and invalid['truth'] == 0
    distance = partial_distance if complete_gt else None
    collision = any(flag for t, flag in collisions if 0 <= t < duration)
    if not collision and not complete_gt:
        collision = None
    for _, msg in records['safety']:
        try:
            status = next(s for s in msg.status if s.name == metadata['roles']['safety']['status'])
            mode = {v.key: v.value for v in status.values}['mode']
            safety.append((relative(stamp(msg.header)), mode))
        except (StopIteration, KeyError):
            invalid['safety'] += 1
    safety.sort()
    events = emergency_events(safety, 0, duration, status_gap)
    safety_coverage = coverage([(t, t+status_gap) for t, _ in safety], 0, duration)/duration
    if safety_coverage < 1-1e-6 or invalid['safety']:
        if not events['observed']:
            events['observed'] = None
        issues.append('Safety status coverage incomplete; event count is an observed lower bound.')
    reference_reason = 'No recorded reference path'
    for _, msg in records['reference']:
        points = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        if msg.header.frame_id != metadata['roles']['reference']['frame'] or len(points) < 2 or not np.all(np.isfinite(points)):
            reference, reference_reason = None, 'Invalid reference frame or geometry'
            break
        if reference is not None and (points.shape != reference.shape or not np.array_equal(points, reference)):
            reference, reference_reason = None, 'Reference geometry changed during recording'
            break
        reference = points
    vehicle_samples = truth[(truth[:, 0] >= 0) & (truth[:, 0] < duration)]
    cross_track = None
    if reference is not None:
        try:
            cross_track = quantiles(segment_distances(vehicle_samples[:, 1:3], reference))
        except ValueError as error:
            reference, reference_reason = None, str(error)
    if reference is None:
        issues.append('Cross-track unavailable: ' + reference_reason)
    if not complete_gt:
        issues.append('GT gaps/invalid samples: total distance and mean speed unavailable; partial distance retained.')
    if np.count_nonzero(~valid):
        issues.append('Some PF samples lack valid GT alignment; error quantiles describe accepted samples only.')
    issues += ['PF TF is odometry-source-stamped; latest-scan association is not synchronized.',
               'GT is latest Gym state stamped by publication time, not physics-step time.',
               'Health is unstamped; bag receive time is its only time basis.',
               'Quantiles are sample-weighted. Reference deviation is not active detour tracking error.',
               'Pose jumps deferred: no justified motion-compensated jump contract yet.']
    metrics = {
        'schema_version': 1, 'analyzer_version': VERSION, 'run_id': root.name,
        'analyzer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'analysis_status': 'PASS', 'recording_status': metadata['recording']['status'],
        'interval': {'start_wall_ns': origin, 'duration_sec': duration}, 'settings': settings,
        'localization': {'position_error_m': quantiles(errors), 'absolute_heading_error_rad': quantiles(heading),
                         'availability_percent': 100*availability, 'pose_jumps': None},
        'vehicle': {'completed': metadata['smoke_result']['result'] == 'PASS', 'collision_observed': collision,
                    'emergency_stops': events, 'distance_travelled_m': distance,
                    'mean_elapsed_time_speed_mps': distance/duration if distance is not None else None,
                    'static_reference_deviation_m': cross_track},
        'data_quality': {'message_counts': {k: len(v) for k, v in records.items()}, 'invalid': invalid,
                         'public_pf_poses_valid': odom_valid, 'public_pf_poses_matching_tf': odom_matches,
                         'pf_evaluation_samples': len(selected), 'aligned_samples': len(paired),
                         'gt_coverage_percent': gt_coverage*100, 'safety_coverage_percent': safety_coverage*100,
                         'partial_observed_distance_m': partial_distance,
                         'max_gt_gap_sec': float(np.max(np.diff(truth[:, 0]))) if len(truth)>1 else None,
                         'max_pf_source_gap_sec': float(np.max(np.diff(pf[:, 0]))) if len(pf)>1 else None},
        'limitations': issues,
    }
    plot_errors = np.full(len(selected), np.nan)
    plot_errors[valid] = errors
    write_outputs(root, metrics, paired, selected[:, 0], plot_errors, vehicle_samples, reference)
    return metrics


def write_outputs(root, metrics, paired, plot_times, errors, vehicle, reference):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plots = root / 'plots'
    plots.mkdir(exist_ok=True)
    def xy(filename, title, series):
        fig, ax = plt.subplots()
        for points, label in series:
            ax.plot(points[:, 0], points[:, 1], label=label)
        ax.set(title=title, xlabel='Map x [m]', ylabel='Map y [m]')
        ax.axis('equal')
        ax.legend()
        fig.tight_layout()
        fig.savefig(plots / filename, dpi=140)
        plt.close(fig)
    xy('trajectory_xy.png', 'Evaluation trajectory: simulator GT and PF',
       [(vehicle[:, 1:3], 'Simulator GT'), (paired[:, 1:3], 'PF')])
    fig, ax = plt.subplots()
    times, values = plot_times.tolist(), errors.tolist()
    for index in range(len(times)-1, 0, -1):
        if times[index]-times[index-1] > metrics['settings']['max_pose_age_sec']:
            times.insert(index, float('nan'))
            values.insert(index, float('nan'))
    ax.plot(times, values, label='PF vs aligned GT')
    ax.set(title='Localization position error during evaluation', xlabel='Elapsed evaluation time [s]', ylabel='Position error [m]')
    ax.set_xlim(0, metrics['interval']['duration_sec'])
    ax.legend()
    fig.tight_layout()
    fig.savefig(plots / 'position_error.png', dpi=140)
    plt.close(fig)
    if reference is not None:
        xy('reference_trajectory.png', 'Static raceline reference and true vehicle trajectory',
           [(reference, 'Reference raceline'), (vehicle[:, 1:3], 'Simulator GT')])
    elif (plots / 'reference_trajectory.png').exists():
        (plots / 'reference_trajectory.png').unlink()  # Remove only our stale generated optional plot.
    loc, car = metrics['localization'], metrics['vehicle']
    def display(value):
        if value is None:
            return 'Unavailable (see limitations)'
        if isinstance(value, bool):
            return 'Yes' if value else 'No'
        return '{:.4f}'.format(value)
    rows = []
    for title, values in [('Position error [m]', loc['position_error_m']),
                          ('Absolute heading error [rad]', loc['absolute_heading_error_rad']),
                          ('Static-reference deviation [m]', car['static_reference_deviation_m'])]:
        for key in ('p50', 'p95', 'max'):
            rows.append((title+' '+key, display(values[key] if values else None)))
    rows += [('Absolute heading error [deg] '+key, display(math.degrees(loc['absolute_heading_error_rad'][key])))
             for key in ('p50', 'p95', 'max')]
    rows += [('Temporal localization availability [%]', display(loc['availability_percent'])),
             ('Pose jumps', 'Unavailable: deferred'), ('Run completed', display(car['completed'])),
             ('Collision observed', display(car['collision_observed'])),
             ('Emergency stop observed', display(car['emergency_stops']['observed'])),
             ('Emergency-stop observed entries', str(car['emergency_stops']['observed_entries'])),
             ('Emergency-stop active at start', display(car['emergency_stops']['active_at_start'])),
             ('Emergency-stop uncertain entries', str(car['emergency_stops']['uncertain_entries'])),
             ('Distance travelled [m]', display(car['distance_travelled_m'])),
             ('Mean elapsed-time speed [m/s]', display(car['mean_elapsed_time_speed_mps']))]
    run_id = metrics.get('run_id', root.name)
    report = '# IFAC RoboRacer PF closed-loop localization baseline — {}\n\n## Summary\n\n'.format(run_id)
    report += ('Offline analysis completed for {:.3f} seconds of evaluation. Smoke completed: {}. '
               'Recording status: {}. Values are measurements, not performance PASS/FAIL thresholds.\n\n').format(
                   metrics['interval']['duration_sec'], display(car['completed']), metrics['recording_status'])
    report += '| Metric | Measured value |\n| --- | --- |\n'
    report += ''.join('| {} | {} |\n'.format(*row) for row in rows)
    quality = metrics['data_quality']
    report += '\n## Data quality and limits\n\n'
    report += 'Aligned PF samples: {}/{}. GT coverage: {:.3f}%; safety coverage: {:.3f}%.\n\n'.format(
        quality['aligned_samples'], quality['pf_evaluation_samples'], quality['gt_coverage_percent'], quality['safety_coverage_percent'])
    report += 'Analysis bounds: GT gap {max_gt_gap_sec} s; pose age {max_pose_age_sec} s; status gap {max_status_gap_sec} s.\n\n'.format(**metrics['settings'])
    report += ''.join('- '+issue+'\n' for issue in metrics['limitations'])
    report += '\n## Plots\n\n![GT and PF](plots/trajectory_xy.png)\n\n![Position error](plots/position_error.png)\n'
    if reference is not None:
        report += '\n![Reference and GT](plots/reference_trajectory.png)\n'
    (root / 'metrics.json').write_text(json.dumps(metrics, indent=2, sort_keys=True, allow_nan=False)+'\n')
    (root / 'report.md').write_text(report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    for name in ('max-gt-gap-sec', 'max-pose-age-sec', 'max-status-gap-sec'):
        parser.add_argument('--' + name, type=float)
    args = parser.parse_args()
    try:
        overrides = {key: value for key, value in vars(args).items()
                     if key != 'run_directory' and value is not None}
        metrics = analyze(args.run_directory.resolve(), overrides)
        print('ANALYSIS PASS: {} aligned samples'.format(metrics['data_quality']['aligned_samples']))
        return 0
    except Exception as error:
        print('ANALYSIS FAIL: ' + str(error))
        if args.run_directory.is_dir():
            (args.run_directory / 'metrics.json').write_text(json.dumps(
                {'analysis_status': 'FAIL', 'reason': str(error)}, indent=2)+'\n')
            (args.run_directory / 'report.md').write_text('# Analysis failed\n\n'+str(error)+'\n')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
