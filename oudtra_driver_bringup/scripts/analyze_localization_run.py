#!/usr/bin/env python3
"""Analyze a completed Foxy SQLite/CDR run bag without ROS replay."""
import argparse
import hashlib
import json
import math
import re
from pathlib import Path
import sqlite3
import sys

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from evaluation_evidence import health_distribution, scalar_signal, imu_diagnostics

VERSION = 4


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


def same_frame(left, right):
    """Compare ROS frame IDs while tolerating the deprecated leading slash."""
    return isinstance(left, str) and isinstance(right, str) and left.lstrip('/') == right.lstrip('/')


# Legacy input adapters retain the original recording interface. New inputs use
# semantic role names directly and do not need PF-specific topics or TF.
ALIASES = {'health': 'localization_health', 'truth': 'ground_truth_pose',
           'reference': 'reference_path', 'command': 'final_drive_command',
           'safety': 'safety_status', 'arbitration': 'control_status'}


def normalize_roles(metadata):
    roles = {ALIASES.get(k, k): dict(v) for k, v in metadata['roles'].items()}
    legacy = metadata.get('schema_version') == 1
    if legacy:
        if 'localization_health' in roles:
            roles['localization_health'].setdefault('states', {'1': 'GOOD', '2': 'DEGRADED', '3': 'INVALID'})
        if 'control_status' in roles:
            roles['control_status'].setdefault('autonomous_states', ['RACELINE', 'REACTIVE'])
        if 'raw_odometry' in roles and metadata.get('platform', 'sim') == 'sim':
            roles.setdefault('vehicle_state', dict(roles['raw_odometry'],
                             velocity_frame='body', physical_time=False))
        if 'ground_truth_pose' in roles:
            roles.setdefault('collision_status', dict(roles['ground_truth_pose']))
    return roles


def read_bag(root, roles, errors=None):
    """Decode declared roles, allowing several roles on one topic; read-only."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    errors = errors if errors is not None else {}
    bag = root / 'rosbag'
    info = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    if info['storage_identifier'] != 'sqlite3' or info.get('compression_format'):
        raise ValueError('Expected uncompressed sqlite3/CDR bag')
    by_topic = {}
    for name, spec in roles.items():
        by_topic.setdefault(spec['topic'], []).append((name, spec))
    records = {name: [] for name in roles}
    for filename in info['relative_file_paths']:
        path = (bag / filename).resolve()
        if bag.resolve() not in path.parents or not path.is_file():
            raise ValueError('Missing or invalid bag database path')
        with sqlite3.connect('file:' + str(path) + '?mode=ro', uri=True) as db:
            if db.execute('PRAGMA integrity_check').fetchone()[0] != 'ok':
                raise ValueError('Invalid SQLite database')
            topics = {}
            for ident, name, typ, encoding in db.execute('SELECT id,name,type,serialization_format FROM topics'):
                matches = []
                for role, spec in by_topic.get(name, []):
                    if typ != spec['type'] or encoding != 'cdr':
                        errors[role] = 'Unexpected topic type/encoding: ' + name
                    else:
                        matches.append(role)
                if matches:
                    topics[ident] = (matches, get_message(typ))
            for ident, timestamp, payload in db.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp,id'):
                if ident not in topics:
                    continue
                names, cls = topics[ident]
                try:
                    msg = deserialize_message(payload, cls)
                    for name in names:
                        records[name].append((timestamp, msg))
                except Exception as error:
                    for name in names:
                        errors[name] = 'Malformed CDR: ' + str(error)
    for rows in records.values():
        rows.sort(key=lambda row: row[0])
    return records


def ordered(rows, columns):
    if not rows:
        return np.empty((0, columns))
    data = np.asarray(rows, dtype=float)
    if not np.all(np.isfinite(data)):
        raise ValueError('Nonfinite numeric sample')
    if np.any(np.diff(data[:, 0]) < 0):
        raise ValueError('Source clock reversed')
    keep = [0]
    for i in range(1, len(data)):
        if data[i, 0] == data[keep[-1], 0]:
            # Last column is receive time, which may differ on duplicate delivery.
            if not np.allclose(data[i, 1:-1], data[keep[-1], 1:-1], atol=1e-9, rtol=0):
                raise ValueError('Conflicting duplicate source timestamp')
        else:
            keep.append(i)
    return data[keep]


def diagnostic(msg, spec):
    status = next(s for s in msg.status if s.name == spec['status'])
    return {v.key: v.value for v in status.values}


def decode(records, roles, origin, errors, duration=None, command_history_sec=.2):
    """Normalize ROS-specific interfaces at the boundary, not inside metrics."""
    data = {}
    for role, spec in roles.items():
        rows = []
        try:
            for received, msg in records[role]:
                receive = (received-origin)*1e-9
                # The smoke runner's shutdown-only zero command has no source
                # stamp. It is outside evaluation and is not control evidence.
                if role in ('final_drive_command', 'autonomy_command') and duration is not None and not -command_history_sec <= receive < duration:
                    continue
                if role == 'reference_path':
                    if not same_frame(msg.header.frame_id, spec['frame']):
                        raise ValueError('Reference frame mismatch')
                    points = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
                    if len(points) < 2 or not np.all(np.isfinite(points)):
                        raise ValueError('Invalid reference geometry')
                    if rows and not np.array_equal(rows[0], points):
                        raise ValueError('Reference geometry changed')
                    rows = [points]
                    continue
                messages = msg.transforms if role == 'source_pose' else [msg]
                for item in messages:
                    if role == 'source_pose' and (not same_frame(item.header.frame_id, spec['parent']) or
                                                  not same_frame(item.child_frame_id, spec['child'])):
                        continue
                    if role == 'localization_health':
                        values = [receive, spec['states'][str(int(msg.data[spec.get('index', 0)]))]]
                        if not math.isfinite(msg.data[spec.get('index', 0)]) or msg.data[spec.get('index', 0)] != int(msg.data[spec.get('index', 0)]):
                            raise ValueError('Invalid health state')
                    else:
                        ns = stamp(item.header)
                        if ns <= 0:
                            raise ValueError('Missing source timestamp')
                        t = (ns-origin)*1e-9
                        if role in ('source_pose', 'estimated_pose', 'raw_odometry'):
                            if role == 'source_pose':
                                position, rotation = item.transform.translation, item.transform.rotation
                            else:
                                if spec.get('frame') and not same_frame(item.header.frame_id, spec['frame']):
                                    raise ValueError('Pose frame mismatch')
                                position, rotation = item.pose.pose.position, item.pose.pose.orientation
                            values = [t, position.x, position.y, yaw(rotation), receive]
                        elif role == 'ground_truth_pose':
                            if spec['type'] == 'nav_msgs/msg/Odometry':
                                if not same_frame(item.header.frame_id, spec['frame']):
                                    raise ValueError('GT pose frame mismatch')
                                p = item.pose.pose
                                values = [t, p.position.x, p.position.y, yaw(p.orientation), receive]
                            else:
                                fields = diagnostic(item, spec)
                                keys = spec.get('pose_keys', ['x_m', 'y_m', 'yaw_rad'])
                                values = [t] + [float(fields[k]) for k in keys] + [receive]
                        elif role in ('forward_speed', 'yaw_rate'):
                            if spec.get('frame') and not same_frame(item.header.frame_id, spec['frame']):
                                raise ValueError('Scalar response frame mismatch')
                            if role == 'forward_speed':
                                if item.child_frame_id != spec.get('body_frame'):
                                    raise ValueError('Forward-speed child/body frame mismatch')
                                value = item.twist.twist.linear.x
                            else:
                                value = item.angular_velocity.z
                            values = [t, value, receive]
                        elif role == 'vehicle_state':
                            if spec.get('frame') and not same_frame(item.header.frame_id, spec['frame']):
                                raise ValueError('Vehicle-state frame mismatch')
                            twist = item.twist.twist
                            values = [t, twist.linear.x, twist.linear.y, twist.angular.z, receive]
                        elif role in ('final_drive_command', 'autonomy_command'):
                            values = [t, item.drive.speed, item.drive.steering_angle, receive]
                        elif role in ('safety_status', 'control_status'):
                            values = [t, diagnostic(item, spec)[spec.get('state_key', 'mode')]]
                        elif role == 'collision_status':
                            flag = diagnostic(item, spec)[spec.get('collision_key', 'collision')].lower()
                            if flag not in ('true', 'false'):
                                raise ValueError('Invalid collision flag')
                            values = [t, flag == 'true']
                        else:
                            continue
                    rows.append(values)
            if role == 'reference_path':
                data[role] = rows[0] if rows else None
            elif role in ('localization_health', 'safety_status', 'control_status', 'collision_status'):
                if any(b[0] < a[0] for a, b in zip(rows, rows[1:])):
                    raise ValueError('Status source clock reversed')
                data[role] = rows
            else:
                data[role] = ordered(rows, 4 if role in ('final_drive_command', 'autonomy_command') else 3 if role in ('forward_speed', 'yaw_rate') else 5)
        except (ValueError, KeyError, IndexError, StopIteration, AttributeError, OverflowError) as error:
            errors[role] = 'Malformed ' + role + ': ' + str(error)
            data[role] = None
    return data


def merged_intervals(intervals, start, end):
    merged = []
    for a, b in sorted(intervals):
        a, b = max(start, a), min(end, b)
        if b <= a:
            continue
        if merged and a <= merged[-1][1] + 1e-12:
            merged[-1][1] = max(b, merged[-1][1])
        else:
            merged.append([a, b])
    return merged


def dropouts(intervals, duration):
    cursor, missing = 0., []
    for a, b in merged_intervals(intervals, 0, duration):
        if a-cursor > 1e-12:
            missing.append([cursor, a])
        cursor = b
    if duration-cursor > 1e-12:
        missing.append([cursor, duration])
    return missing


def state_summary(samples, duration, gap):
    durations, entries = {}, {}
    previous = None
    for i, (t, state) in enumerate(samples):
        end = min(t+gap, samples[i+1][0] if i+1 < len(samples) else duration, duration)
        span = max(0., end-max(0., t))
        if span > 0:
            durations[state] = durations.get(state, 0.) + span
        if 0 <= t < duration and previous and t-previous[0] <= gap and state != previous[1]:
            entries[state] = entries.get(state, 0) + 1
        previous = (t, state)
    return {'duration_sec': durations, 'observed_entries': entries,
            'unknown_duration_sec': max(0., duration-sum(durations.values()))}


def consistency(pose, odom, duration, settings):
    window = settings['consistency_window_sec']
    times = np.arange(window, duration, settings['analysis_grid_sec'])
    output = []
    for t in times:
        pair = []
        for series in (pose, odom):
            ends = interpolate(series, [t-window, t], settings['max_increment_gap_sec'])
            dx, dy = ends[1, :2]-ends[0, :2]
            angle = ends[0, 2]
            pair.append([math.cos(angle)*dx + math.sin(angle)*dy,
                         -math.sin(angle)*dx + math.cos(angle)*dy,
                         wrapped(ends[1, 2]-angle)])
        pair = np.asarray(pair)
        if np.all(np.isfinite(pair)):
            output.append([t, np.linalg.norm(pair[1, :2]-pair[0, :2]), abs(wrapped(pair[1, 2]-pair[0, 2]))])
        else:
            output.append([t, np.nan, np.nan])
    return np.asarray(output).reshape((-1, 3))


def windowed_dynamics(state, duration, settings):
    """Physical body velocities only; caller must verify frame/time provenance."""
    result = []
    half = settings['smoothness_window_sec']/2
    for t in np.arange(half, duration-half + 1e-9, settings['analysis_grid_sec']):
        samples = state[(state[:, 0] >= t-half-1e-9) & (state[:, 0] <= t+half+1e-9)]
        if (len(samples) < 5 or np.ptp(samples[:, 0]) < settings['smoothness_min_span_sec']-1e-9 or
                not samples[0, 0] < t < samples[-1, 0] or
                np.max(np.diff(samples[:, 0])) > settings['smoothness_max_gap_sec']+1e-9):
            continue
        u = samples[:, 0]-t
        design = np.column_stack((np.ones(len(u)), u, u*u))
        coeff = np.linalg.lstsq(design, samples[:, 1:4], rcond=None)[0]
        result.append([t, coeff[1, 0], 2*coeff[2, 0], coeff[1, 1]+coeff[0, 0]*coeff[0, 2]])
    return np.asarray(result).reshape((-1, 4))


def driven_progress(track, duration, max_gap):
    """Return source time and cumulative driven distance for a covered trajectory."""
    if track is None or len(track) < 2:
        return np.empty((0, 2)), 0.
    coverage_fraction = coverage(
        [(a, b) for a, b in zip(track[:-1, 0], track[1:, 0]) if 0 < b-a <= max_gap],
        0, duration) / duration
    ends = interpolate(track, [0., duration], max_gap)
    inside = track[(track[:, 0] > 0) & (track[:, 0] < duration), :4]
    bounded = np.vstack(([0., *ends[0]], inside, [duration, *ends[1]]))
    if coverage_fraction < 1-1e-6 or not np.all(np.isfinite(bounded[:, 1:3])):
        return np.empty((0, 2)), coverage_fraction
    station = np.r_[0., np.cumsum(np.linalg.norm(np.diff(bounded[:, 1:3], axis=0), axis=1))]
    return np.column_stack((bounded[:, 0], station)), coverage_fraction


def station_oscillation(signal, value_column, progress, settings):
    """Remove a local linear trend from a signal sampled over driven distance."""
    empty = np.empty((0, 4))
    if signal is None or len(signal) < 2 or len(progress) < 2 or progress[-1, 1] <= 0:
        return None, empty, 0.
    step = settings['oscillation_station_step_m']
    window = settings['oscillation_trend_window_m']
    max_gap = settings['oscillation_max_interpolation_gap_m']
    times = signal[:, 0]
    inside = (times >= progress[0, 0]) & (times <= progress[-1, 0])
    samples = signal[inside]
    if len(samples) < 2:
        return None, empty, 0.
    station = np.interp(samples[:, 0], progress[:, 0], progress[:, 1])
    # Several messages while stationary have the same station. Their mean is a
    # deterministic station-domain value; time-domain chatter while stopped is
    # intentionally outside this metric.
    unique, inverse = np.unique(np.round(station, 9), return_inverse=True)
    values = np.zeros(len(unique))
    counts = np.zeros(len(unique))
    np.add.at(values, inverse, samples[:, value_column])
    np.add.at(counts, inverse, 1)
    values /= counts
    if len(unique) < 3 or unique[-1]-unique[0] < window/2:
        return None, empty, 0.
    first = math.ceil(unique[0]/step-1e-9)*step
    last = math.floor(unique[-1]/step+1e-9)*step
    grid = np.arange(first, last+step/2, step)
    sampled = np.interp(grid, unique, values)
    right = np.searchsorted(unique, grid, side='left')
    exact = (right < len(unique)) & (np.abs(unique[np.minimum(right, len(unique)-1)]-grid) <= 1e-8)
    bracketed = (right > 0) & (right < len(unique))
    gaps = np.full(len(grid), np.inf)
    gaps[exact] = 0.
    gaps[bracketed] = unique[right[bracketed]]-unique[right[bracketed]-1]
    sampled[gaps > max_gap+1e-9] = np.nan
    residual_rows = []
    half = window/2
    finite_grid = np.isfinite(sampled)
    segment_ids = np.cumsum(~finite_grid)
    for index, (station_value, value) in enumerate(zip(grid, sampled)):
        if not finite_grid[index]:
            continue
        local = (finite_grid & (segment_ids == segment_ids[index]) &
                 (np.abs(grid-station_value) <= half+1e-9))
        if (not math.isfinite(value) or np.count_nonzero(local) < 3 or
                np.ptp(grid[local]) < half-1e-9):
            continue
        offsets = grid[local]-station_value
        design = np.column_stack((np.ones(np.count_nonzero(local)), offsets))
        trend = float(np.linalg.lstsq(design, sampled[local], rcond=None)[0][0])
        residual_rows.append([station_value, value, trend, value-trend])
    trace = np.asarray(residual_rows).reshape((-1, 4))
    expected = max(1, int(math.floor(progress[-1, 1]/step+1e-9))+1)
    coverage_fraction = len(trace)/expected
    if not len(trace):
        return None, trace, coverage_fraction
    residual = trace[:, 3]
    return {'rms': float(np.sqrt(np.mean(residual*residual))),
            'p95_absolute': float(np.percentile(np.abs(residual), 95))}, trace, coverage_fraction


def discontinuities(selected, settings):
    dt = np.diff(selected[:, 0])
    adjacent = (dt > 0) & (dt <= settings['max_increment_gap_sec'])
    dp = np.linalg.norm(np.diff(selected[:, 1:3], axis=0), axis=1)
    dyaw = np.abs(wrapped(np.diff(selected[:, 3])))
    flags = adjacent & ((dp > settings['jump_position_margin_m']+settings['jump_speed_bound_mps']*dt) |
                        (dyaw > settings['jump_yaw_margin_rad']+settings['jump_yaw_rate_bound_radps']*dt))
    return adjacent, dp, dyaw, flags


def reference_errors(vehicle, reference, return_tangents=False):
    """Existing distance/absolute-heading errors; optional tangent for displays only."""
    starts, vectors = reference[:-1], np.diff(reference, axis=0)
    lengths = np.sum(vectors*vectors, axis=1)
    keep = lengths > 1e-12
    starts, vectors, lengths = starts[keep], vectors[keep], lengths[keep]
    if not len(starts):
        raise ValueError('Reference has no nondegenerate segments')
    distances, headings, reference_yaw = [], [], []
    for row in vehicle:
        fractions = np.clip(np.sum((row[1:3]-starts)*vectors, axis=1)/lengths, 0, 1)
        d = np.linalg.norm(starts+fractions[:, None]*vectors-row[1:3], axis=1)
        candidates = np.flatnonzero(np.abs(d-d.min()) <= 1e-8)
        tangents = np.arctan2(vectors[candidates, 1], vectors[candidates, 0])
        distances.append(d.min())
        # At corners/equidistant branches there is no unique reference heading.
        unique = np.all(np.abs(wrapped(tangents-tangents[0])) < 1e-6)
        headings.append(abs(wrapped(row[3]-tangents[0])) if unique else np.nan)
        reference_yaw.append(float(tangents[0]) if unique else np.nan)
    result = (np.asarray(distances), np.asarray(headings))
    return result + (np.asarray(reference_yaw),) if return_tangents else result


def held(samples, t, gap):
    if samples is None or not len(samples):
        return None
    index = np.searchsorted([row[0] for row in samples], t, side='right')-1
    return samples[index] if index >= 0 and t-samples[index][0] <= gap else None


def stall_episodes(command, state, authority, safety, duration, settings, active_states, emergency_state):
    """Piecewise-constant evidence; explicitly stop holding stale observations."""
    gap = settings['max_increment_gap_sec']
    boundaries = {0., duration}
    for samples in (command, state, authority, safety):
        for row in samples:
            for t in (row[0], row[0]+gap):
                if 0 < t < duration:
                    boundaries.add(float(t))
    episodes, start, known = [], None, 0.
    times = sorted(boundaries)
    for a, b in zip(times, times[1:]):
        values = [held(s, (a+b)/2, gap) for s in (command, state, authority, safety)]
        valid = all(v is not None for v in values)
        if valid:
            known += b-a
        qualifies = valid and (values[2][1] in active_states and values[3][1] != emergency_state and
                              abs(values[0][1]) >= settings['stall_command_mps'] and
                              abs(values[1][1]) <= settings['stall_speed_mps'])
        if qualifies and start is None:
            start = a
        if not qualifies and start is not None:
            if a-start >= settings['stall_duration_sec']-1e-9:
                episodes.append([start, a])
            start = None
    if start is not None and duration-start >= settings['stall_duration_sec']-1e-9:
        episodes.append([start, duration])
    return episodes, known/duration


def load_settings(metadata, overrides):
    source = Path(__file__).resolve().parents[1] / 'config/evaluation_analysis.yaml'
    if not source.is_file():
        from ament_index_python.packages import get_package_share_directory
        source = Path(get_package_share_directory('oudtra_driver_bringup')) / 'config/evaluation_analysis.yaml'
    settings = yaml.safe_load(source.read_text())
    settings.update(metadata.get('analysis', {}))
    unknown = set(overrides or {})-set(settings)
    if unknown:
        raise ValueError('Unknown analysis settings: ' + ', '.join(sorted(unknown)))
    settings.update(overrides or {})
    if not all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) and v > 0 for v in settings.values()):
        raise ValueError('Analysis bounds must be finite and positive')
    if settings['smoothness_min_span_sec'] > settings['smoothness_window_sec']:
        raise ValueError('Smoothness span exceeds window')
    if settings['oscillation_trend_window_m'] < 2*settings['oscillation_station_step_m']:
        raise ValueError('Oscillation trend window must span at least two station steps')
    if settings['oscillation_max_interpolation_gap_m'] < settings['oscillation_station_step_m']:
        raise ValueError('Oscillation interpolation gap must cover at least one station step')
    return settings


def analyze(root, overrides=None):
    metadata = yaml.safe_load((root / 'metadata.yaml').read_text())
    if metadata.get('schema_version') not in (1, 2):
        raise ValueError('Unsupported metadata schema')
    platform = metadata.get('platform', 'sim' if metadata['schema_version'] == 1 else None)
    if platform not in ('sim', 'onboard'):
        raise ValueError('Platform must be sim or onboard')
    phases = {p['state']: p for p in metadata.get('phases', [])}
    if 'interval' in metadata:
        origin, end = metadata['interval']['start_wall_ns'], metadata['interval']['end_wall_ns']
        duration = (end-origin)*1e-9
    else:
        if not {'RUNNING', 'EVALUATION_END'} <= phases.keys():
            raise ValueError('No evaluation interval: RUNNING/EVALUATION_END missing')
        begin, finish = phases['RUNNING'], phases['EVALUATION_END']
        origin = begin['wall_ns']
        duration = (finish['wall_ns']-origin)*1e-9
        mono = (finish['monotonic_ns']-begin['monotonic_ns'])*1e-9
        if abs(duration-mono) > .05:
            raise ValueError('Wall/monotonic clock mismatch')
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('Invalid evaluation duration')
    settings = load_settings(metadata, overrides)
    roles, errors = normalize_roles(metadata), {}
    records = read_bag(root, roles, errors)
    data = decode(records, roles, origin, errors, duration, settings['max_increment_gap_sec'])
    pose_role = 'source_pose' if 'source_pose' in roles else 'estimated_pose'
    pose = data.get(pose_role)
    if pose is None or not len(pose) or pose_role in errors:
        raise ValueError('No usable core estimated pose: ' + errors.get(pose_role, 'missing samples'))
    selected = pose[(pose[:, 0] >= 0) & (pose[:, 0] < duration)]
    if not len(selected):
        raise ValueError('No estimated pose in evaluation interval')
    if pose_role == 'source_pose':
        public = data.get('estimated_pose')
        if public is not None and len(public):
            values = {tuple(np.round(row[1:4], 8)) for row in pose}
            if not any(tuple(np.round(row[1:4], 8)) in values for row in public):
                errors['estimated_pose'] = 'No public/source pose agreement'
    score = {'localization': {g: {} for g in ('accuracy', 'continuity', 'availability', 'consistency')},
             'vehicle': {g: {} for g in ('robustness', 'tracking', 'smoothness', 'dynamics_diagnostics', 'pace')}}

    def put(section, group, name, value, unit, evidence, method, deps=(), reason='', status=None, cov=None, time_basis='source_wall'):
        bad = [errors[r] for r in deps if r in errors]
        state = 'ANALYSIS_ERROR' if bad else status or ('AVAILABLE' if value is not None else 'UNAVAILABLE_DATA')
        score[section][group][name] = {
            'status': state, 'value': value if state == 'AVAILABLE' else None,
            'unit': unit, 'reason': '; '.join(bad) or reason,
            'evidence': evidence, 'method': method, 'time_basis': time_basis, 'coverage': cov}

    truth = data.get('ground_truth_pose')
    pose_frame = roles[pose_role].get('parent', roles[pose_role].get('frame'))
    if truth is not None and not same_frame(roles.get('ground_truth_pose', {}).get('frame'), pose_frame):
        errors['ground_truth_pose'] = 'GT and estimated pose frames differ; no implicit transform'
        truth = None
    truth_ok = truth is not None and len(truth) > 0
    aligned = interpolate(truth, selected[:, 0], settings['max_gt_gap_sec']) if truth_ok else np.full((len(selected), 3), np.nan)
    valid = np.all(np.isfinite(aligned), axis=1)
    position_error = np.linalg.norm(selected[valid, 1:3]-aligned[valid, :2], axis=1)
    heading_error = np.abs(wrapped(selected[valid, 3]-aligned[valid, 2]))
    accuracy_state = 'NOT_APPLICABLE' if platform == 'onboard' and 'ground_truth_pose' not in roles else None
    for name, values, unit in [('position_error_m', position_error, 'm'), ('absolute_heading_error_rad', heading_error, 'rad')]:
        put('localization', 'accuracy', name, quantiles(values), unit, 'independent_gt', 'source_pose_gt_interpolation_v1',
            (pose_role, 'estimated_pose', 'ground_truth_pose'), 'Requires aligned independent GT' if not len(values) else '',
            accuracy_state, float(np.mean(valid)))

    intervals = [(r[4], r[0]+settings['max_pose_age_sec']) for r in pose if r[0] <= r[4]]
    missing = dropouts(intervals, duration)
    available = coverage(intervals, 0, duration)/duration
    common = dict(evidence='estimated_pose', method='delivery_source_age_v1', deps=(pose_role, 'estimated_pose'), cov=available)
    for name, value, unit in [('availability_percent', available*100, '%'),
                              ('dropout_count', len(missing), 'count'),
                              ('longest_dropout_sec', max([b-a for a, b in missing] or [0.]), 's')]:
        put('localization', 'availability', name, value, unit, **common)
    readiness = None
    if {'LOCALIZATION_READY', 'SEND_INITIAL_POSE'} <= phases.keys():
        readiness = (phases['LOCALIZATION_READY']['wall_ns']-phases['SEND_INITIAL_POSE']['wall_ns'])*1e-9
        if not math.isfinite(readiness) or readiness < 0:
            raise ValueError('Invalid readiness interval')
    put('localization', 'availability', 'readiness_time_sec', readiness, 's', 'run_events', 'initialization_to_ready_v1', reason='Requires initialization and readiness events' if readiness is None else '')
    health = data.get('localization_health')
    put('localization', 'availability', 'health_states', state_summary(health, duration, settings['max_status_gap_sec']) if health else None,
        's/count', 'published_health', 'fresh_state_hold_v1', ('localization_health',), 'Unstamped health uses bag receive time', time_basis='bag_receive')

    if health:
        summary = state_summary(health, duration, settings['max_status_gap_sec'])
        unknown = dropouts([(t, t+settings['max_status_gap_sec']) for t, _ in health], duration)
        distribution = health_distribution(summary, duration, unknown)
    else:
        distribution = None
    put('localization', 'availability', 'health_time_distribution', distribution, 's/%/count',
        'published_health', 'interval_normalized_health_v1', ('localization_health',),
        'Health quality is separate from fresh-pose delivery; historical threshold equivalence is not established',
        cov=1-distribution['duration_sec']['UNKNOWN']/duration if distribution else 0., time_basis='bag_receive')

    adjacent, dp, dyaw, flags = discontinuities(selected, settings)
    for name, value, unit in [('pose_jump_count', int(sum(flags)) if np.any(adjacent) else None, 'count'),
                              ('largest_position_jump_m', float(max(dp[adjacent])) if np.any(adjacent) else None, 'm'),
                              ('largest_yaw_jump_rad', float(max(dyaw[adjacent])) if np.any(adjacent) else None, 'rad')]:
        put('localization', 'continuity', name, value, unit, 'estimated_pose', 'motion_bound_discontinuity_v1', (pose_role,),
            'Largest jump is largest adjacent increment, not necessarily a flagged discontinuity; resets may be legitimate', cov=float(np.mean(adjacent)) if len(adjacent) else 0.)

    odom = data.get('raw_odometry')
    residuals = consistency(pose, odom, duration, settings) if odom is not None and len(odom) else np.empty((0, 3))
    finite = np.all(np.isfinite(residuals), axis=1)
    for col, name, unit in [(1, 'pose_increment_disagreement_m', 'm'), (2, 'yaw_increment_disagreement_rad', 'rad')]:
        put('localization', 'consistency', name, quantiles(residuals[finite, col]), unit, 'localization_vs_raw_odometry',
            'body_relative_se2_window_v1', (pose_role, 'raw_odometry'), 'Consistency only; raw odometry is not truth', cov=float(np.mean(finite)) if len(finite) else 0.)
    disagreement = finite & ((residuals[:, 1] > settings['consistency_position_threshold_m']) | (residuals[:, 2] > settings['consistency_yaw_threshold_rad']))
    entries = sum(bool(disagreement[i] and i > 0 and finite[i-1] and not disagreement[i-1]) for i in range(len(disagreement)))
    uncertain = sum(bool(disagreement[i] and (i == 0 or not finite[i-1])) for i in range(len(disagreement)))
    put('localization', 'consistency', 'disagreement_events', {'observed_entries': entries, 'initial_or_after_gap': uncertain} if np.any(finite) else None,
        'count', 'localization_vs_raw_odometry', 'threshold_window_entries_v1', (pose_role, 'raw_odometry'))

    completed = metadata.get('outcome', {}).get('completed')
    if completed is None and 'smoke_result' in metadata:
        completed = metadata['smoke_result']['result'] == 'PASS'
    if completed is not None and not isinstance(completed, bool):
        raise ValueError('Completion must be boolean')
    put('vehicle', 'robustness', 'completed', completed, 'bool', 'run_outcome', 'declared_completion_v1')
    collisions = data.get('collision_status')
    collision_cov = coverage([(t, t+settings['max_gt_gap_sec']) for t, _ in collisions], 0, duration)/duration if collisions else 0.
    collision = any(v for t, v in collisions if 0 <= t < duration) if collisions else None
    if collision is False and collision_cov < 1-1e-6:
        collision = None
    put('vehicle', 'robustness', 'collision_observed', collision, 'bool', 'collision_sensor', 'observed_collision_flag_v1',
        ('collision_status',), 'A negative observation requires coverage; absence of a sensor is not collision freedom', cov=collision_cov)
    safety = data.get('safety_status')
    emergency_state = roles.get('safety_status', {}).get('emergency_state', 'EMERGENCY_STOP')
    events = emergency_events([(t, 'EMERGENCY_STOP' if s == emergency_state else s) for t, s in safety], 0, duration, settings['max_status_gap_sec']) if safety else None
    safety_cov = coverage([(t, t+settings['max_status_gap_sec']) for t, _ in safety], 0, duration)/duration if safety else 0.
    if events and safety_cov < 1-1e-6 and not events['observed']:
        events['observed'] = None
    put('vehicle', 'robustness', 'emergency_stops', events, 'count', 'safety_state', 'observed_entries_v1', ('safety_status',),
        'Entries are observed lower bounds when coverage is incomplete', cov=safety_cov)
    command, state, authority = [data.get(r) for r in ('final_drive_command', 'vehicle_state', 'control_status')]
    active_states = roles.get('control_status', {}).get('autonomous_states', [])
    episodes, stall_cov = ([], 0.)
    if (all(s is not None and len(s) for s in (command, state, authority, safety)) and
            active_states and roles.get('vehicle_state', {}).get('velocity_frame') == 'body'):
        episodes, stall_cov = stall_episodes(command, state, authority, safety, duration, settings, active_states, emergency_state)
    stall_value = {'observed_count': len(episodes), 'observed_duration_sec': sum(b-a for a, b in episodes), 'episodes_sec': episodes} if stall_cov > 0 else None
    put('vehicle', 'robustness', 'commanded_motion_stalls', stall_value, 'count/s', 'command_vs_vehicle_speed_with_authority', 'sustained_commanded_motion_stall_v1',
        ('final_drive_command', 'vehicle_state', 'control_status', 'safety_status'),
        'Narrow observable stall, not all unexpected stops; incomplete coverage gives lower bounds. Onboard requires qualified speed and mux authority evidence; arbitration mode alone does not establish actual mux authority', cov=stall_cov)

    # Preserve the original GT distance calculation, including its boundary rules.
    vehicle_role = 'ground_truth_pose' if platform == 'sim' else 'raw_odometry'
    vehicle = data.get(vehicle_role)
    vehicle_gap = settings['max_gt_gap_sec'] if platform == 'sim' else settings['max_increment_gap_sec']
    distance, partial_distance, vehicle_cov = None, None, 0.
    if vehicle is not None and len(vehicle):
        vehicle_cov = coverage([(a, b) for a, b in zip(vehicle[:-1, 0], vehicle[1:, 0]) if 0 < b-a <= vehicle_gap], 0, duration)/duration
        ends = interpolate(vehicle, [0., duration], vehicle_gap)
        inside = vehicle[(vehicle[:, 0] > 0) & (vehicle[:, 0] < duration), :4]
        bounded = np.vstack(([0., *ends[0]], inside, [duration, *ends[1]]))
        usable = (np.diff(bounded[:, 0]) <= vehicle_gap) & np.all(np.isfinite(bounded[:-1, 1:3]), axis=1) & np.all(np.isfinite(bounded[1:, 1:3]), axis=1)
        partial_distance = float(np.sum(np.linalg.norm(np.diff(bounded[:, 1:3], axis=0)[usable], axis=1)))
        if vehicle_cov >= 1-1e-6:
            distance = partial_distance
    evidence = 'gt_trajectory' if platform == 'sim' else 'odometry_trajectory_proxy'
    for name, value, unit in [('distance_travelled_m', distance, 'm'), ('mean_elapsed_time_speed_mps', distance/duration if distance is not None else None, 'm/s')]:
        put('vehicle', 'pace', name, value, unit, evidence, 'trajectory_segments_elapsed_v1', (vehicle_role,), cov=vehicle_cov)

    track_role = 'ground_truth_pose' if platform == 'sim' else pose_role
    track = data.get(track_role)
    track = track[(track[:, 0] >= 0) & (track[:, 0] < duration)] if track is not None else np.empty((0, 5))
    reference = data.get('reference_path')
    track_frame = roles.get(track_role, {}).get('parent', roles.get(track_role, {}).get('frame'))
    if reference is not None and not same_frame(roles['reference_path'].get('frame'), track_frame):
        errors['reference_path'] = 'Reference and vehicle pose frames differ; no implicit transform'
        reference = None
    deviations, headings = np.array([]), np.array([])
    if reference is not None and len(track):
        try:
            deviations, headings = reference_errors(track, reference)
        except ValueError as error:
            errors['reference_path'] = str(error)
    for name, vals, unit in [('static_reference_deviation_m', deviations, 'm'), ('heading_to_reference_rad', headings[np.isfinite(headings)], 'rad')]:
        put('vehicle', 'tracking', name, quantiles(vals), unit, 'gt_map_pose' if platform == 'sim' else 'estimated_map_pose_proxy',
            'nearest_static_segment_v1', (track_role, 'reference_path'), 'Static distance to raceline includes intentional avoidance; ambiguous segment headings excluded',
            cov=len(vals)/len(track) if len(track) else 0.)

    # Public PF pose is deliberately identical evidence on both platforms.
    public = data.get('estimated_pose')
    public = public[(public[:, 0] >= 0) & (public[:, 0] < duration)] if public is not None else np.empty((0, 5))
    pf_deviation, pf_heading = np.array([]), np.array([])
    if reference is not None and len(public):
        if not same_frame(roles['estimated_pose'].get('frame'), roles['reference_path'].get('frame')):
            errors['estimated_pose'] = 'PF/reference frame mismatch; no implicit transform'
        else:
            try:
                pf_deviation, pf_heading = reference_errors(public, reference)
            except ValueError as error:
                errors['reference_path'] = str(error)
    for name, vals, unit in [('pf_reference_position_deviation_m', pf_deviation, 'm'),
                             ('pf_heading_to_reference_rad', pf_heading[np.isfinite(pf_heading)], 'rad')]:
        put('vehicle', 'tracking', name, quantiles(vals), unit, 'estimated_map_pose_proxy',
            'nearest_static_segment_v1', ('estimated_pose', 'reference_path'),
            'Where PF believes the vehicle is; independent GT accuracy is a separate metric',
            cov=len(vals)/len(public) if len(public) else 0.)

    # Raw-odometry arc length is the common sim/onboard station coordinate. It
    # keeps these vehicle metrics independent of simulator-only GT.
    progress_role = 'raw_odometry'
    progress, progress_coverage = driven_progress(
        data.get(progress_role), duration, settings['max_increment_gap_sec'])
    oscillation_traces = {}
    state_evidence = roles.get('vehicle_state', {}).get(
        'evidence', 'simulator_odometry_body_twist' if platform == 'sim' else 'measured_vehicle_state')
    oscillation_inputs = [
        ('speed_command_oscillation', command, 1, 'm/s', 'final_speed_command', 'final_drive_command'),
        ('steering_command_oscillation', command, 2, 'rad', 'final_steering_command', 'final_drive_command'),
        ('forward_speed_oscillation', state, 1, 'm/s', state_evidence + '_forward_speed', 'vehicle_state'),
        ('yaw_rate_oscillation', state, 3, 'rad/s', state_evidence + '_yaw_rate', 'vehicle_state')]
    adapter_reasons = {}
    for role, name, unit in [('forward_speed', 'forward_speed_oscillation', 'm/s'),
                              ('yaw_rate', 'yaw_rate_oscillation', 'rad/s')]:
        if role in roles:
            samples, adapter_reasons[name] = scalar_signal(data.get(role), roles[role], role)
            oscillation_inputs = [item for item in oscillation_inputs if item[0] != name]
            oscillation_inputs.append((name, samples, 1, unit, roles[role].get('evidence', role), role))
    upstream = data.get('autonomy_command')
    for name, column, unit, evidence in [
            ('autonomy_speed_command_oscillation', 1, 'm/s', 'upstream_autonomy_speed_command'),
            ('autonomy_steering_command_oscillation', 2, 'rad', 'upstream_autonomy_steering_command')]:
        oscillation_inputs.append((name, upstream, column, unit, evidence, 'autonomy_command'))
    for name, samples, column, unit, signal_evidence, signal_role in oscillation_inputs:
        value, trace, signal_coverage = station_oscillation(samples, column, progress, settings)
        oscillation_traces[name] = trace
        reason = ('Requires continuous driven-progress pose and sufficient signal coverage over distance'
                  if value is None else
                  'Residual around a local distance-domain trend; lower generally means less short-scale oscillation')
        reason = adapter_reasons.get(name) or reason
        put('vehicle', 'smoothness', name, value, unit, signal_evidence,
            'uniform_station_local_linear_residual_v1', (signal_role, progress_role), reason,
            cov=min(progress_coverage, signal_coverage), time_basis='driven_distance')
    spec = roles.get('vehicle_state', {})
    qualified = spec.get('physical_time') is True and spec.get('velocity_frame') == 'body' and spec.get('lateral_velocity_observed') is True
    dynamics = windowed_dynamics(state, duration, settings) if state is not None and len(state) and spec.get('physical_time') is True and spec.get('velocity_frame') == 'body' else np.empty((0, 4))
    for name, col, unit in [('longitudinal_acceleration_mps2', 1, 'm/s²'), ('longitudinal_jerk_mps3', 2, 'm/s³'), ('lateral_acceleration_mps2', 3, 'm/s²')]:
        vals = np.abs(dynamics[:, col]) if col != 3 or qualified else []
        q = quantiles(vals)
        if q and col == 2:
            q.pop('p50')
        put('vehicle', 'dynamics_diagnostics', name, q, unit, spec.get('evidence', 'vehicle_state'), 'windowed_body_velocity_quadratic_v1', ('vehicle_state',),
            '' if q else 'Requires body velocity with verified physical timestamps and window support; lateral metric also requires observed lateral velocity', time_basis='physical_source')

    metrics = {'schema_version': 2, 'analyzer_version': VERSION, 'run_id': metadata.get('run_id', root.name),
               'title': root.name, 'description': metadata.get('description', ''), 'platform': platform,
               'analyzer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               'evidence_adapter_sha256': hashlib.sha256(Path(__file__).with_name('evaluation_evidence.py').read_bytes()).hexdigest(),
               'analysis_status': 'FAIL' if errors else 'PASS', 'recording_status': metadata.get('recording', {}).get('status', 'UNKNOWN'),
               'interval': {'start_wall_ns': origin, 'duration_sec': duration}, 'settings': settings,
               'identity': metadata.get('comparison_identity', {}),
               'provenance': {k: metadata.get(k) for k in ('main_revision', 'pf_revision', 'scenario', 'configs')},
               **score,
               'data_quality': {'errors': errors, 'message_counts': {k: len(v) for k, v in records.items()},
                                'aligned_samples': int(sum(valid)), 'pose_evaluation_samples': len(selected),
                                'dropout_intervals_sec': missing, 'partial_observed_distance_m': partial_distance,
                                'discontinuity_unobserved_pairs': int(sum(~adjacent))},
               'limitations': ['Consistency is not truth error; odometry may be an input to localization.',
                               'Onboard tracking uses estimated map pose and can hide localization bias.',
                               'Commanded-motion stalls exclude intentional zero commands and emergency stops.',
                               'Oscillation uses uniform driven-distance sampling; it does not measure chatter while stationary.',
                               'Legacy simulator physics steps are not publication timestamps; physical acceleration/jerk may be unavailable.',
                               'Health timing uses bag receive time; state transitions between samples can be missed.']}
    metrics['limitations'].extend(metadata.get('limitations', []))
    metrics['data_quality']['health_definition'] = metadata.get('health_definition', {'qualification': 'UNKNOWN', 'reason': 'Historical deployed health thresholds not captured'})
    if 'yaw_rate' in roles:
        metrics['data_quality']['raw_imu_diagnostics'] = imu_diagnostics(records.get('yaw_rate', []), origin, duration, roles['yaw_rate'])
    metrics['provenance']['reanalysis'] = metadata.get('reanalysis')
    metrics['provenance']['historical_audit'] = metadata.get('historical_audit', {})
    write_outputs(root, metrics, metadata, selected, aligned, valid, track, reference, residuals, oscillation_traces, public, health)
    return metrics


def display(metric):
    if metric['status'] != 'AVAILABLE':
        return metric['status'] + ': ' + (metric['reason'] or 'insufficient declared data')
    value = metric['value']
    if isinstance(value, bool):
        return 'Yes' if value else 'No'
    if isinstance(value, dict):
        labels = {'rms': 'RMS', 'p95_absolute': 'P95 absolute'}
        return '; '.join('{}: {}'.format(labels.get(k, k), ('Yes' if v else 'No') if isinstance(v, bool) else format(v, '.4g') if isinstance(v, (float, int)) else json.dumps(v, sort_keys=True)) for k, v in value.items())
    return format(value, '.5g') if isinstance(value, (float, int)) else str(value)


def human_result(name, metric):
    """Render structured machine values as concise engineering prose."""
    if metric['status'] != 'AVAILABLE':
        return display(metric)
    value = metric['value']
    yes_no = lambda item: 'Unknown' if item is None else 'Yes' if item else 'No'
    if name == 'health_time_distribution':
        return '<br>'.join('{}: {:.2f}% ({:.3f} s)'.format(s, value['percent'][s], value['duration_sec'][s])
                          for s in ('GOOD', 'DEGRADED', 'INVALID', 'UNKNOWN')) + '<br>Health evidence gaps: {} (longest {:.3f} s)'.format(value['unknown_interval_count'], value['longest_unknown_interval_sec'])
    if name == 'health_states':
        durations = value.get('duration_sec', {})
        known = ['GOOD', 'DEGRADED', 'INVALID']
        rows = ['{}: {:.2f} s'.format(state, durations.get(state, 0.)) for state in known]
        rows += ['{}: {:.2f} s'.format(state, seconds)
                 for state, seconds in sorted(durations.items()) if state not in known]
        rows.append('Unknown: {:.2f} s'.format(value.get('unknown_duration_sec', 0.)))
        entries = value.get('observed_entries', {})
        rows.append('Observed state transitions: ' +
                    (', '.join('{}={}'.format(k, v) for k, v in sorted(entries.items())) or 'none'))
        return '<br>'.join(rows)
    if name == 'emergency_stops':
        return ('Emergency observed: {}<br>Entries: {}<br>Active at interval start: {}'
                '<br>Uncertain entries after gaps: {}').format(
                    yes_no(value.get('observed')), value.get('observed_entries', 0),
                    yes_no(value.get('active_at_start')), value.get('uncertain_entries', 0))
    if name == 'commanded_motion_stalls':
        intervals = value.get('episodes_sec', [])
        interval_text = ', '.join('{:.2f}–{:.2f} s'.format(a, b) for a, b in intervals) or 'none'
        return ('Observed episodes: {}<br>Observed duration: {:.2f} s<br>Intervals: {}').format(
            value.get('observed_count', 0), value.get('observed_duration_sec', 0.), interval_text)
    if name == 'disagreement_events':
        return ('Observed entries: {}<br>Initial/after-gap exceedances: {}').format(
            value.get('observed_entries', 0), value.get('initial_or_after_gap', 0))
    return display(metric)


GROUP_EXPLANATIONS = {
    ('localization', 'accuracy'): ('Question: how close was the estimated global pose to independent simulator truth? Source-timed PF poses are aligned with interpolated ground truth, then position and wrapped heading errors are summarized. Lower is more accurate. This absolute result requires independent GT and therefore does not exist for an ordinary onboard run.'),
    ('localization', 'continuity'): ('Question: did localization change abruptly? Adjacent estimated poses are checked against time-scaled position and yaw motion bounds; flagged transitions and the largest observed increments are reported. Fewer/smaller jumps are generally preferable, but gaps are unknown and a legitimate localization reset can also look discontinuous.'),
    ('localization', 'availability'): ('Question: how much of the window had fresh pose delivery, and what quality did PF report? Freshness is not localization quality; GOOD/DEGRADED/INVALID/UNKNOWN percentages use the entire evaluation interval. Source and receive timestamps define freshness intervals; their uncovered complement gives dropout count and duration. Higher availability and shorter dropouts are preferable. Health-state timing uses bag receive time, so transitions between messages may be missed.'),
    ('localization', 'consistency'): ('Question: did PF motion agree with raw-odometry motion over short windows? Body-relative translation and yaw increments are compared every 0.10 s over 0.50 s windows. Lower disagreement is more internally consistent, but this is correlated evidence because PF may consume the same odometry; it is not absolute accuracy.'),
    ('vehicle', 'robustness'): ('Question: did the bounded run complete without observed collision, emergency-stop entry, or a sustained commanded-motion stall? Results combine declared run outcome, collision observations, lower-safety state, final command, vehicle speed, and control authority. Zero events is favorable only within the stated evidence coverage and does not prove long-run robustness.'),
    ('vehicle', 'tracking'): ('Question: how closely did the vehicle follow the recorded static raceline? PF-estimated map poses use the same nearest-static-segment projection on both platforms: this is where localization believes the car is. Legacy platform tracking remains unchanged (GT in simulation, PF onboard); simulated GT tracking describes actual vehicle position. PF-versus-GT accuracy is computed independently. Lower is closer to the reference, although intentional obstacle avoidance can correctly increase both values.'),
    ('vehicle', 'dynamics_diagnostics'): ('These secondary physical acceleration and jerk diagnostics require body velocity and qualified physical source timing. They do not substitute wall time or an assumed no-slip model when evidence is insufficient.'),
    ('vehicle', 'pace'): ('Question: how much motion occurred during the bounded interval? Consecutive trajectory segments give distance, and distance divided by elapsed evaluation time gives mean speed. Larger values mean more distance or pace in this run, not a lap-time or racing-performance result.')}

GROUP_LABELS = {'accuracy': 'Accuracy', 'continuity': 'Continuity', 'availability': 'Availability',
                'consistency': 'Consistency', 'robustness': 'Robustness', 'tracking': 'Tracking',
                'smoothness': 'Smoothness', 'dynamics_diagnostics': 'Additional dynamics diagnostics',
                'pace': 'Pace'}

METRIC_LABELS = {
    'health_time_distribution': 'PF quality: interval-normalized health',
    'pf_reference_position_deviation_m': 'PF-estimated distance to raceline',
    'pf_heading_to_reference_rad': 'PF-estimated absolute heading difference from reference',
    'autonomy_speed_command_oscillation': 'Upstream autonomy speed-command oscillation',
    'autonomy_steering_command_oscillation': 'Upstream autonomy steering-command oscillation',
    'position_error_m': 'Position error', 'absolute_heading_error_rad': 'Absolute heading error',
    'pose_jump_count': 'Pose jumps', 'largest_position_jump_m': 'Largest position jump',
    'largest_yaw_jump_rad': 'Largest yaw jump', 'availability_percent': 'Fresh-pose availability',
    'dropout_count': 'Localization dropouts', 'longest_dropout_sec': 'Longest localization dropout',
    'readiness_time_sec': 'Localization readiness time', 'health_states': 'Localization health states',
    'pose_increment_disagreement_m': 'Position-increment disagreement',
    'yaw_increment_disagreement_rad': 'Yaw-increment disagreement',
    'disagreement_events': 'Motion disagreement events', 'completed': 'Run completed',
    'collision_observed': 'Collision observed', 'emergency_stops': 'Emergency stops',
    'commanded_motion_stalls': 'Commanded-motion stalls',
    'static_reference_deviation_m': 'Legacy distance to raceline (GT in sim; PF onboard)',
    'heading_to_reference_rad': 'Legacy platform-reference heading difference (GT in sim; PF onboard)',
    'speed_command_oscillation': 'Speed-command oscillation',
    'steering_command_oscillation': 'Steering-command oscillation',
    'forward_speed_oscillation': 'Forward-speed oscillation',
    'yaw_rate_oscillation': 'Yaw-rate oscillation',
    'longitudinal_acceleration_mps2': 'Longitudinal acceleration',
    'longitudinal_jerk_mps3': 'Longitudinal jerk',
    'lateral_acceleration_mps2': 'Lateral acceleration',
    'distance_travelled_m': 'Distance travelled',
    'mean_elapsed_time_speed_mps': 'Mean elapsed-time speed'}

EVIDENCE_LABELS = {
    'vesc_odometry_body_forward_speed': 'ERPM-derived forward-speed estimate (`/odom` body linear.x)',
    'unqualified_imu_yaw_rate': 'Raw gyro z: units/body-axis/bias unqualified',
    'upstream_autonomy_speed_command': 'Autonomy `/drive` speed, before onboard mux',
    'upstream_autonomy_steering_command': 'Autonomy `/drive` steering, before onboard mux',
    'independent_gt': 'Independent ground truth', 'estimated_pose': 'Estimated pose',
    'localization_vs_raw_odometry': 'PF pose and raw odometry', 'published_health': 'Published localization health',
    'run_events': 'Recorded startup events', 'run_outcome': 'Declared run outcome',
    'collision_sensor': 'Collision-status observations', 'safety_state': 'Lower-safety state',
    'command_vs_vehicle_speed_with_authority': 'Final command, vehicle speed, control authority, and safety state',
    'gt_map_pose': 'Simulator GT map pose', 'estimated_map_pose_proxy': 'Estimated map pose proxy',
    'final_speed_command': 'Final drive speed command', 'final_steering_command': 'Final drive steering command',
    'simulator_odometry_body_twist_forward_speed': 'Simulated forward speed in the vehicle body frame (`/ego_racecar/odom` `twist.linear.x`)',
    'simulator_odometry_body_twist_yaw_rate': 'Simulated yaw rate in the vehicle body frame (`/ego_racecar/odom` `twist.angular.z`)',
    'gt_trajectory': 'Simulator GT trajectory', 'odometry_trajectory_proxy': 'Local odometry trajectory proxy'}

PLOT_CAPTIONS = {
    'pf_reference_trajectory.png': 'Public PF map pose versus the recorded reference, using the same evidence on sim and onboard. This shows believed tracking; it can hide localization bias.',
    'localization_health.png': 'Reported health is held only across fresh status intervals; absent segments are UNKNOWN. GOOD health is not established by 100% fresh-pose delivery.',
    'trajectory_xy.png': ('GT and PF estimated map-frame trajectories. Separation between the lines is absolute position error; equal axis scaling preserves geometry.'),
    'position_error.png': ('Absolute PF-versus-GT position error over elapsed evaluation time. Lower is more accurate; line breaks would mark unsupported intervals.'),
    'reference_trajectory.png': ('Recorded static raceline and driven trajectory in map coordinates. Separation includes both tracking error and intentional avoidance.'),
    'pose_increment_disagreement.png': ('PF-versus-odometry position and yaw increment disagreement over time, with the unchanged 0.30 m / 0.25 rad event thresholds. This is correlated consistency evidence, not truth error.'),
    'smoothness_oscillations.png': ('Each panel shows only the oscillation residual—evaluated signal minus its local linear trend—against raw-odometry driven progress. Zero follows the local trend; excursions show shorter-scale variation. The raw signal and fitted trend are used by analysis but are not drawn.')}

PLOT_TITLES = {'pf_reference_trajectory.png': 'PF-estimated raceline tracking',
               'localization_health.png': 'PF health over the evaluation interval',
               'trajectory_xy.png': 'GT and estimated trajectory',
               'position_error.png': 'Absolute position error',
               'reference_trajectory.png': 'Reference and driven trajectory',
               'pose_increment_disagreement.png': 'Localization/odometry consistency',
               'smoothness_oscillations.png': 'Smoothness residuals over driven progress'}


def report_context(metadata, metrics):
    """Describe execution from preserved metadata without changing metric JSON."""
    platform = metrics['platform']
    scenario = metadata.get('scenario') or metrics.get('identity', {}).get('scenario_id') or 'Not recorded'
    initial = metadata.get('initial_pose')
    initialization = ('One-shot initial pose: x={:.4f} m, y={:.4f} m, yaw={:.4f} rad.'.format(*initial)
                      if isinstance(initial, list) and len(initial) == 3 else
                      'No localization-initialization event was recorded for this artifact.')
    commands = metadata.get('smoke_result', {}).get('commands', [])
    launches = []
    for command in commands:
        if len(command) >= 5 and command[:2] == ['ros2', 'launch']:
            launches.append('`{}/{}`'.format(command[2], command[3]))
    if launches:
        flow = ' → '.join(launches)
    elif metadata.get('description'):
        flow = metadata['description']
    else:
        flow = 'Execution flow was not recorded in this run metadata.'
    if platform == 'sim':
        purpose = ('A bounded localization-in-the-loop simulation run exercised the Gym simulator, '
                   'particle-filter localization, and the existing autonomous full stack in closed loop. '
                   'Simulator ground truth was recorded only as evaluation evidence.')
    else:
        purpose = (metadata.get('description') or
                   'A preserved onboard-style run was analyzed without assuming simulator ground truth.')
    roles = []
    for name, spec in metadata.get('roles', {}).items():
        roles.append('`{}` → `{}`'.format(ALIASES.get(name, name), spec.get('topic', 'topic not recorded')))
    producers = []
    for name in metadata.get('software_sha256', {}):
        if name in ('run_localization_smoke.py', 'localization_recording.py'):
            producers.append('[{}](../../scripts/{})'.format(name, name))
    producers.append('[analyze_localization_run.py](../../scripts/analyze_localization_run.py)')
    return {'purpose': purpose, 'scenario': str(scenario), 'initialization': initialization,
            'flow': flow, 'roles': roles, 'producers': producers}


def report_metric_table(items, names=None):
    """Human Markdown view; structured values remain unchanged in metrics.json."""
    names = names or list(items)
    report = '| Metric [unit] | Result | Evidence | Coverage |\n| --- | --- | --- | --- |\n'
    for name in names:
        metric = items[name]
        coverage_text = ('{:.1f}%'.format(100*metric['coverage'])
                         if metric.get('coverage') is not None else 'Not quantified')
        evidence_text = EVIDENCE_LABELS.get(metric['evidence'], metric['evidence'].replace('_', ' '))
        report += '| {} [{}] | {} | {} | {} |\n'.format(
            METRIC_LABELS.get(name, name.replace('_', ' ').title()), metric['unit'],
            human_result(name, metric).replace('|', '/'), evidence_text, coverage_text)
    return report + '\n'


def write_outputs(root, metrics, metadata, selected, aligned, valid, track, reference, residuals, oscillation_traces, public, health):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plots = root / 'plots'
    plots.mkdir(exist_ok=True)
    generated = []
    def save(fig, name):
        fig.tight_layout()
        fig.savefig(plots / name, dpi=140)
        plt.close(fig)
        generated.append(name)
    def xy(name, title, series):
        fig, ax = plt.subplots()
        for rows, label in series:
            ax.plot(rows[:, 0], rows[:, 1], label=label)
        ax.set(title=title, xlabel='Map x [m]', ylabel='Map y [m]')
        ax.axis('equal')
        ax.legend()
        save(fig, name)
    series = [(selected[:, 1:3], 'Estimated pose')]
    if np.any(valid):
        series.insert(0, (aligned[valid, :2], 'Simulator GT' if metrics['platform'] == 'sim' else 'Independent GT'))
    xy('trajectory_xy.png', 'Evaluation trajectory — ' + metrics['platform'], series)
    def line(name, title, times, values, label):
        fig, ax = plt.subplots()
        ax.plot(times, values)
        ax.set(title=title, xlabel='Elapsed evaluation time [s]', ylabel=label, xlim=(0, metrics['interval']['duration_sec']))
        save(fig, name)
    if np.any(valid):
        values = np.full(len(selected), np.nan)
        values[valid] = np.linalg.norm(selected[valid, 1:3]-aligned[valid, :2], axis=1)
        # Break lines over absent pose intervals as well as failed GT alignment.
        times = selected[:, 0].tolist()
        values = values.tolist()
        for i in range(len(times)-1, 0, -1):
            if times[i]-times[i-1] > metrics['settings']['max_pose_age_sec']:
                times.insert(i, float('nan'))
                values.insert(i, float('nan'))
        line('position_error.png', 'Absolute localization position error', times, values, 'Position error [m]')
    if reference is not None and len(track) and 'reference_path' not in metrics['data_quality']['errors']:
        xy('reference_trajectory.png', 'Static reference and vehicle trajectory', [(reference, 'Static reference'), (track[:, 1:3], 'GT' if metrics['platform'] == 'sim' else 'Estimated pose (proxy)')])
    if reference is not None and len(public) and not any(k in metrics['data_quality']['errors'] for k in ('estimated_pose', 'reference_path')):
        xy('pf_reference_trajectory.png', 'PF-estimated tracking (not absolute accuracy)',
           [(reference, 'Recorded static reference'), (public[:, 1:3], 'Public PF map pose')])
    if health:
        fig, ax = plt.subplots(figsize=(7, 3))
        levels = {'UNKNOWN': 0, 'INVALID': 1, 'DEGRADED': 2, 'GOOD': 3}
        end = metrics['interval']['duration_sec']
        # Explicit finite holds; gaps remain UNKNOWN rather than drawing bridges.
        ax.axhline(0, color='gray', linewidth=.5)
        for i, (t, state) in enumerate(health):
            stop = min(end, t+metrics['settings']['max_status_gap_sec'], health[i+1][0] if i+1 < len(health) else end)
            if stop > max(0., t):
                ax.plot([max(0., t), stop], [levels.get(state, 0)]*2, color='tab:blue', linewidth=2)
        ax.set(yticks=list(levels.values()), yticklabels=list(levels), xlim=(0, end),
               xlabel='Elapsed evaluation time [s]', title='PF reported quality; fresh pose delivery is separate')
        save(fig, 'localization_health.png')
    if len(residuals) and np.any(np.isfinite(residuals[:, 1])):
        fig, axes = plt.subplots(2, 1, sharex=True, figsize=(7, 5))
        for ax, col, unit, threshold in [(axes[0], 1, 'm', metrics['settings']['consistency_position_threshold_m']),
                                         (axes[1], 2, 'rad', metrics['settings']['consistency_yaw_threshold_rad'])]:
            ax.plot(residuals[:, 0], residuals[:, col])
            ax.axhline(threshold, color='red', linestyle='--', label='Event threshold')
            ax.set(ylabel='Increment disagreement ['+unit+']', xlim=(0, metrics['interval']['duration_sec']))
            ax.legend()
        axes[0].set_title('Correlated PF / raw-odometry consistency (not accuracy)')
        axes[-1].set_xlabel('Elapsed evaluation time [s]')
        save(fig, 'pose_increment_disagreement.png')
    plotted = [(name, trace) for name, trace in oscillation_traces.items() if len(trace)]
    if plotted:
        fig, axes = plt.subplots(len(plotted), 1, sharex=True, figsize=(7, 2.2*len(plotted)))
        axes = np.atleast_1d(axes)
        for ax, (name, trace) in zip(axes, plotted):
            metric = metrics['vehicle']['smoothness'][name]
            ax.plot(trace[:, 0], trace[:, 3], linewidth=1)
            ax.axhline(0, color='black', linewidth=.6)
            ax.set(ylabel='Residual [{}]'.format(metric['unit']), title=METRIC_LABELS[name])
        axes[-1].set_xlabel('Raw-odometry driven progress [m]')
        fig.suptitle('Short-scale smoothness residuals after local linear trend removal')
        save(fig, 'smoothness_oscillations.png')
    for name in ('trajectory_xy.png', 'position_error.png', 'reference_trajectory.png',
                 'pose_increment_disagreement.png', 'smoothness_oscillations.png', 'pf_reference_trajectory.png', 'localization_health.png'):
        if name not in generated and (plots / name).exists():
            (plots / name).unlink()
    metrics['plots'] = generated
    context = report_context(metadata, metrics)
    position_metric = metrics['localization']['accuracy']['position_error_m']
    position_p95 = ('{:.4g} m'.format(position_metric['value']['p95'])
                    if position_metric['status'] == 'AVAILABLE' else position_metric['status'])
    report = '# Evaluation scorecard — {}\n\n## 1. Summary\n\n'.format(metrics['title'])
    report += '{} run, {:.3f} s evaluation; analysis **{}**, recording **{}**. '.format(metrics['platform'], metrics['interval']['duration_sec'], metrics['analysis_status'], metrics['recording_status'])
    report += 'The scorecard confirms whether recorded evidence was valid; it does not apply performance acceptance thresholds.\n\n'
    report += ('Run completed: **{}**. Fresh-pose availability: **{}%**. Observed pose jumps: **{}**. '
               'Absolute position-error P95: **{}**. All four spatial oscillation metrics were **{}**.\n\n').format(
        display(metrics['vehicle']['robustness']['completed']),
        display(metrics['localization']['availability']['availability_percent']),
        display(metrics['localization']['continuity']['pose_jump_count']),
        position_p95,
        'AVAILABLE' if all(metrics['vehicle']['smoothness'][n]['status'] == 'AVAILABLE' for n in ('speed_command_oscillation', 'steering_command_oscillation', 'forward_speed_oscillation', 'yaw_rate_oscillation'))
        else 'partially available')
    report += ('The numbers describe this bounded run only. They do not by themselves prove controller quality, '
               'long-run robustness, onboard equivalence, or causal performance improvement.\n\n')
    health_metric = metrics['localization']['availability']['health_time_distribution']
    report += '**PF quality (separate from pose freshness):** ' + human_result('health_time_distribution', health_metric) + '\n\n'
    report += 'Health-state threshold equivalence across historical deployments is not established.\n\n'
    report += '## 2. What this run tested\n\n'+context['purpose']+'\n\n'
    if metrics['description'] and metrics['description'] != context['purpose']:
        report += metrics['description']+'\n\n'
    report += '## 3. How this experiment produces the scorecard\n\n'
    if metrics['platform'] == 'sim':
        report += ('The live closed loop was Gym simulator → scan/raw odometry → particle filter → estimated map pose → '
                   'existing planning-and-control stack → final drive command → Gym simulator. During the bounded evaluation '
                   'interval, selected semantic ROS signals were recorded to rosbag. Simulator ground truth was recorded on a '
                   'validation-only branch and never fed to PF or control.\n\n')
        report += ('```mermaid\nflowchart LR\n'
                   '  SIM["Gym simulator"] -->|"scan and raw odometry"| PF["Particle filter"]\n'
                   '  PF -->|"estimated map pose"| PNC["Existing PnC stack"]\n'
                   '  PNC -->|"final drive command"| SIM\n'
                   '  SIM -->|"validation-only GT"| BAG["Recorded rosbag"]\n'
                   '  PF --> BAG\n'
                   '  PNC --> BAG\n'
                   '  META["Run metadata"] --> ANALYZER["Offline analyzer after shutdown"]\n'
                   '  BAG --> ANALYZER\n'
                   '  ANALYZER --> OUTPUTS["metrics.json, report.md, plots"]\n'
                   '```\n\n')
    else:
        report += ('The recorded platform was the physical vehicle: LiDAR and raw odometry → PF → estimated map pose → '
                   'PnC → autonomy /drive → mux /ackermann_cmd → VESC. Selected topics were passively recorded. '
                   'No simulator or independent ground truth was involved. Imported raw bags have retrospective evaluation '
                   'bounds and do not establish the deployed map, source revision, initialization or task completion.\n\n')
        report += ('```mermaid\nflowchart LR\n'
                   '  CAR["Vehicle sensors and odometry"] --> PF["Particle filter"]\n'
                   '  PF --> PNC["PnC"]\n'
                   '  PNC --> MUX["Ackermann mux and VESC"]\n'
                   '  CAR --> BAG["Passive recording"]\n'
                   '  PF --> BAG\n'
                   '  MUX --> BAG\n'
                   '  BAG --> ANALYZER["Offline analyzer"]\n'
                   '  META["Metadata and explicit limitations"] --> ANALYZER\n'
                   '  ANALYZER --> OUTPUTS["metrics.json, report.md, plots"]\n'
                   '```\n\n')
    report += ('After shutdown, the offline analyzer validates declared topic types, frames, timestamps, and coverage; aligns '
               'compatible evidence; then derives localization and vehicle metrics. The analyzer never publishes into the '
               'live graph and cannot influence localization or control.\n\n')
    report += '| Item | Recorded context |\n| --- | --- |\n'
    report += '| Platform | `{}` |\n'.format(metrics['platform'])
    report += '| Scenario / map | `{}` |\n'.format(context['scenario'].replace('|', '/'))
    report += '| Evaluation window | {:.3f} s |\n'.format(metrics['interval']['duration_sec'])
    report += '| Localization initialization | {} |\n'.format(context['initialization'])
    report += '| Launched stack | {} |\n'.format(context['flow'].replace('|', '/'))
    report += '| Recorded semantic signals | {} |\n\n'.format('<br>'.join(context['roles']) if context['roles'] else 'Not recorded')
    report += 'The run and report were produced by {}.\n\n'.format(', '.join(context['producers']))
    report += ('## 4. How to read the results\n\n'
               '| Term | Meaning in this report |\n| --- | --- |\n'
               '| p50 | Median: the middle evaluated value; a useful typical level. |\n'
               '| p95 | 95% of evaluated values are at or below this level. |\n'
               '| RMS | Root-mean-square magnitude; used here as typical oscillation amplitude. |\n'
               '| max | Largest observed value in the supported evidence. |\n'
               '| Coverage | Fraction of the relevant evaluation evidence valid enough to support the metric. |\n'
               '| `AVAILABLE` | Declared evidence supports the result. |\n'
               '| `NOT_APPLICABLE` | The metric is not meaningful for this evidence design. |\n'
               '| `UNAVAILABLE_DATA` | The metric is meaningful, but required evidence is missing or insufficient. |\n'
               '| `ANALYSIS_ERROR` | A declared input exists but violates its expected data, frame, or time contract. |\n\n'
               '| Evidence term | Meaning |\n| --- | --- |\n'
               '| Estimated map pose | PF estimate of vehicle position and heading in the global map frame. |\n'
               '| Ground truth (GT) | Simulator internal true pose, recorded only for independent evaluation. |\n'
               '| Raw odometry | Local motion/pose input used by PF; it can drift and is not absolute truth. |\n'
               '| Vehicle body frame | Coordinates fixed to the car: x is forward and angular z is yaw rate. |\n'
               '| Station / driven progress | Cumulative distance along the raw-odometry trajectory actually driven; it is a spatial coordinate, not publication time. |\n'
               '| Local trend | Slowly varying behavior estimated from a neighborhood around a station. |\n'
               '| Oscillation residual | Evaluated signal minus its local trend; shorter-scale variation around the intended maneuver. |\n\n')
    for number, section in ((5, 'localization'), (6, 'vehicle')):
        report += '## {}. {} results\n\n'.format(number, section.title())
        for group, items in metrics[section].items():
            report += '### ' + GROUP_LABELS.get(group, group.replace('_', ' ').title()) + '\n\n'
            if section == 'vehicle' and group == 'smoothness':
                report += ('Smoothness asks whether commands and vehicle response contain short-scale wiggle or chatter around an intended maneuver. '
                           'The analyzer associates each recorded sample with station, resamples it on a uniform spatial grid, estimates a slowly varying '
                           'local linear trend, and computes **oscillation residual = signal − local trend**. Removing the trend prevents normal acceleration, '
                           'braking, and steering through corners from automatically counting as vibration.\n\n')
                report += ('The {:.2f} m station spacing evaluates the signal about every {:.0f} cm of driven progress. The {:.2f} m trend window estimates '
                           'intended behavior from roughly a one-metre neighborhood. Gaps above {:.2f} m are not bridged. RMS is typical residual amplitude; '
                           'P95 absolute is the level below which 95% of absolute residuals fall. Lower values generally mean less short-scale chatter, but '
                           'do not alone prove better tracking, faster driving, or better control.\n\n').format(
                               metrics['settings']['oscillation_station_step_m'],
                               metrics['settings']['oscillation_station_step_m']*100,
                               metrics['settings']['oscillation_trend_window_m'],
                               metrics['settings']['oscillation_max_interpolation_gap_m'])
                report += '#### Command behavior — final plant-input command\n\nSimulation uses `/drive`; onboard uses post-mux `/ackermann_cmd` delivered toward VESC. These are commands, not measured steering.\n\n'
                report += report_metric_table(items, ['speed_command_oscillation', 'steering_command_oscillation'])
                report += '#### Upstream autonomy command\n\nThese optional rows use `/drive` on both platforms. Onboard mux output may differ; no servo command is treated as physical steering feedback.\n\n'
                report += report_metric_table(items, ['autonomy_speed_command_oscillation', 'autonomy_steering_command_oscillation'])
                report += ('#### Vehicle response\n\n' + ('These rows describe simulated vehicle motion in the vehicle body frame.' if metrics['platform'] == 'sim' else 'Forward speed uses ERPM-derived `/odom` body speed. Body yaw rate requires separately qualified gyro units, axis/sign, bias and timing; unknown qualification remains unavailable.') + '\n\n')
                report += report_metric_table(items, ['forward_speed_oscillation', 'yaw_rate_oscillation'])
                continue
            if section == 'vehicle' and group == 'tracking':
                report += GROUP_EXPLANATIONS[(section, group)] + '\n\n'
                report += '#### Shared PF-estimated tracking\n\n'
                report += report_metric_table(items, ['pf_reference_position_deviation_m', 'pf_heading_to_reference_rad'])
                report += ('#### Simulation-only GT tracking diagnostics\n\n' if metrics['platform'] == 'sim' else
                           '#### Legacy PF tracking fields (compatibility)\n\n')
                report += report_metric_table(items, ['static_reference_deviation_m', 'heading_to_reference_rad'])
                continue
            if section == 'vehicle' and group == 'dynamics_diagnostics' and all(
                    metric['status'] == 'UNAVAILABLE_DATA' for metric in items.values()):
                report += ('Longitudinal acceleration, longitudinal jerk, and lateral acceleration are all `UNAVAILABLE_DATA`. ' +
                           ('They require qualified physical source timing, while the legacy simulator publications use wall-clock stamps. ' if metrics['platform'] == 'sim' else 'This onboard run has no declared qualified vehicle-state adapter and physical-time evidence for these derivatives. ') +
                           'They remain in `metrics.json` as secondary diagnostics and are not fabricated from unqualified timing.\n\n')
                continue
            report += GROUP_EXPLANATIONS.get((section, group), '') + '\n\n'
            report += report_metric_table(items)
    report += '## 7. Data quality, conclusions, and limitations\n\n'
    report += 'Aligned absolute-accuracy samples: {}/{}. See `metrics.json` for effective thresholds, methods, complete structured values, and provenance.\n\n'.format(metrics['data_quality']['aligned_samples'], len(selected))
    report += ('**Supported:** the displayed measurements were '
               'derived under their declared evidence contracts. Completion is established only when the run-completed metric is AVAILABLE and true. **Not supported:** performance acceptance, controller causality, long-run racing robustness, '
               'or equivalence to future onboard evidence.\n\n')
    imu = metrics['data_quality'].get('raw_imu_diagnostics')
    if imu:
        report += '**Raw IMU diagnostics (not body-yaw metrics):** {} samples; observed frames `{}`; raw z range {} to {} (raw units {}). {}.\n\n'.format(
            imu.get('samples', 0), imu.get('observed_frames'), imu.get('raw_xyz_min', [None]*3)[2],
            imu.get('raw_xyz_max', [None]*3)[2], imu.get('unit', 'UNKNOWN'), imu.get('reason'))
    audit = metrics['provenance'].get('historical_audit', {})
    if audit:
        report += '| Historical deployment fact | Evidence classification |\n| --- | --- |\n'
        report += ''.join('| {} | {} |\n'.format(k.replace('_', ' '), v) for k, v in audit.items()) + '\n'
    report += ''.join('- '+v+'\n' for v in metrics['limitations'])
    report += ''.join('- ANALYSIS_ERROR: '+k+': '+v+'\n' for k, v in metrics['data_quality']['errors'].items())
    report += ('\n## 8. Reproduction and source references\n\n'
               '- [RoboRacer operational command reference](../../../docs/ROBORACER_OPERATIONAL_COMMAND_REFERENCE.md) '
               'contains the maintained recording, analysis, and comparison commands.\n'
               '- [Localization simulation and shared evaluation](../../../docs/LOCALIZATION_SIMULATION.md) '
               'defines the workflow, semantic signals, metric meanings, and evidence limits.\n'
               '- [Development environment](../../../docs/DEVELOPMENT_ENVIRONMENT.md) identifies the canonical ROS 2 Foxy container and workspace.\n\n')
    report += '## 9. Deliverables and plots\n\n`metrics.json` contains machine-readable results; `metadata.yaml`, `config/` and `rosbag/` retain run evidence. Plots are in `plots/` next to this report.\n\n'
    for name in generated:
        caption = PLOT_CAPTIONS.get(name, '')
        title = PLOT_TITLES.get(name, name.replace('_', ' ').title())
        if metrics['platform'] == 'onboard' and name == 'trajectory_xy.png':
            caption = 'Estimated map-frame trajectory only; no independent truth is available, so this plot cannot establish absolute localization accuracy.'
            title = 'Estimated map trajectory (proxy)'
        report += '### {}\n\n{}\n\n![{}](plots/{})\n\n'.format(
            title, caption, title, name)
        if name == 'pf_reference_trajectory.png' and (root/'pf_reference_deviation.md').is_file():
            report += (root/'pf_reference_deviation.md').read_text()+'\n'
        if name == 'pf_reference_trajectory.png' and (root/'simulation_trajectory_overlay.md').is_file():
            report += (root/'simulation_trajectory_overlay.md').read_text()+'\n'
        if name == 'pf_reference_trajectory.png' and (root/'heading_analysis.md').is_file():
            report += (root/'heading_analysis.md').read_text()+'\n'
        if name == 'pf_reference_trajectory.png' and (root/'control_analysis.md').is_file():
            report += (root/'control_analysis.md').read_text()+'\n'
        if name == 'localization_health.png' and metrics['platform'] == 'onboard' and (root/'pf_health_explanation.md').is_file():
            report += (root/'pf_health_explanation.md').read_text()+'\n'
    contents = '**Table of contents**\n\n' + ''.join(
        '- [{}](#{})\n'.format(h, re.sub(r'[^\w\- ]', '', h.lower()).replace(' ', '-'))
        for h in re.findall(r'^## (.+)$', report, flags=re.M)) + '\n'
    report = report.replace('## 2. What this run tested', contents + '## 2. What this run tested', 1)
    (root / 'metrics.json').write_text(json.dumps(metrics, indent=2, sort_keys=True, allow_nan=False)+'\n')
    (root / 'report.md').write_text(report)
    if (public is not None and len(public) and reference is not None and len(reference) >= 4
            and 'localization_health' in normalize_roles(metadata)):
        import evaluation_particle_spread
        evaluation_particle_spread.render_run(root)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    parser.add_argument('--analysis-config', type=Path, help='YAML overrides for documented analysis settings')
    for name in ('max-gt-gap-sec', 'max-pose-age-sec', 'max-status-gap-sec'):
        parser.add_argument('--' + name, type=float)
    args = parser.parse_args()
    try:
        input_metadata = yaml.safe_load((args.run_directory / 'metadata.yaml').read_text())
        if isinstance(input_metadata, dict) and 'rosbag2_bagfile_information' in input_metadata:
            print('ANALYSIS FAIL: raw rosbag detected; run import_onboard_bag.py first. Raw input unchanged.')
            return 1
        overrides = yaml.safe_load(args.analysis_config.read_text()) if args.analysis_config else {}
        overrides.update({key: value for key, value in vars(args).items() if key not in ('run_directory', 'analysis_config') and value is not None})
        metrics = analyze(args.run_directory.resolve(), overrides)
        print('ANALYSIS {}: {} pose samples, {} GT-aligned samples; outputs in {}'.format(metrics['analysis_status'], metrics['data_quality']['pose_evaluation_samples'], metrics['data_quality']['aligned_samples'], args.run_directory))
        return 0 if metrics['analysis_status'] == 'PASS' else 1
    except Exception as error:
        print('ANALYSIS FAIL: ' + str(error))
        if args.run_directory.is_dir():
            (args.run_directory / 'metrics.json').write_text(json.dumps({'schema_version': 2, 'analysis_status': 'FAIL', 'reason': str(error)}, indent=2)+'\n')
            (args.run_directory / 'report.md').write_text(
                '# Evaluation analysis failed\n\n## 1. Summary\n\nThe offline analysis did not complete.\n\n'
                '## 2. Failure reason\n\n'+str(error)+'\n\n'
                '## 3. Reproduction and source references\n\n'
                'See [the operational command reference](../../../docs/ROBORACER_OPERATIONAL_COMMAND_REFERENCE.md) '
                'and [evaluation contract](../../../docs/LOCALIZATION_SIMULATION.md).\n')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
