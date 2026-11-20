"""Additive shared-evidence adapters; no live ROS interfaces or controller logic."""
import math
import numpy as np

HEALTH_STATES = ('GOOD', 'DEGRADED', 'INVALID', 'UNKNOWN')


def health_distribution(summary, duration, unknown_intervals):
    durations = {s: float(summary['duration_sec'].get(s, 0.)) for s in HEALTH_STATES[:-1]}
    durations['UNKNOWN'] = max(0., duration - sum(durations.values()))
    return {'duration_sec': durations,
            'percent': {s: 100.*v/duration for s, v in durations.items()},
            'observed_entries': summary['observed_entries'],
            'unknown_intervals_sec': unknown_intervals,
            'unknown_interval_count': len(unknown_intervals),
            'longest_unknown_interval_sec': max([b-a for a, b in unknown_intervals] or [0.])}


def qualify_scalar(spec, role):
    """Explicit semantic qualification, never a unit/frame guess from values."""
    if role == 'forward_speed':
        if spec.get('unit') != 'm/s' or spec.get('velocity_frame') != 'body':
            return None, 'Forward speed requires declared m/s body-frame evidence'
        return (1., 0.), ''
    q = spec.get('qualification', {})
    required = ('units', 'body_axis', 'bias', 'timestamp')
    if any(q.get(k) != 'VERIFIED' for k in required):
        return None, 'IMU units, body axis/sign, bias and timestamp qualification are not all VERIFIED'
    if not spec.get('frame') or spec.get('velocity_frame') != 'body' or spec.get('axis') != 'z':
        return None, 'Qualified gyro requires a nonempty observed body frame and z axis'
    unit = spec.get('unit')
    if unit not in ('rad/s', 'deg/s') or spec.get('sign') not in (-1, 1):
        return None, 'Invalid qualified gyro unit or sign'
    bias = spec.get('bias_radps')
    if not isinstance(bias, (int, float)) or not math.isfinite(bias):
        return None, 'Qualified gyro requires an explicit finite bias in rad/s'
    return (spec['sign'] * (math.pi/180 if unit == 'deg/s' else 1.), bias), ''


def scalar_signal(samples, spec, role):
    conversion, reason = qualify_scalar(spec, role)
    if conversion is None or samples is None or not len(samples):
        return None, reason or 'Missing scalar response samples'
    result = samples.copy()
    result[:, 1] = result[:, 1]*conversion[0] - conversion[1]
    return result, ''


def imu_diagnostics(records, origin, duration, spec):
    rows = [(received, msg) for received, msg in records if 0 <= (received-origin)*1e-9 < duration]
    if not rows:
        return {'qualification': 'UNKNOWN', 'reason': 'No IMU messages in evaluation interval'}
    values = np.asarray([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for _, m in rows])
    stamps = np.asarray([m.header.stamp.sec + m.header.stamp.nanosec*1e-9 for _, m in rows])
    if not np.all(np.isfinite(values)):
        return {'qualification': 'UNKNOWN', 'reason': 'Malformed nonfinite raw gyro evidence; see role ANALYSIS_ERROR'}
    age = np.asarray([r*1e-9 for r, _ in rows])-stamps
    conversion, reason = qualify_scalar(spec, 'yaw_rate')
    qualified = conversion is not None
    return {'qualification': 'VERIFIED' if qualified else 'UNKNOWN',
            'unit': spec.get('unit') if qualified else 'UNKNOWN',
            'body_axis_sign': 'VERIFIED' if qualified else 'UNKNOWN', 'bias': 'VERIFIED' if qualified else 'UNKNOWN',
            'observed_frames': sorted({m.header.frame_id for _, m in rows}), 'samples': len(rows),
            'raw_xyz_min': values.min(axis=0).tolist(), 'raw_xyz_max': values.max(axis=0).tolist(),
            'receive_age_sec_p50_p95_max': np.percentile(age, [50, 95, 100]).tolist(),
            'source_timestamps_strictly_increasing': bool(np.all(np.diff(stamps) > 0)),
            'reason': reason or 'Explicit adapter qualification supplied; raw diagnostics retain source units'}
