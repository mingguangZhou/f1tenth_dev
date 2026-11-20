"""Shared additive metrics and fail-closed onboard sensor qualification."""
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts'

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

A = load('shared_analyzer', SCRIPT / 'analyze_localization_run.py')
E = load('shared_evidence', SCRIPT / 'evaluation_evidence.py')
R = load('reanalysis', SCRIPT / 'reanalyze_localization_run.py')
F = load('fixtures', Path(__file__).with_name('test_localization_measurements.py'))
C = load('comparison_fixture', Path(__file__).with_name('test_evaluation_comparison.py'))


def test_pf_tracking_symmetric_and_distinct_from_gt(tmp_path):
    root = F.make_run(tmp_path)
    import sqlite3
    from rclpy.serialization import deserialize_message, serialize_message
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    with sqlite3.connect(str(root / 'rosbag/fixture.db3')) as db:
        topics = dict(db.execute('SELECT id,type FROM topics'))
        for ident, topic, payload in db.execute('SELECT id,topic_id,data FROM messages').fetchall():
            cls = {'nav_msgs/msg/Odometry': Odometry, 'tf2_msgs/msg/TFMessage': TFMessage}.get(topics[topic])
            if cls is None:
                continue
            msg = deserialize_message(payload, cls)
            if cls is TFMessage:
                msg.transforms[0].transform.translation.y = .4
            elif msg.header.frame_id.lstrip('/') == 'map':
                msg.pose.pose.position.y = .4
            db.execute('UPDATE messages SET data=? WHERE id=?', (serialize_message(msg), ident))
    result = A.analyze(root)
    track = result['vehicle']['tracking']
    assert track['static_reference_deviation_m']['evidence'] == 'gt_map_pose'
    assert track['pf_reference_position_deviation_m']['evidence'] == 'estimated_map_pose_proxy'
    assert track['static_reference_deviation_m']['value']['p50'] == pytest.approx(0.)
    assert track['pf_reference_position_deviation_m']['value']['p50'] == pytest.approx(.4)
    # A deliberately displaced reference exposes separate GT/PF position evidence.
    ref = np.array([[0., 1.], [3., 1.]])
    assert A.reference_errors(np.array([[0., 1., 0., 0., 0.]]), ref)[0][0] == 1.
    assert track['pf_heading_to_reference_rad']['value']['p95'] == pytest.approx(.1)
    old_accuracy = result['localization']['accuracy']
    def onboard(m):
        m['roles'] = A.normalize_roles(m)
        m['schema_version'] = 2
        m['platform'] = 'onboard'
        for key in ('ground_truth_pose', 'source_pose', 'collision_status'):
            m['roles'].pop(key, None)
    F.edit_metadata(root, onboard)
    onboard_result = A.analyze(root)
    assert onboard_result['vehicle']['tracking']['pf_heading_to_reference_rad'] == track['pf_heading_to_reference_rad']
    assert old_accuracy['position_error_m']['status'] == 'AVAILABLE'
    assert onboard_result['localization']['accuracy']['position_error_m']['status'] == 'NOT_APPLICABLE'


def test_health_percentages_keep_unknown_and_transitions():
    rows = [(-.1, 'GOOD'), (.1, 'INVALID'), (.9, 'DEGRADED')]
    summary = A.state_summary(rows, 1., .2)
    unknown = A.dropouts([(t, t+.2) for t, _ in rows], 1.)
    value = E.health_distribution(summary, 1., unknown)
    assert sum(value['percent'].values()) == pytest.approx(100.)
    assert value['percent']['UNKNOWN'] == pytest.approx(60.)
    assert value['duration_sec']['INVALID'] == pytest.approx(.2)
    assert value['observed_entries'] == {'INVALID': 1}
    assert value['unknown_interval_count'] == 1
    assert value['longest_unknown_interval_sec'] == pytest.approx(.6)
    text = A.human_result('health_time_distribution', {'status': 'AVAILABLE', 'value': value})
    assert 'UNKNOWN: 60.00%' in text and '{' not in text


def gyro_spec():
    return dict(unit='deg/s', frame='base_link', velocity_frame='body', axis='z', sign=-1,
                bias_radps=.1, qualification={k: 'VERIFIED' for k in ('units', 'body_axis', 'bias', 'timestamp')})


def test_split_sensor_units_and_bias_conversion():
    data = np.array([[0., 180., 0.], [1., 0., 1.]])
    result, reason = E.scalar_signal(data, gyro_spec(), 'yaw_rate')
    assert reason == ''
    assert result[:, 1] == pytest.approx([-np.pi-.1, -.1])
    assert np.array_equal(data[:, 1], [180., 0.])
    speed, reason = E.scalar_signal(data, dict(unit='m/s', velocity_frame='body'), 'forward_speed')
    assert np.array_equal(speed, data) and not reason
    missing, reason = E.scalar_signal(None, gyro_spec(), 'yaw_rate')
    assert missing is None and 'Missing' in reason


@pytest.mark.parametrize('key,value', [('unit', 'UNKNOWN'), ('unit', 'rpm'), ('frame', ''),
                                      ('sign', 0), ('bias_radps', float('nan')),
                                      ('qualification', {})])
def test_unqualified_gyro_never_claims_body_yaw(key, value):
    spec = gyro_spec()
    spec[key] = value
    samples, reason = E.scalar_signal(np.array([[0., 1., 0.]]), spec, 'yaw_rate')
    assert samples is None and reason


def test_decode_split_speed_imu_and_upstream_final_are_separate():
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from ackermann_msgs.msg import AckermannDriveStamped
    odom = Odometry()
    odom.header.stamp.sec = 100
    odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
    odom.twist.twist.linear.x = .7
    imu = Imu()
    imu.header.stamp.sec = 100
    imu.angular_velocity.z = 40.
    final, auto = AckermannDriveStamped(), AckermannDriveStamped()
    for m, v in [(final, 0.), (auto, 1.)]:
        m.header.stamp.sec = 100
        m.drive.speed = v
    specs = {'forward_speed': dict(frame='odom', body_frame='base_link'), 'yaw_rate': {},
             'final_drive_command': {}, 'autonomy_command': {}}
    messages = dict(zip(specs, [odom, imu, final, auto]))
    errors = {}
    data = A.decode({k: [(100000000000, m)] for k, m in messages.items()}, specs, 100000000000, errors, 1.)
    assert not errors
    assert data['forward_speed'][0, 1] == .7
    assert data['yaw_rate'][0, 1] == 40.
    assert data['autonomy_command'][0, 1] == 1. and data['final_drive_command'][0, 1] == 0.
    odom.child_frame_id = 'unqualified_sensor'
    A.decode({'forward_speed': [(100000000000, odom)]}, {'forward_speed': specs['forward_speed']}, 100000000000, errors, 1.)
    assert 'forward_speed' in errors
    imu.angular_velocity.z = float('nan')
    A.decode({'yaw_rate': [(100000000000, imu)]}, {'yaw_rate': {}}, 100000000000, errors, 1.)
    assert 'yaw_rate' in errors


def test_reanalysis_preserves_original_identity_and_rejects_overwrite(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    F.make_run(source)
    before = (source / 'metadata.yaml').read_bytes()
    contract = SCRIPT.parent / 'config/shared_evaluation_signals.yaml'
    first, second = tmp_path / 'first', tmp_path / 'second'
    a = R.prepare(source, first, contract)
    b = R.prepare(source, second, contract)
    assert a == b
    assert a.get('comparison_identity') is None
    assert (source / 'metadata.yaml').read_bytes() == before
    assert R.sha256(source / 'rosbag/fixture.db3') == R.sha256(first / 'rosbag/fixture.db3')
    with pytest.raises(ValueError, match='already exists'):
        R.prepare(source, first, contract)
    with pytest.raises(ValueError, match='separate'):
        R.prepare(source, source / 'nested', contract)


def test_optional_commands_missing_and_unknown_provenance_suppress_deltas(tmp_path):
    run = tmp_path / 'run'
    run.mkdir()
    F.make_run(run)
    result = A.analyze(run)
    assert result['vehicle']['smoothness']['autonomy_speed_command_oscillation']['status'] == 'UNAVAILABLE_DATA'
    assert result['localization']['availability']['health_time_distribution']['value']['percent']['GOOD'] == pytest.approx(100)
    assert result['data_quality']['health_definition']['qualification'] == 'UNKNOWN'
    left, right = tmp_path / 'left', tmp_path / 'right'
    for path in (left, right):
        path.mkdir()
        (path / 'metrics.json').write_text(json.dumps(result))
    comparison = C.C.compare(left, right, tmp_path / 'comparison')
    assert all(row['delta_right_minus_left'] is None for row in comparison['metrics'].values())


def test_health_deltas_require_verified_matching_thresholds(tmp_path):
    paths = [tmp_path / name for name in ('left', 'right')]
    for path in paths:
        result = C.fixture(path, 90.)
        metric = result['localization']['availability'].pop('percent')
        result['localization']['availability']['health_time_distribution'] = metric
        (path / 'metrics.json').write_text(json.dumps(result))
    output = tmp_path / 'comparison'
    result = C.C.compare(*paths, output)
    metric = result['metrics']['localization.availability.health_time_distribution']
    assert metric['delta_right_minus_left'] is None
    assert any('Health definitions' in reason for reason in metric['reasons'])
