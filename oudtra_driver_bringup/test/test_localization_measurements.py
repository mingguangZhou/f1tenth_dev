"""Deterministic observable metric contracts, independent of a running ROS graph."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

SPEC = importlib.util.spec_from_file_location(
    'analyzer', Path(__file__).resolve().parents[1] / 'scripts/analyze_localization_run.py')
A = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(A)


def test_heading_wrap_and_interpolation_cross_pi():
    data = np.array([[0, 0, 0, np.deg2rad(179)], [1, 2, 0, np.deg2rad(-179)]])
    values = A.interpolate(data, np.array([.5]), 1.1)
    assert values[0, 0] == 1
    assert abs(abs(values[0, 2])-np.pi) < 1e-10
    assert abs(A.wrapped(np.deg2rad(358))) == pytest.approx(np.deg2rad(2))


def test_alignment_never_bridges_gaps_or_extrapolates():
    data = np.array([[0, 0, 0, 0], [1, 2, 0, 0]])
    values = A.interpolate(data, np.array([-.1, 0, .5, 1, 1.1]), .1)
    assert np.all(np.isnan(values[[0, 2, 4]]))
    assert np.all(np.isfinite(values[[1, 3]]))


def test_availability_clips_and_unions_without_double_counting():
    assert A.coverage([(-1, .2), (.1, .5), (.8, 2)], 0, 1) == pytest.approx(.7)


def test_reference_distance_uses_segments_not_waypoints():
    assert A.segment_distances(np.array([[5, 2], [12, 0]]), np.array([[0, 0], [10, 0]])).tolist() == [2, 2]
    with pytest.raises(ValueError):
        A.segment_distances(np.array([[0, 0]]), np.array([[1, 1], [1, 1]]))


def test_emergency_count_is_entries_not_samples_and_gaps_are_unknown():
    result = A.emergency_events([(-.1, 'NOMINAL'), (.1, 'EMERGENCY_STOP'),
                                 (.2, 'EMERGENCY_STOP'), (.3, 'NOMINAL'),
                                 (.4, 'EMERGENCY_STOP'), (1.5, 'EMERGENCY_STOP')], 0, 2, .5)
    assert result == {'observed': True, 'observed_entries': 2,
                      'active_at_start': False, 'uncertain_entries': 1}


def test_missing_run_metadata_is_rejected(tmp_path):
    with pytest.raises(FileNotFoundError):
        A.analyze(tmp_path)


def make_run(root):
    """Small real CDR/SQLite fixture: straight one-metre motion, known 0.2 m PF error."""
    import math
    import sqlite3
    import yaml
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message
    from geometry_msgs.msg import TransformStamped, PoseStamped
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    contract = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/localization_recording.yaml').read_text())
    (root / 'rosbag').mkdir()
    metadata = dict(schema_version=1, roles=contract['roles'], analysis=contract['analysis'],
                    phases=[dict(state='RUNNING', wall_ns=100000000000, monotonic_ns=0),
                            dict(state='EVALUATION_END', wall_ns=101000000000, monotonic_ns=1000000000)],
                    recording={'status': 'PASS'}, smoke_result={'result': 'PASS'})
    (root / 'metadata.yaml').write_text(yaml.safe_dump(metadata))
    (root / 'rosbag/metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
        'storage_identifier': 'sqlite3', 'relative_file_paths': ['fixture.db3']}}))
    with sqlite3.connect(str(root / 'rosbag/fixture.db3')) as db:
        db.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT)')
        db.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)')
        for ident, (role, spec) in enumerate(contract['roles'].items(), 1):
            db.execute('INSERT INTO topics VALUES(?,?,?,?)', (ident, spec['topic'], spec['type'], 'cdr'))
            for i in range(-1, 22):
                elapsed = i * .05
                ns = 100000000000 + i*50000000
                msg = get_message(spec['type'])()
                def set_header(header):
                    header.frame_id = spec.get('frame', 'map')
                    header.stamp.sec, header.stamp.nanosec = divmod(ns, 1000000000)
                if hasattr(msg, 'header'):
                    set_header(msg.header)
                if role == 'truth':
                    status = DiagnosticStatus(name='simulator/ego')
                    status.values = [KeyValue(key=k, value=str(v)) for k, v in
                                     {'x_m': elapsed, 'y_m': 0, 'yaw_rad': 0, 'collision': False}.items()]
                    msg.status = [status]
                elif role in ('estimated_pose', 'raw_odometry'):
                    msg.pose.pose.position.x = elapsed + .2
                    msg.pose.pose.orientation.z = math.sin(.05)
                    msg.pose.pose.orientation.w = math.cos(.05)
                elif role == 'source_pose':
                    tf = TransformStamped(child_frame_id='ego_racecar/base_link')
                    set_header(tf.header)
                    tf.transform.translation.x = elapsed + .2
                    tf.transform.rotation.z = math.sin(.05)
                    tf.transform.rotation.w = math.cos(.05)
                    msg.transforms = [tf]
                elif role == 'health':
                    msg.data = [1.]
                elif role == 'arbitration':
                    status = DiagnosticStatus(name=spec['status'])
                    status.values = [KeyValue(key='mode', value='RACELINE')]
                    msg.status = [status]
                elif role == 'safety':
                    status = DiagnosticStatus(name=spec['status'])
                    status.values = [KeyValue(key='mode', value='EMERGENCY_STOP' if i in (10, 11) else 'NOMINAL')]
                    msg.status = [status]
                elif role == 'reference':
                    for x in (0., 2.):
                        pose = PoseStamped()
                        pose.pose.position.x = x
                        pose.pose.orientation.w = 1.
                        msg.poses.append(pose)
                db.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)',
                           (ident, ns+20000000, serialize_message(msg)))
    return root


def test_real_cdr_run_metrics_and_reanalysis_are_deterministic(tmp_path):
    import json
    root = make_run(tmp_path)
    result = A.analyze(root)
    assert result['localization']['accuracy']['position_error_m']['value']['p95'] == pytest.approx(.2)
    assert result['localization']['accuracy']['absolute_heading_error_rad']['value']['p95'] == pytest.approx(.1)
    assert result['vehicle']['pace']['distance_travelled_m']['value'] == pytest.approx(1)
    assert result['vehicle']['pace']['mean_elapsed_time_speed_mps']['value'] == pytest.approx(1)
    assert result['vehicle']['robustness']['emergency_stops']['value']['observed_entries'] == 1
    assert result['vehicle']['robustness']['collision_observed']['value'] is False
    first = [(root / name).read_bytes() for name in ('metrics.json', 'report.md')]
    report = first[1].decode()
    assert '## 2. What this run tested' in report
    assert '## 3. How the run was executed' in report
    assert '## 4. Localization results' in report
    assert '## 5. Vehicle results' in report
    assert '## 7. Reproduction and source references' in report
    assert '`estimated_pose`' in report and '`/pf/pose/odom`' in report
    assert '../../../docs/ROBORACER_OPERATIONAL_COMMAND_REFERENCE.md' in report
    A.analyze(root)
    assert first == [(root / name).read_bytes() for name in ('metrics.json', 'report.md')]
    assert len(list((root / 'plots').glob('*.png'))) == 4
    assert json.loads(first[0])['analysis_status'] == 'PASS'


def test_missing_gt_messages_fail_instead_of_reporting_zero_error(tmp_path):
    import sqlite3
    root = make_run(tmp_path)
    with sqlite3.connect(str(root / 'rosbag/fixture.db3')) as db:
        db.execute("DELETE FROM messages WHERE topic_id IN (SELECT id FROM topics WHERE name='/simulator/agent_status')")
    result = A.analyze(root)
    assert result['localization']['accuracy']['position_error_m']['status'] == 'UNAVAILABLE_DATA'
    assert result['localization']['accuracy']['position_error_m']['value'] is None


def settings():
    return A.load_settings({}, {})


def edit_metadata(root, change):
    import yaml
    path = root / 'metadata.yaml'
    metadata = yaml.safe_load(path.read_text())
    change(metadata)
    path.write_text(yaml.safe_dump(metadata))


def test_no_gt_onboard_and_no_tf_are_valid(tmp_path):
    root = make_run(tmp_path)
    def onboard(m):
        m['platform'] = 'onboard'
        m['roles'].pop('truth')
        m['roles'].pop('source_pose')
    edit_metadata(root, onboard)
    result = A.analyze(root)
    assert result['analysis_status'] == 'PASS'
    assert result['localization']['accuracy']['position_error_m']['status'] == 'NOT_APPLICABLE'
    assert result['vehicle']['pace']['distance_travelled_m']['value'] == pytest.approx(1)
    assert result['vehicle']['tracking']['static_reference_deviation_m']['status'] == 'AVAILABLE'
    assert result['vehicle']['smoothness']['longitudinal_jerk_mps3']['status'] == 'UNAVAILABLE_DATA'
    assert not (root / 'plots/position_error.png').exists()


def test_shared_topic_and_physical_state_adapter(tmp_path):
    root = make_run(tmp_path)
    def change(m):
        m['roles']['vehicle_state'] = dict(m['roles']['raw_odometry'], physical_time=True,
                                         velocity_frame='body', lateral_velocity_observed=True)
    edit_metadata(root, change)
    result = A.analyze(root)
    assert result['vehicle']['smoothness']['longitudinal_acceleration_mps2']['value']['max'] == 0
    assert result['data_quality']['message_counts']['vehicle_state'] == result['data_quality']['message_counts']['raw_odometry']


def test_malformed_optional_signal_marks_analysis_error(tmp_path):
    root = make_run(tmp_path)
    edit_metadata(root, lambda m: m['roles']['command'].update(type='std_msgs/msg/Float32'))
    result = A.analyze(root)
    assert result['analysis_status'] == 'FAIL'
    assert result['vehicle']['smoothness']['command_steering_rate_radps']['status'] == 'ANALYSIS_ERROR'
    assert result['localization']['accuracy']['position_error_m']['status'] == 'AVAILABLE'


def test_invalid_cdr_is_not_missing_data(tmp_path):
    import sqlite3
    root = make_run(tmp_path)
    with sqlite3.connect(str(root / 'rosbag/fixture.db3')) as db:
        db.execute("UPDATE messages SET data=? WHERE topic_id IN (SELECT id FROM topics WHERE name='/drive')", (b'broken',))
    result = A.analyze(root)
    assert result['vehicle']['smoothness']['command_steering_rate_radps']['status'] == 'ANALYSIS_ERROR'


def test_unknown_health_is_error_not_healthy(tmp_path):
    root = make_run(tmp_path)
    edit_metadata(root, lambda m: m['roles']['health'].update(states={'2': 'DEGRADED'}))
    assert A.analyze(root)['localization']['availability']['health_states']['status'] == 'ANALYSIS_ERROR'


def test_dropout_complement_and_state_unknown_duration():
    assert A.dropouts([(.2, .4), (.3, .5), (.8, 1.2)], 1) == [[0., .2], [.5, .8]]
    summary = A.state_summary([(-.1, 'GOOD'), (.1, 'INVALID'), (.9, 'GOOD')], 1, .2)
    assert summary['duration_sec']['GOOD'] == pytest.approx(.2)
    assert summary['unknown_duration_sec'] == pytest.approx(.6)
    assert summary['observed_entries'] == {'INVALID': 1}


def test_relative_increment_invariant_to_global_rigid_frame():
    t = np.arange(0, 2.01, .05)
    odom = np.column_stack((t, t, t*0, t*0))
    pose = np.column_stack((t, t*0+3, t+4, t*0+np.pi/2))
    result = A.consistency(pose, odom, 2, settings())
    assert np.nanmax(result[:, 1:]) < 1e-12
    pose[:, 2] *= 2
    result = A.consistency(pose, odom, 2, settings())
    assert result[0, 1] == pytest.approx(.5)
    odom = odom[(t < .4) | (t > 1.5)]
    assert np.any(np.isnan(A.consistency(pose, odom, 2, settings())[:, 1]))


def test_windowed_acceleration_jerk_and_lateral_units():
    t = np.arange(0, 3.001, .025)
    state = np.column_stack((t, 2+3*t+4*t*t, .5*t, t*0+.2, t))
    result = A.windowed_dynamics(state, 3, settings())
    assert result[:, 1] == pytest.approx(3+8*result[:, 0])
    assert result[:, 2] == pytest.approx(np.full(len(result), 8.))
    assert result[:, 3] == pytest.approx(.5 + .2*(2+3*result[:, 0]+4*result[:, 0]**2))
    assert not len(A.windowed_dynamics(state[::10], 3, settings()))


def test_reference_heading_ambiguous_segments_excluded():
    vehicle = np.array([[0, 1, 1, 0], [1, 2, 0, 0]])
    distance, heading = A.reference_errors(vehicle, np.array([[0, 0], [2, 0], [2, 2]]))
    assert distance == pytest.approx([1, 0])
    assert np.all(np.isnan(heading))
    _, heading = A.reference_errors(vehicle[:1], np.array([[0, 0], [2, 0]]))
    assert heading[0] == 0


def test_stall_requires_command_authority_and_not_emergency():
    t = np.arange(0, 3.001, .05)
    command = np.column_stack((t, t*0+1, t*0, t))
    state = np.column_stack((t, t*0, t*0, t*0, t))
    control = [(x, 'AUTO') for x in t]
    safety = [(x, 'NOMINAL') for x in t]
    episodes, covered = A.stall_episodes(command, state, control, safety, 3, settings(), ['AUTO'], 'STOP')
    assert episodes == [[0, 3]] and covered == pytest.approx(1)
    command[:, 1] = 0
    assert not A.stall_episodes(command, state, control, safety, 3, settings(), ['AUTO'], 'STOP')[0]
    command[:, 1] = 1
    assert not A.stall_episodes(command, state, control, [(x, 'STOP') for x in t], 3, settings(), ['AUTO'], 'STOP')[0]
    assert not A.stall_episodes(command, state, control, safety, 3, settings(), ['OTHER'], 'STOP')[0]


def test_source_time_contract_rejects_conflicting_duplicates_and_reversal():
    with pytest.raises(ValueError, match='Conflicting'):
        A.ordered([[0, 0, 0, 0, .1], [0, 1, 0, 0, .2]], 5)
    with pytest.raises(ValueError, match='reversed'):
        A.ordered([[1, 0, 0, 0, 1.1], [0, 0, 0, 0, 1.2]], 5)
    assert len(A.ordered([[0, 0, 0, 0, .1], [0, 0, 0, 0, .2]], 5)) == 1


def test_fast_motion_wrap_and_gaps_do_not_become_jumps():
    poses = np.array([[0, 0, 0, 3.13], [.1, .9, 0, -3.13], [.2, 5, 0, -3.13], [2, 100, 0, 0]])
    adjacent, dp, heading, flagged = A.discontinuities(poses, settings())
    assert adjacent.tolist() == [True, True, False]
    assert flagged.tolist() == [False, True, False]
    assert heading[0] < .03


def test_unstamped_cleanup_command_excluded_but_in_window_rejected(tmp_path):
    import sqlite3
    from ackermann_msgs.msg import AckermannDriveStamped
    from rclpy.serialization import serialize_message
    root = make_run(tmp_path)
    with sqlite3.connect(str(root / 'rosbag/fixture.db3')) as db:
        topic = db.execute("SELECT id FROM topics WHERE name='/drive'").fetchone()[0]
        db.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)',
                   (topic, 102000000000, serialize_message(AckermannDriveStamped())))
    assert A.analyze(root)['analysis_status'] == 'PASS'
    with sqlite3.connect(str(root / 'rosbag/fixture.db3')) as db:
        db.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)',
                   (topic, 100500000000, serialize_message(AckermannDriveStamped())))
    assert A.analyze(root)['vehicle']['smoothness']['command_steering_rate_radps']['status'] == 'ANALYSIS_ERROR'


def test_command_ramp_reports_rate_not_unset_rate_field(tmp_path):
    import sqlite3
    from rclpy.serialization import deserialize_message, serialize_message
    from ackermann_msgs.msg import AckermannDriveStamped
    root = make_run(tmp_path)
    with sqlite3.connect(str(root / 'rosbag/fixture.db3')) as db:
        for ident, payload in db.execute("SELECT messages.id,data FROM messages JOIN topics ON messages.topic_id=topics.id WHERE topics.name='/drive'").fetchall():
            msg = deserialize_message(payload, AckermannDriveStamped)
            msg.drive.steering_angle = (A.stamp(msg.header)-100000000000)*1e-9*.2
            db.execute('UPDATE messages SET data=? WHERE id=?', (serialize_message(msg), ident))
    result = A.analyze(root)['vehicle']['smoothness']['command_steering_rate_radps']
    assert result['value']['p95'] == pytest.approx(.2, abs=1e-6)


def test_missing_optional_reference_is_unavailable_and_stale_plot_removed(tmp_path):
    import sqlite3
    root = make_run(tmp_path)
    A.analyze(root)
    with sqlite3.connect(str(root / 'rosbag/fixture.db3')) as db:
        db.execute("DELETE FROM messages WHERE topic_id IN (SELECT id FROM topics WHERE name='/raceline_path')")
    result = A.analyze(root)
    assert result['vehicle']['tracking']['static_reference_deviation_m']['status'] == 'UNAVAILABLE_DATA'
    assert not (root / 'plots/reference_trajectory.png').exists()


def test_sim_wall_clock_cannot_claim_physical_derivatives(tmp_path):
    result = A.analyze(make_run(tmp_path))
    for name in ('longitudinal_acceleration_mps2', 'longitudinal_jerk_mps3', 'lateral_acceleration_mps2'):
        assert result['vehicle']['smoothness'][name]['status'] == 'UNAVAILABLE_DATA'


def test_cli_malformed_optional_data_returns_failure(tmp_path, monkeypatch):
    root = make_run(tmp_path)
    edit_metadata(root, lambda m: m['roles']['command'].update(type='std_msgs/msg/Float32'))
    monkeypatch.setattr('sys.argv', ['analyze_localization_run.py', str(root)])
    assert A.main() == 1
    assert 'ANALYSIS_ERROR' in (root / 'report.md').read_text()


def test_invalid_interval_and_no_core_data_are_fatal(tmp_path):
    import sqlite3
    root = make_run(tmp_path)
    with sqlite3.connect(str(root / 'rosbag/fixture.db3')) as db:
        db.execute("DELETE FROM messages WHERE topic_id IN (SELECT id FROM topics WHERE name='/tf')")
    with pytest.raises(ValueError, match='core estimated pose'):
        A.analyze(root)
    edit_metadata(root, lambda m: m.update(interval={'start_wall_ns': 100, 'end_wall_ns': 100}))
    with pytest.raises(ValueError, match='duration'):
        A.analyze(root)


def test_declared_different_frames_are_not_compared_numerically(tmp_path):
    root = make_run(tmp_path)
    edit_metadata(root, lambda m: m['roles']['truth'].update(frame='different_map'))
    result = A.analyze(root)
    assert result['localization']['accuracy']['position_error_m']['status'] == 'ANALYSIS_ERROR'
    assert result['vehicle']['tracking']['static_reference_deviation_m']['status'] == 'ANALYSIS_ERROR'
