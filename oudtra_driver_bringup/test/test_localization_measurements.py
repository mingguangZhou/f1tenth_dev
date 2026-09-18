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
                    header.frame_id = 'map'
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
    assert result['localization']['position_error_m']['p95'] == pytest.approx(.2)
    assert result['localization']['absolute_heading_error_rad']['p95'] == pytest.approx(.1)
    assert result['vehicle']['distance_travelled_m'] == pytest.approx(1)
    assert result['vehicle']['mean_elapsed_time_speed_mps'] == pytest.approx(1)
    assert result['vehicle']['emergency_stops']['observed_entries'] == 1
    assert result['vehicle']['collision_observed'] is False
    first = [(root / name).read_bytes() for name in ('metrics.json', 'report.md')]
    A.analyze(root)
    assert first == [(root / name).read_bytes() for name in ('metrics.json', 'report.md')]
    assert len(list((root / 'plots').glob('*.png'))) == 3
    assert json.loads(first[0])['analysis_status'] == 'PASS'


def test_missing_gt_messages_fail_instead_of_reporting_zero_error(tmp_path):
    import sqlite3
    root = make_run(tmp_path)
    with sqlite3.connect(str(root / 'rosbag/fixture.db3')) as db:
        db.execute("DELETE FROM messages WHERE topic_id IN (SELECT id FROM topics WHERE name='/simulator/agent_status')")
    with pytest.raises(ValueError, match='Missing recorded role: truth'):
        A.analyze(root)
