"""Passive onboard recorder contract tests; no physical vehicle is required."""
import importlib.util
from pathlib import Path
import sqlite3
from unittest.mock import Mock

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'recording_support', ROOT / 'scripts/recording_support.py')
R = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(R)
CONTRACT = ROOT / 'config/onboard_localization_recording.yaml'


def ready_graph(topics):
    graph = {topic: {'types': [spec['type']], 'publishers': 1}
             for topic, spec in topics.items()}
    observed = {
        topic: {'message_count': 1,
                **({'frame_id': spec['frames'][0]} if spec['frames'] else {}),
                **({'diagnostic_names': list(spec['diagnostic_names'])}
                   if spec['diagnostic_names'] else {})}
        for topic, spec in topics.items()}
    return graph, observed


def test_onboard_contract_has_no_gt_and_uses_mux_output():
    contract, topics = R.load_contract(CONTRACT)
    assert (contract, topics) == R.load_contract(CONTRACT)
    assert contract['platform'] == 'onboard'
    assert 'ground_truth_pose' not in contract['roles']
    assert contract['roles']['final_drive_command']['topic'] == '/ackermann_cmd'
    assert contract['evidence']['required']['upstream_autonomy_command']['topic'] == '/drive'
    assert contract['roles']['raw_odometry']['topic'] == '/odom'
    assert contract['evidence']['required']['measured_imu']['topic'] == '/sensors/imu/raw'
    assert topics['/ackermann_cmd']['required'] is True
    assert topics['/initialpose']['required'] is False


def test_required_preflight_success_and_failure():
    _, topics = R.load_contract(CONTRACT)
    graph, observed = ready_graph(topics)
    assert R.classify_preflight(topics, graph, observed)['status'] == 'PASS'
    del graph['/pf/pose/odom']
    result = R.classify_preflight(topics, graph, observed)
    assert result['status'] == 'FAIL'
    assert any('/pf/pose/odom: MISSING' in value for value in result['required_failures'])


def test_optional_absence_is_warning_only():
    _, topics = R.load_contract(CONTRACT)
    graph, observed = ready_graph(topics)
    del graph['/initialpose']
    observed.pop('/initialpose')
    result = R.classify_preflight(topics, graph, observed)
    assert result['status'] == 'PASS'
    assert result['optional_warnings'] == [
        '/initialpose: MISSING (no publisher discovered)']


def test_type_mismatch_and_no_message_are_distinct():
    _, topics = R.load_contract(CONTRACT)
    graph, observed = ready_graph(topics)
    graph['/scan']['types'] = ['std_msgs/msg/String']
    observed['/odom']['message_count'] = 0
    result = R.classify_preflight(topics, graph, observed)
    assert result['topics']['/scan']['status'] == 'TYPE_MISMATCH'
    assert result['topics']['/odom']['status'] == 'NO_MESSAGE'


def test_declared_frame_mismatch_fails_preflight():
    _, topics = R.load_contract(CONTRACT)
    graph, observed = ready_graph(topics)
    observed['/odom']['frame_id'] = 'unexpected_odom'
    result = R.classify_preflight(topics, graph, observed)
    assert result['status'] == 'FAIL'
    assert result['topics']['/odom']['status'] == 'FRAME_MISMATCH'


def test_existing_directory_rejection_contract(tmp_path):
    root = tmp_path / 'run'
    assert R.create_run_directory(root) == root.resolve()
    with pytest.raises(FileExistsError):
        R.create_run_directory(root)


def test_git_and_asset_provenance_are_explicit_and_deterministic(tmp_path):
    asset = tmp_path / 'asset.yaml'
    asset.write_text('value: 1\n')
    assert R.asset_record(asset) == R.asset_record(asset)
    assert R.asset_record(asset)['state'] == 'available'
    assert R.asset_record(tmp_path / 'missing')['state'] == 'unknown'
    unknown = R.git_identity(tmp_path / 'missing-repository')
    assert unknown['revision'] is None and unknown['source_state'] == 'unknown'


def test_atomic_metadata_write_is_deterministic(tmp_path):
    destination = tmp_path / 'metadata.yaml'
    value = {'schema_version': 2, 'platform': 'onboard', 'roles': {'a': {'topic': '/a'}}}
    R.atomic_yaml(destination, value)
    first = destination.read_bytes()
    R.atomic_yaml(destination, value)
    assert destination.read_bytes() == first
    assert yaml.safe_load(first) == value
    assert not destination.with_suffix('.yaml.tmp').exists()


def test_clean_rosbag_finalization_and_counts(tmp_path, monkeypatch):
    bag = tmp_path / 'rosbag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text('ok: true\n')
    with sqlite3.connect(str(bag / 'run.db3')) as db:
        db.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT)')
        db.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER)')
        db.execute("INSERT INTO topics VALUES(1, '/odom')")
        db.execute('INSERT INTO messages VALUES(1, 1)')
    process = Mock()
    process.poll.return_value = None
    process.pid = 123
    process.wait.return_value = 2
    monkeypatch.setattr(R.os, 'killpg', Mock())
    code, counts = R.stop_rosbag(process, bag)
    assert code == 2 and counts == {'/odom': 1}
    R.os.killpg.assert_called_once_with(123, R.signal.SIGINT)


def test_recorder_source_has_no_application_publishers_or_mutations():
    source = (ROOT / 'scripts/record_onboard_localization.py').read_text()
    assert 'create_publisher' not in source
    assert '.publish(' not in source
    assert 'set_parameters' not in source
    assert 'create_client' not in source
    assert '/initialpose' not in source
