"""Raw bag import preserves evidence and never invents a successful vehicle run."""
import importlib.util
from pathlib import Path
import sqlite3

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('onboard_import', ROOT / 'scripts/import_onboard_bag.py')
I = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(I)


@pytest.fixture
def bag(tmp_path):
    source = tmp_path / 'raw'
    source.mkdir()
    with sqlite3.connect(str(source / 'data.db3')) as db:
        db.execute('CREATE TABLE topics (id INTEGER,name TEXT,type TEXT,serialization_format TEXT)')
        db.execute('CREATE TABLE messages (id INTEGER,topic_id INTEGER,timestamp INTEGER,data BLOB)')
        db.execute("INSERT INTO topics VALUES(1,'/pf/pose/odom','nav_msgs/msg/Odometry','cdr')")
        db.executemany('INSERT INTO messages VALUES(?,1,?,?)', [(1,1000000000,b'raw'),(2,3000000000,b'raw')])
    info = {'storage_identifier': 'sqlite3', 'compression_format': '', 'relative_file_paths': ['data.db3'],
            'message_count': 2, 'topics_with_message_count': [{'topic_metadata': {'name': '/pf/pose/odom'}, 'message_count': 2}]}
    (source / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': info}))
    return source


def test_import_preserves_originals_and_unknown_provenance(bag, tmp_path):
    before = {p.name: I.sha256(p) for p in bag.iterdir()}
    output = tmp_path / 'analysis_run'
    data = I.import_bag(bag, output, ROOT / 'config/onboard_localization_recording.yaml', .1, 1.9)
    assert data['interval'] == {'start_wall_ns': 1100000000, 'end_wall_ns': 2900000000}
    assert data['platform'] == 'onboard' and 'ground_truth_pose' not in data['roles']
    assert data['main_revision'] == data['pf_revision'] == 'unknown'
    assert data['outcome']['completed'] is None
    assert 'reference_sha256' not in data['comparison_identity']
    assert 'vehicle_state' not in data['roles']
    assert before == {p.name: I.sha256(p) for p in bag.iterdir()}
    assert before == {p.name: I.sha256(p) for p in (output/'rosbag').iterdir()}
    copied = I.inspect_bag(output/'rosbag')
    copied['source'] = str(bag)
    assert I.inspect_bag(bag) == copied
    with pytest.raises(ValueError, match='already exists'):
        I.import_bag(bag, output, ROOT / 'config/onboard_localization_recording.yaml')


@pytest.mark.parametrize('start,end', [(-1,1),(0,0),(2,1),(0,4),(float('nan'),1)])
def test_bad_interval_rejected_before_output(bag, tmp_path, start, end):
    output = tmp_path / 'bad'
    with pytest.raises(ValueError, match='offset'):
        I.import_bag(bag, output, ROOT/'config/onboard_localization_recording.yaml', start,end)
    assert not output.exists()


def test_inconsistent_counts_and_type_are_rejected(bag, tmp_path):
    with sqlite3.connect(str(bag/'data.db3')) as db:
        db.execute("UPDATE topics SET type='std_msgs/msg/Float32'")
    with pytest.raises(ValueError, match='type mismatch'):
        I.import_bag(bag,tmp_path/'bad',ROOT/'config/onboard_localization_recording.yaml')
    with sqlite3.connect(str(bag/'data.db3')) as db:
        db.execute('DELETE FROM messages WHERE id=2')
    with pytest.raises(ValueError, match='counts disagree'):
        I.inspect_bag(bag)


def test_unsafe_database_and_recursive_output_rejected(bag, tmp_path):
    with pytest.raises(ValueError, match='inside'):
        I.import_bag(bag,bag/'nested',ROOT/'config/onboard_localization_recording.yaml')
    data=yaml.safe_load((bag/'metadata.yaml').read_text())
    data['rosbag2_bagfile_information']['relative_file_paths']=['../missing.db3']
    (bag/'metadata.yaml').write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='unsafe'):
        I.inspect_bag(bag)


def test_unfinalized_wal_is_not_silently_ignored(bag):
    Path(str(bag/'data.db3')+'-wal').write_bytes(b'pending evidence')
    with pytest.raises(ValueError, match='Nonempty SQLite WAL'):
        I.inspect_bag(bag)


def test_direct_raw_analysis_explains_import_and_does_not_write_inputs(bag, monkeypatch, capsys):
    spec = importlib.util.spec_from_file_location('raw_guard_analyzer', ROOT/'scripts/analyze_localization_run.py')
    analyzer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(analyzer)
    before = {p.name: I.sha256(p) for p in bag.iterdir()}
    monkeypatch.setattr('sys.argv', ['analyze_localization_run.py', str(bag)])
    assert analyzer.main() == 1
    assert 'import_onboard_bag.py first' in capsys.readouterr().out
    assert before == {p.name: I.sha256(p) for p in bag.iterdir()}
