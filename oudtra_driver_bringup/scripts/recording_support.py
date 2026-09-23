"""Pure contract, provenance, and rosbag helpers for passive onboard recording."""
import hashlib
import os
from pathlib import Path
import signal
import sqlite3
import subprocess

import yaml


def file_sha256(path):
    path = Path(path)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_identity(path):
    """Return explicit revision/state or unknown; never infer missing Git facts."""
    path = Path(path).resolve()
    result = {'path': str(path), 'revision': None, 'source_state': 'unknown'}
    try:
        revision = subprocess.run(
            ['git', '-C', str(path), 'rev-parse', 'HEAD'], check=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True).stdout.strip()
        status = subprocess.run(
            ['git', '-C', str(path), 'status', '--porcelain'], check=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True).stdout
        result.update(revision=revision, source_state='dirty' if status else 'clean')
    except (OSError, subprocess.SubprocessError):
        result['reason'] = 'path is unavailable or not a readable Git worktree'
    return result


def load_contract(path):
    contract = yaml.safe_load(Path(path).read_text())
    if contract.get('schema_version') != 2 or contract.get('platform') != 'onboard':
        raise ValueError('Expected schema-2 onboard recording contract')
    roles = contract.get('roles', {})
    evidence = contract.get('evidence', {})
    required = dict(roles)
    required.update(evidence.get('required', {}))
    optional = evidence.get('optional', {})
    for group in (required, optional):
        for name, spec in group.items():
            if not spec.get('topic', '').startswith('/') or '/msg/' not in spec.get('type', ''):
                raise ValueError('Invalid topic/type for ' + name)
    topics = {}
    for required_flag, group in ((True, required), (False, optional)):
        for name, spec in group.items():
            topic = spec['topic']
            if topic in topics and topics[topic]['type'] != spec['type']:
                raise ValueError('Conflicting declared types for ' + topic)
            entry = topics.setdefault(topic, {'type': spec['type'], 'required': False, 'uses': [],
                                              'frames': [], 'diagnostic_names': []})
            entry['required'] = entry['required'] or required_flag
            entry['uses'].append(name)
            if spec.get('frame') and spec['frame'] not in entry['frames']:
                entry['frames'].append(spec['frame'])
            if spec.get('status') and spec['status'] not in entry['diagnostic_names']:
                entry['diagnostic_names'].append(spec['status'])
    return contract, topics


def classify_preflight(expected, graph, observations):
    """Classify graph/type/message evidence without requiring a live ROS import."""
    result = {'required_failures': [], 'optional_warnings': [], 'topics': {}}
    for topic, spec in sorted(expected.items()):
        actual_types = sorted(graph.get(topic, {}).get('types', []))
        publishers = int(graph.get(topic, {}).get('publishers', 0))
        observed = observations.get(topic, {})
        status = 'READY'
        reason = ''
        if publishers < 1 or not actual_types:
            status, reason = 'MISSING', 'no publisher discovered'
        elif spec['type'] not in actual_types:
            status, reason = 'TYPE_MISMATCH', 'expected {}; observed {}'.format(
                spec['type'], ', '.join(actual_types))
        elif observed.get('message_count', 0) < 1:
            status, reason = 'NO_MESSAGE', 'publisher/type present but no message observed'
        elif (spec['frames'] and observed.get('frame_id') and
              observed['frame_id'].lstrip('/') not in {value.lstrip('/') for value in spec['frames']}):
            status, reason = 'FRAME_MISMATCH', 'expected {}; observed {}'.format(
                ', '.join(spec['frames']), observed['frame_id'])
        elif (spec['diagnostic_names'] and
              not set(spec['diagnostic_names']) <= set(observed.get('diagnostic_names', []))):
            status, reason = 'DIAGNOSTIC_MISMATCH', 'expected status {}; observed {}'.format(
                ', '.join(spec['diagnostic_names']),
                ', '.join(observed.get('diagnostic_names', [])) or 'none')
        row = dict(spec)
        row.update(status=status, reason=reason, actual_types=actual_types,
                   publisher_count=publishers, observation=observed)
        result['topics'][topic] = row
        if status != 'READY':
            destination = 'required_failures' if spec['required'] else 'optional_warnings'
            result[destination].append('{}: {} ({})'.format(topic, status, reason))
    result['status'] = 'PASS' if not result['required_failures'] else 'FAIL'
    return result


def asset_record(path):
    if not path:
        return {'path': None, 'sha256': None, 'state': 'unknown'}
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        return {'path': str(resolved), 'sha256': None, 'state': 'unknown',
                'reason': 'file not found'}
    return {'path': str(resolved), 'sha256': file_sha256(resolved), 'state': 'available'}


def atomic_yaml(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(yaml.safe_dump(value, sort_keys=False))
    temporary.replace(path)


def create_run_directory(path):
    root = Path(path).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=False)
    (root / 'config').mkdir()
    (root / 'logs').mkdir()
    return root


def bag_counts(directory):
    counts = {}
    for path in Path(directory).glob('*.db3'):
        with sqlite3.connect('file:' + str(path) + '?mode=ro', uri=True, timeout=.1) as db:
            for topic, count in db.execute(
                    'SELECT topics.name, COUNT(messages.id) FROM topics '
                    'LEFT JOIN messages ON topics.id=messages.topic_id GROUP BY topics.id'):
                counts[topic] = counts.get(topic, 0) + count
    return counts


def stop_rosbag(process, bag_directory, timeout_sec=12):
    """Request Foxy rosbag finalization and validate the resulting SQLite bag."""
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
    try:
        code = process.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)
        raise RuntimeError('Recorder did not finalize on SIGINT')
    if code not in (0, 2):
        raise RuntimeError('Unexpected rosbag return code: ' + str(code))
    bag_directory = Path(bag_directory)
    if not (bag_directory / 'metadata.yaml').is_file():
        raise RuntimeError('Recorder did not write rosbag metadata.yaml')
    for path in bag_directory.glob('*.db3'):
        with sqlite3.connect('file:' + str(path) + '?mode=ro', uri=True) as db:
            if db.execute('PRAGMA integrity_check').fetchone()[0] != 'ok':
                raise RuntimeError('Bag SQLite integrity check failed')
    return code, bag_counts(bag_directory)
