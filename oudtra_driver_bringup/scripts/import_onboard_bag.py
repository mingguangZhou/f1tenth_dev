#!/usr/bin/env python3
"""Prepare a raw onboard SQLite bag for offline analysis, without ROS replay."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3

import yaml


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def inspect_bag(source):
    """Verify immutable raw files and metadata counts; do not claim CDR validity."""
    source = Path(source).expanduser().resolve()
    info = yaml.safe_load((source / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    if info['storage_identifier'] != 'sqlite3' or info.get('compression_format'):
        raise ValueError('Expected an uncompressed SQLite/CDR raw bag')
    counts, types, hashes, first, last = {}, {}, {}, None, None
    for name in info['relative_file_paths']:
        path = (source / name).resolve()
        if source not in path.parents or not path.is_file():
            raise ValueError('Missing or unsafe database path: ' + name)
        wal = Path(str(path) + '-wal')
        if wal.exists() and wal.stat().st_size:
            raise ValueError('Nonempty SQLite WAL; finalize/checkpoint the bag before import')
        hashes[name] = sha256(path)
        with sqlite3.connect('file:' + str(path) + '?mode=ro&immutable=1', uri=True) as db:
            if db.execute('PRAGMA integrity_check').fetchone()[0] != 'ok':
                raise ValueError('Invalid SQLite database: ' + name)
            for topic, typ, encoding, count in db.execute(
                    'SELECT t.name,t.type,t.serialization_format,count(m.id) FROM topics t '
                    'LEFT JOIN messages m ON t.id=m.topic_id GROUP BY t.id'):
                if encoding != 'cdr' or topic in types and types[topic] != typ:
                    raise ValueError('Inconsistent topic type/encoding: ' + topic)
                types[topic] = typ
                counts[topic] = counts.get(topic, 0) + count
            begin, end = db.execute('SELECT min(timestamp),max(timestamp) FROM messages').fetchone()
            if begin is not None:
                first = begin if first is None else min(first, begin)
                last = end if last is None else max(last, end)
    declared = {r['topic_metadata']['name']: r['message_count'] for r in info['topics_with_message_count']}
    if counts != declared or sum(counts.values()) != info['message_count']:
        raise ValueError('Bag metadata/message counts disagree')
    if first is None or last <= first:
        raise ValueError('Bag has no nonzero recording interval')
    hashes['metadata.yaml'] = sha256(source / 'metadata.yaml')
    return {'source': str(source), 'start_ns': first, 'end_ns': last,
            'duration_sec': (last-first)*1e-9, 'message_count': sum(counts.values()),
            'topics': {t: {'type': types[t], 'count': counts[t]} for t in sorted(types)},
            'sha256': hashes, 'integrity': 'PASS'}


def import_bag(source, output, contract_path, start_offset_sec=0., end_offset_sec=None, scenario_id=None):
    """Copy into a self-contained run; completion and deployed provenance stay unknown."""
    audit = inspect_bag(source)
    source, output = Path(audit['source']), Path(output).expanduser().resolve()
    if output.exists():
        raise ValueError('Output already exists; choose a new run directory')
    if source in output.parents:
        raise ValueError('Output must not be inside the source raw bag')
    end_offset_sec = audit['duration_sec'] if end_offset_sec is None else end_offset_sec
    if not 0 <= start_offset_sec < end_offset_sec <= audit['duration_sec']:
        raise ValueError('Require 0 <= start offset < end offset <= bag duration')
    contract_path = Path(contract_path)
    contract = yaml.safe_load(contract_path.read_text())
    if contract.get('platform') != 'onboard' or 'ground_truth_pose' in contract['roles']:
        raise ValueError('Require an onboard contract without simulator GT')
    roles, absent = {}, []
    for role, spec in contract['roles'].items():
        topic = audit['topics'].get(spec['topic'])
        if not topic or not topic['count']:
            absent.append(role)
        elif topic['type'] != spec['type']:
            raise ValueError('Topic type mismatch: ' + spec['topic'])
        else:
            roles[role] = dict(spec)
    if 'estimated_pose' not in roles:
        raise ValueError('No estimated_pose evidence in raw bag')
    notes = [
        'Imported raw rosbag: evaluation bounds were selected offline, not operator run-state events.',
        'Deployed source/PF revisions, map hashes and PF/PnC parameters were not recorded; they remain unknown.',
        'Completion, collision absence and initialization/readiness are not established by a readable bag.',
        'No ground truth onboard: tracking and odometry consistency are proxy evidence, not absolute accuracy.',
        'Availability measures fresh pose delivery, not GOOD health; read the health-state durations separately.',
        'Arbitration modes do not prove mux/joystick authority; that control-source state was not recorded.',
        'Measured IMU and odometry are preserved; the split vehicle-state adapter is not implemented, so vehicle-response smoothness is unavailable.']
    if scenario_id:
        notes.append('Scenario label supplied during import; it does not prove the loaded PF map or configuration.')
    metadata = {'schema_version': 2, 'platform': 'onboard', 'run_id': output.name,
                'scenario': scenario_id, 'main_revision': 'unknown', 'pf_revision': 'unknown',
                'source_state': 'unknown', 'configs': {}, 'roles': roles,
                'comparison_identity': {'scenario_id': scenario_id},
                'recording': {'status': 'IMPORTED', 'meaning': 'SQLite integrity/counts verified; CDR validated by analyzer'},
                'interval': {'start_wall_ns': audit['start_ns'] + round(start_offset_sec*1e9),
                             'end_wall_ns': audit['start_ns'] + round(end_offset_sec*1e9)},
                'outcome': {'completed': None}, 'limitations': notes,
                'description': 'Real-car scan/raw odometry fed PF; estimated map pose fed PnC; '
                               'autonomy /drive passed through mux /ackermann_cmd toward VESC. '
                               'A passive raw rosbag was imported offline; actual stack configuration is unverified.',
                'raw_import': dict(audit, missing_analysis_roles=absent,
                                   start_offset_sec=start_offset_sec, end_offset_sec=end_offset_sec),
                'software_sha256': {'import_onboard_bag.py': sha256(__file__)}}
    output.mkdir(parents=True)
    shutil.copytree(source, output / 'rosbag')
    (output / 'config').mkdir()
    shutil.copyfile(contract_path, output / 'config/onboard_localization_recording.yaml')
    analysis = contract_path.with_name('evaluation_analysis.yaml')
    if analysis.is_file():
        shutil.copyfile(analysis, output / 'config/evaluation_analysis.yaml')
        metadata['analysis'] = yaml.safe_load(analysis.read_text())
    (output / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    (output / 'import_audit.json').write_text(json.dumps(audit, indent=2, sort_keys=True) + '\n')
    for name, value in audit['sha256'].items():
        if sha256(output / 'rosbag' / name) != value:
            raise ValueError('Copied raw file hash mismatch: ' + name)
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('raw_bag')
    parser.add_argument('--inspect-only', action='store_true')
    parser.add_argument('--output-dir')
    parser.add_argument('--contract', help='Optional semantic-role YAML; defaults to installed onboard contract')
    parser.add_argument('--start-offset-sec', type=float, default=0.)
    parser.add_argument('--end-offset-sec', type=float)
    parser.add_argument('--scenario-id', help='Operator-reported scenario label, not verified provenance')
    args = parser.parse_args()
    try:
        if args.inspect_only:
            if args.output_dir:
                raise ValueError('Do not use --output-dir with --inspect-only')
            print(json.dumps(inspect_bag(args.raw_bag), indent=2, sort_keys=True))
        else:
            if not args.output_dir:
                raise ValueError('--output-dir is required unless --inspect-only is used')
            if not args.contract:
                from ament_index_python.packages import get_package_share_directory
                args.contract = str(Path(get_package_share_directory('oudtra_driver_bringup')) /
                                    'config/onboard_localization_recording.yaml')
            import_bag(args.raw_bag, args.output_dir, args.contract,
                       args.start_offset_sec, args.end_offset_sec, args.scenario_id)
            print('IMPORT PASS: ' + str(Path(args.output_dir).resolve()))
    except (OSError, ValueError, KeyError, TypeError, sqlite3.DatabaseError, yaml.YAMLError) as error:
        parser.exit(1, 'IMPORT FAIL: ' + str(error) + '\n')


if __name__ == '__main__':
    main()
