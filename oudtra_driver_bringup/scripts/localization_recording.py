"""Optional rosbag lifecycle and run provenance; no simulator launch logic."""
import hashlib
from pathlib import Path
import os
import sqlite3
import subprocess
import time

import yaml

from recording_support import bag_counts, stop_rosbag


class RunRecording:
    def __init__(self, directory, contract, args, configs):
        self.root = Path(directory).resolve()
        self.root.mkdir(parents=True, exist_ok=False)
        (self.root / 'config').mkdir()
        (self.root / 'logs').mkdir()
        self.contract = yaml.safe_load(Path(contract).read_text())
        self.metadata = {
            'schema_version': 1, 'run_id': self.root.name, 'platform': 'sim',
            'created_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'clock': 'ROS system/wall time; interval duration checked against monotonic time',
            'main_revision': args.source_revision or None,
            'pf_revision': args.pf_revision or None,
            'source_state': args.source_state,
            'provenance_note': 'Revisions supplied by caller; build freshness is caller responsibility',
            'software_sha256': {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                                for name in ('run_localization_smoke.py', 'localization_recording.py')},
            'requested_duration_sec': args.run_duration_sec,
            'ros_domain_id': args.ros_domain_id,
            'initial_offsets': [args.x_offset_m, args.y_offset_m, args.yaw_offset_rad],
            'roles': self.contract['roles'], 'analysis': self.contract['analysis'],
            'phases': [], 'configs': {}, 'recording': {'status': 'INCOMPLETE'},
        }
        for label, path in configs.items():
            path = Path(path)
            data = path.read_bytes()
            destination = self.root / 'config' / (label + path.suffix)
            destination.write_bytes(data)
            self.metadata['configs'][label] = {
                'source': str(path), 'copy': str(destination.relative_to(self.root)),
                'sha256': hashlib.sha256(data).hexdigest()}
        self.process = None
        self.log = None
        self.save()

    def save(self):
        temporary = self.root / 'metadata.yaml.tmp'
        temporary.write_text(yaml.safe_dump(self.metadata, sort_keys=False))
        temporary.replace(self.root / 'metadata.yaml')

    def mark(self, state, node):
        self.metadata['phases'].append({
            'state': state, 'wall_ns': node.get_clock().now().nanoseconds,
            'monotonic_ns': time.monotonic_ns()})
        self.save()

    def start(self):
        qos = self.root / 'config/recorder_qos.yaml'
        qos.write_text(yaml.safe_dump(self.contract['qos']))
        command = ['ros2', 'bag', 'record', '-o', str(self.root / 'rosbag'),
                   '--qos-profile-overrides-path', str(qos)]
        command += [role['topic'] for role in self.contract['roles'].values()]
        self.metadata['recording']['command'] = command
        self.log = (self.root / 'logs/recorder.log').open('w')
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        self.save()

    def check(self):
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError('Recorder exited before requested stop')

    def counts(self):
        return bag_counts(self.root / 'rosbag')

    def ready(self, roles):
        self.check()
        try:
            counts = self.counts()
        except sqlite3.OperationalError:
            return False  # Database may be opening; caller's readiness deadline applies.
        return all(counts.get(self.contract['roles'][role]['topic'], 0) > 0 for role in roles)

    def stop(self, result):
        self.metadata['smoke_result'] = result
        try:
            self.check()
            code, counts = stop_rosbag(self.process, self.root / 'rosbag')
            try:
                os.killpg(self.process.pid, 0)
            except ProcessLookupError:
                pass
            else:
                raise RuntimeError('Recorder process group survived shutdown')
            self.metadata['recording']['return_code'] = code
            missing = [r['topic'] for r in self.contract['roles'].values()
                       if counts.get(r['topic'], 0) == 0]
            self.metadata['recording']['message_counts'] = counts
            if missing:
                raise RuntimeError('Bag missing required topics: ' + ', '.join(missing))
            self.metadata['recording']['status'] = 'PASS'
        except Exception as error:
            self.metadata['recording'].update(status='FAIL', reason=str(error))
        finally:
            if self.log:
                self.log.close()
            self.save()
        return self.metadata['recording']['status'] == 'PASS'
