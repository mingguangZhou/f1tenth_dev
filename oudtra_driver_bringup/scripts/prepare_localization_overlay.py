#!/usr/bin/env python3
"""Write the same simulation/speed overlay used by the timed runner; no ROS nodes."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import yaml


def load_speed_profile(speed_profile):
    """Validate the speed-only contract used by both platform adapters."""
    profile = yaml.safe_load(Path(speed_profile).read_text())
    expected = {'/**': {'command_speed_min_mps', 'command_speed_max_mps'},
                'lower_safety_controller': {'absolute_speed_limit_mps'}}
    if not isinstance(profile, dict) or set(profile) != set(expected):
        raise ValueError('Speed profile requires shared command limits and lower-safety cap')
    for node, keys in expected.items():
        block = profile[node]
        if not isinstance(block, dict) or set(block) != {'ros__parameters'}:
            raise ValueError('Invalid speed profile node: ' + node)
        params = block['ros__parameters']
        if not isinstance(params, dict) or set(params) != keys:
            raise ValueError('Speed profile may contain only declared speed limits')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) for v in params.values()):
            raise ValueError('Speed limits must be finite numbers')
    limits = profile['/**']['ros__parameters']
    cap = profile['lower_safety_controller']['ros__parameters']['absolute_speed_limit_mps']
    if not 0 < limits['command_speed_min_mps'] <= limits['command_speed_max_mps'] == cap:
        raise ValueError('Require positive min <= max and identical final speed cap')
    return profile


def speed_profile_overlay(evaluation_overlay, speed_profile):
    """Merge explicit speed limits without changing frames, health or GT isolation."""
    base = yaml.safe_load(Path(evaluation_overlay).read_text())
    profile = load_speed_profile(speed_profile)
    for node, block in profile.items():
        base.setdefault(node, {'ros__parameters': {}})['ros__parameters'].update(
            block['ros__parameters'])
    return base


def write_overlay(evaluation_config, speed_profile, output):
    """Keep effective settings and their source hashes at an explicit local path."""
    output = Path(output).expanduser().resolve()
    provenance = output.with_suffix('.provenance.yaml')
    if output.exists() or provenance.exists():
        raise ValueError('Output already exists; choose a new session path: ' + str(output))
    merged = speed_profile_overlay(evaluation_config, speed_profile)
    content = yaml.safe_dump(merged, sort_keys=False)
    sources = {}
    for label, value in [('evaluation_config', evaluation_config), ('speed_profile', speed_profile)]:
        path = Path(value).expanduser().resolve()
        sources[label] = {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    evidence = {'sources': sources,
                'output': {'path': str(output), 'sha256': hashlib.sha256(content.encode()).hexdigest()},
                'helper_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        stream.write(content)
    with provenance.open('x') as stream:
        stream.write(yaml.safe_dump(evidence, sort_keys=False))
    return {'result': 'PASS', 'overlay': str(output), 'provenance': str(provenance)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluation-config', required=True)
    parser.add_argument('--speed-profile', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    try:
        print(json.dumps(write_overlay(args.evaluation_config, args.speed_profile, args.output), indent=2))
    except (OSError, ValueError, yaml.YAMLError) as error:
        parser.error(str(error))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
