"""Explicit session selection, verified before handoff; no ROS or execution side effects."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shlex
import sys

import yaml


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verified_file(selection, key):
    value = selection[key]
    path = Path(value)
    if not path.is_absolute() or not path.is_file():
        raise ValueError(f"Selected {key} is missing or not absolute: {value}")
    if sha256(path) != selection[key + '_sha256']:
        raise ValueError(f"Selected {key} hash mismatch: {value}")
    return path


def resolve_selection(manifest, allow_experimental=False, map_yaml=None):
    """Verify exact assets and evidence. Never select a map or a fallback CSV."""
    try:
        manifest = Path(manifest).expanduser().resolve()
        data = yaml.safe_load(manifest.read_text())
        if data['format_version'] != 1:
            raise ValueError('Unsupported selected-raceline format')
        selection = data['selection']
        status = selection['status']
        if status not in ('validated', 'experimental'):
            raise ValueError('Selection status must be validated or experimental')
        if status == 'experimental' and not allow_experimental:
            raise ValueError('Experimental selection requires --allow-experimental; not validated for onboard use')
        if selection['direction'] not in ('csv', 'normal', 'reverse'):
            raise ValueError('Invalid selected direction')
        if selection['generator'] not in ('global_optimizer', 'manual_editor') or not selection['candidate_id']:
            raise ValueError('Selection must identify an actual optimizer/manual candidate')
        csv_path = verified_file(selection, 'raceline_csv')
        verified_file(selection, 'map_yaml')
        verified_file(selection, 'map_image')
        validation_path = verified_file(selection, 'validation_report')
        map_data = yaml.safe_load(Path(selection['map_yaml']).read_text())
        image = (Path(selection['map_yaml']).parent / map_data['image']).resolve()
        if sha256(image) != selection['map_image_sha256']:
            raise ValueError('Selected map YAML references a different image')
        if map_yaml is not None and sha256(Path(map_yaml).expanduser().resolve()) != selection['map_yaml_sha256']:
            raise ValueError('Selected raceline map differs from explicitly supplied PF map')
        with csv_path.open(newline='') as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != ['index', 'x', 'y', 'yaw', 'curvature', 'curvature_abs']:
                raise ValueError('Selected raceline requires the standard six-column CSV schema')
            rows = list(reader)
            if len(rows) < 3 or not all(math.isfinite(float(row[key])) for row in rows for key in reader.fieldnames):
                raise ValueError('Selected raceline geometry is insufficient or nonfinite')
        evidence = yaml.safe_load(validation_path.read_text())
        constraints = evidence['vehicle_constraints']
        constraints_hash = hashlib.sha256(json.dumps(constraints, sort_keys=True).encode()).hexdigest()
        if (evidence['candidate_id'] != selection['candidate_id'] or
                evidence['raceline_sha256'] != selection['raceline_csv_sha256'] or
                evidence['status'] != status or evidence['direction'] != selection['direction'] or
                constraints_hash != selection['vehicle_profile_sha256'] or
                evidence['inputs']['map_yaml_sha256'] != selection['map_yaml_sha256'] or
                evidence['inputs']['map_image_sha256'] != selection['map_image_sha256']):
            raise ValueError('Selection and validation evidence disagree')
        if (status == 'validated') != bool(evidence['valid']):
            raise ValueError('Selection misrepresents production validation')
        return dict(selection, manifest=str(manifest), manifest_sha256=sha256(manifest))
    except (OSError, KeyError, TypeError, AttributeError, yaml.YAMLError) as error:
        raise ValueError('Invalid selected raceline: ' + str(error)) from error


def handoff_arguments(selection, target):
    """Use existing interfaces: runner, full stack, or standalone publisher."""
    csv_path, direction = selection['raceline_csv'], selection['direction']
    if target == 'runner':
        return ['--raceline-csv', csv_path, '--raceline-direction', direction]
    if target == 'pnc':
        return ['raceline_csv_path:=' + csv_path, 'raceline_direction:=' + direction]
    if target == 'publisher':
        return ['csv_path:=' + csv_path, 'direction:=' + direction]
    raise ValueError('Unknown handoff target')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest')
    parser.add_argument('--allow-experimental', action='store_true')
    parser.add_argument('--map-yaml', help='Verify against the caller-selected map; never replaces it')
    parser.add_argument('--format', choices=('summary', 'json', 'runner', 'pnc', 'publisher'), default='summary')
    args = parser.parse_args(argv)
    try:
        selection = resolve_selection(args.manifest, args.allow_experimental, args.map_yaml)
        if args.format == 'json':
            print(json.dumps(selection, indent=2))
        elif args.format != 'summary':
            print(' '.join(shlex.quote(v) for v in handoff_arguments(selection, args.format)))
        else:
            print(f"SELECTED RACELINE\nStatus: {selection['status'].upper()}\nCandidate: {selection['candidate_id']}"
                  f"\nCSV: {selection['raceline_csv']}\nDirection: {selection['direction']}\nManifest: {selection['manifest']}")
            if selection.get('custom_vehicle_profile'):
                print('CUSTOM offline vehicle profile; validation does not establish physical vehicle capability.')
            if selection['status'] == 'experimental':
                print('Experimental simulation only. Not validated for onboard use.')
        return 0
    except ValueError as error:
        print(str(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
