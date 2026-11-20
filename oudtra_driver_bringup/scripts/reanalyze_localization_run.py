#!/usr/bin/env python3
"""Copy preserved inputs to a new analysis directory; originals remain untouched."""
import argparse
import hashlib
from pathlib import Path
import shutil

import yaml


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def default_contract():
    path = Path(__file__).resolve().parents[1] / 'config/shared_evaluation_signals.yaml'
    if path.is_file():
        return path
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('oudtra_driver_bringup')) / 'config/shared_evaluation_signals.yaml'


def prepare(source, output, contract):
    source, output, contract = map(lambda p: Path(p).resolve(), (source, output, contract))
    if output == source or source in output.parents or output in source.parents:
        raise ValueError('Reanalysis output must be separate from the preserved source')
    if output.exists():
        raise ValueError('Reanalysis output already exists; choose a new directory')
    metadata = yaml.safe_load((source / 'metadata.yaml').read_text())
    signals = yaml.safe_load(contract.read_text())
    platform = metadata.get('platform', 'sim' if metadata.get('schema_version') == 1 else None)
    if signals.get('schema_version') != 1 or platform not in signals.get('platforms', {}):
        raise ValueError('Unsupported analysis signal contract or platform')
    if not (source / 'rosbag/metadata.yaml').is_file():
        raise ValueError('Preserved run has no rosbag metadata')
    metadata.setdefault('roles', {}).update(signals['platforms'][platform])
    metadata['reanalysis'] = {
        'source_run': str(source), 'source_metadata_sha256': sha256(source / 'metadata.yaml'),
        'signal_contract_sha256': sha256(contract),
        'raw_files_sha256': {str(p.relative_to(source)): sha256(p) for p in sorted((source / 'rosbag').iterdir()) if p.is_file()},
        'tool_sha256': sha256(Path(__file__))}
    # Preserve superseded analysis-capability notes as history, not current limits.
    obsolete = [v for v in metadata.get('limitations', []) if 'split vehicle-state adapter is not' in v]
    if obsolete:
        metadata['prior_analysis_limitations'] = obsolete
        metadata['limitations'] = [v for v in metadata['limitations'] if v not in obsolete]
    # Historical deployment claims remain unchanged.
    output.mkdir(parents=True)
    shutil.copytree(source / 'rosbag', output / 'rosbag')
    if (source / 'config').is_dir():
        shutil.copytree(source / 'config', output / 'config')
    shutil.copy2(contract, output / 'analysis_signals.yaml')
    (output / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source_run', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--signal-contract', type=Path, default=default_contract())
    args = parser.parse_args()
    try:
        prepare(args.source_run, args.output_dir, args.signal_contract)
        from analyze_localization_run import analyze
        result = analyze(args.output_dir.resolve())
    except (ValueError, OSError, KeyError, yaml.YAMLError) as error:
        parser.exit(1, 'REANALYSIS FAIL: '+str(error)+'\n')
    print('REANALYSIS '+result['analysis_status']+': '+str(args.output_dir.resolve() / 'report.md'))
    return 0 if result['analysis_status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
