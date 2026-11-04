"""Deployment assets survive relocation without weakening selection evidence."""
import hashlib
from pathlib import Path
import shutil

import pytest
import yaml

from centerline_tools.raceline_selection import resolve_selection, resolve_track

PACKAGE = Path(__file__).resolve().parents[1]
CSV_SHA = '0aab3968b64537b608ff8821ff8b1bd28ec3253d3c12719d542ad3ae6a71e88e'


def copy_assets(destination):
    shutil.copytree(PACKAGE / 'tracks', destination / 'tracks')
    for name in ('tempo_coffee_scanned_20261002_00.yaml', 'tempo_coffee_scanned_20261002_00.pgm'):
        shutil.copyfile(PACKAGE / name, destination / name)
    return destination / 'tracks/tempo_coffee/selected_raceline.yaml'


def test_curated_tempo_keeps_exact_geometry_and_failed_production_evidence():
    manifest = PACKAGE / 'tracks/tempo_coffee/selected_raceline.yaml'
    selected = resolve_selection(manifest, True)
    assert selected['status'] == 'experimental'
    assert selected['candidate_id'] == '008_geometry'
    assert selected['raceline_csv_sha256'] == CSV_SHA
    assert hashlib.sha256(Path(selected['raceline_csv']).read_bytes()).hexdigest() == CSV_SHA
    evidence = yaml.safe_load(Path(selected['validation_report']).read_text())
    assert not evidence['valid'] and not evidence['curvature_valid']
    assert evidence['curvature_abs_max_inv_m'] > evidence['curvature_limit_inv_m'] == 1.082
    assert selected['direction'] == 'csv'
    assert all(not Path(yaml.safe_load(manifest.read_text())['selection'][key]).is_absolute()
               for key in ('raceline_csv', 'map_yaml', 'map_image', 'validation_report'))


def test_same_selection_resolves_from_two_independent_workspaces(tmp_path):
    for folder in ('sim_ws', 'f1tenth_ws'):
        target = tmp_path / folder
        target.mkdir()
        selection = resolve_selection(copy_assets(target), True)
        for key in ('raceline_csv', 'map_yaml', 'map_image', 'validation_report'):
            assert target in Path(selection[key]).parents
        assert selection['raceline_csv_sha256'] == CSV_SHA


@pytest.mark.parametrize('asset', ['raceline_csv', 'map_yaml', 'map_image', 'validation_report'])
@pytest.mark.parametrize('change', ['missing', 'changed'])
def test_portable_selection_still_rejects_broken_assets(tmp_path, asset, change):
    manifest = copy_assets(tmp_path)
    data = yaml.safe_load(manifest.read_text())['selection']
    path = (manifest.parent / data[asset]).resolve()
    if change == 'missing':
        path.unlink()
    else:
        path.write_bytes(path.read_bytes() + b'changed')
    with pytest.raises(ValueError, match='missing|hash mismatch'):
        resolve_selection(manifest, True)


def test_installed_named_track_requires_acknowledgment():
    with pytest.raises(ValueError, match='requires --allow-experimental'):
        resolve_track('tempo_coffee')
    assert resolve_track('tempo_coffee', True)['raceline_csv_sha256'] == CSV_SHA
    for track in ('../tempo_coffee', '/tmp/tempo_coffee', ''):
        with pytest.raises(ValueError, match='simple track name'):
            resolve_track(track, True)
    with pytest.raises(ValueError, match='Invalid selected raceline'):
        resolve_track('missing_track', True)
