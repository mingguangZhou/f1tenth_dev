"""Input and isolation contracts for the opt-in integration check."""
import importlib.util
import math
from pathlib import Path
import shutil

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('smoke', ROOT / 'scripts/run_localization_smoke.py')
SMOKE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SMOKE)


def test_initial_pose_offsets_are_additive_without_truth_subscription():
    assert SMOKE.initial_pose({'sx': 1, 'sy': 2, 'stheta': .5}, (.1, -.2, .3)) == [1.1, 1.8, .8]


@pytest.mark.parametrize('value', [math.inf, -math.inf, math.nan])
def test_nonfinite_initialization_rejected(value):
    with pytest.raises(ValueError):
        SMOKE.initial_pose({'sx': 1, 'sy': 2, 'stheta': 0}, (value, 0, 0))


def test_evaluation_profile_preserves_health_and_isolates_truth():
    profile_path = ROOT / 'config/localization_eval_sim.yaml'
    assert not (ROOT / 'config/localization_eval.yaml').exists()
    profile = yaml.safe_load(profile_path.read_text())
    lower = profile['lower_safety_controller']['ros__parameters']
    assert lower['enable_sim_reverse_swept_gate'] is False
    assert lower['reverse_swept_agent_status_topic'] != '/simulator/agent_status'
    assert lower['odom_topic'] == '/pf/pose/odom'
    assert profile['drive_arbitrator']['ros__parameters']['require_pf_health'] is True
    assert all(node['ros__parameters']['use_sim_time'] is False for node in profile.values())
    text = profile_path.read_text().lower()
    assert 'ifac' not in text
    assert 'map_path' not in text
    assert 'raceline_csv_path' not in text
    assert 'raceline_points_optimized' not in text


def _track_fixture(tmp_path):
    image = tmp_path / 'track.png'
    image.write_bytes(b'fixture-map')
    map_yaml = tmp_path / 'track.yaml'
    map_yaml.write_text(
        'image: track.png\nresolution: 0.05\norigin: [0, 0, 0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    sim = tmp_path / 'sim.yaml'
    sim.write_text(
        'bridge:\n  ros__parameters:\n    map_path: "{}"\n'
        '    map_img_ext: ".png"\n    sx: 1.0\n    sy: 2.0\n    stheta: 0.5\n'.format(
            tmp_path / 'track'))
    raceline = tmp_path / 'raceline.csv'
    raceline.write_text('index,x,y,yaw\n0,0,0,0\n1,1,0,0\n')
    localize = tmp_path / 'localize.yaml'
    localize.write_text('particle_filter:\n  ros__parameters: {}\n')
    return sim, map_yaml, raceline, localize


def test_explicit_track_preflight_and_provenance_are_deterministic(tmp_path):
    inputs = _track_fixture(tmp_path)
    first = SMOKE.validate_track_inputs('test-track', *inputs[:3], 'csv', inputs[3])
    second = SMOKE.validate_track_inputs('test-track', *inputs[:3], 'csv', inputs[3])
    assert first == second
    assert first['scenario_id'] == 'test-track'
    assert first['raceline_direction'] == 'csv'
    assert set(first['assets']) == {
        'simulator_config', 'pf_map_yaml', 'raceline_csv', 'localize_config',
        'simulator_map_yaml', 'simulator_map_image', 'pf_map_image'}
    assert all(set(value) == {'path', 'sha256'} for value in first['assets'].values())


def test_mismatched_simulator_and_pf_maps_are_rejected(tmp_path):
    sim, map_yaml, raceline, localize = _track_fixture(tmp_path)
    other = tmp_path / 'other'
    other.mkdir()
    other_image = other / 'track.png'
    other_image.write_bytes(b'different-map')
    other_map = other / 'track.yaml'
    shutil.copyfile(map_yaml, other_map)
    with pytest.raises(ValueError, match='different maps'):
        SMOKE.validate_track_inputs(
            'test-track', sim, other_map, raceline, 'csv', localize)


@pytest.mark.parametrize('missing_index, expected', [
    (0, 'simulator_config does not exist'),
    (1, 'pf_map_yaml does not exist'),
    (2, 'raceline_csv does not exist'),
    (3, 'localize_config does not exist'),
])
def test_missing_explicit_track_input_fails_clearly(tmp_path, missing_index, expected):
    inputs = list(_track_fixture(tmp_path))
    inputs[missing_index].unlink()
    with pytest.raises(ValueError, match=expected):
        SMOKE.validate_track_inputs('test-track', *inputs[:3], 'csv', inputs[3])


def test_missing_map_image_and_bad_raceline_fail_clearly(tmp_path):
    sim, map_yaml, raceline, localize = _track_fixture(tmp_path)
    (tmp_path / 'track.png').unlink()
    with pytest.raises(ValueError, match='simulator map image does not exist'):
        SMOKE.validate_track_inputs('test-track', sim, map_yaml, raceline, 'csv', localize)
    (tmp_path / 'track.png').write_bytes(b'fixture-map')
    raceline.write_text('index,yaw\n0,0\n')
    with pytest.raises(ValueError, match='requires x and y columns'):
        SMOKE.validate_track_inputs('test-track', sim, map_yaml, raceline, 'csv', localize)


def test_explicit_pf_and_pnc_arguments_are_forwarded(tmp_path):
    localize = tmp_path / 'localize.yaml'
    map_yaml = tmp_path / 'map.yaml'
    raceline = tmp_path / 'raceline.csv'
    overlay = tmp_path / 'localization_eval_sim.yaml'
    pf = SMOKE.pf_launch_arguments(localize, map_yaml, overlay)
    pnc = SMOKE.pnc_launch_arguments(raceline, 'reverse', overlay)
    assert pf == ['localize_config:=' + str(localize), 'map_yaml:=' + str(map_yaml),
                  'parameter_overlay:=' + str(overlay)]
    assert 'raceline_csv_path:=' + str(raceline) in pnc
    assert 'raceline_direction:=reverse' in pnc
    assert sum(str(overlay) in value for value in pnc) == 4
