"""Input and isolation contracts for the opt-in integration check."""
import importlib.util
import math
from pathlib import Path

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
    profile = yaml.safe_load((ROOT / 'config/localization_eval.yaml').read_text())
    lower = profile['lower_safety_controller']['ros__parameters']
    assert lower['enable_sim_reverse_swept_gate'] is False
    assert lower['reverse_swept_agent_status_topic'] != '/simulator/agent_status'
    assert lower['odom_topic'] == '/pf/pose/odom'
    assert profile['drive_arbitrator']['ros__parameters']['require_pf_health'] is True
    assert all(node['ros__parameters']['use_sim_time'] is False for node in profile.values())
