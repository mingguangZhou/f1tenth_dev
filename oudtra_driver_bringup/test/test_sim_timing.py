"""Wall-time bag measurements remain explicit about evidence and missing data."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
SPEC = importlib.util.spec_from_file_location(
    'sim_timing', ROOT / 'scripts/analyze_sim_timing.py')
TIMING = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TIMING)


def _message(stamp_ns):
    return SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=stamp_ns // 1000000000,
                              nanosec=stamp_ns % 1000000000)))


def test_rate_jitter_and_wall_stamp_age_are_measured_within_interval():
    rows = [(900000000, _message(895000000)),
            (1010000000, _message(1008000000)),
            (1020000000, _message(1017000000)),
            (1040000000, _message(1036000000)),
            (1100000000, _message(1099000000))]
    result = TIMING.summarize_stream(rows, 1000000000, 1050000000)
    assert result['count'] == 3
    assert result['rate_hz'] == 60
    assert result['interarrival_ms']['p50'] == 15
    assert result['interarrival_ms']['max'] == 20
    assert result['interarrival_jitter_ms']['max'] == 5
    assert result['source_to_bag_age_ms'] == {'p50': 3, 'p95': 3.9, 'max': 4}


def test_missing_samples_are_unavailable_and_bad_values_fail():
    empty = TIMING.summarize_stream([], 1000000000, 2000000000)
    assert empty['count'] == 0
    assert empty['interarrival_ms'] is None
    assert empty['source_to_bag_age_ms'] is None
    with pytest.raises(ValueError, match='Nonfinite'):
        TIMING.distribution([1, float('nan')])
