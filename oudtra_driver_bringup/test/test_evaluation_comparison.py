"""Evidence compatibility is required for numerical deltas, not just matching names."""
import importlib.util
import json
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location('comparison', Path(__file__).resolve().parents[1] / 'scripts/compare_localization_runs.py')
C = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(C)


def fixture(root, value, platform='sim'):
    root.mkdir()
    report = {'schema_version': 2, 'analysis_status': 'PASS', 'title': root.name, 'platform': platform,
              'identity': dict(scenario_id='same', reference_sha256='ref', algorithm_id='alg', algorithm_config_sha256='cfg'),
              'settings': {'window_sec': .5}, 'interval': {'duration_sec': 10}, 'vehicle': {},
              'localization': {'availability': {'percent': dict(status='AVAILABLE', value=value, unit='%', method='v1', evidence='pose', time_basis='source', coverage=1)}}}
    (root / 'metrics.json').write_text(json.dumps(report))
    return report


def test_comparison_deltas_and_repeatability(tmp_path):
    a, b, out = [tmp_path / p for p in ('sim', 'onboard', 'comparison')]
    fixture(a, 90)
    fixture(b, 95, 'onboard')
    result = C.compare(a, b, out)
    metric = result['metrics']['localization.availability.percent']
    assert metric['delta_right_minus_left'] == 5
    assert metric['delta_unit'] == 'percentage points'
    before = [(out / f).read_bytes() for f in ('report.md', 'comparison.json')]
    report = before[0].decode()
    assert '## 2. Runs compared' in report
    assert '## 3. Comparability assessment' in report
    assert '## 4. Localization comparison' in report
    assert '## 5. Vehicle comparison' in report
    assert '## 7. Reproduction and source references' in report
    assert '../../scripts/compare_localization_runs.py' in report
    assert '../sim/report.md' in report and '../onboard/report.md' in report
    C.compare(a, b, out)
    assert before == [(out / f).read_bytes() for f in ('report.md', 'comparison.json')]


@pytest.mark.parametrize('change', ['identity', 'settings', 'evidence', 'status', 'coverage', 'time_basis'])
def test_comparison_suppresses_unjustified_delta(tmp_path, change):
    a, b = tmp_path / 'a', tmp_path / 'b'
    fixture(a, 90)
    report = fixture(b, 95)
    metric = report['localization']['availability']['percent']
    if change == 'identity':
        report['identity'] = {}
    elif change == 'settings':
        report['settings'] = {}
    elif change == 'coverage':
        metric[change] = .5
    else:
        metric[change] = 'NOT_APPLICABLE' if change == 'status' else 'different'
    (b / 'metrics.json').write_text(json.dumps(report))
    result = C.compare(a, b, tmp_path / 'comparison')
    assert result['metrics']['localization.availability.percent']['delta_right_minus_left'] is None


def test_cannot_overwrite_source_run(tmp_path):
    a, b = tmp_path / 'a', tmp_path / 'b'
    fixture(a, 90)
    fixture(b, 95)
    with pytest.raises(ValueError, match='separate directory'):
        C.compare(a, b, a)


def test_unequal_duration_counts_are_not_compared(tmp_path):
    a, b = tmp_path / 'a', tmp_path / 'b'
    for root, duration in ((a, 10), (b, 20)):
        report = fixture(root, 2)
        report['interval']['duration_sec'] = duration
        report['localization']['availability']['percent']['unit'] = 'count'
        (root / 'metrics.json').write_text(json.dumps(report))
    result = C.compare(a, b, tmp_path / 'comparison')
    assert result['metrics']['localization.availability.percent']['delta_right_minus_left'] is None
