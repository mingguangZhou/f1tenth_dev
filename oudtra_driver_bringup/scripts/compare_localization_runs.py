#!/usr/bin/env python3
"""Compare completed schema-2 scorecards; no ROS dependencies or composite score."""
import argparse
import json
import math
import os
from pathlib import Path

GROUP_LABELS = {'dynamics_diagnostics': 'Secondary dynamics diagnostics'}
METRIC_LABELS = {
    'speed_command_oscillation': 'Speed-command oscillation',
    'steering_command_oscillation': 'Steering-command oscillation',
    'forward_speed_oscillation': 'Forward-speed oscillation',
    'yaw_rate_oscillation': 'Yaw-rate oscillation'}


def flatten(report):
    return {'.'.join((section, group, name)): metric
            for section in ('localization', 'vehicle')
            for group, metrics in report[section].items()
            for name, metric in metrics.items()}


def subtract(left, right):
    if isinstance(left, bool) or isinstance(right, bool):
        return None
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return right-left if math.isfinite(left) and math.isfinite(right) else None
    if isinstance(left, dict) and isinstance(right, dict) and left.keys() == right.keys():
        result = {key: subtract(left[key], right[key]) for key in left}
        return result if all(v is not None for v in result.values()) else None
    return None


def compare(left_root, right_root, output):
    reports = [json.loads((root / 'metrics.json').read_text()) for root in (left_root, right_root)]
    if any(r.get('schema_version') != 2 or r.get('analysis_status') != 'PASS' for r in reports):
        raise ValueError('Both runs require successful schema-2 analysis; reanalyze older runs')
    left, right = reports
    shared_reasons = []
    for key in ('scenario_id', 'reference_sha256', 'algorithm_id', 'algorithm_config_sha256'):
        a, b = left.get('identity', {}).get(key), right.get('identity', {}).get(key)
        if not a or not b:
            shared_reasons.append('Unknown identity: ' + key)
        elif a != b:
            shared_reasons.append('Different identity: ' + key)
    if left['settings'] != right['settings']:
        shared_reasons.append('Different effective analysis settings')
    a_metrics, b_metrics = map(flatten, reports)
    rows = {}
    for name in sorted(a_metrics.keys() | b_metrics.keys()):
        a, b = a_metrics.get(name), b_metrics.get(name)
        reasons = list(shared_reasons)
        if a is None or b is None:
            reasons.append('Metric absent from one schema')
        else:
            if a['status'] != 'AVAILABLE' or b['status'] != 'AVAILABLE':
                reasons.append('Metric not AVAILABLE in both runs')
            for key in ('unit', 'method', 'time_basis', 'evidence'):
                if a[key] != b[key]:
                    reasons.append('Different ' + key)
            if ('count' in a['unit'] or name.endswith('distance_travelled_m')) and abs(left['interval']['duration_sec']-right['interval']['duration_sec']) > 1e-6:
                reasons.append('Different interval durations for a cumulative metric')
            if any(m.get('coverage') is not None and m['coverage'] < 1-1e-6 for m in (a, b)):
                reasons.append('Incomplete coverage; suppress delta')
        delta = None if reasons else subtract(a['value'], b['value'])
        if not reasons and delta is None:
            reasons.append('Values are categorical or non-scalar event summaries')
        rows[name] = {'left': a, 'right': b, 'delta_right_minus_left': delta,
                      'delta_unit': 'percentage points' if a and a['unit'] == '%' else a['unit'] if a else None,
                      'comparability': 'COMPARABLE' if not reasons else 'NOT_ESTABLISHED', 'reasons': reasons}
    result = {'schema_version': 1, 'title': 'Evaluation comparison — {} versus {}'.format(left['title'], right['title']),
              'left': {key: left.get(key) for key in ('title', 'platform', 'identity', 'provenance', 'interval', 'settings')},
              'right': {key: right.get(key) for key in ('title', 'platform', 'identity', 'provenance', 'interval', 'settings')},
              'metrics': rows, 'limitations': ['Deltas are right minus left and are not causal platform effects.',
                                               'Unknown legacy identity suppresses deltas; do not infer identity from directory names.']}
    # Never overwrite an analyzed run's report with a comparison report.
    output = output.resolve()
    if output in (left_root.resolve(), right_root.resolve()) or (output / 'metadata.yaml').exists() or (output / 'metrics.json').exists():
        raise ValueError('Comparison output must be a separate directory')
    output.mkdir(parents=True, exist_ok=True)
    def value(metric):
        if metric is None:
            return 'MISSING'
        if metric['status'] != 'AVAILABLE':
            return metric['status']
        item = metric['value']
        if isinstance(item, bool):
            return 'Yes' if item else 'No'
        if isinstance(item, (int, float)):
            return format(item, '.5g')
        if isinstance(item, dict):
            return ', '.join('{}={}'.format(
                key, ('Yes' if val else 'No') if isinstance(val, bool) else
                format(val, '.4g') if isinstance(val, (int, float)) else
                json.dumps(val, sort_keys=True)) for key, val in item.items())
        return json.dumps(item, sort_keys=True)
    report = '# '+result['title']+'\n\n## 1. Summary\n\nSemantic metrics are aligned side by side. Deltas require matching evidence, definitions, settings, coverage, and known identity. No composite score is calculated.\n\n'
    report += '## 2. Runs compared\n\n| Side | Run | Platform | Evaluation duration |\n| --- | --- | --- | --- |\n'
    left_report = os.path.relpath(left_root.resolve() / 'report.md', output)
    right_report = os.path.relpath(right_root.resolve() / 'report.md', output)
    report += '| Left | [{}]({}) | `{}` | {:.3f} s |\n'.format(left['title'], left_report, left['platform'], left['interval']['duration_sec'])
    report += '| Right | [{}]({}) | `{}` | {:.3f} s |\n\n'.format(right['title'], right_report, right['platform'], right['interval']['duration_sec'])
    report += '## 3. Comparability assessment\n\n'
    if shared_reasons:
        report += 'Numerical deltas are globally suppressed for these reasons:\n\n'
        report += ''.join('- '+reason+'\n' for reason in shared_reasons)
    else:
        report += 'Run identity and effective analysis settings match. Individual metrics still require compatible evidence, units, timing, and coverage.\n'
    for number, section in ((4, 'localization'), (5, 'vehicle')):
        report += '\n## {}. {} comparison\n\n'.format(number, section.title())
        groups = sorted({name.split('.')[1] for name in rows if name.startswith(section+'.')})
        for group in groups:
            report += '### '+GROUP_LABELS.get(group, group.replace('_', ' ').title())+'\n\n'
            report += '| Metric | Left | Right | Delta (right - left) | Metric-specific limit |\n| --- | --- | --- | --- | --- |\n'
            for name, row in rows.items():
                if not name.startswith(section+'.'+group+'.'):
                    continue
                local = [reason for reason in row['reasons'] if reason not in shared_reasons]
                limit = '; '.join(local) or ('See global assessment' if shared_reasons else 'Comparable')
                metric_name = name.split('.')[-1]
                report += '| {} | {} | {} | {} | {} |\n'.format(
                    METRIC_LABELS.get(metric_name, metric_name.replace('_', ' ').title()), value(row['left']), value(row['right']),
                    json.dumps(row['delta_right_minus_left']) if row['delta_right_minus_left'] is not None else 'N/A', limit)
            report += '\n'
    report += ('## 6. Interpretation limitations\n\n'
               '- Deltas are right minus left and are not causal platform effects.\n'
               '- Different evidence, such as simulator GT versus onboard estimated pose, is shown side by side without a numerical delta.\n'
               '- `NOT_APPLICABLE`, `UNAVAILABLE_DATA`, and `ANALYSIS_ERROR` retain their source-run meanings.\n\n'
               '## 7. Reproduction and source references\n\n'
               '- [compare_localization_runs.py](../../scripts/compare_localization_runs.py) generated this report from the two existing `metrics.json` files.\n'
               '- [Operational commands](../../../docs/ROBORACER_OPERATIONAL_COMMAND_REFERENCE.md) explain how to analyze and compare runs.\n'
               '- [Shared evaluation contract](../../../docs/LOCALIZATION_SIMULATION.md) defines comparability and evidence rules.\n\n'
               '## 8. Deliverables\n\n`comparison.json` retains units, evidence, applicability states, methods, and provenance. This `report.md` is the human-readable view. Source run artifacts remain unchanged.\n')
    (output / 'comparison.json').write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False)+'\n')
    (output / 'report.md').write_text(report)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('left_run', type=Path)
    parser.add_argument('right_run', type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    try:
        compare(args.left_run, args.right_run, args.output_dir)
        print('COMPARISON PASS: ' + str(args.output_dir))
        return 0
    except Exception as error:
        print('COMPARISON FAIL: ' + str(error))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
