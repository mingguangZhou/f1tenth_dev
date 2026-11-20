#!/usr/bin/env python3
"""Exploratory reference-section comparison; existing certified gates stay separate."""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze_localization_run as A
import compare_localization_runs as strict
from reanalyze_localization_run import prepare, sha256, default_contract


METRIC_ORDER = ['vehicle.tracking.pf_reference_position_deviation_m', 'vehicle.tracking.pf_heading_to_reference_rad',
              'localization.consistency.pose_increment_disagreement_m', 'localization.consistency.yaw_increment_disagreement_rad',
              'localization.availability.availability_percent', 'localization.availability.health_time_distribution',
              'vehicle.smoothness.autonomy_speed_command_oscillation', 'vehicle.smoothness.autonomy_steering_command_oscillation',
              'vehicle.smoothness.forward_speed_oscillation']

def defaults():
    path = Path(__file__).resolve().parents[1] / 'config/matched_section_analysis.yaml'
    if not path.is_file():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('oudtra_driver_bringup')) / 'config/matched_section_analysis.yaml'
    return yaml.safe_load(path.read_text())


def geometry(reference):
    if (reference is None or len(reference) < 4 or not np.all(np.isfinite(reference)) or
            np.linalg.norm(reference[0]-reference[-1]) > 1e-6):
        raise ValueError('Matching requires a finite, explicitly closed ordered reference')
    vectors = np.diff(reference, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    keep = lengths > 1e-9
    if sum(keep) < 3:
        raise ValueError('Degenerate reference')
    starts, vectors, lengths = reference[:-1][keep], vectors[keep], lengths[keep]
    stations = np.r_[0., np.cumsum(lengths)]
    return starts, vectors, lengths, stations


def project_continuously(pose, reference, settings):
    """Resolve only geometrically and temporally supported branches; gaps reset laps."""
    starts, vectors, lengths, stations = geometry(reference)
    period = stations[-1]
    rows, previous, segment = [], None, -1
    for row in pose:
        t = float(row[0])
        reason = ''
        reset = previous is not None and t-previous['t'] > settings['max_pose_gap_sec']
        if reset:
            previous = None
        fractions = np.clip(np.sum((row[1:3]-starts)*vectors, axis=1)/lengths**2, 0, 1)
        distances = np.linalg.norm(starts+fractions[:, None]*vectors-row[1:3], axis=1)
        candidates = np.flatnonzero(distances <= distances.min()+settings['ambiguity_distance_m'])
        ss = (stations[candidates]+fractions[candidates]*lengths[candidates]) % period
        if distances.min() > settings['max_projection_distance_m']:
            reason = 'outside_projection_corridor'
        if previous is not None:
            unwrapped = previous['s'] + (ss-previous['s']+period/2) % period-period/2
            delta = unwrapped-previous['s']
            feasible = ((delta >= -settings['reverse_tolerance_m']) &
                        (delta <= settings['max_station_speed_mps']*(t-previous['t'])+settings['station_slack_m']))
            candidates, ss = candidates[feasible], unwrapped[feasible]
            if not len(ss):
                reason = reason or 'unsupported_station_jump_or_reverse'
        elif len(ss):
            # Put candidates around the nearest candidate to handle the 0/L seam.
            anchor = ss[np.argmin(distances[candidates])]
            ss = anchor + (ss-anchor+period/2) % period-period/2
        if len(ss) and np.ptp(ss) > settings['branch_separation_m']:
            reason = reason or 'ambiguous_nearby_branches'
        if reason or not len(ss):
            rows.append(dict(t=t, s=None, distance_m=float(distances.min()), segment=None, reason=reason))
            previous = None
            continue
        best = int(np.argmin(distances[candidates]))
        if previous is None:
            segment += 1
        value = dict(t=t, s=float(ss[best]), distance_m=float(distances[candidates[best]]),
                     segment=segment, reason='', follows_source_gap=bool(reset))
        rows.append(value)
        previous = value
    return rows, float(period)


def select_first_pass(rows, period, start, length, settings):
    if not 0 <= start < period or not 0 < length <= period+1e-8:
        raise ValueError('Section start must be in [0,L); length must be in (0,L]')
    groups = {}
    for row in rows:
        if row['s'] is not None:
            groups.setdefault(row['segment'], []).append(row)
    rejected = []
    for group in groups.values():
        # First-crossing envelope: keep increasing stations, never warp timestamps.
        increasing = []
        for row in group:
            if not increasing or row['s'] > increasing[-1]['s']+1e-9:
                increasing.append(row)
        if len(increasing) < 2:
            continue
        s = np.array([r['s'] for r in increasing]); t = np.array([r['t'] for r in increasing])
        target = start + math.ceil((s[0]-start-1e-9)/period)*period
        while target+length <= s[-1]+1e-9:
            a, b = np.interp([target, target+length], s, t)
            lo = max(0, np.searchsorted(t, a)-1)
            hi = min(len(t), np.searchsorted(t, b)+1)
            if (np.max(np.diff(s[lo:hi])) > settings['max_station_gap_m'] or
                    np.max(np.diff(t[lo:hi])) > settings['max_pose_gap_sec']):
                rejected.append({'start_s': float(target), 'reason': 'missing_station_or_time_coverage'})
                target += period
                continue
            raw = [r for r in group if a <= r['t'] <= b]
            return {'start_sec': float(a), 'end_sec': float(b), 'elapsed_sec': float(b-a),
                    'start_unwrapped_m': float(target), 'length_m': float(length),
                    'station_modulo_start_m': float(start), 'segment': group[0]['segment'],
                    'increasing_samples': [[float(x), float(y)] for x, y in zip(t[lo:hi], s[lo:hi])],
                    'non_increasing_station_samples': int(len(raw)-sum(a <= r['t'] <= b for r in increasing)),
                    'rejected_earlier_passes': rejected}
    raise ValueError('No continuous complete section: '+json.dumps(rejected))


def load_run(root):
    meta = yaml.safe_load((root/'metadata.yaml').read_text())
    phases = {r['state']: r for r in meta.get('phases', [])}
    interval = meta.get('interval') or {'start_wall_ns': phases['RUNNING']['wall_ns'],
                                      'end_wall_ns': phases['EVALUATION_END']['wall_ns']}
    origin = interval['start_wall_ns']; duration = (interval['end_wall_ns']-origin)*1e-9
    roles = A.normalize_roles(meta); errors = {}
    signal_defaults = yaml.safe_load(default_contract().read_text())['platforms'][meta.get('platform', 'sim')]
    for name, spec in signal_defaults.items():
        roles.setdefault(name, spec)
    data = A.decode(A.read_bag(root, roles, errors), roles, origin, errors, duration, .2)
    if errors:
        raise ValueError('Input contract error: '+json.dumps(errors))
    pose = data.get('estimated_pose')
    if pose is None or not len(pose):
        raise ValueError('No public estimated pose')
    pose = pose[(pose[:, 0] >= 0) & (pose[:, 0] < duration)]
    if not A.same_frame(roles['estimated_pose'].get('frame'), roles['reference_path'].get('frame')):
        raise ValueError('PF/reference frames differ')
    return meta, data, pose, origin, duration


def rebase_generated_links(report, source_run):
    """Rehome legacy guide/script links without modifying analyzer/comparator code."""
    pattern = r'\[([^\]]+)\]\(([^)]+)\)'
    original = source_run/'report.md'
    source_links = dict(re.findall(pattern, original.read_text())) if original.is_file() else {}
    def replace(match):
        label, target = match.groups()
        if not target.startswith(('../../../docs/', '../../scripts/')):
            return match.group(0)
        source_target = source_links.get(label, target)
        path, separator, anchor = source_target.partition('#')
        resolved = (source_run/path).resolve()
        relative = os.path.relpath(resolved, report.parent)
        return '[{}]({}{})'.format(label, relative, '#'+anchor if separator else '')
    report.write_text(re.sub(pattern, replace, report.read_text()))


def create_section(source, output, selected, origin):
    meta = prepare(source, output, default_contract())
    meta['interval'] = {'start_wall_ns': origin+round(selected['start_sec']*1e9),
                        'end_wall_ns': origin+round(selected['end_sec']*1e9)}
    meta['source_outcome'] = {'outcome': meta.pop('outcome', None), 'smoke_result': meta.pop('smoke_result', None)}
    # Completion of a parent run is not completion of this analytically cropped interval.
    meta['outcome'] = {'completed': None}
    meta['matched_section'] = selected
    meta['description'] = 'Offline reference-section crop of preserved evidence; not a new vehicle run.'
    (output/'metadata.yaml').write_text(yaml.safe_dump(meta, sort_keys=False))
    result = A.analyze(output)
    rebase_generated_links(output/'report.md', source)
    return result


def station_grid(selected, settings):
    length = selected['length_m']; step = settings['station_step_m']
    grid = np.unique(np.r_[np.arange(0., length, step), length])
    pairs = np.array(selected['increasing_samples'])
    times = np.interp(selected['start_unwrapped_m']+grid, pairs[:, 1], pairs[:, 0])
    return grid, times


def signal_at(samples, column, times, gap):
    if samples is None or not len(samples):
        return np.full(len(times), np.nan)
    packed = np.column_stack((samples[:, 0], samples[:, column], np.zeros((len(samples), 2))))
    return A.interpolate(packed, times, gap)[:, 0]


def exploratory(left, right, name, minimum_coverage):
    """No certified delta is ever produced here, even if source identity is known."""
    reasons = []
    if name.endswith(('health_time_distribution', 'health_states')):
        reasons.append('Health threshold equivalence is not established')
    if any(m['status'] != 'AVAILABLE' for m in (left, right)):
        reasons.append('Metric not AVAILABLE in both sections')
    for key in ('unit', 'method', 'time_basis'):
        if left[key] != right[key]:
            reasons.append('Different '+key)
    if left['evidence'] != right['evidence']:
        reasons.append('Different measurement evidence; display observations only')
    if any(m.get('coverage') is not None and m['coverage'] < minimum_coverage for m in (left, right)):
        reasons.append('Insufficient metric coverage')
    difference = None if reasons else strict.subtract(left['value'], right['value'])
    return {'left': left, 'right': right, 'exploratory_onboard_minus_sim': difference,
            'classification': 'EXPLORATORY_UNVERIFIED_DEPLOYMENT' if difference is not None else 'OBSERVATIONS_ONLY',
            'certified_delta': None, 'reasons': reasons or ['Historical deployment unknown; PF-aligned, not physically matched truth']}


def histories(data, pose, selected, settings, metrics):
    grid, times = station_grid(selected, settings)
    output = {'station_m': grid, 'elapsed_sec': times-selected['start_sec']}
    deviation, heading = A.reference_errors(pose, data['reference_path'])
    for name, values in [('pf_position_m', deviation), ('pf_heading_rad', heading)]:
        samples = np.column_stack((pose[:, 0], values))
        output[name] = signal_at(samples, 1, times, settings['max_pose_gap_sec'])
    for name, role, column in [('autonomy_speed_mps', 'autonomy_command', 1),
                                ('autonomy_steering_rad', 'autonomy_command', 2),
                                ('forward_speed_mps', 'forward_speed', 1)]:
        samples = data.get(role)
        if role == 'forward_speed' and samples is None:
            samples = data.get('vehicle_state')
        if role == 'forward_speed' and metrics['vehicle']['smoothness']['forward_speed_oscillation']['status'] != 'AVAILABLE':
            samples = None
        output[name] = signal_at(samples, column, times, settings['max_pose_gap_sec'])
    # The existing consistency calculation is evaluated on the cropped interval;
    # station is only a display coordinate, never an input to its mathematics.
    shifted = {}
    for role in ('source_pose', 'estimated_pose', 'raw_odometry'):
        if data.get(role) is not None:
            shifted[role] = data[role].copy(); shifted[role][:, 0] -= selected['start_sec']
    residual = A.consistency(shifted.get('source_pose', shifted['estimated_pose']),
                             shifted.get('raw_odometry'), selected['elapsed_sec'], metrics['settings'])
    if len(residual):
        residual[:, 0] += selected['start_sec']
    for name, column in [('consistency_position_m', 1), ('consistency_yaw_rad', 2)]:
        output[name] = signal_at(residual, column, times, settings['max_pose_gap_sec'])
    health = data.get('localization_health', [])
    levels = {'GOOD': 3, 'DEGRADED': 2, 'INVALID': 1}
    output['health_state'] = np.array([levels.get(A.held(health, t, metrics['settings']['max_status_gap_sec'])[1], 0)
                                      if A.held(health, t, metrics['settings']['max_status_gap_sec']) else 0 for t in times])
    return output


def json_safe(value):
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, (float, np.floating)) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def compare_sections(sim_root, onboard_root, output, start=0., length=None, overrides=None):
    if output.exists():
        raise ValueError('Output exists; choose a new comparison directory')
    if any(root.resolve() in output.resolve().parents for root in (sim_root, onboard_root)):
        raise ValueError('Output must be separate from preserved input directories')
    settings = defaults()
    if set(overrides or {})-set(settings):
        raise ValueError('Unknown matched-section settings')
    settings.update(overrides or {})
    if any(not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v) or v <= 0 for v in settings.values()):
        raise ValueError('Matching settings must be finite and positive')
    if settings['minimum_metric_coverage'] > 1:
        raise ValueError('Metric coverage threshold must be <= 1')
    inputs = []
    for root in (sim_root, onboard_root):
        meta, data, pose, origin, duration = load_run(root)
        inputs.append((meta, data, pose, origin, duration))
    if [i[0].get('platform', 'sim') for i in inputs] != ['sim', 'onboard']:
        raise ValueError('Expected simulation then onboard inputs')
    refs = [i[1]['reference_path'] for i in inputs]
    if refs[0].shape != refs[1].shape or not np.allclose(refs[0], refs[1], rtol=0, atol=1e-9):
        raise ValueError('Recorded ordered reference geometry/direction differs; no implicit alignment')
    projected, passes = [], []
    period = geometry(refs[0])[-1][-1]
    for meta, data, pose, origin, duration in inputs:
        rows, _ = project_continuously(pose, refs[0], settings)
        projected.append(rows)
        passes.append(select_first_pass(rows, period, start, period if length is None else length, settings))
    output.mkdir(parents=True)
    metrics, curves = [], []
    for index, side in enumerate(('sim', 'onboard')):
        root = (sim_root, onboard_root)[index]; selected = passes[index]
        metric = create_section(root, output/(side+'_section'), selected, inputs[index][3])
        if metric['analysis_status'] != 'PASS':
            raise ValueError('Cropped section analysis failed: '+json.dumps(metric['data_quality']['errors']))
        metrics.append(metric)
        curves.append(histories(inputs[index][1], inputs[index][2], selected, settings, metric))
    if metrics[0]['settings'] != metrics[1]['settings']:
        raise ValueError('Different effective shared analysis settings; no common metric definitions')
    certified = strict.compare(output/'sim_section', output/'onboard_section', output/'strict_comparison')
    rebase_generated_links(output/'strict_comparison/report.md', sim_root)
    flat = list(map(strict.flatten, metrics)); comparisons = {}
    wanted = METRIC_ORDER
    for name in wanted:
        comparisons[name] = exploratory(flat[0][name], flat[1][name], name, settings['minimum_metric_coverage'])
    result = {'schema_version': 1, 'classification': 'MATCHED_SECTION_EXPLORATORY',
              'selection_rule': 'Earliest complete contiguous reference traversal on each input; no health selection',
              'reference_length_m': float(period), 'section_start_m': start, 'settings': settings,
              'source_sha256': sha256(Path(__file__)),
              'inputs': {}, 'metrics': comparisons,
              'elapsed_section_sec': {'sim': passes[0]['elapsed_sec'], 'onboard': passes[1]['elapsed_sec'],
                                      'exploratory_onboard_minus_sim': passes[1]['elapsed_sec']-passes[0]['elapsed_sec']},
              'strict_certified_deltas_available': sum(r['delta_right_minus_left'] is not None for r in certified['metrics'].values()),
              'limitations': ['PF-based reference alignment is not independent physical section truth.',
                              'Historical onboard deployed configuration/health threshold equivalence remains unknown.',
                              'Smoothness still uses raw-odometry cumulative driven distance, not raceline station.',
                              'Headline metrics retain original sample/time definitions; only plots use a common station grid.',
                              'Forward response is Gym body speed versus ERPM-derived onboard estimate; no numeric response delta.',
                              'No GT accuracy, body-IMU yaw, collision or completion comparison is fabricated.']}
    for side, root, inp, selected, projection, curve in zip(('sim', 'onboard'), (sim_root, onboard_root), inputs, passes, projected, curves):
        in_section = [r for r in projection if selected['start_sec'] <= r['t'] <= selected['end_sec']]
        health = inp[1].get('localization_health', [])
        summary = A.state_summary(health, inp[4], metrics[0]['settings']['max_status_gap_sec']) if health else None
        full_health = A.health_distribution(summary, inp[4], A.dropouts([(t, t+metrics[0]['settings']['max_status_gap_sec']) for t, _ in health], inp[4])) if summary else None
        observed = {}
        for role, columns in [('autonomy_command', [('speed_mps', 1), ('steering_rad', 2)]),
                              ('final_drive_command', [('speed_mps', 1)])]:
            samples = inp[1].get(role)
            if samples is not None:
                samples = samples[(samples[:, 0] >= selected['start_sec']) & (samples[:, 0] < selected['end_sec'])]
                for label, col in columns:
                    observed[role+'_'+label] = {'min': float(np.min(samples[:, col])), 'max': float(np.max(samples[:, col]))} if len(samples) else None
        result['inputs'][side] = {'full_input_health': full_health, 'observed_command_ranges': observed,
                                 'source_smoke_result': inp[0].get('smoke_result', {}).get('result', 'UNKNOWN'),
                                 'source_recording_status': inp[0].get('recording', {}).get('status', 'UNKNOWN'),
                                 'source_visualization_mode': inp[0].get('visualization_mode', 'UNKNOWN'),
                                 'source_directory': str(root.resolve()),
                                 'source_provenance': {k: inp[0].get(k) for k in ('main_revision', 'pf_revision', 'source_state', 'configs', 'track')}, 'source_metadata_sha256': sha256(root/'metadata.yaml'),
                                 'comparison_identity': inp[0].get('comparison_identity', {}), 'selected_interval': selected,
                                 'projection_sample_count': len(projection),
                                 'projection_excluded_reasons': dict(Counter(r['reason'] for r in projection if r['reason'])),
                                 'section_projection_samples': len(in_section),
                                 'section_projection_coverage': sum(r['s'] is not None for r in in_section)/len(in_section),
                                 'outside_selected_time_samples': len(projection)-len(in_section),
                                 'grid_coverage': {k: float(np.mean(np.isfinite(v))) for k, v in curve.items()},
                                 'projection': projection}
    sim_meta = inputs[0][0]
    runtime_root = Path(sim_meta.get('reanalysis', {}).get('source_run', str(sim_root)))
    runtime = runtime_root/'runtime_parameter_evidence.json'
    if runtime.is_file():
        result['sim_runtime_parameters'] = {'source': str(runtime), 'sha256': sha256(runtime),
                                            'values': json.loads(runtime.read_text())}
    # Passive independent GT cross-check of SIM alignment only. Never selects sections.
    truth = inputs[0][1].get('ground_truth_pose')
    if truth is not None:
        pose = inputs[0][2]; gt = A.interpolate(truth, pose[:, 0], metrics[0]['settings']['max_gt_gap_sec'])
        mask = ((pose[:, 0] >= passes[0]['start_sec']) & (pose[:, 0] < passes[0]['end_sec']) & np.all(np.isfinite(gt), axis=1))
        gt_pose = np.c_[pose[mask, 0], gt[mask], pose[mask, 0]]
        gt_rows, _ = project_continuously(gt_pose, refs[0], settings)
        pf_s = np.array([r['s'] for r, keep in zip(projected[0], mask) if keep])
        gt_s = np.array([r['s'] if r['s'] is not None else np.nan for r in gt_rows])
        discrepancy = np.abs((pf_s-gt_s+period/2) % period-period/2)
        result['sim_gt_crosscheck'] = {'station_disagreement_m': A.quantiles(discrepancy[np.isfinite(discrepancy)]),
                                      'coverage': float(np.mean(np.isfinite(discrepancy))),
                                      'absolute_accuracy': metrics[0]['localization']['accuracy']}
    render(output, result, metrics, curves)
    (output/'comparison.json').write_text(json.dumps(json_safe(result), indent=2, sort_keys=True, allow_nan=False)+'\n')
    (output/'station_signals.json').write_text(json.dumps(json_safe(dict(zip(('sim', 'onboard'), curves))), indent=2, sort_keys=True, allow_nan=False)+'\n')
    return result


def ordered_summary(value):
    preferred = ('p50', 'p95', 'max', 'rms', 'p95_absolute')
    keys = [k for k in preferred if k in value] + sorted(set(value)-set(preferred))
    return {k: value[k] for k in keys}


def human_metric(name, metric):
    view = dict(metric)
    if isinstance(view.get('value'), dict):
        view['value'] = ordered_summary(view['value'])
    return A.human_result(name, view)


def onboard_name(result):
    """Identify the preserved input in human reports; never assume Session 3."""
    source = Path(result['inputs']['onboard']['source_directory']).name
    match = re.search(r'session(\d+)', source, re.IGNORECASE)
    return 'Onboard Session '+match.group(1) if match else 'Onboard run'


def direction_segments(points, fractions, arrow_length_m=.22):
    """Display-only arrows along ordered geometry, with no fit or trajectory change."""
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return []
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    keep = np.r_[True, np.diff(distance) > 1e-9]
    points, distance = points[keep], distance[keep]
    if distance[-1] <= 1e-9:
        return []
    segments = []
    for fraction in fractions:
        start = float(fraction)*distance[-1]
        end = min(start+arrow_length_m, distance[-1])
        if not 0 <= start < end:
            continue
        segment = np.column_stack([np.interp([start, end], distance, points[:, col]) for col in (0,1)])
        if np.linalg.norm(segment[1]-segment[0]) > 1e-9:
            segments.append(segment)
    return segments


def occupancy_background(map_yaml, image_path):
    """ROS occupancy semantics and pixel-to-map transform, for display only."""
    import matplotlib.image as mpimg
    info = yaml.safe_load(map_yaml.read_text())
    resolution = float(info['resolution']); origin = np.asarray(info['origin'], dtype=float)
    if resolution <= 0 or not math.isfinite(resolution) or origin.shape != (3,) or not np.all(np.isfinite(origin)):
        raise ValueError('Invalid display map resolution/origin')
    pixels = mpimg.imread(str(image_path))
    if pixels.ndim == 3:
        pixels = pixels[:, :, :3].mean(axis=2)
    pixels = pixels.astype(float)/(255. if pixels.max() > 1 else 1.)
    occupancy = pixels if info.get('negate', 0) else 1.-pixels
    occupied = occupancy > float(info['occupied_thresh'])
    free = occupancy < float(info['free_thresh'])
    shade = np.where(occupied, .32, np.where(free, .99, .84))
    return shade, resolution, origin


def spatial_plots(output, result):
    """Read selected section evidence; never alter alignment or headline metrics."""
    import matplotlib.pyplot as plt
    from matplotlib.transforms import Affine2D
    if not all((output/(side+'_section')/'metadata.yaml').is_file() for side in ('sim','onboard')):
        return [], 'Selected-section bags unavailable; no XY plot generated.'
    sections = [load_run(output/(side+'_section')) for side in ('sim','onboard')]
    reference = sections[0][1]['reference_path']
    poses = [item[2] for item in sections]
    truth = sections[0][1].get('ground_truth_pose')
    if truth is not None:
        truth = truth[(truth[:, 0] >= 0) & (truth[:, 0] < sections[0][4])]
    series = [(poses[0][:, 1:3], 'Simulation PF', 'tab:blue', '-'),
              (poses[1][:, 1:3], onboard_name(result)+' PF', 'tab:orange', '-')]
    if truth is not None and len(truth):
        series.append((truth[:, 1:3], 'Simulation GT (independent)', 'tab:green', ':'))
    points = np.vstack([reference, *[item[0] for item in series]])
    low, high = points.min(axis=0)-.65, points.max(axis=0)+.65
    def draw(ax, lines, bounds=None):
        ax.plot(reference[:, 0], reference[:, 1], 'k--', linewidth=1.2, label='Recorded raceline', zorder=3)
        def arrows(xy, color, fractions):
            for begin, end in direction_segments(xy, fractions):
                ax.annotate('', xy=end, xytext=begin, zorder=7,
                            arrowprops=dict(arrowstyle='-|>', color=color, lw=1.3, mutation_scale=13))
        arrows(reference, 'black', (.18, .51, .84))
        for xy, label, color, style in lines:
            ax.plot(xy[:, 0], xy[:, 1], color=color, linestyle=style, linewidth=1.5, label=label, zorder=4)
            fractions = {'tab:blue': (.10,.43,.76), 'tab:orange': (.26,.59,.90), 'tab:green': (.34,.67,.95)}[color]
            arrows(xy, color, fractions)
            ax.scatter(*xy[0], marker='o', color=color, s=28, zorder=5)
            ax.scatter(*xy[-1], marker='x', color=color, s=35, zorder=5)
        left, right = (low, high) if bounds is None else bounds
        ax.set(xlim=(left[0], right[0]), ylim=(left[1], right[1]), xlabel='Map x [m]', ylabel='Map y [m]')
        ax.set_aspect('equal', adjustable='box'); ax.legend(fontsize=11, loc='best')
        ax.grid(alpha=.15)
    fig, ax = plt.subplots(figsize=(8, 6))
    draw(ax, series)
    ax.set_title('Matched-section trajectories relative to the recorded raceline\n'+onboard_name(result))
    fig.text(.5, .015, 'PF lines are estimates; GT is simulation only. Arrows: point-order direction. Circles/crosses: first/last samples.', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .04, 1, 1)); fig.savefig(output/'plots/trajectory_xy.png', dpi=160); plt.close(fig)
    files = ['trajectory_xy.png']
    # Use only the captured simulation map; no claim about the historical onboard map.
    meta = sections[0][0]; configs = meta.get('configs', {})
    if not all(k in configs for k in ('map_yaml', 'map_image')):
        return files, 'No captured map pair; map-free XY comparison remains available.'
    paths = [output/'sim_section'/configs[k]['copy'] for k in ('map_yaml', 'map_image')]
    if any(not path.is_file() or sha256(path) != configs[k]['sha256'] for path, k in zip(paths, ('map_yaml','map_image'))):
        return files, 'Map snapshot missing/hash mismatch; map overlay omitted rather than substituting current assets.'
    shade, resolution, origin = occupancy_background(*paths)
    fig, axes = plt.subplots(1, 3, figsize=(15, 7), sharex=True, sharey=True)
    # Wider map context than the map-free detail plot: keep circuit walls visible.
    corners = np.array([[0,0], [shade.shape[1]*resolution,0],
                        [0,shade.shape[0]*resolution], [shade.shape[1]*resolution,shade.shape[0]*resolution]])
    rotation = np.array([[np.cos(origin[2]), -np.sin(origin[2])], [np.sin(origin[2]), np.cos(origin[2])]])
    world_corners = corners @ rotation.T + origin[:2]
    context = (np.maximum(points.min(axis=0)-1.5, world_corners.min(axis=0)),
               np.minimum(points.max(axis=0)+1.5, world_corners.max(axis=0)))
    panels = [(series[:2], 'Both PF estimates'),
              ([series[0]]+series[2:], 'Simulation PF and independent GT'),
              ([series[1]], onboard_name(result)+' PF (no GT)')]
    for ax, (lines, title) in zip(axes, panels):
        transform = Affine2D().rotate(origin[2]).translate(origin[0], origin[1])+ax.transData
        ax.imshow(shade, cmap='gray', vmin=0, vmax=1, origin='upper', interpolation='nearest',
                  extent=(0, shade.shape[1]*resolution, 0, shade.shape[0]*resolution), transform=transform, zorder=0)
        draw(ax, lines, context); ax.set_title(title, fontsize=11)
    fig.suptitle('Tempo circuit context — captured simulation occupancy map\n'+onboard_name(result), fontsize=14)
    fig.text(.5, .035, 'Dark: mapped obstacles/boundaries. Light gray: unknown. White: mapped free space.\n'
             'Onboard traces are PF estimates; its historical loaded map identity and actual vehicle path remain unverified.',
             ha='center', fontsize=10)
    fig.tight_layout(rect=(0, .09, 1, .94)); fig.savefig(output/'plots/trajectory_map.png', dpi=160); plt.close(fig)
    return files+['trajectory_map.png'], 'Map background: hash-verified captured simulation map; historical onboard map identity UNKNOWN.'


HEALTH_STYLE = {'GOOD': ('#208443', 'o', 10), 'DEGRADED': ('#e49b00', '^', 24),
                'UNKNOWN': ('#7b43a1', '+', 38), 'INVALID': ('#d62728', 'X', 42)}


def pose_health_states(pose, health, gap):
    """Unstamped health: causal last published state at pose bag-receive time."""
    states = []
    for row in pose:
        state = A.held(health, float(row[4]), gap)
        states.append(state[1] if state and state[1] in HEALTH_STYLE else 'UNKNOWN')
    return np.asarray(states)


def health_position_plots(output, result, metrics):
    """Spatial annotations of reported quality; no scorecard/selection changes."""
    import matplotlib.pyplot as plt
    from matplotlib.transforms import Affine2D
    source = Path(result['inputs']['onboard']['source_directory'])
    if not (source/'metadata.yaml').is_file() or not (output/'onboard_section/metadata.yaml').is_file():
        return [], 'Health-position evidence unavailable.'
    full = load_run(source); selected = load_run(output/'onboard_section')
    sim = load_run(output/'sim_section'); gap = metrics[1]['settings']['max_status_gap_sec']
    reference = full[1]['reference_path']; period = geometry(reference)[-1][-1]
    full_states = pose_health_states(full[2], full[1].get('localization_health', []), gap)
    selected_states = pose_health_states(selected[2], selected[1].get('localization_health', []), gap)
    def stations(item):
        rows, _ = project_continuously(item[2], reference, result['settings'])
        return np.array([r['s'] % period if r['s'] is not None else np.nan for r in rows])
    full_s, selected_s = stations(full), stations(selected)
    sim_s = stations(sim)
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(onboard_name(result)+' — reported PF quality at estimated vehicle positions', fontsize=15)
    corners = np.vstack((reference, full[2][:, 1:3], selected[2][:, 1:3]))
    low, high = corners.min(axis=0)-1.0, corners.max(axis=0)+1.0
    map_note = 'No map background available.'
    configs = sim[0].get('configs', {})
    if all(k in configs for k in ('map_yaml','map_image')):
        paths = [output/'sim_section'/configs[k]['copy'] for k in ('map_yaml','map_image')]
        if all(p.is_file() and sha256(p)==configs[k]['sha256'] for p,k in zip(paths,('map_yaml','map_image'))):
            shade, resolution, origin = occupancy_background(*paths)
            for ax in axes[0]:
                transform = Affine2D().rotate(origin[2]).translate(origin[0],origin[1])+ax.transData
                ax.imshow(shade,cmap='gray',vmin=0,vmax=1,origin='upper',interpolation='nearest',
                          extent=(0,shade.shape[1]*resolution,0,shade.shape[0]*resolution),transform=transform,zorder=0)
            map_note = 'Background: captured simulation map; historical onboard map identity unverified.'
    def scatter(ax, x, y, states, counts=False):
        for name,(color,marker,size) in HEALTH_STYLE.items():
            mask = (states==name) & np.isfinite(x) & np.isfinite(y)
            display_name = 'UNKNOWN / stale' if name=='UNKNOWN' else name
            label = display_name+' ({} poses)'.format(int(sum(mask))) if counts else display_name
            ax.scatter(x[mask],y[mask],color=color,marker=marker,s=size,
                       alpha=.65 if name=='GOOD' else .95,label=label,zorder=4+list(HEALTH_STYLE).index(name))
    for ax in axes[0]:
        ax.plot(reference[:,0],reference[:,1],'k--',lw=1.1,label='Recorded raceline',zorder=2)
        for start,end in direction_segments(reference,(.18,.51,.84)):
            ax.annotate('',xy=end,xytext=start,arrowprops=dict(arrowstyle='-|>',color='black',lw=1.2),zorder=8)
        ax.set(xlabel='Map x [m]',ylabel='Map y [m]',xlim=(low[0],high[0]),ylim=(low[1],high[1]))
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.12)
    scatter(axes[0,0],full[2][:,1],full[2][:,2],full_states,True)
    axes[0,0].set_title('Full reviewed onboard window — {:.1f} s'.format(full[4]))
    axes[0,1].plot(sim[2][:,1],sim[2][:,2],color='tab:blue',lw=1.1,label='Matched simulation PF',zorder=3)
    scatter(axes[0,1],selected[2][:,1],selected[2][:,2],selected_states,True)
    axes[0,1].set_title('Matched traversal — onboard reported quality')
    for ax in axes[0]:ax.legend(fontsize=11,loc='best')
    # Time separates repeated visits/laps; no connecting line bridges a seam or gap.
    scatter(axes[1,0],full[2][:,4],full_s,full_states)
    health = full[1].get('localization_health', [])
    unknown = A.dropouts([(t,t+gap) for t,_ in health],full[4])
    for i,(start,end) in enumerate(unknown):
        axes[1,0].axvspan(start,end,color=HEALTH_STYLE['UNKNOWN'][0],alpha=.15,
                         label='No fresh health / no pose position inferred' if i==0 else None)
    axes[1,0].set(xlabel='Pose receive time in reviewed window [s]',ylabel='Raceline station [m]',
                  title='Full window: repeated visits and quality over time',xlim=(0,full[4]),ylim=(0,period))
    axes[1,0].legend(fontsize=11,loc='best');axes[1,0].grid(alpha=.15)
    import evaluation_particle_spread as P
    onboard_cloud=P.load(output/'onboard_section')[3]['pose_samples']
    sim_cloud=P.load(output/'sim_section')[3]['pose_samples']
    onboard_spread=np.array([p['spread_m'] if p['spread_m'] is not None else np.nan for p in onboard_cloud])
    sim_spread=np.array([p['spread_m'] if p['spread_m'] is not None else np.nan for p in sim_cloud])
    axes[1,1].scatter(sim_s,sim_spread,color='tab:blue',s=8,label='Simulation PF particle spread')
    scatter(axes[1,1],selected_s,onboard_spread,selected_states)
    axes[1,1].set(xlabel='Raceline station [m]',ylabel='PF particle spread [m]',
                  title='Matched traversal: internal uncertainty and quality',xlim=(0,period))
    axes[1,1].legend(fontsize=11,loc='best');axes[1,1].grid(alpha=.15)
    fig.text(.5,.02,'PF-reported states, not accuracy; UNKNOWN means missing/stale health. INVALID markers are drawn last across overlapping laps.\n'
             'Last health at pose receipt, age <= {:.2f} s; no future state or spatial interpolation. Blank gaps have no invented vehicle position.\n'.format(gap)+map_note,
             ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.08,1,.95));fig.savefig(output/'plots/pf_health_positions.png',dpi=160);plt.close(fig)
    evidence = {'method':'last_published_health_at_pose_receive_v1','max_health_age_sec':gap,
                'qualification':'Health has no source stamp; pose/state association is approximate, not calibrated confidence.',
                'map_context':map_note,'unknown_health_intervals_sec':unknown}
    for name,item,states,station in [('full_onboard',full,full_states,full_s),('matched_onboard',selected,selected_states,selected_s)]:
        evidence[name] = {'pose_counts':{k:int(sum(states==k)) for k in HEALTH_STYLE},
                          'station_unavailable_pose_count':int(sum(~np.isfinite(station))),
                          'samples': [{'source_sec':float(row[0]),'receive_sec':float(row[4]),'x_m':float(row[1]),'y_m':float(row[2]),
                                       'state':str(state),'station_m':float(ss) if math.isfinite(ss) else None}
                                      for row,state,ss in zip(item[2],states,station)]}
    (output/'pf_health_positions.json').write_text(json.dumps(evidence,indent=2,sort_keys=True)+'\n')
    return ['pf_health_positions.png'], ('Health markers use the most recent published state at pose receipt, held at most {:.2f} s; '
              'the health topic is unstamped, so this is approximate association. Counts are pose samples, not time percentages. '
              'The full window retains later INVALID states; the matched traversal is shown separately. '
              'Repeated laps overlap on the map, so INVALID is drawn last; the time/station panel separates visits. '
              'No position is inferred during gaps without PF poses. See `pf_health_positions.json` for exact associations.').format(gap)


def deviation_position_plots(output, result):
    """Color existing PF/reference distances on maps; never change metric math."""
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize
    from matplotlib.transforms import Affine2D
    if not all((Path(result['inputs'][side]['source_directory'])/'metadata.yaml').is_file() and (output/(side+'_section')/'metadata.yaml').is_file() for side in ('sim','onboard')):
        return [], 'Full and matched PF pose evidence unavailable; no deviation map fabricated.'
    roots = {side: Path(result['inputs'][side]['source_directory']) for side in ('sim','onboard')}
    full_reviews = sorted(output.glob('full_onboard_session*'))
    if len(full_reviews)==1 and (full_reviews[0]/'metadata.yaml').is_file():
        roots['onboard']=full_reviews[0]
    runs = {side: load_run(root) for side,root in roots.items()}
    selected = {side: load_run(output/(side+'_section')) for side in runs}
    reference = selected['sim'][1]['reference_path']
    views = [('full_'+side, runs[side]) for side in runs]+[('matched_'+side, selected[side]) for side in runs]
    errors = {name: A.reference_errors(run[2], run[1]['reference_path'])[0] for name,run in views}
    maximum = max(float(np.max(values)) for values in errors.values() if len(values))
    norm = Normalize(vmin=0., vmax=max(maximum, 1e-6))
    points = np.vstack([reference]+[run[2][:,1:3] for _,run in views])
    low, high = points.min(axis=0)-1., points.max(axis=0)+1.
    configs = selected['sim'][0].get('configs', {})
    background = None
    if all(k in configs for k in ('map_yaml','map_image')):
        paths = [output/'sim_section'/configs[k]['copy'] for k in ('map_yaml','map_image')]
        if all(p.is_file() and sha256(p)==configs[k]['sha256'] for p,k in zip(paths,('map_yaml','map_image'))):
            background = occupancy_background(*paths)
    note = ('Background: hash-verified captured simulation occupancy map; historical onboard loaded map is unverified.'
            if background is not None else 'Map snapshot unavailable or mismatched: no substituted background.')
    evidence = {'method':'existing_pf_reference_errors_display_v1', 'color_range_m':[0.,norm.vmax],
                'map_context':note,'qualification':'PF-estimated distance to raceline, not PF accuracy or wall clearance.', 'views':{}}
    def draw(ax, name, run, title):
        if background is not None:
            shade,resolution,origin=background
            transform=Affine2D().rotate(origin[2]).translate(origin[0],origin[1])+ax.transData
            ax.imshow(shade,cmap='gray',vmin=0,vmax=1,origin='upper',interpolation='nearest',
                      extent=(0,shade.shape[1]*resolution,0,shade.shape[0]*resolution),transform=transform,zorder=0)
        ref=run[1]['reference_path'];pose=run[2];values=errors[name]
        ax.plot(ref[:,0],ref[:,1],'k--',lw=1.,label='Recorded raceline',zorder=2)
        for start,end in direction_segments(ref,(.18,.51,.84)):
            ax.annotate('',xy=end,xytext=start,arrowprops=dict(arrowstyle='-|>',color='black',lw=1.1),zorder=6)
        # Nearest point on exactly the same polyline used by reference_errors.
        starts,vectors,lengths,_=geometry(ref)
        links=[]
        for row in pose[::max(1,len(pose)//35)]:
            fraction=np.clip(np.sum((row[1:3]-starts)*vectors,axis=1)/lengths**2,0,1)
            feet=starts+fraction[:,None]*vectors
            foot=feet[np.argmin(np.linalg.norm(feet-row[1:3],axis=1))]
            ax.plot([row[1],foot[0]],[row[2],foot[1]],color='gray',lw=.6,alpha=.5,zorder=3)
            links.append(foot.tolist())
        # High deviations last: overlapping laps cannot hide the largest excursions.
        order=np.argsort(values,kind='stable')
        scatter=ax.scatter(pose[order,1],pose[order,2],c=values[order],norm=norm,cmap='viridis',s=13,zorder=4)
        ax.scatter(*pose[0,1:3],marker='o',s=48,facecolors='none',edgecolors='black',label='First pose',zorder=5)
        ax.scatter(*pose[-1,1:3],marker='x',s=40,color='black',label='Last pose',zorder=5)
        ax.set(title=title,xlabel='Map x [m]',ylabel='Map y [m]',xlim=(low[0],high[0]),ylim=(low[1],high[1]))
        ax.set_aspect('equal',adjustable='box');ax.legend(fontsize=11);ax.grid(alpha=.1)
        evidence['views'][name]={'source_directory':str(roots['sim' if name.endswith('sim') else 'onboard']),
            'samples':[{'source_sec':float(row[0]),'receive_sec':float(row[4]),'x_m':float(row[1]),'y_m':float(row[2]),'reference_deviation_m':float(value)} for row,value in zip(pose,values)],
            'projection_link_count':len(links)}
        return scatter
    caption = ('Color = distance from PF-estimated position to recorded raceline [m], not localization error or wall clearance.\n'
               'Gray spokes show sparse nearest-reference projections; black arrows show raceline driving direction.\n'
               'Overlapping visits: larger deviations drawn last. Shared color scale across full and matched views.\n'+note)
    for side in ('sim','onboard'):
        root=roots[side]
        fig,ax=plt.subplots(figsize=(8,7))
        sc=draw(ax,'full_'+side,runs[side],('Simulation PF' if side=='sim' else onboard_name(result)+' PF')+' — full evaluation distance to raceline')
        fig.colorbar(sc,ax=ax,label='PF-estimated distance to raceline [m]',shrink=.8)
        fig.text(.5,.02,caption,ha='center',fontsize=11)
        fig.tight_layout(rect=(0,.12,1,1));fig.savefig(root/'plots/pf_reference_deviation_positions.png',dpi=160);plt.close(fig)
        fragment='#### Where the estimated trajectory deviates from the raceline\n\n![PF-estimated distance to raceline on map](plots/pf_reference_deviation_positions.png)\n\n'+caption.replace('\n',' ')+'\n\nLighter/yellower positions are farther from the reference. This is tracking in the PF-estimated map frame; it cannot independently reveal actual vehicle position. Static-distance to raceline may include intentional avoidance. Boundary pixels are context, not a signed clearance measurement.\n\n'
        attach_report_fragment(root,'pf_reference_deviation.md',fragment,'(plots/pf_reference_trajectory.png)')
    fig,axes=plt.subplots(1,2,figsize=(13,7),sharex=True,sharey=True)
    for ax,side in zip(axes,('sim','onboard')):
        sc=draw(ax,'matched_'+side,selected[side],('Simulation PF' if side=='sim' else onboard_name(result)+' PF')+' — matched traversal')
    color_axis=fig.add_axes([.90,.25,.018,.55])
    fig.colorbar(sc,cax=color_axis,label='PF-estimated distance to raceline [m]')
    fig.suptitle('Matched PF-estimated tracking — same reference and distance color scale')
    fig.subplots_adjust(bottom=.18,top=.88,wspace=.22,right=.86)
    fig.text(.5,.025,caption,ha='center',fontsize=9)
    fig.savefig(output/'plots/pf_reference_deviation_positions.png',dpi=160);plt.close(fig)
    (output/'pf_reference_deviation_positions.json').write_text(json.dumps(evidence,indent=2,sort_keys=True,allow_nan=False)+'\n')
    return ['pf_reference_deviation_positions.png'], caption.replace('\n',' ')


def attach_report_fragment(root, filename, fragment, marker):
    """Idempotent optional presentation addendum; raw evidence remains untouched."""
    path=root/filename
    previous=path.read_text() if path.exists() else None
    report=root/'report.md';text=report.read_text()
    if previous and previous in text:
        text=text.replace(previous,fragment)
    elif fragment not in text:
        lines=text.splitlines(keepends=True)
        index=next((i for i,line in enumerate(lines) if marker in line),None)
        if index is None:raise ValueError('Expected figure marker missing: '+marker)
        lines.insert(index+1,'\n'+fragment);text=''.join(lines)
    path.write_text(fragment);report.write_text(text)


def heading_series(metadata, data, pose, settings):
    """Reuse the analyzer's reference tangent and shortest-arc GT alignment."""
    _, error, tangent = A.reference_errors(pose, data['reference_path'], return_tangents=True)
    output = dict(time_sec=pose[:,0], pf_yaw_rad=A.wrapped(pose[:,3]), reference_at_pf_rad=tangent,
                  pf_reference_signed_rad=A.wrapped(pose[:,3]-tangent), pf_reference_absolute_rad=error)
    truth = data.get('ground_truth_pose') if metadata.get('platform','sim')=='sim' else None
    if truth is not None and len(truth):
        roles=A.normalize_roles(metadata)
        if not A.same_frame(roles['estimated_pose'].get('frame'),roles['ground_truth_pose'].get('frame')):
            raise ValueError('Heading/trajectory GT and PF frames differ; no implicit transform')
        aligned=A.interpolate(truth,pose[:,0],settings['max_gt_gap_sec'])
        valid=np.all(np.isfinite(aligned),axis=1)
        gt_tangent=np.full(len(pose),np.nan)
        if any(valid):
            virtual_pose=np.column_stack((pose[valid,0],aligned[valid]))
            _,_,gt_tangent[valid]=A.reference_errors(virtual_pose,data['reference_path'],return_tangents=True)
        output.update(gt_yaw_rad=A.wrapped(aligned[:,2]),reference_at_gt_rad=gt_tangent,
                      pf_gt_signed_rad=A.wrapped(pose[:,3]-aligned[:,2]),
                      gt_reference_signed_rad=A.wrapped(aligned[:,2]-gt_tangent),gt_alignment_valid=valid)
    return output


def angle_display(times, angles, gap=.5):
    """Break lines at pi wrap boundaries and evidence gaps, without losing samples."""
    tt=[];aa=[]
    for i,(t,a) in enumerate(zip(times,angles)):
        if i and (t-times[i-1]>gap or (np.isfinite(a) and np.isfinite(angles[i-1]) and abs(a-angles[i-1])>np.pi)):
            tt.append(np.nan);aa.append(np.nan)
        tt.append(float(t));aa.append(float(a))
    return np.asarray(tt),np.degrees(aa)


def plot_heading_run(root, run, settings):
    import matplotlib.pyplot as plt
    meta,data,pose,_,duration=run
    series=heading_series(meta,data,pose,settings)
    simulation='gt_yaw_rad' in series
    fig,axes=plt.subplots(3 if simulation else 2,1,sharex=True,figsize=(10,8 if simulation else 6))
    def line(ax,key,label,color,style='-'):
        x,y=angle_display(series['time_sec'],series[key]);ax.plot(x,y,color=color,ls=style,lw=1.,label=label)
    line(axes[0],'pf_yaw_rad','PF map yaw','tab:blue')
    if simulation:line(axes[0],'gt_yaw_rad','Simulator GT yaw','tab:green')
    line(axes[0],'reference_at_pf_rad','Raceline tangent at PF position','black','--')
    axes[0].set(title='Map orientation; gaps at wrap boundaries are deliberate',ylabel='Map yaw [deg]')
    if simulation:
        line(axes[1],'pf_gt_signed_rad','PF minus aligned GT','tab:blue')
        axes[1].set(title='Localization heading error — independent simulator truth',ylabel='Signed yaw error [deg]')
    line(axes[-1],'pf_reference_signed_rad','PF minus tangent at PF position','tab:blue')
    if simulation:line(axes[-1],'gt_reference_signed_rad','GT minus tangent at GT position','tab:green')
    axes[-1].set(title='Heading relative to the raceline — tracking, not localization accuracy',ylabel='Signed heading difference [deg]',xlabel='Elapsed evaluation time [s]')
    for ax in axes:
        ax.grid(alpha=.15);ax.legend(fontsize=11);ax.set_xlim(0,duration)
    fig.suptitle('Heading analysis — '+('simulation PF / GT / raceline' if simulation else 'onboard PF / raceline; no independent GT'))
    fig.tight_layout();fig.savefig(root/'plots/heading_analysis.png',dpi=160);plt.close(fig)
    # Exact arrays retain gaps as null rather than fabricating zero.
    serial={key:[float(x) if np.isfinite(x) else None for x in value] for key,value in series.items()}
    (root/'heading_analysis.json').write_text(json.dumps(dict(method='existing_wrapped_heading_display_v1',series=serial,
        note='Reference ambiguities, source gaps and GT alignment failures remain missing. No IMU yaw-rate qualification implied.'),indent=2,sort_keys=True,allow_nan=False)+'\n')
    metrics=json.loads((root/'metrics.json').read_text())
    fragment='#### Heading analysis: localization error versus raceline tracking\n\n'
    fragment+=('Heading is the vehicle orientation in the map frame, derived from the recorded pose quaternion; it is not steering angle or IMU yaw rate. '
               'The reference heading is the tangent of the nearest unambiguous recorded raceline segment, respecting recorded direction. '
               'Positive signed difference means counterclockwise relative to the comparator. Wrapped differences use the shortest angle; a 359°/1° crossing is 2°, not 358°.\n\n')
    fragment+='![Heading relative to truth and reference](plots/heading_analysis.png)\n\n'
    fragment+=('Top: map-frame orientation. '+('Middle: PF-versus-GT heading error. ' if simulation else '')+
               'Bottom: each pose’s heading minus the raceline tangent at its own position. '
               'Disagreement with the raceline can reflect cornering, avoidance or reverse maneuvers; it does not by itself prove localization failure. '
               'Missing/equidistant reference tangents and source gaps stay blank; wrap boundaries are broken instead of drawing artificial vertical jumps.\n\n')
    fragment+='| Existing scorecard measurement | Median [deg] | P95 [deg] | Max [deg] | Meaning |\n| --- | --- | --- | --- | --- |\n'
    rows=[('PF / reference',metrics['vehicle']['tracking']['pf_heading_to_reference_rad'],'Where PF believes the car points relative to its intended path')]
    if simulation:
        rows += [('GT / reference',metrics['vehicle']['tracking']['heading_to_reference_rad'],'Actual simulated orientation relative to the path'),
                 ('PF / GT',metrics['localization']['accuracy']['absolute_heading_error_rad'],'Independent simulator localization heading error')]
    for label,metric,meaning in rows:
        values=['{:.2f}'.format(np.degrees(metric['value'][k])) for k in ('p50','p95','max')] if metric['status']=='AVAILABLE' else [metric['status']]*3
        fragment+='| {} | {} | {} | {} | {} |\n'.format(label,*values,meaning)
    if not simulation:fragment+='| PF / GT | NOT_APPLICABLE | NOT_APPLICABLE | NOT_APPLICABLE | No onboard independent heading truth |\n'
    if simulation and all(metric['status']=='AVAILABLE' for _,metric,_ in rows):
        accuracy=metrics['localization']['accuracy']['absolute_heading_error_rad']['value']['p95']
        tracking=metrics['vehicle']['tracking']['heading_to_reference_rad']['value']['p95']
        fragment+=('\n**What this run shows:** PF/GT heading-error P95 is {:.2f}°, while actual GT/reference heading-difference P95 is {:.2f}°. '
                   'PF estimates therefore follow simulator orientation much more closely than the vehicle orientation follows the static reference. '
                   'This distinguishes localization disagreement from tracking/heading behavior; it does not identify a controller defect. '
                   'These separate percentiles are not additive and must not be subtracted as an error budget.\n').format(np.degrees(accuracy),np.degrees(tracking))
    fragment+=('\nTable values are the existing absolute-angle scorecard values converted from radians to degrees; no new thresholds or metrics replace them. '
               'The simulation display aligns GT to public-PF timestamps with the existing gap-limited shortest-arc method; scorecard rows retain their original sample clocks/evidence. '
               'Onboard provides estimated orientation versus intended reference only, not actual heading accuracy. '
               '[Exact display evidence](heading_analysis.json).\n\n')
    attach_report_fragment(root,'heading_analysis.md',fragment,'(plots/pf_reference_trajectory.png)')
    return series


def simulation_trajectory_overlay(root, run, comparison):
    """One deliberately simple overlay: raceline, actual simulator path, PF path."""
    import matplotlib.pyplot as plt
    from matplotlib.transforms import Affine2D
    meta,data,pose,_,duration=run
    truth=data.get('ground_truth_pose')
    if truth is None or not len(truth):return
    # plot_heading_run already validates PF/GT frame identity.
    truth=truth[(truth[:,0]>=0)&(truth[:,0]<duration)]
    ref=data['reference_path'];fig,ax=plt.subplots(figsize=(9,7))
    configs=yaml.safe_load((comparison/'sim_section/metadata.yaml').read_text()).get('configs',{})
    if all(k in configs for k in ('map_yaml','map_image')):
        paths=[comparison/'sim_section'/configs[k]['copy'] for k in ('map_yaml','map_image')]
        if all(path.is_file() and sha256(path)==configs[k]['sha256'] for path,k in zip(paths,('map_yaml','map_image'))):
            shade,res,origin=occupancy_background(*paths)
            transform=Affine2D().rotate(origin[2]).translate(origin[0],origin[1])+ax.transData
            ax.imshow(shade,cmap='gray',vmin=0,vmax=1,origin='upper',interpolation='nearest',extent=(0,shade.shape[1]*res,0,shade.shape[0]*res),transform=transform,zorder=0)
    ax.plot(ref[:,0],ref[:,1],'k--',lw=1.6,label='Raceline reference',zorder=2)
    for start,end in direction_segments(ref,(.18,.51,.84)):
        ax.annotate('',xy=end,xytext=start,arrowprops=dict(arrowstyle='-|>',color='black',lw=1.2),zorder=6)
    for values,color,label,style,fractions in [(truth,'tab:green','Simulator ground truth','-',(.32,.65)),(pose,'tab:blue','PF estimated position',':',(.12,.48))]:
        xy=values[:,1:3].copy()
        for i in range(len(values)-1,0,-1):
            if values[i,0]-values[i-1,0]>.5:xy=np.insert(xy,i,[np.nan,np.nan],axis=0)
        ax.plot(xy[:,0],xy[:,1],color=color,ls=style,lw=1.6,label=label,zorder=3)
        for start,end in direction_segments(values[:,1:3],fractions):
            ax.annotate('',xy=end,xytext=start,arrowprops=dict(arrowstyle='-|>',color=color,lw=1.2),zorder=6)
    points=np.vstack((ref,pose[:,1:3],truth[:,1:3]));low,high=points.min(axis=0)-1.,points.max(axis=0)+1.
    ax.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),xlabel='Map x [m]',ylabel='Map y [m]',title='Full simulation: raceline reference, ground truth and PF estimate')
    ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.15);ax.legend(fontsize=10,loc='upper right')
    fig.tight_layout();fig.savefig(root/'plots/raceline_gt_pf_overlay.png',dpi=160);plt.close(fig)
    fragment='#### Full simulation trajectory: reference, GT and PF\n\n![Raceline, simulator truth and PF localization](plots/raceline_gt_pf_overlay.png)\n\n'
    fragment+=('Black dashed: intended raceline; green solid: actual simulator position; blue dotted: PF-estimated position. '
               'Equal map axes preserve geometry; arrows show driving direction. The captured occupancy map supplies circuit context. '
               'PF-to-GT separation is localization disagreement; GT-to-reference separation is actual simulated tracking; PF-to-reference separation is believed tracking. '
               'Repeated laps overlap. No fitted translation/rotation or GT control input is introduced.\n\n')
    attach_report_fragment(root,'simulation_trajectory_overlay.md',fragment,'(plots/pf_reference_trajectory.png)')


def heading_report_plots(output, result):
    """Full-run and matched heading displays; metric files are read-only."""
    import matplotlib.pyplot as plt
    if not all((Path(result['inputs'][side]['source_directory'])/'metadata.yaml').is_file() for side in ('sim','onboard')):
        return [],'Heading source evidence unavailable.'
    roots={side:Path(result['inputs'][side]['source_directory']) for side in ('sim','onboard')}
    copies=sorted(output.glob('full_onboard_session*'))
    if len(copies)==1:roots['onboard']=copies[0]
    full={side:load_run(root) for side,root in roots.items()}
    settings={side:json.loads((root/'metrics.json').read_text())['settings'] for side,root in roots.items()}
    series={side:plot_heading_run(roots[side],full[side],settings[side]) for side in roots}
    simulation_trajectory_overlay(roots['sim'],full['sim'],output)
    waves={};fig,axes=plt.subplots(2,2,figsize=(12,9),sharex=True)
    for side,column,color in [('sim',0,'tab:blue'),('onboard',1,'tab:orange')]:
        grid,times=station_grid(result['inputs'][side]['selected_interval'],result['settings'])
        pose=full[side][2];values=series[side];wave={'station_m':grid}
        for key in ('pf_yaw_rad','reference_at_pf_rad','gt_yaw_rad'):
            if key not in values:continue
            packed=np.column_stack((pose[:,0],np.zeros((len(pose),2)),values[key]))
            wave[key]=A.wrapped(A.interpolate(packed,times,result['settings']['max_pose_gap_sec'])[:,2])
        wave['pf_reference_absolute_rad']=signal_at(np.column_stack((pose[:,0],values['pf_reference_absolute_rad'])),1,times,result['settings']['max_pose_gap_sec'])
        for key,label,cc,style in [('pf_yaw_rad','PF yaw',color,'-'),('gt_yaw_rad','Simulator GT yaw','tab:green','-'),('reference_at_pf_rad','Reference tangent at PF','black','--')]:
            if key in wave:
                x,y=angle_display(grid,wave[key],gap=np.inf);axes[0,column].plot(x,y,color=cc,ls=style,lw=1.,label=label)
        axes[0,column].set(title='Simulation heading' if side=='sim' else onboard_name(result)+' heading',ylabel='Map yaw [deg]')
        axes[1,0].plot(grid,np.degrees(wave['pf_reference_absolute_rad']),color=color,label='Simulation PF' if side=='sim' else onboard_name(result)+' PF')
        if side=='sim' and 'pf_gt_signed_rad' in values:
            for key,label,cc in [('pf_gt_signed_rad','PF / GT localization error','tab:blue'),('gt_reference_signed_rad','GT / reference tracking','tab:green')]:
                values_abs=np.abs(values[key]);sampled=signal_at(np.column_stack((pose[:,0],values_abs)),1,times,result['settings']['max_pose_gap_sec'])
                wave[key+'_absolute']=sampled
                axes[1,1].plot(grid,np.degrees(sampled),color=cc,label=label)
        waves[side]={key:[float(v) if np.isfinite(v) else None for v in values] for key,values in wave.items()}
    axes[1,0].set(title='PF-believed heading relative to raceline',ylabel='Absolute heading difference [deg]')
    axes[1,1].set(title='Simulation-only independent truth checks',ylabel='Absolute heading difference [deg]')
    for ax in axes.flat:ax.grid(alpha=.15);ax.legend(fontsize=11);ax.set_xlabel('Reference station from matched-section start [m]')
    fig.suptitle('Matched-section heading: orientation, believed tracking and simulator truth')
    fig.tight_layout();fig.savefig(output/'plots/heading_comparison.png',dpi=160);plt.close(fig)
    # Two wider quantity-focused figures replace the crowded four-panel report default.
    for filename,kind in [('heading_orientation_comparison.png','orientation'),('heading_deviation_comparison.png','deviation')]:
        fig,pair_axes=plt.subplots(1,2,figsize=(13,5))
        if kind=='orientation':
            for ax,side,color in zip(pair_axes,('sim','onboard'),('tab:blue','tab:orange')):
                wave=waves[side];grid=np.asarray(wave['station_m'],dtype=float)
                for key,label,cc,style in [('pf_yaw_rad','PF yaw',color,'-'),('gt_yaw_rad','Simulator GT yaw','tab:green','-'),('reference_at_pf_rad','Reference tangent at PF','black','--')]:
                    if key in wave:
                        x,y=angle_display(grid,np.asarray(wave[key],dtype=float),gap=np.inf)
                        ax.plot(x,y,color=cc,ls=style,lw=1.3,label=label)
                ax.set(title='Simulation' if side=='sim' else onboard_name(result),ylabel='Map yaw [deg]')
        else:
            for side,color in [('sim','tab:blue'),('onboard','tab:orange')]:
                wave=waves[side]
                pair_axes[0].plot(wave['station_m'],np.degrees(np.asarray(wave['pf_reference_absolute_rad'],dtype=float)),color=color,label='Simulation PF' if side=='sim' else onboard_name(result)+' PF')
            wave=waves['sim']
            for key,label,color in [('pf_gt_signed_rad_absolute','PF / GT localization error','tab:blue'),('gt_reference_signed_rad_absolute','GT / reference tracking','tab:green')]:
                if key in wave:pair_axes[1].plot(wave['station_m'],np.degrees(np.asarray(wave[key],dtype=float)),color=color,label=label)
            pair_axes[0].set(title='Believed tracking on both platforms',ylabel='Absolute PF/reference heading difference [deg]')
            pair_axes[1].set(title='Independent simulator truth checks',ylabel='Absolute heading difference [deg]')
        for ax in pair_axes:
            ax.set_xlabel('Matched reference station [m]',fontsize=12);ax.tick_params(labelsize=11);ax.grid(alpha=.15);ax.legend(fontsize=11)
        fig.tight_layout();fig.savefig(output/'plots'/filename,dpi=160);plt.close(fig)
    (output/'heading_comparison.json').write_text(json.dumps(dict(method='existing_heading_metrics_station_display_v1',waves=waves,
        qualification='Exploratory display; no onboard truth or certified deltas. Common station remains separate from driven-distance smoothness.'),indent=2,sort_keys=True,allow_nan=False)+'\n')
    text='### Heading comparison: what can be compared\n\n![Matched map-frame headings](plots/heading_orientation_comparison.png)\n\n![Believed tracking versus simulator truth checks](plots/heading_deviation_comparison.png)\n\n[Optional four-panel overview](plots/heading_comparison.png). The separate figures above keep legends and axes readable.\n\n'
    text+=('First figure: map yaw and the nearest reference tangent at each PF position. Second figure, left: compares the existing absolute PF/reference heading-difference definition on the same matched station grid. '
           'Second figure, right: separates simulation PF/GT localization error from actual GT/reference tracking. Onboard has no equivalent independent truth. '
           'Yaw wraps use shortest-angle handling; ambiguous tangents and evidence gaps remain blank. '
           'This is heading from map poses, not steering angle, velocity direction or unqualified onboard IMU yaw rate. '
           'Reverse recovery/avoidance can increase distance to raceline without localization failure. Differences remain exploratory; historical identity gates and metric definitions are unchanged.\n\n')
    text+='| Existing matched-section heading result | Median [deg] | P95 [deg] | Max [deg] |\n| --- | --- | --- | --- |\n'
    for side,label,group,name in [('sim','Simulation PF / reference','tracking','pf_heading_to_reference_rad'),
                                  ('onboard','Onboard PF / reference','tracking','pf_heading_to_reference_rad'),
                                  ('sim','Simulation GT / reference','tracking','heading_to_reference_rad'),
                                  ('sim','Simulation PF / GT','accuracy','absolute_heading_error_rad')]:
        metric=json.loads((output/(side+'_section')/'metrics.json').read_text())
        entry=metric['vehicle' if group=='tracking' else 'localization'][group][name]
        values=['{:.2f}'.format(np.degrees(entry['value'][k])) for k in ('p50','p95','max')] if entry['status']=='AVAILABLE' else [entry['status']]*3
        text+='| {} | {} | {} | {} |\n'.format(label,*values)
    text+='\nValues are unchanged scorecard statistics converted to degrees; there is no new certified delta. A smaller onboard PF/distance to raceline does not prove smaller onboard localization error.\n\n'
    for side in ('sim','onboard'):
        link=os.path.relpath(roots[side]/'report.md',output)+'#heading-analysis-localization-error-versus-raceline-tracking'
        text+='[Full {} heading analysis]({}) · '.format(side,link)
    text+='[Exact plotted evidence](heading_comparison.json).\n\n'
    attach_report_fragment(output,'heading_analysis.md',text,'## 4. Trajectories and station-domain plots')
    return ['heading_comparison.png'],'Existing wrapped-heading metrics displayed in degrees; no numerical scorecard changes.'


def render(output, result, metrics, curves):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plots = output/'plots'; plots.mkdir(exist_ok=True)
    specs = [('tracking.png', [('pf_position_m','PF-estimated distance to raceline [m]'), ('pf_heading_rad','PF/reference heading difference [rad]')]),
             ('command_response.png', [('autonomy_speed_mps','Autonomy speed [m/s]'), ('autonomy_steering_rad','Autonomy steering [rad]'), ('forward_speed_mps','Forward speed [m/s]')]),
             ('consistency.png', [('consistency_position_m','Position disagreement [m]'), ('consistency_yaw_rad','Yaw disagreement [rad]')]),
             ('elapsed_health.png', [('elapsed_sec','Section elapsed time [s]'), ('health_state','Reported PF quality')])]
    for name, panels in specs:
        fig, axes = plt.subplots(len(panels), 1, sharex=True, figsize=(8, 2.6*len(panels)))
        for ax, (signal, label) in zip(np.atleast_1d(axes), panels):
            for side, curve in zip(('Simulation', onboard_name(result)), curves):
                if signal == 'health_state':
                    ax.step(curve['station_m'], curve[signal], where='post', label=side)
                else:
                    ax.plot(curve['station_m'], curve[signal], label=side)
            ax.set_ylabel(label); ax.legend()
            if signal == 'health_state':
                ax.set(yticks=[0, 1, 2, 3], yticklabels=['UNKNOWN', 'INVALID', 'DEGRADED', 'GOOD'])
        axes[-1].set_xlabel('Reference station from section start [m]')
        fig.suptitle('PF-aligned reference section — exploratory, not controlled platform deltas')
        fig.tight_layout(); fig.savefig(plots/name, dpi=140); plt.close(fig)
    spatial_files, spatial_note = spatial_plots(output, result)
    health_files, health_note = health_position_plots(output, result, metrics)
    deviation_files, deviation_note = deviation_position_plots(output, result)
    report = '# Tempo matched-reference-section comparison — '+onboard_name(result)+'\n\n## 1. Engineering result\n\n'
    report += ('A recorded closed-loop simulation and historical '+onboard_name(result)+' are aligned by public PF pose to the same recorded reference geometry and direction. '
               'The first complete continuous traversal is used on each platform; health does not select the interval. '
               'This is **exploratory engineering evidence**, not a provenance-qualified controlled experiment. Software/data processing PASS does not establish vehicle performance or simulator fidelity.\n\n')
    report += '**Matched length:** {:.3f} m. **Elapsed:** sim {:.3f} s; onboard {:.3f} s (descriptive difference {:+.3f} s). **Certified deltas:** {}.\n\n'.format(
        result['inputs']['sim']['selected_interval']['length_m'], result['elapsed_section_sec']['sim'], result['elapsed_section_sec']['onboard'], result['elapsed_section_sec']['exploratory_onboard_minus_sim'], result['strict_certified_deltas_available'])
    report += '| Measurement | Simulation | Onboard | Exploratory onboard − sim |\n| --- | --- | --- | --- |\n'
    for name in METRIC_ORDER:
        if name not in result['metrics']:
            continue
        row = result['metrics'][name]
        metric_name = name.split('.')[-1]
        label = A.METRIC_LABELS.get(metric_name, metric_name)
        diff = row['exploratory_onboard_minus_sim']
        diff_text = ', '.join('{}: {:+.4g}'.format(k,v) for k,v in ordered_summary(diff).items()) if isinstance(diff,dict) else '{:+.4g}'.format(diff) if isinstance(diff,(float,int)) else 'Not qualified; observations only'
        report += '| {} [{}] | {} | {} | {} |\n'.format(label, row['left']['unit'], human_metric(metric_name,row['left']), human_metric(metric_name,row['right']), diff_text)
    tracking = result['metrics']['vehicle.tracking.pf_reference_position_deviation_m']
    steering = result['metrics']['vehicle.smoothness.autonomy_steering_command_oscillation']
    if tracking['exploratory_onboard_minus_sim'] is not None and steering['exploratory_onboard_minus_sim'] is not None:
        report += ('\n**Main observations:** onboard PF-estimated distance-to-raceline P95 was {:+.1f} cm relative to simulation; '
                   'upstream steering-command oscillation RMS differed by {:+.4f} rad. These are descriptive differences of '
                   'the matched intervals, not isolated platform effects.\n\n').format(
            tracking['exploratory_onboard_minus_sim']['p95']*100, steering['exploratory_onboard_minus_sim']['rms'])
    report += ('\nSmaller tracking distance means closer *believed* raceline following. Lower consistency disagreement means stronger PF/odometry agreement, not absolute accuracy. '
               'Lower oscillation means less short-scale variation around local trend; it does not prove better control. Fresh pose delivery and reported health quality answer different questions. '
               'Health fractions and Gym-versus-ERPM speed response are shown without numeric deltas because evidence equivalence is not established.\n\n')
    cross = result.get('sim_gt_crosscheck')
    if cross and cross.get('station_disagreement_m'):
        report += '**Independent simulator GT check:** PF/GT reference-station disagreement P50/P95/max = '+', '.join('{:.4g}'.format(cross['station_disagreement_m'][k]) for k in ('p50','p95','max'))+' m; coverage {:.1f}%. '.format(cross['coverage']*100)
        report += 'Absolute accuracy is retained in the [simulation section scorecard](sim_section/report.md). GT did not select the section or feed PF/control.\n\n'
    report += '## 2. Setup and section selection\n\n'
    report += '| Platform | Original evaluation offsets selected [s] | Duration [s] | Projection samples / coverage |\n| --- | --- | --- | --- |\n'
    for side in ('sim','onboard'):
        item=result['inputs'][side];sel=item['selected_interval']
        report += '| {} | {:.6f}–{:.6f} | {:.3f} | {} / {:.1f}% |\n'.format(side,sel['start_sec'],sel['end_sec'],sel['elapsed_sec'],item['section_projection_samples'],item['section_projection_coverage']*100)
    report += ('\nOnboard offsets above are relative to the reviewed 7.2–64.8 s raw-bag interval, not to bag start. '
               'Simulation uses the straight-start Tempo configuration, experimental 008_geometry and intended comparison profile 0.5–2.0 m/s (final cap ±2.0). '
               'The onboard historical deployed speed profile and configuration hashes remain unknown; recorded commands are the evidence. The experimental raceline is not production-validated.\n\n')
    for side in ('sim','onboard'):
        item=result['inputs'][side]
        report += '- {}: exclusions {}; {} samples outside selected interval; {} local non-increasing station samples omitted only from first-crossing station resampling. All interval samples, including health events, remain in metric analysis.\n'.format(side,item['projection_excluded_reasons'] or 'none',item['outside_selected_time_samples'],item['selected_interval']['non_increasing_station_samples'])
    report += '\n| Waveform station-grid coverage | Simulation | Onboard |\n| --- | --- | --- |\n'
    for signal in ('pf_position_m', 'pf_heading_rad', 'autonomy_speed_mps', 'autonomy_steering_rad', 'forward_speed_mps', 'consistency_position_m', 'consistency_yaw_rad'):
        report += '| {} | {:.2f}% | {:.2f}% |\n'.format(signal.replace('_', ' '), result['inputs']['sim']['grid_coverage'][signal]*100, result['inputs']['onboard']['grid_coverage'][signal]*100)
    report += '\nMissing heading support and the 0.5 s consistency-window startup remain gaps in plots; no zero fill is used. Headline metric coverage is reported independently in the section scorecards.\n\n'
    full_health = result['inputs']['onboard'].get('full_input_health')
    if full_health:
        report += ('The entire reviewed onboard interval contains {:.2f}% DEGRADED and {:.2f}% INVALID reported health. '
                   'The selected earliest complete traversal was not chosen for health; later events remain visible in the full-input scorecard.\n\n').format(full_health['percent']['DEGRADED'], full_health['percent']['INVALID'])
    report += '| Observed selected-interval command ranges | Simulation | Onboard |\n| --- | --- | --- |\n'
    for role in ('autonomy_command_speed_mps', 'autonomy_command_steering_rad', 'final_drive_command_speed_mps'):
        values = [result['inputs'][side]['observed_command_ranges'].get(role) for side in ('sim','onboard')]
        report += '| {} | {} | {} |\n'.format(role.replace('_',' '), *['{:.4g} to {:.4g}'.format(v['min'],v['max']) if v else 'UNAVAILABLE_DATA' for v in values])
    report += '\nFinal command rows are observations at different command stages, not a numerical cross-platform comparison.\n'
    report += '\n## 3. Evidence and metric mechanics\n\n'
    report += ('Recorded PF map pose → bounded continuous projection onto ordered raceline → explicit lap unwrapping and first complete traversal → cropped metadata/bag copies → existing shared analyzer. '
               'Reference station is distance along the raceline; it is **not** the raw-odometry cumulative driven distance used by existing smoothness. '
               'The common {:.2f} m station grid is used only for waveform display. Original metric formulas, time/sample weighting, 0.5 s SE(2) consistency windows and smoothness definitions are reused unchanged. '.format(result['settings']['station_step_m'])+
               'Nearby branches unresolved by continuity are rejected; source gaps split traversals; interpolation cannot bridge missing station/time coverage. No healthy-interval filtering occurs.\n\n')
    report += ('Upstream commands use `/drive` on both platforms. Onboard final `/ackermann_cmd` stays distinct in the section scorecard. Forward response is Gym body speed versus ERPM-derived odometry speed; neither steering command nor servo echo is physical steering feedback. '
               'Raw onboard gyro units/frame remain unqualified, so body-yaw metrics are unavailable. Onboard GT accuracy, task completion and collision absence are not invented.\n\n')
    report += '## 4. Trajectories and station-domain plots\n\n'
    report += spatial_note+'\n\n'
    for name in spatial_files:
        caption = ('Top-down comparison of the same selected interval: both PF trajectories, recorded raceline, and independent simulator GT. Equal axes preserve geometry. Start/end markers are sample endpoints, not inferred start-line events.' if name == 'trajectory_xy.png' else 'The captured simulation occupancy grid supplies circuit context, not independent onboard truth or a production safety corridor. Panels separate overlapping PF/GT traces. Arrows follow the ordered raceline and chronological pose samples; they are not measured velocity vectors. No data-driven translation/rotation was fitted to improve agreement.')
        report += '![{}](plots/{})\n\n{}\n\n'.format(name, name, caption)
    report += '### PF-estimated distance to raceline on the circuit map\n\n'
    for name in deviation_files:
        report += '![PF-estimated distance to raceline](plots/{})\n\n{}\n\n'.format(name, deviation_note)
    report += '### Reported PF quality at estimated vehicle positions\n\n'
    for name in health_files:
        report += '![Reported PF quality at vehicle positions](plots/{})\n\n{}\n\n'.format(name, health_note)
    for name, caption in [('tracking.png','PF-believed tracking relative to the same static raceline; larger values mean greater tracking distance, including intentional avoidance.'),('command_response.png','The same upstream autonomous command stage on both platforms, followed by differently sourced forward-speed response. Neither command is measured steering.'),('consistency.png','Unchanged body-relative PF/odometry disagreement; the first 0.5 s lacks a full in-section window. Agreement is correlated evidence.'),('elapsed_health.png','Elapsed time at each reference station and all reported health states. Health classifications are descriptive; deployed threshold equivalence is unknown.')]:
        report+='![{}](plots/{})\n\n{}\n\n'.format(name.replace('.png','').replace('_',' '),name,caption)
    report += '## 5. Conclusions and limitations\n\n'+''.join('- '+v+'\n' for v in result['limitations'])
    report += ('\nThe [strict provenance-qualified comparison](strict_comparison/report.md) is separate and uses unchanged gates. '
               'Zero certified deltas means controlled comparability is not established, **not** analyzer failure. '
               'Exploratory differences above are observations of these PF-aligned intervals; they do not isolate physical platform effects, prove equal deployed tuning, or certify simulator fidelity.\n\n')
    report += ('## 6. Reproduction, provenance and detailed evidence\n\n'
               '- [Canonical commands](../../../docs/ROBORACER_OPERATIONAL_COMMAND_REFERENCE.md#matched-reference-section-review) and [metric definitions](../../../docs/LOCALIZATION_SIMULATION.md#matched-reference-section-comparison).\n'
               '- [Simulation section](sim_section/report.md), [onboard section](onboard_section/report.md), [comparison JSON](comparison.json), [station waveforms](station_signals.json).\n'
               '- `comparison.json` preserves source metadata hashes, original identities, intervals, projection reasons, complete settings and grid coverage. Section metadata retains source/raw-file hashes and original source outcomes separately.\n')
    if result.get('sim_runtime_parameters'):
        report += '\nThe supplemental parameter probe is retained in `comparison.json` (`sim_runtime_parameters`) with its file hash. Inspect per-service errors: an unavailable post-run service does not qualify runtime settings. Where recorded, the runner’s own live checks are preserved separately in the source metadata (`smoke_result.effective_parameters`).\n'
    report += '\nSource revisions, dirty state, map/raceline and configuration snapshot hashes are retained under each input’s `source_provenance`; unknown onboard facts are not populated from simulation.\n'
    report += '\n| Side | Preserved source directory |\n| --- | --- |\n'
    for side in ('sim','onboard'):
        report+='| {} | `{}` |\n'.format(side,result['inputs'][side]['source_directory'])
    report += '\n## 7. Software and evidence validation\n\n| Check | Result / evidence |\n| --- | --- |\n'
    report += '| Source simulation smoke / recording | {} / {} |\n'.format(result['inputs']['sim'].get('source_smoke_result','UNKNOWN'),result['inputs']['sim'].get('source_recording_status','UNKNOWN'))
    report += '| Simulation visualization mode | {} (visualization is not measurement evidence) |\n'.format(result['inputs']['sim'].get('source_visualization_mode','UNKNOWN'))
    report += '| Cropped section analysis | sim {}; onboard {} |\n'.format(metrics[0]['analysis_status'],metrics[1]['analysis_status'])
    report += '| Selected projection coverage | sim {:.1f}%; onboard {:.1f}% |\n'.format(result['inputs']['sim']['section_projection_coverage']*100,result['inputs']['onboard']['section_projection_coverage']*100)
    report += '| Strict certified differences | {}; unknown provenance remains gated |\n\n'.format(result['strict_certified_deltas_available'])
    report += 'These checks qualify software and recorded evidence. They do not qualify physical performance, healthy operation throughout a session, historical deployed settings, or simulator fidelity. Exact unit-test/repetition results belong to the task handoff; a single successful invocation does not claim repeated testing.\n'
    headings=re.findall(r'^## (.+)$',report,re.M)
    toc='**Table of contents**\n\n'+''.join('- [{}](#{})\n'.format(h,re.sub(r'[^\w\- ]','',h.lower()).replace(' ','-')) for h in headings)+'\n'
    p=report.index('## 2.');report=report[:p]+toc+report[p:]
    (output/'report.md').write_text(report)
    rebase_generated_links(output/'report.md', Path(result['inputs']['sim']['source_directory']))
    heading_report_plots(output,result)
    import evaluation_control_plots
    evaluation_control_plots.render_pair(output,result)
    import evaluation_particle_spread
    evaluation_particle_spread.render_pair(output,result)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('simulation_run',type=Path);parser.add_argument('onboard_run',type=Path)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--section-start-m',type=float,default=0.)
    parser.add_argument('--section-length-m',type=float,default=None)
    parser.add_argument('--matching-config',type=Path)
    args=parser.parse_args()
    try:
        overrides=yaml.safe_load(args.matching_config.read_text()) if args.matching_config else None
        result=compare_sections(args.simulation_run.resolve(),args.onboard_run.resolve(),args.output_dir.resolve(),args.section_start_m,args.section_length_m,overrides)
    except (OSError,ValueError,KeyError,yaml.YAMLError) as error:
        parser.exit(1,'MATCHED SECTION FAIL: '+str(error)+'\n')
    print('MATCHED SECTION EXPLORATORY PASS: '+str(args.output_dir.resolve()/'report.md'))
    return 0

if __name__=='__main__':
    raise SystemExit(main())
