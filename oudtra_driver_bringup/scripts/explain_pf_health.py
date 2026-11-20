#!/usr/bin/env python3
"""Optional PF-specific report addendum; offline reference-profile replay, not tuning."""
import argparse
import ast
import hashlib
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

FIELDS = ('static_match_ratio','static_scan_error_median','obstacle_short_ratio','xy_spread',
          'longitudinal_residual','lateral_residual','yaw_residual','n_eff_ratio')
STATES = {1:'GOOD',2:'DEGRADED',3:'INVALID'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def classifier(source):
    """Execute the inspected monitor's unchanged suffix, not a second classifier."""
    tree=ast.parse(Path(source).read_text())
    fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='evaluate_pf_health')
    def assign(node,name):
        return isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in node.targets)
    start=next(i for i,node in enumerate(fn.body) if assign(node,'scan_good_evidence'))
    end=next(i for i in range(start+1,len(fn.body)) if assign(fn.body[i],'metrics'))
    return compile(ast.Module(body=fn.body[start:end],type_ignores=[]),str(source),'exec')


def new_state(profile):
    state=SimpleNamespace(**{k.upper():v for k,v in profile.items()})
    state.PF_STATUS_GOOD=1;state.PF_STATUS_DEGRADED=2;state.PF_STATUS_INVALID=3
    state.health_status=1;state.good_count=0;state.degraded_count=0
    state.invalid_count=0;state.degraded_status_count=0
    return state


def replay(rows, profile, code, max_gap=.5):
    """Unknown prehistory/gaps are not guessed: re-anchor on a healthy streak."""
    state=new_state(profile);anchored=False;streak=0;result=[]
    required=max(profile['recover_good_count'],profile['manual_reset_grace_updates'])+1
    for i,row in enumerate(rows):
        if len(row)!=10 or not np.all(np.isfinite(row)) or row[1] not in STATES:
            raise ValueError('Expected finite timestamp plus nine PF health fields')
        if i and row[0]-rows[i-1][0]<0:raise ValueError('Reversed receive clock')
        if i and row[0]-rows[i-1][0]>max_gap:
            anchored=False;streak=0;state=new_state(profile)
        local=dict(zip(FIELDS,row[2:]));local.update(self=state,scan_has_static_candidates=True)
        before=state.degraded_status_count;prior=state.health_status
        exec(code,{},local)
        trusted=anchored
        if not anchored:
            streak=streak+1 if local['good_evidence'] and row[1]==1 else 0
            if streak>=required:
                anchored=True;state=new_state(profile);state.good_count=streak
        result.append(dict(time_sec=float(row[0]),reported=int(row[1]),predicted=int(state.health_status) if trusted else None,
                           anchored=trusted,good_evidence=bool(local['good_evidence']),
                           hard_invalid_evidence=bool(local['invalid_evidence']),
                           scan_good=bool(local['scan_good_evidence']),
                           obstacle_override=bool(local['obstacle_good_evidence']),
                           counter_before=before,counter_after=state.degraded_status_count,
                           watchdog=bool(trusted and prior==2 and before+1>=profile['max_degraded_count'] and
                                         state.health_status==3 and not local['invalid_evidence'])))
    return result


# These colors describe explanations, not a second PF status scale.
MODE_STYLE = {
    'match_only': ('#c98b00', 'o', 'Too few matching rays'),
    'range_only': ('#dc6d27', '^', 'Range error too large'),
    'both': ('#8f4a20', 's', 'Both scan checks fail'),
    'recovery': ('#2475b0', 'D', 'Good evidence; recovery pending'),
    'watchdog': ('#d62728', 'X', 'INVALID after prolonged DEGRADED'),
    'unknown': ('#7550a3', '+', 'No fresh health evidence'),
}


def associate_explanations(pose, rows, trace, profile, max_age=.5):
    """Attach preceding diagnostic evidence only; never infer locations in gaps.

    Scan reasons and state persistence are separate: an INVALID point may also
    have poor scan evidence. UNKNOWN is missing evidence, not a PF status code.
    """
    invalid_origin = None
    origins = []
    for i, item in enumerate(trace):
        if i and rows[i, 0]-rows[i-1, 0] > max_age:
            invalid_origin = None
        if item['reported'] != 3:
            invalid_origin = None
        elif i == 0 or trace[i-1]['reported'] != 3:
            invalid_origin = ('watchdog' if item['watchdog'] and item['anchored']
                              and item['predicted'] == item['reported'] else None)
        origins.append(invalid_origin)
    samples = []
    for row in pose:
        index = int(np.searchsorted(rows[:, 0], row[4], side='right')-1)
        sample = dict(source_sec=float(row[0]), receive_sec=float(row[4]),
                      x_m=float(row[1]), y_m=float(row[2]),
                      scan_reason=None, mechanism=None, status='UNKNOWN', health_index=None)
        if index < 0 or row[4]-rows[index, 0] > max_age:
            sample['mechanism'] = 'unknown'
        else:
            health = rows[index]; item = trace[index]
            sample.update(status=STATES[item['reported']], health_index=index,
                          health_age_sec=float(row[4]-health[0]))
            if item['reported'] != 1:
                low_match = health[2] < profile['good_match_ratio']
                high_error = health[3] > profile['good_scan_error_m']
                sample['scan_reason'] = ('both' if low_match and high_error else
                                         'match_only' if low_match else
                                         'range_only' if high_error else None)
                if item['anchored'] and item['predicted'] == item['reported']:
                    if item['reported'] == 3 and origins[index] == 'watchdog':
                        sample['mechanism'] = 'watchdog'
                    elif item['good_evidence']:
                        sample['mechanism'] = 'recovery'
                    elif item['hard_invalid_evidence']:
                        sample['mechanism'] = 'hard_evidence'
                else:
                    sample['mechanism'] = 'unqualified_replay'
            sample['good_evidence_now'] = bool(item['good_evidence'])
        samples.append(sample)
    return samples


def render_failure_modes(root, comparison, pose, data, rows, trace, profile, duration, evidence):
    """Separate map panels for observed scan checks and qualified state history."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.transforms import Affine2D
    import analyze_localization_run as A
    import compare_matched_sections as M

    samples = associate_explanations(pose, rows, trace, profile)
    unknown = A.dropouts([(float(row[0]), float(row[0])+.5) for row in rows], duration)
    reference = data['reference_path']
    fig, axes = plt.subplots(2, 2, figsize=(14, 11))
    points = np.vstack((reference, pose[:, 1:3]))
    low, high = points.min(axis=0)-1., points.max(axis=0)+1.
    sim_meta = yaml.safe_load((comparison/'sim_section/metadata.yaml').read_text())
    configs = sim_meta.get('configs', {})
    background = None
    if all(k in configs for k in ('map_yaml', 'map_image')):
        paths = [comparison/'sim_section'/configs[k]['copy'] for k in ('map_yaml', 'map_image')]
        if all(path.is_file() and sha(path) == configs[k]['sha256']
               for path, k in zip(paths, ('map_yaml', 'map_image'))):
            background = M.occupancy_background(*paths)
    for ax in axes.flat:
        if background is not None:
            shade, resolution, origin = background
            transform = Affine2D().rotate(origin[2]).translate(origin[0], origin[1])+ax.transData
            ax.imshow(shade, cmap='gray', vmin=0, vmax=1, origin='upper', interpolation='nearest',
                      extent=(0, shade.shape[1]*resolution, 0, shade.shape[0]*resolution),
                      transform=transform, zorder=0)
        ax.scatter(pose[:, 1], pose[:, 2], c='#b5b5b5', s=4, alpha=.35, zorder=1)
        ax.plot(reference[:, 0], reference[:, 1], 'k--', lw=1., zorder=2)
        for start, end in M.direction_segments(reference, (.18, .51, .84)):
            ax.annotate('', xy=end, xytext=start,
                        arrowprops=dict(arrowstyle='-|>', color='black', lw=1.2), zorder=7)
        ax.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), xlabel='Map x [m]', ylabel='Map y [m]')
        ax.set_aspect('equal', adjustable='box'); ax.grid(alpha=.12)

    def mark(ax, mode, field):
        color, marker, label = MODE_STYLE[mode]
        chosen = [item for item in samples if item[field] == mode]
        ax.scatter([item['x_m'] for item in chosen], [item['y_m'] for item in chosen],
                   c=color, marker=marker, s=30 if mode != 'watchdog' else 48,
                   label='{} ({} {})'.format(label, len(chosen), 'pose' if len(chosen)==1 else 'poses'), zorder=4)
        return len(chosen)

    counts = {}
    for mode in ('match_only', 'range_only', 'both'):
        counts[mode] = mark(axes[0, 0], mode, 'scan_reason')
    counts['recovery'] = mark(axes[0, 1], 'recovery', 'mechanism')
    counts['watchdog'] = mark(axes[1, 0], 'watchdog', 'mechanism')
    counts['unknown'] = mark(axes[1, 1], 'unknown', 'mechanism')
    axes[0, 0].set_title('1. Scan disagreement at non-GOOD positions')
    axes[0, 1].set_title('2. Evidence improved; state still recovering')
    axes[1, 0].set_title('3. INVALID episodes after the DEGRADED timeout')
    axes[1, 1].set_title('4. UNKNOWN: no fresh health to classify')
    events = [event for event in evidence['transitions'] if event['reported'] == 3]
    for number, event in enumerate(events, 1):
        loc = event['approximate_pose']
        if loc:
            axes[1, 0].annotate('I{}: {:.2f}s'.format(number, event['time_sec']),
                                xy=(loc['x_m'], loc['y_m']), xytext=(.50, .34+.07*(number-1)),
                                textcoords='axes fraction', fontsize=10, color='#a51a1a',
                                bbox=dict(facecolor='white', alpha=.9, edgecolor='none'),
                                arrowprops=dict(arrowstyle='-', color='#a51a1a'), zorder=8)
    missing_sec = sum(end-start for start, end in unknown)
    axes[1, 1].text(.03, .03,
                   'Missing fresh health: {:.3f} s in {} interval(s).\n'.format(missing_sec, len(unknown))+
                   ('No recorded position inside the missing interval:\nno UNKNOWN location is invented.'
                    if unknown and not counts['unknown'] else
                    'Purple marks only positions actually recorded\nwithout a fresh preceding health message.'),
                   transform=axes[1, 1].transAxes, fontsize=10,
                   bbox=dict(facecolor='white', alpha=.95, edgecolor='#bbbbbb'), zorder=9)
    for ax in axes.flat:
        ax.legend(loc='upper right', fontsize=11, framealpha=.95)
    fig.suptitle('Why the reported PF quality changed — full onboard evaluation', fontsize=15)
    note = ('Background: captured simulation occupancy map; historical onboard map identity unverified.'
            if background is not None else 'No qualified captured map: map background omitted.')
    fig.text(.5, .02, 'Gray: all recorded estimated positions. Black dashed: raceline and driving direction.\n'
             'Panels are separate explanations, not mutually exclusive classes: one position can appear in panels 1 and 3.\n'
             'Counters/thresholds use the inspected reference settings; colors do not establish physical localization failure.\n'+note,
             ha='center', fontsize=10)
    fig.tight_layout(rect=(0, .10, 1, .95))
    fig.savefig(root/'plots/pf_health_failure_modes.png', dpi=160); plt.close(fig)
    result = dict(method='causal_health_diagnostic_modes_v1', counts=counts, samples=samples,
                  max_health_age_sec=.5, unknown_intervals_sec=unknown,
                  unknown_duration_sec=missing_sec, map_context=note,
                  source_sha256=evidence['source_sha256'], profile_sha256=evidence['profile_sha256'],
                  qualification='Reference-profile mechanisms and recorded scan checks; not physical root-cause labels.')
    (root/'pf_health_failure_modes.json').write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False)+'\n')
    return result


def descriptive_correlation(x, y):
    """Paired Pearson r is descriptive, not a significance/causality test."""
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    valid = np.isfinite(x) & np.isfinite(y)
    x, y = x[valid], y[valid]
    if len(x) < 3 or np.ptp(x) == 0 or np.ptp(y) == 0:
        return dict(value=None, samples=len(x), reason='Insufficient pairs or constant signal')
    return dict(value=float(np.corrcoef(x, y)[0, 1]), samples=len(x), reason=None)


def spread_quality_evidence(rows, samples, reference_errors, duration, profile):
    """Same-message status/spread; separately qualified causal pose association."""
    reviewed = rows[(rows[:, 0] >= 0) & (rows[:, 0] < duration)]
    states = {}
    for code, name in STATES.items():
        values = reviewed[reviewed[:, 1] == code, 5]
        states[name] = dict(samples=len(values), p50_m=float(np.median(values)) if len(values) else None,
                            p95_m=float(np.percentile(values, 95)) if len(values) else None,
                            max_m=float(np.max(values)) if len(values) else None,
                            above_good_bound=int(sum(values > profile['max_good_xy_spread_m'])),
                            above_invalid_bound=int(sum(values > profile['max_degraded_xy_spread_m'])))
    paired = []
    for sample, error in zip(samples, reference_errors):
        index = sample['health_index']
        item = dict(sample, pf_reference_deviation_m=float(error), xy_spread_m=None)
        if index is not None:
            item['xy_spread_m'] = float(rows[index, 5])
        paired.append(item)
    supported = [item for item in paired if item['xy_spread_m'] is not None]
    return dict(method='recorded_weighted_xy_spread_quality_v1', states=states,
                good_spread_bound_m=profile['max_good_xy_spread_m'],
                invalid_spread_bound_m=profile['max_degraded_xy_spread_m'],
                spread_vs_match=descriptive_correlation(reviewed[:, 5], reviewed[:, 2]),
                spread_vs_range_error=descriptive_correlation(reviewed[:, 5], reviewed[:, 3]),
                spread_vs_pf_reference_deviation=descriptive_correlation(
                    [item['xy_spread_m'] for item in supported],
                    [item['pf_reference_deviation_m'] for item in supported]),
                samples=paired, unknown_pose_samples=sum(item['xy_spread_m'] is None for item in paired),
                qualification='Particle-cloud dispersion, not absolute pose error, calibrated covariance or a confidence radius. Pearson r is sample-weighted and descriptive; updates are autocorrelated.')


def render_spread_quality(root, comparison, pose, data, rows, trace, profile, duration):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize
    from matplotlib.cm import ScalarMappable
    from matplotlib.lines import Line2D
    from matplotlib.transforms import Affine2D
    import analyze_localization_run as A
    import compare_matched_sections as M

    associated = associate_explanations(pose, rows, trace, profile)
    errors, _ = A.reference_errors(pose, data['reference_path'])
    evidence = spread_quality_evidence(rows, associated, errors, duration, profile)
    samples = evidence['samples']; reference = data['reference_path']
    configs = yaml.safe_load((comparison/'sim_section/metadata.yaml').read_text()).get('configs', {})
    background = None
    if all(k in configs for k in ('map_yaml', 'map_image')):
        paths = [comparison/'sim_section'/configs[k]['copy'] for k in ('map_yaml', 'map_image')]
        if all(path.is_file() and sha(path) == configs[k]['sha256']
               for path, k in zip(paths, ('map_yaml', 'map_image'))):
            background = M.occupancy_background(*paths)
    points = np.vstack((reference, pose[:, 1:3])); low, high = points.min(axis=0)-1., points.max(axis=0)+1.
    norm = Normalize(0., profile['max_good_xy_spread_m']); cmap = plt.get_cmap('viridis')
    fig, axes = plt.subplots(1, 3, figsize=(18, 8))
    for ax in axes:
        if background is not None:
            shade, resolution, origin = background
            transform = Affine2D().rotate(origin[2]).translate(origin[0], origin[1])+ax.transData
            ax.imshow(shade, cmap='gray', vmin=0, vmax=1, origin='upper', interpolation='nearest',
                      extent=(0, shade.shape[1]*resolution, 0, shade.shape[0]*resolution), transform=transform)
        ax.plot(reference[:, 0], reference[:, 1], 'k--', lw=1.)
        for start, end in M.direction_segments(reference, (.18, .51, .84)):
            ax.annotate('', xy=end, xytext=start, arrowprops=dict(arrowstyle='-|>', color='black', lw=1.2), zorder=8)
        ax.set(xlabel='Map x [m]', ylabel='Map y [m]', xlim=(low[0], high[0]), ylim=(low[1], high[1]))
        ax.set_aspect('equal', adjustable='box'); ax.grid(alpha=.1)
    def reason(item):
        if item['status'] == 'UNKNOWN': return 'unknown'
        if item['status'] == 'GOOD': return 'good'
        if item['mechanism'] in ('watchdog', 'recovery'): return item['mechanism']
        return item['scan_reason'] or 'other'
    state_markers = [('GOOD','o','GOOD'),('DEGRADED','^','DEGRADED'),('INVALID','X','INVALID'),('UNKNOWN','+','UNKNOWN / no spread')]
    reason_markers = [('good','o','GOOD'),('match_only','^','Few matching rays'),('range_only','>','High range error'),
                      ('both','s','Both scan checks'),('recovery','D','Recovery pending'),('watchdog','X','INVALID after timeout'),
                      ('other','p','Other / unqualified'),('unknown','+','Missing health')]
    valid=[item for item in samples if item['xy_spread_m'] is not None]
    axes[0].scatter([item['x_m'] for item in valid], [item['y_m'] for item in valid],
                    c=[item['xy_spread_m'] for item in valid], norm=norm, cmap=cmap, s=14, zorder=4)
    axes[0].set_title('1. Particle spread only [m]')
    for ax, specs, field in zip(axes[1:], (state_markers, reason_markers), ('status','reason')):
        handles=[]
        for key, marker, label in specs:
            chosen=[item for item in samples if (item['status'] if field=='status' else reason(item)) == key]
            if not chosen and field=='reason': continue
            color = ({'GOOD':'#208443','DEGRADED':'#e49b00','INVALID':'#d62728','UNKNOWN':'#7550a3'}.get(key,'#777777')
                     if field=='status' else MODE_STYLE.get(key,('#208443' if key=='good' else '#777777',))[0])
            ax.scatter([item['x_m'] for item in chosen], [item['y_m'] for item in chosen],
                       color=color, marker=marker, s=35 if marker!='o' else 10, zorder=4+int(key in ('INVALID','watchdog')))
            handles.append(Line2D([],[],marker=marker,ls='',color=color,label='{} ({})'.format(label,len(chosen)),markersize=7))
        ax.legend(handles=handles,loc='upper right',fontsize=11,framealpha=.95)
    axes[1].set_title('2. Reported quality only')
    axes[2].set_title('3. Warning explanation only')
    fig.subplots_adjust(left=.04,right=.94,top=.85,bottom=.22,wspace=.16)
    color_ax=fig.add_axes([.075,.16,.23,.018])
    fig.colorbar(ScalarMappable(norm=norm,cmap=cmap),cax=color_ax,orientation='horizontal',
                 label='Recorded weighted XY particle spread [m]')
    fig.suptitle('Same estimated positions and map axes — read one quantity per panel',fontsize=16)
    fig.text(.62,.06,'Left: spread magnitude only; center: GOOD / DEGRADED / INVALID / UNKNOWN.\n'
             'Right: scan checks / recovery / timeout / missing health, using the existing explanation colors.\n'
             'No spread is filled into UNKNOWN. Markers are not uncertainty radii.\n'
             'Captured simulation map provides context; actual onboard error/map identity remain unknown.',ha='center',fontsize=10)
    fig.savefig(root/'plots/pf_spread_quality_map.png',dpi=160);plt.close(fig)

    # Full-size single-quantity views are the human report defaults; retain the overview as a link.
    for kind,filename,title in [('spread','pf_particle_spread_map.png','Particle spread only'),
                                 ('status','pf_quality_only_map.png','PF-reported quality only'),
                                 ('reason','pf_warning_reason_map.png','Warning explanation only')]:
        fig,ax=plt.subplots(figsize=(10,9))
        if background is not None:
            shade,resolution,origin=background
            transform=Affine2D().rotate(origin[2]).translate(origin[0],origin[1])+ax.transData
            ax.imshow(shade,cmap='gray',vmin=0,vmax=1,origin='upper',interpolation='nearest',
                      extent=(0,shade.shape[1]*resolution,0,shade.shape[0]*resolution),transform=transform)
        ax.plot(reference[:,0],reference[:,1],'k--',lw=1.2)
        for start,end in M.direction_segments(reference,(.18,.51,.84)):
            ax.annotate('',xy=end,xytext=start,arrowprops=dict(arrowstyle='-|>',color='black',lw=1.3),zorder=8)
        if kind=='spread':
            valid=[item for item in samples if item['xy_spread_m'] is not None]
            sc=ax.scatter([item['x_m'] for item in valid],[item['y_m'] for item in valid],
                          c=[item['xy_spread_m'] for item in valid],norm=norm,cmap=cmap,s=18,zorder=4)
            fig.colorbar(sc,ax=ax,label='Recorded weighted XY particle spread [m]',shrink=.8)
        else:
            specs=state_markers if kind=='status' else reason_markers
            for key,marker,label in specs:
                chosen=[item for item in samples if (item['status'] if kind=='status' else reason(item))==key]
                if not chosen:continue
                color=({'GOOD':'#208443','DEGRADED':'#e49b00','INVALID':'#d62728','UNKNOWN':'#7550a3'}.get(key,'#777777')
                       if kind=='status' else MODE_STYLE.get(key,('#208443' if key=='good' else '#777777',))[0])
                ax.scatter([item['x_m'] for item in chosen],[item['y_m'] for item in chosen],color=color,
                           marker=marker,s=35 if marker!='o' else 12,label='{} ({})'.format(label,len(chosen)),zorder=5)
            ax.legend(loc='upper center',bbox_to_anchor=(.5,-.13),ncol=2,fontsize=12)
        ax.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),xlabel='Map x [m]',ylabel='Map y [m]',title=title)
        ax.set_aspect('equal',adjustable='box');ax.tick_params(labelsize=12);ax.grid(alpha=.12)
        fig.text(.5,.015,'Same recorded PF positions and axis limits. Circuit context is the captured simulation map.\n'
                 'UNKNOWN has no fresh spread value; no confidence radius or true onboard error is implied.',ha='center',fontsize=11)
        fig.tight_layout(rect=(0,.10,1,1));fig.savefig(root/'plots'/filename,dpi=160,bbox_inches='tight',pad_inches=.2);plt.close(fig)

    reviewed=rows[(rows[:,0]>=0)&(rows[:,0]<duration)]
    fig,axes=plt.subplots(2,2,figsize=(13,11))
    color={'GOOD':'#208443','DEGRADED':'#e49b00','INVALID':'#d62728'}
    for code,name in STATES.items():
        group=reviewed[reviewed[:,1]==code]; i=code-1
        if len(group):
            box=axes[0,0].boxplot(group[:,5],positions=[i],widths=.5,patch_artist=True,showfliers=False)
            box['boxes'][0].set(facecolor=color[name],alpha=.5)
            axes[0,1].scatter(group[:,5],group[:,2],s=7,alpha=.4,c=color[name],label=name)
            axes[1,1].scatter(group[:,0],group[:,5],s=7,alpha=.6,c=color[name],label=name)
        pairs=[item for item in samples if item['status']==name and item['xy_spread_m'] is not None]
        axes[1,0].scatter([item['xy_spread_m'] for item in pairs], [item['pf_reference_deviation_m'] for item in pairs],s=7,alpha=.4,c=color[name],label=name)
    axes[0,0].set(xticks=[0,1,2],xticklabels=['GOOD','DEGRADED','INVALID'],ylabel='XY spread [m]',title='Do non-GOOD states have a wider cloud?')
    axes[0,1].axhline(profile['good_match_ratio'],ls='--',color='black',lw=.8,label='Reference GOOD match bound')
    axes[0,1].set(xlabel='XY spread [m]',ylabel='Recorded static match fraction',title='Cloud width versus scan/map agreement')
    axes[1,0].set(xlabel='Causally associated XY spread [m]',ylabel='PF-estimated distance to raceline [m]',title='Spread versus believed tracking (not truth error)')
    axes[1,1].set(xlabel='Evaluation receive time [s]',ylabel='XY spread [m]',title='Spread over time; colors show reported state')
    gaps=A.dropouts([(float(row[0]),float(row[0])+.5) for row in rows],duration)
    for start,end in gaps:axes[1,1].axvspan(start,end,color='#777777',alpha=.2)
    for ax in axes.flat:ax.grid(alpha=.15)
    for ax in (axes[0,1],axes[1,0],axes[1,1]):ax.legend(fontsize=11)
    fig.suptitle('Particle spread and reported quality — distributions, relationships and timing')
    fig.text(.5,.015,'Box: middle 50%, line: median, whiskers: 1.5 IQR rule; outliers omitted only from the box display, not analysis.\n'
             'UNKNOWN has no fresh spread value. Gray time bands retain evidence gaps. Scatter/box samples are not independent trials.',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.07,1,.95));fig.savefig(root/'plots/pf_spread_quality_relationships.png',dpi=160);plt.close(fig)
    evidence['report_tool_sha256']=sha(Path(__file__))
    evidence['map_context']='Hash-verified captured simulation map' if background is not None else 'Map omitted: missing/mismatched snapshot'
    (root/'pf_spread_quality.json').write_text(json.dumps(evidence,indent=2,sort_keys=True,allow_nan=False)+'\n')
    return evidence


def spread_quality_fragment(evidence):
    states=evidence['states']
    non_good=sum(states[name]['samples'] for name in ('DEGRADED','INVALID'))
    above_good=sum(item['above_good_bound'] for item in states.values())
    highest=max(item['max_m'] for item in states.values() if item['max_m'] is not None)
    text='<a id="particle-spread-reported-quality-and-warning-reasons-together"></a>\n\n#### Particle spread, quality and reasons side by side\n\n'
    text+=('**Three different questions:** particle spread asks “how tightly are the PF particles clustered?”; '
           'reported quality asks “do the scan/map and motion checks remain acceptable over successive updates?”; '
           'distance to raceline asks “where does PF believe the vehicle is relative to the intended path?” '
           'None is an independent measurement of actual onboard position error.\n\n')
    text+='![Particle spread, full-size view](plots/pf_particle_spread_map.png)\n\n![PF quality, full-size view](plots/pf_quality_only_map.png)\n\n![Warning reasons, full-size view](plots/pf_warning_reason_map.png)\n\n[Optional side-by-side overview](plots/pf_spread_quality_map.png). Full-size views above keep legends readable.\n\n'
    text+=('**Read the full-size maps in order:** all three use identical map limits and the same estimated positions. '
           '**First full-size map:** color means particle spread only, using the same 0–0.25 m scale across sessions. '
           '**Second full-size map:** categorical colors show reported quality. **Third full-size map:** categorical colors show the warning explanation, '
           'with timeout/recovery taking precedence over overlapping scan failures. '
           'No single marker combines continuous spread and categorical quality. UNKNOWN has no fresh spread value; no uncertainty radius is drawn.\n\n')
    text+=('**What the log shows in this session:** maximum recorded spread is **{:.4f} m**. '
           '**{}** health messages exceed the reference GOOD spread bound of {:.2f} m, '
           'even though **{}** messages report DEGRADED or INVALID. ').format(highest,above_good,evidence['good_spread_bound_m'],non_good)
    if above_good==0:
        text+='Thus the warnings did **not** require a wide particle cloud: the spread gate stayed satisfied while scan evidence and warning history drove the state. '
    text+='The following table uses spread and quality from the **same health message**, so it does not depend on the approximate pose-to-health join.\n\n'
    text+='| Reported state | Health messages | Spread median [m] | Spread P95 [m] | Spread maximum [m] | Above GOOD spread bound |\n| --- | --- | --- | --- | --- | --- |\n'
    for name,item in states.items():
        values=['{:.4f}'.format(item[key]) if item[key] is not None else 'Unavailable' for key in ('p50_m','p95_m','max_m')]
        text+='| {} | {} | {} | {} | {} | {} |\n'.format(name,item['samples'],*values,item['above_good_bound'])
    text+='| UNKNOWN | No fresh diagnostic | Unavailable | Unavailable | Unavailable | Not evaluated |\n\n'
    if all(states[name]['p50_m'] is not None for name in STATES.values()) and states['GOOD']['p50_m'] > max(states[name]['p50_m'] for name in ('DEGRADED','INVALID')):
        text+=('**Direct answer for this run:** the median cloud is actually **wider during GOOD** than during either DEGRADED or INVALID. '
               'So the logs do not support the simple expectation “larger particle spread means worse reported quality.” '
               'They show compact non-GOOD clouds, with scan agreement and state history explaining the alarms. '
               'This does not mean that increasing spread would improve localization.\n\n')
    text+='![Recorded spread relationships and quality over time](plots/pf_spread_quality_relationships.png)\n\n'
    text+=('**How to read these checks:** top left compares spread distributions by reported state; overlapping ranges mean cloud width alone cannot separate those states. '
           'Top right compares spread with scan-match fraction. Bottom left compares spread with **PF-believed** distance to raceline, not actual localization error. '
           'Bottom right keeps the time order and the missing-evidence intervals. No spread value is filled into UNKNOWN intervals.\n\n'
           '**Do they correlate?** There can be some statistical association without a common trigger. '
           'The descriptive coefficients below quantify the observed co-variation; they do not prove that increased spread caused a warning, '
           'or that compact particles imply correct localization. We do not treat the numeric state codes as a continuous uncertainty scale.\n\n')
    text+='| Paired quantities | Pearson r | Pairs | Interpretation |\n| --- | --- | --- | --- |\n'
    for key,label,meaning in [
            ('spread_vs_match','Spread / static scan-match fraction','Negative: wider cloud accompanies worse matching; positive: wider cloud accompanies better matching.'),
            ('spread_vs_range_error','Spread / median scan range error','Positive: wider cloud accompanies greater scan discrepancy.'),
            ('spread_vs_pf_reference_deviation','Spread / PF-estimated distance to raceline','Positive: wider cloud accompanies larger believed tracking deviation; this is not truth error.')]:
        item=evidence[key];value='{:+.3f}'.format(item['value']) if item['value'] is not None else 'Unavailable: '+item['reason']
        text+='| {} | {} | {} | {} |\n'.format(label,value,item['samples'],meaning)
    text+=('\nHere r describes linear association: near zero means little **linear** co-variation, not independence; its sign gives direction. '
           'These are sample-weighted, autocorrelated updates from one run, not independent statistical trials. '
           'No significance test, controlled comparison delta or causal claim is attached. The pose/raceline pair additionally uses the causal, at-most-0.5 s health association.\n\n')
    text+='| PF mechanism | Why spread and quality need not move together |\n| --- | --- |\n'
    text+='| Particle-cloud dispersion | The code logs `sqrt(weighted covariance xx + yy)` after the MCL update. It measures scatter around the particle cloud’s own center; it excludes yaw spread. A tightly clustered cloud can still be centered at a biased position. |\n'
    text+='| Relative particle weighting | MCL resamples, applies motion/sensor models, then normalizes weights to sum to one. Relative concentration can remain strong even when absolute scan/map agreement is inadequate. This is a mechanism that permits a compact but poor match, not proof of particle impoverishment here. |\n'
    text+='| Multiple health checks | GOOD requires adequate scan agreement **and** bounded particle spread **and** acceptable per-update motion (with a separate obstacle override). Low spread alone is insufficient. In these data, scan checks fail while spread/motion remain within bounds. |\n'
    text+='| Recovery and timeout memory | The reported state retains preceding evidence: five good updates are needed to recover, and prolonged DEGRADED escalates after 50 cycles. It need not track instantaneous spread or scan quality monotonically. |\n'
    text+='| Shared, limited evidence | Spread, scan checks and PF tracking all come from the same localization system. Agreement is not independent verification; missing health supplies no spread evidence at all. |\n\n'
    text+=('**Engineering conclusion:** use particle spread as an internal concentration diagnostic, reported quality as the monitor’s multi-check warning, '
           'and distance to raceline as believed tracking. Read them together; none replaces the others. '
           'The bags do not establish whether tight non-GOOD clouds corresponded to actual global position error. '
           'The plotted scalar spread is **not** a calibrated confidence radius/ellipse, and no physical failure cause is inferred. '
           'The reference thresholds reproduce these alarms, but historical deployed settings remain unverified. '
           '[Exact paired samples and descriptive statistics](pf_spread_quality.json).\n\n')
    return text


def make_addendum(root, source, profile_path, comparison):
    import analyze_localization_run as A
    import compare_matched_sections as M
    metadata=yaml.safe_load((root/'metadata.yaml').read_text())
    if metadata.get('platform')!='onboard':raise ValueError('This addendum targets recorded onboard evidence')
    roles=A.normalize_roles(metadata)
    records=A.read_bag(root,{'health':roles['localization_health']})['health']
    origin=metadata['interval']['start_wall_ns'];duration=(metadata['interval']['end_wall_ns']-origin)*1e-9
    rows=np.array([[(t-origin)*1e-9,*msg.data] for t,msg in records])
    profile=yaml.safe_load(profile_path.read_text())['particle_filter']['ros__parameters']
    expected={'good_match_ratio':.60,'degraded_match_ratio':.40,'good_scan_error_m':.10,
              'degraded_scan_error_m':.20,'max_good_xy_spread_m':.25,'max_degraded_xy_spread_m':.50,
              'max_good_lateral_residual_m':.10,'max_degraded_lateral_residual_m':.35,
              'max_good_yaw_residual_rad':.15,'max_degraded_yaw_residual_rad':.45,
              'lost_bad_count':8,'degraded_bad_count':5,'recover_good_count':5,
              'max_degraded_count':50,'manual_reset_grace_updates':10,
              'obstacle_short_error_m':.50,'obstacle_present_ratio':.03,'obstacle_good_match_ratio':.90}
    if any(profile.get(k)!=v for k,v in expected.items()):
        raise ValueError('Explanation wording targets the inspected onboard reference profile; supplied values differ. Do not reinterpret this as deployed tuning.')
    trace=replay(rows,profile,classifier(source));selected=(rows[:,0]>=0)&(rows[:,0]<duration)
    covered=[item for item in trace if item['anchored'] and 0<=item['time_sec']<duration]
    matches=sum(item['reported']==item['predicted'] for item in covered)
    full_covered=[item for item in trace if item['anchored']]
    metrics=rows[selected];transitions=[]
    meta,data,pose,_,_=M.load_run(root)
    projected,period=M.project_continuously(pose,data['reference_path'],M.defaults())
    received=np.array([row[4] for row in pose])
    for i,(row,item) in enumerate(zip(rows,trace)):
        if not selected[i] or i==0 or row[1]==rows[i-1,1]:continue
        event=dict(item,previous_reported=int(rows[i-1,1]),diagnostics=dict(zip(FIELDS,map(float,row[2:]))))
        j=int(np.searchsorted(received,row[0],side='right')-1)
        event['approximate_pose']=({'x_m':float(pose[j,1]),'y_m':float(pose[j,2]),'station_m':projected[j]['s']%period if projected[j]['s'] is not None else None,'age_sec':float(row[0]-received[j])} if j>=0 and row[0]-received[j]<=.5 else None)
        transitions.append(event)
    failures={}
    for code,name in STATES.items():
        samples=metrics[metrics[:,1]==code]
        failures[name]={'samples':len(samples),'match_below_good':int(sum(samples[:,2]<profile['good_match_ratio'])),
                        'median_error_above_good':int(sum(samples[:,3]>profile['good_scan_error_m'])),
                        'spread_above_good':int(sum(samples[:,5]>profile['max_good_xy_spread_m'])),
                        'lateral_above_good':int(sum(abs(samples[:,7])>profile['max_good_lateral_residual_m'])),
                        'yaw_above_good':int(sum(abs(samples[:,8])>profile['max_good_yaw_residual_rad']))}
    evidence=dict(qualification='REFERENCE_PROFILE_REPLAY_NOT_DEPLOYMENT_PROOF',
                  source_path=str(source),source_sha256=sha(source),profile_path=str(profile_path),profile_sha256=sha(profile_path),
                  profile={k:v for k,v in profile.items() if k.startswith(('good_','degraded_','max_good_','max_degraded_','lost_','recover_','manual_reset_','obstacle_'))},
                  full_bag_anchored=len(full_covered),full_bag_matches=sum(x['reported']==x['predicted'] for x in full_covered),
                  reviewed_anchored=len(covered),reviewed_matches=matches,reviewed_samples=len(metrics),
                  diagnostic_counts=failures,transitions=transitions,trace=trace,
                  note='Counters replayed from published diagnostics using pinned code/current onboard YAML; no deployed parameter identity is inferred. Unknown history/gaps require re-anchoring. This does not recover historical configuration hashes or prove physical localization failure.')
    invalid_entries=[e for e in transitions if e['reported']==3]
    if (matches!=len(covered) or not invalid_entries or not all(e['watchdog'] for e in invalid_entries) or
            any(r['hard_invalid_evidence'] for r in covered)):
        raise ValueError('Recorded sequence does not support the targeted scan/watchdog explanation; inspect diagnostics instead of claiming this cause')
    (root/'pf_health_explanation.json').write_text(json.dumps(evidence,indent=2,sort_keys=True,allow_nan=False)+'\n')
    render_timeline(root,rows,trace,profile,duration)
    import shutil
    shutil.copy2(comparison/'plots/pf_health_positions.png',root/'plots/pf_health_positions.png')
    source_link=os.path.relpath(source,root);profile_link=os.path.relpath(profile_path,root)
    modes = render_failure_modes(root, comparison, pose, data, rows, trace, profile, duration, evidence)
    frag = '#### Reported PF quality at estimated vehicle positions\n\n![Reported PF quality at estimated vehicle positions](plots/pf_health_positions.png)\n\n'
    frag += ('This first figure answers **what quality did PF report, and where did it believe the car was?** '
             'Green = GOOD, amber = DEGRADED, red = INVALID. Purple = UNKNOWN because the recording lacks fresh health. '
             'The lower time/station view separates repeated visits to the same place. These are estimated positions, not independently measured vehicle locations.\n\n')
    frag += '#### Why did PF become DEGRADED, INVALID or UNKNOWN?\n\n'
    frag += ('**Short answer:** the scan sometimes disagreed with what the map predicted. This first produced **DEGRADED**. '
             'When DEGRADED lasted too many update cycles without sufficient recovery, the monitor escalated to **INVALID**. '
             '**UNKNOWN is different:** the offline analyzer had no fresh health message; PF did not publish an UNKNOWN failure code.\n\n'
             '**How to read this explanation:** the recorded diagnostic values and reported states are real bag evidence. '
             'The threshold/counter explanation below is reproduced using the pinned PF code and its current onboard **reference** settings. '
             'The historical deployed settings were not recorded, so this is a supported mechanism explanation, not recovered deployment proof.\n\n')
    frag += '| Displayed state | Plain-language meaning | What explains it in this session |\n| --- | --- | --- |\n'
    frag += '| GOOD | The monitor accepts the current quality, or is tolerating a brief disturbance. | A few poor scans can occur before the state changes. Green does not independently prove the global pose is accurate. |\n'
    frag += '| DEGRADED | The scan/map agreement has been unsatisfactory for several updates, but the stronger spread/motion rejection has not fired. | Five consecutive updates without GOOD evidence change GOOD to DEGRADED. Here, scan disagreement is the failing evidence; particle spread and per-update motion checks remain within their GOOD bounds. |\n'
    frag += '| INVALID | The monitor escalates its warning; this is not a direct measurement that the car is lost. | All {} entries are reproduced by the **50-update DEGRADED timeout**. None is explained by excessive particle spread or motion residual. |\n'.format(len(invalid_entries))
    frag += '| UNKNOWN | We cannot determine the state from fresh recorded evidence. | {:.3f} s in {} missing/stale-health interval(s). A gap could be recording, communication or publication loss; these bags do not establish which. It is not proof that PF failed or the car stopped. |\n\n'.format(modes['unknown_duration_sec'],len(modes['unknown_intervals_sec']))
    frag += ('**What does “scan disagreement” mean?** At its inferred vehicle pose, PF predicts the laser distances to the static map. '
             'It compares those predictions with the measured scan, excluding obstacle-like rays much shorter than predicted. '
             'Under the reference settings, at least **60% of retained rays must match within 10 cm**, and the **median range error must be no more than 10 cm**. '
             'This checks scan-to-map agreement; it does **not** measure distance to the raceline.\n\n'
             '**Why can an improving scan still have a bad state?** Recovery requires **five consecutive GOOD-evidence updates**. '
             'One better scan is insufficient. Meanwhile, if the state reaches its 50th would-be DEGRADED cycle, the timeout changes it to INVALID. '
             'Thus an INVALID entry can coincide with a favorable current scan. These are counts of PF updates, not seconds.\n\n')
    frag += '| Explanation | What to look for | Recorded evidence in this session |\n| --- | --- | --- |\n'
    deg=failures['DEGRADED']
    frag += '| Too few matching rays | The match fraction drops below 0.60. | {}/{} DEGRADED health messages. |\n'.format(deg['match_below_good'],deg['samples'])
    frag += '| Range discrepancies too large | Median retained-ray error rises above 0.10 m. This can overlap the preceding condition. | {}/{} DEGRADED health messages. |\n'.format(deg['median_error_above_good'],deg['samples'])
    frag += '| Recovery still pending | Current evidence is GOOD, but the required streak has not completed. | {} recorded pose locations show this while still DEGRADED (blue in the map below). INVALID episodes remain red in their separate panel even if a scan improves. |\n'.format(modes['counts']['recovery'])
    frag += '| DEGRADED timeout | State stays DEGRADED through 49 published updates; the next would-be DEGRADED update escalates. | All {} INVALID entries reproduce counter **49 → 50**. |\n'.format(len(invalid_entries))
    frag += '| Strong spread/motion rejection | Excessive particle dispersion or implausible lateral/yaw motion can also cause INVALID in the code. | **Not observed here under the reference thresholds.** No reviewed message exceeds those hard bounds, or even the GOOD spread/motion bounds. |\n\n'
    frag += ('**What is still unexplained?** The bags establish scan disagreement, not its physical origin. '
             'Map inaccuracies, laser calibration, scan distortion, map-raycast approximation, reflective surfaces and changes in the scene remain possibilities—not diagnosed causes. '
             'There is no onboard ground truth here to determine the actual global pose error.\n\n')
    frag += '#### Where the different explanations occur on the track\n\n![Explained PF degradation and missing-evidence modes on the map](plots/pf_health_failure_modes.png)\n\n'
    frag += ('Read the four panels in order. **1:** amber = too few matching rays only; orange = excessive range error only; brown = both, at non-GOOD positions. '
             '**2:** blue = evidence improved while the DEGRADED state awaits recovery. '
             '**3:** red crosses = INVALID episodes following the replayed timeout; I1/I2/I3 label entry times and approximate positions from the latest preceding PF pose (at most 0.5 s old). '
             '**4:** purple = recorded positions lacking fresh preceding health. Missing-pose intervals have no invented location.\n\n'
             'Gray dots give the full estimated trajectory; black dashes/arrows give the raceline and driving direction. '
             'These panels explain **different aspects**: a position with poor scan evidence can appear in both panels 1 and 3. '
             'Repeated laps overlap; use the quality/time plot above to separate visits. The background is the hash-verified captured simulation map, '
             'not proof of the historically loaded onboard map or measured wall clearance.\n\n'
             'Legend counts are **pose samples**, while the table above counts **health messages**. PF publishes the pose before health, '
             'so a pose can inherit the preceding health cycle. All spatial associations use the most recent health at pose receipt, at most 0.5 s old; no future message is used. '
             '[Exact mapped explanations](pf_health_failure_modes.json).\n\n')
    spread_evidence = render_spread_quality(root, comparison, pose, data, rows, trace, profile, duration)
    frag += spread_quality_fragment(spread_evidence)
    frag += '#### When the warning built up and recovered\n\n![Recorded PF evidence and reference classifier replay](plots/pf_health_trigger_evidence.png)\n\n'
    frag += ('From top to bottom: scan-match fraction, median scan error, reconstructed DEGRADED dwell count, and reported/replayed state. '
             'The first two panels show the observed evidence. The third explains the timeout. The final panel checks whether that mechanism reproduces the reported states. '
             'Gray intervals lack fresh evidence; counters there are not guessed.\n\n')
    frag += '| INVALID entry | Time from evaluation start [s] | Approx. raceline station [m] | Why it turns red |\n| --- | --- | --- | --- |\n'
    for number,event in enumerate(invalid_entries,1):
        loc=event['approximate_pose'];station='{:.2f}'.format(loc['station_m']) if loc and loc['station_m'] is not None else 'Unavailable'
        scan_now='already favorable, but recovery incomplete' if event['good_evidence'] else 'still below GOOD evidence'
        frag += '| I{} | {:.3f} | {} | DEGRADED counter 49 → 50; current evidence {}. |\n'.format(number,event['time_sec'],station,scan_now)
    frag += '\n#### Technical evidence and limits\n\n'
    frag += ('The unchanged classifier reproduces **{}/{} anchored reviewed messages** and {}/{} anchored full-bag messages. '
             'After unknown initial history or gaps, replay becomes trusted only after {} consecutive reported-GOOD/GOOD-evidence messages. '
             'No PF console/counter logs were recorded: the counters are reconstructed from the nine diagnostic fields, not directly logged.\n\n').format(matches,len(covered),evidence['full_bag_matches'],len(full_covered),max(profile['recover_good_count'],profile['manual_reset_grace_updates'])+1)
    frag += ('Other inspected code paths: hard-invalid evidence is spread >0.50 m, absolute lateral residual >0.35 m or yaw residual >0.45 rad, '
             'feeding an invalid counter with threshold 8; intermediate evidence decays that counter and GOOD evidence clears it. '
             'GOOD spread/lateral/yaw bounds are 0.25 m / 0.10 m / 0.15 rad. '
             'The obstacle override needs short-ray fraction ≥0.03 and match fraction ≥0.90 with bounded spread/motion; it is inactive in these windows. '
             'Low effective particle count and longitudinal residual are diagnostic fields, not state triggers. '
             'The loose scan-bad test (<0.40 match or >0.20 m error) is not an independent INVALID gate.\n\n'
             'Manual initialization provides ten GOOD grace updates; global initialization sets INVALID. Missing map-raycast buffers preserve state with infinite diagnostic placeholders. '
             'The recorded finite vectors and replay do not support these as the reviewed alarm mechanism. No initial-pose messages were captured; earlier resets remain unknown. '
             'The monitor does not itself change particles, inferred pose, TF or pose publication. Controller response requires separate evidence.\n\n'
             'Monitor lateral/yaw residuals compare a **single PF update** with accumulated raw-odometry action; they are not the scorecard’s **0.5 s SE(2) consistency** measurement. '
             'Python defaults differ from onboard YAML, and simulation uses different scan/recovery settings (0.85 / 8 versus onboard reference 0.60 / 5). '
             'Replay agreement neither identifies the historical deployment uniquely nor authorizes certified cross-platform health deltas. '
             'No PF/control parameters, metric formulas or original data were changed.\n\n')
    frag += '[Pinned PF mechanism]({}) · [reference onboard YAML]({}) · [exact diagnostic/replay evidence](pf_health_explanation.json).\n\n'.format(source_link,profile_link)
    M.attach_report_fragment(root,'pf_health_explanation.md',frag,
                             '![PF health over the evaluation interval](plots/localization_health.png)')
    # The paired report links the explanation and repeats its map for comparison context.
    shutil.copy2(root/'plots/pf_health_failure_modes.png', comparison/'plots/pf_health_failure_modes.png')
    link = os.path.relpath(root/'report.md', comparison)+'#why-did-pf-become-degraded-invalid-or-unknown'
    summary = ('### Why the onboard PF quality changed\n\n'
               'Scan disagreement first leads to DEGRADED; prolonged DEGRADED can time out to INVALID. '
               'UNKNOWN instead means missing fresh recorded health. These are distinct explanations, not measured global pose errors. '
               '[Read the state-by-state explanation and session-specific evidence]('+link+').\n\n'
               '![Explained onboard PF quality changes on the map](plots/pf_health_failure_modes.png)\n\n'
               'Amber/orange/brown: scan checks failing at non-GOOD positions; blue: DEGRADED with improved evidence awaiting recovery; '
               'red: INVALID episodes after the replayed timeout; purple: recorded positions without fresh health. '
               'Panels can overlap in meaning. No missing position is invented. Source/configuration identity remains unverified historically.\n\n')
    summary += ('See also the [particle-spread/quality overlay and correlation analysis]('+os.path.relpath(root/'report.md', comparison)+'#particle-spread-quality-and-reasons-side-by-side).\n\n')
    M.attach_report_fragment(comparison, 'pf_health_failure_modes.md', summary,
                             '## 4. Trajectories and station-domain plots')
    return evidence


def render_timeline(root,rows,trace,p,duration):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,1,sharex=True,figsize=(10,9))
    axes[0].plot(rows[:,0],rows[:,2],lw=.8,label='Recorded static match ratio');axes[0].axhline(p['good_match_ratio'],color='green',ls='--',label='GOOD reference');axes[0].axhline(p['degraded_match_ratio'],color='red',ls=':',label='Loose scan-bad reference');axes[0].set(ylabel='Match fraction',ylim=(0,1));axes[0].legend(fontsize=11)
    axes[1].plot(rows[:,0],rows[:,3],lw=.8);axes[1].axhline(p['good_scan_error_m'],color='green',ls='--');axes[1].axhline(p['degraded_scan_error_m'],color='red',ls=':');axes[1].set(ylabel='Median ray error [m]')
    t=np.array([r['time_sec'] for r in trace]);counter=np.array([r['counter_after'] if r['anchored'] else np.nan for r in trace]);pred=np.array([r['predicted'] if r['predicted'] is not None else np.nan for r in trace])
    axes[2].plot(t,counter,lw=.9,label='Replayed DEGRADED-status dwell counter');axes[2].axhline(p['max_degraded_count'],color='red',ls='--',label='Reference watchdog');axes[2].set(ylabel='Update cycles');axes[2].legend(fontsize=11)
    # Unobserved intervals are shaded instead of implying continuous status evidence.
    import analyze_localization_run as A
    gaps=A.dropouts([(float(r[0]),float(r[0])+.5) for r in rows],duration)
    for i in range(1,len(rows)):
        if rows[i,0]-rows[i-1,0]>.5:pred[i]=np.nan
    axes[3].scatter(rows[:,0],rows[:,1],s=6,label='Reported');axes[3].plot(t,pred,lw=.8,color='purple',ls='--',label='Anchored reference replay');axes[3].set(yticks=[1,2,3],yticklabels=['GOOD','DEGRADED','INVALID'],xlabel='Receive time within reviewed evaluation [s]',ylabel='PF status');axes[3].legend(fontsize=11)
    for ax in axes:
        ax.set_xlim(0,duration);ax.grid(alpha=.15)
        for start,end in gaps:ax.axvspan(start,end,color='gray',alpha=.15)
    fig.suptitle('Reported PF diagnostics versus pinned reference-profile monitor (not deployment proof)')
    fig.tight_layout();fig.savefig(root/'plots/pf_health_trigger_evidence.png',dpi=150);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run_directory',type=Path);parser.add_argument('--pf-source',type=Path,required=True);parser.add_argument('--health-profile',type=Path,required=True);parser.add_argument('--comparison-dir',type=Path,required=True);args=parser.parse_args()
    result=make_addendum(args.run_directory.resolve(),args.pf_source.resolve(),args.health_profile.resolve(),args.comparison_dir.resolve())
    print('PF HEALTH EXPLANATION: {}/{} anchored reviewed statuses reproduced; reference-profile analysis only'.format(result['reviewed_matches'],result['reviewed_anchored']))

if __name__=='__main__':main()
