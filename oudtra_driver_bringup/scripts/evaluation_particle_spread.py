"""Report-only PF cloud spread, kept separate from truth error and tracking distance.

Adapter: existing /pf/health Float32MultiArray, status=data[0], XY spread=data[4].
No covariance is read from ROS Odometry (its legacy packing is not trustworthy).
"""
import json
from pathlib import Path
import numpy as np

STATES = {1: 'GOOD', 2: 'DEGRADED', 3: 'INVALID'}
COLORS = {'GOOD': '#208443', 'DEGRADED': '#e49b00', 'INVALID': '#d62728', 'UNKNOWN': '#7550a3'}


def summarize(rows, duration):
    """Sample statistics from the same health messages; never fill missing spread."""
    a = np.asarray(rows, dtype=float).reshape((-1, 3))
    selected = a[(a[:, 0] >= 0) & (a[:, 0] < duration)]
    valid = np.all(np.isfinite(selected), axis=1) & (selected[:, 2] >= 0) & np.isin(selected[:, 1], list(STATES))
    def stats(values):
        return dict(samples=len(values), **({k: float(v) for k, v in zip(('p50_m', 'p95_m', 'max_m'), (*np.percentile(values, [50, 95]), np.max(values)))} if len(values) else dict(p50_m=None, p95_m=None, max_m=None)))
    return dict(status='AVAILABLE' if np.any(valid) else 'UNAVAILABLE_DATA',
                definition='sqrt(weighted_var_x + weighted_var_y)', unit='m',
                evidence='Recorded /pf/health data[4]; particle dispersion, not calibrated error/confidence',
                overall=stats(selected[valid, 2]),
                by_quality={name: stats(selected[valid & (selected[:, 1] == code), 2]) for code, name in STATES.items()},
                invalid_message_count=int(sum(~valid)),
                samples=selected[valid].tolist())


def associate(rows, times, max_age=.5):
    """Causal, freshness-bounded association. Invalid/stale messages stay unknown."""
    a = np.asarray(rows, dtype=float).reshape((-1, 3)); result = []
    for t in times:
        i = int(np.searchsorted(a[:, 0], t, side='right') - 1)
        row = a[i] if i >= 0 and 0 <= t-a[i, 0] <= max_age else None
        valid = row is not None and np.all(np.isfinite(row)) and row[1] in STATES and row[2] >= 0
        result.append(dict(spread_m=float(row[2]) if valid else None,
                           quality=STATES[int(row[1])] if valid else 'UNKNOWN'))
    return result


def load(root):
    import analyze_localization_run as A
    import compare_matched_sections as M
    meta, data, pose, origin, duration = M.load_run(root)
    spec = A.normalize_roles(meta).get('localization_health')
    rows = []
    if spec and spec.get('type') == 'std_msgs/msg/Float32MultiArray':
        for received, msg in A.read_bag(root, {'health': spec})['health']:
            if len(msg.data) > 4:
                rows.append([(received-origin)*1e-9, msg.data[0], msg.data[4]])
    evidence = summarize(rows, duration)
    associated = associate(rows, pose[:, 4])
    evidence['pose_samples'] = [dict(x_m=float(p[1]), y_m=float(p[2]), receive_sec=float(p[4]), **s) for p, s in zip(pose, associated)]
    evidence['pose_association_coverage_percent'] = 100*sum(s['spread_m'] is not None for s in associated)/len(associated) if len(associated) else 0.
    evidence['duration_sec'] = duration
    return meta, data, pose, evidence


def context(ax, reference, background=None, positions=None):
    import compare_matched_sections as M
    from matplotlib.transforms import Affine2D
    if background:
        image, res, origin = background
        ax.imshow(image, cmap='gray', vmin=0, vmax=1, origin='upper', interpolation='nearest',
                  extent=(0, image.shape[1]*res, 0, image.shape[0]*res),
                  transform=Affine2D().rotate(origin[2]).translate(origin[0], origin[1])+ax.transData, zorder=0)
    ax.plot(reference[:, 0], reference[:, 1], 'k--', lw=1.2, label='Recorded raceline')
    for start, end in M.direction_segments(reference, (.18, .51, .84)):
        ax.annotate('', xy=end, xytext=start, arrowprops=dict(arrowstyle='-|>', color='black', lw=1.3))
    extent=reference if positions is None else np.vstack((reference,positions))
    low,high=np.min(extent,axis=0)-.8,np.max(extent,axis=0)+.8
    ax.set(xlabel='Map x [m]', ylabel='Map y [m]',xlim=(low[0],high[0]),ylim=(low[1],high[1])); ax.set_aspect('equal'); ax.grid(alpha=.15)


def background(meta, root):
    import compare_matched_sections as M
    configs = meta.get('configs', {})
    if all(k in configs for k in ('map_yaml', 'map_image')):
        paths = [Path(root)/configs[k]['copy'] for k in ('map_yaml', 'map_image')]
        if all(p.is_file() and M.sha256(p) == configs[k]['sha256'] for p,k in zip(paths, ('map_yaml', 'map_image'))):
            return M.occupancy_background(*paths)
    return None


def table(evidence, name='This run'):
    text = '| Evidence / PF quality | Samples | Particle spread median [m] | P95 [m] | Max [m] |\n| --- | ---: | ---: | ---: | ---: |\n'
    for state, item in [('All', evidence['overall']), *evidence['by_quality'].items()]:
        values = ['{:.4f}'.format(item[k]) if item[k] is not None else 'Unavailable' for k in ('p50_m','p95_m','max_m')]
        text += '| {} — {} | {} | {} | {} | {} |\n'.format(name, state, item['samples'], *values)
    return text


def render_run(root, comparison=None, attach=True):
    import matplotlib.pyplot as plt
    import compare_matched_sections as M
    root=Path(root); meta,data,pose,e=load(root); plots=root/'plots'; plots.mkdir(exist_ok=True)
    bg=background(meta,root)
    if bg is None and comparison is not None:
        import yaml
        base=Path(comparison)/'sim_section'; captured=yaml.safe_load((base/'metadata.yaml').read_text()); bg=background(captured,base)
    samples=e['pose_samples']; valid=[s for s in samples if s['spread_m'] is not None]
    for kind in ('spread','quality'):
        fig,ax=plt.subplots(figsize=(9,8));context(ax,data['reference_path'],bg,pose[:,1:3])
        if kind=='spread' and valid:
            points=ax.scatter([s['x_m'] for s in valid],[s['y_m'] for s in valid],c=[s['spread_m'] for s in valid],cmap='viridis',vmin=0,vmax=.25,s=18,zorder=4)
            fig.colorbar(points,ax=ax,label='PF particle spread [m]',shrink=.8,extend='max')
        else:
            for state,color in COLORS.items():
                points=[s for s in samples if s['quality']==state]
                if points:ax.scatter([s['x_m'] for s in points],[s['y_m'] for s in points],c=color,s=20,marker='+' if state=='UNKNOWN' else 'o',label=state,zorder=4)
        ax.set_title('PF internal particle spread' if kind=='spread' else 'PF-reported quality at the same estimated positions',pad=14)
        ax.legend(loc='upper center',bbox_to_anchor=(.5,-.12),ncol=2,fontsize=11)
        fig.text(.5,.025,'All evaluation visits; arrows show reference direction. Map identity is captured simulation context.\nSpread is cloud dispersion, not raceline distance, wall clearance or absolute localization error.',ha='center',fontsize=10)
        fig.tight_layout(rect=(0,.10,1,1));fig.savefig(plots/('particle_'+kind+'_map.png'),dpi=160,bbox_inches='tight');plt.close(fig)
    fig,axes=plt.subplots(2,1,sharex=True,figsize=(10,7))
    a=np.asarray(e['samples']).reshape((-1,3))
    if len(a):
        for code,name in STATES.items():
            v=a[a[:,1]==code];axes[0].scatter(v[:,0],v[:,2],s=9,c=COLORS[name],label=name)
        # No interpolation through stale evidence, nor categorical state interpolation.
        for row,end in zip(a,np.r_[a[1:,0],e['duration_sec']]):
            axes[1].hlines(row[1],max(0,row[0]),min(end,row[0]+.5,e['duration_sec']),color=COLORS[STATES[int(row[1])]],lw=3)
    axes[0].set(ylabel='PF particle spread [m]',title='Cloud width and quality — same health messages');axes[0].legend(loc='upper center',bbox_to_anchor=(.5,1.23),ncol=3)
    axes[1].set(yticks=[1,2,3],yticklabels=['GOOD','DEGRADED','INVALID'],xlabel='Elapsed evaluation receive time [s]',ylabel='PF quality')
    for ax in axes:ax.grid(alpha=.15);ax.set_xlim(0,e['duration_sec'])
    fig.tight_layout();fig.savefig(plots/'particle_spread_quality_time.png',dpi=160,bbox_inches='tight');plt.close(fig)
    (root/'particle_spread.json').write_text(json.dumps(e,indent=2,sort_keys=True,allow_nan=False)+'\n')
    frag='### PF internal uncertainty: particle spread and reported quality\n\n'
    frag+='**Particle spread** is `sqrt(weighted var(x) + weighted var(y))` in metres, calculated from the particle cloud. It is neither distance to the raceline nor a calibrated truth-error bound. A compact cloud can be biased; scan mismatch and hysteresis can report DEGRADED/INVALID without a wide cloud.\n\n'
    frag+=table(e)+'\nPose association coverage: **{:.1f}%** (latest received health held at most 0.5 s). Missing evidence stays unknown. Statistics use health-message samples, not duration weighting.\n\n'.format(e['pose_association_coverage_percent'])
    for filename,label in [('particle_spread_map.png','Particle spread'),('particle_quality_map.png','PF quality'),('particle_spread_quality_time.png','Spread and quality through time')]:frag+='![{}](plots/{})\n\n'.format(label,filename)
    frag+='Map color shows one quantity per view; both maps use the same estimated positions. The 0–0.25 m spread color scale is shared across reports, with values above the scale retained in the statistics. No confidence circle or onboard truth is implied.\n\n'
    (root/'particle_spread.md').write_text(frag)
    if attach:M.attach_report_fragment(root,'particle_spread.md',frag,'## 5. Localization results')
    return e


def render_pair(output,result):
    import matplotlib.pyplot as plt
    import compare_matched_sections as M
    output=Path(output)
    if not all((output/(side+'_section')/'metadata.yaml').is_file() for side in ('sim','onboard')):
        return None  # Optional presentation requires preserved section evidence.
    datasets={side:load(output/(side+'_section')) for side in ('sim','onboard')}
    frag='### Matched-section PF particle spread and quality\n\n**Internal uncertainty replaces tracking distance in this quality comparison.** PF-estimated distance to raceline remains in Tracking. The section and original strict certified-delta gates are unchanged; health thresholds/deployed identity are not established equivalent.\n\n'
    curves={};fig,axes=plt.subplots(2,1,sharex=True,figsize=(10,7))
    for side,color in [('sim','tab:blue'),('onboard','tab:orange')]:
        meta,data,pose,e=datasets[side];render_run(output/(side+'_section'),output)
        frag+=table(e,side)+'\n'
        selection=result['inputs'][side]['selected_interval']
        grid,times=M.station_grid(selection,result['settings'])
        receive=M.signal_at(np.c_[pose[:,0],pose[:,4]],1,times-selection['start_sec'],result['settings']['max_pose_gap_sec'])
        paired=associate(e['samples'],receive)
        vals=np.array([p['spread_m'] if p['spread_m'] is not None else np.nan for p in paired])
        axes[0].scatter(grid,vals,c=color,s=10,label=side)
        for code,name in STATES.items():
            yes=np.array([p['quality']==name for p in paired]);axes[1].scatter(grid[yes],[code+(0 if side=='sim' else .08)]*sum(yes),s=12,c=color,marker='o' if side=='sim' else 'x')
        curves[side]=dict(station_m=grid.tolist(),particle_spread_m=[float(v) if np.isfinite(v) else None for v in vals],quality=[p['quality'] for p in paired],summary=e['overall'])
        # Full-size paired maps: adjacent in the report rather than cramped panels.
        for filename,label in [('particle_spread_map.png','particle spread'),('particle_quality_map.png','reported quality')]:frag+='![{} {}]({}_section/plots/{})\n\n'.format(side,label,side,filename)
    axes[0].set(ylabel='PF particle spread [m]',title='Matched reference traversal: internal cloud width');axes[0].legend()
    axes[1].set(yticks=[1,2,3],yticklabels=['GOOD','DEGRADED','INVALID'],xlabel='Matched reference progress [m]',ylabel='Reported PF quality')
    for ax in axes:ax.grid(alpha=.15)
    fig.tight_layout();fig.savefig(output/'plots/particle_spread_comparison.png',dpi=160,bbox_inches='tight');plt.close(fig)
    frag+='![Matched spread and PF quality](plots/particle_spread_comparison.png)\n\nThe small vertical offset separates platforms on categorical rows; it is not a quality magnitude. The existing common station grid and first-crossing time mapping are reused. Health is causally associated at the corresponding pose receipt time, held at most 0.5 s; gaps are not filled. All original section samples remain in the scorecard.\n\n'
    (output/'particle_spread_comparison.json').write_text(json.dumps(curves,indent=2,sort_keys=True,allow_nan=False)+'\n')
    M.attach_report_fragment(output,'particle_spread_comparison.md',frag,'## 4. Trajectories and station-domain plots')
