"""Passive, receive-time control/quality displays from the existing run contract."""
import json
from pathlib import Path
import numpy as np
import yaml

MODE_COLORS = {'RACELINE':'#2878b5', 'REACTIVE':'#e67e22', 'STOP':'#c62828',
               'WAITING':'#777777', 'UNKNOWN':'#7550a3'}
QUALITY_COLORS = {'GOOD':'#208443','DEGRADED':'#e49b00','INVALID':'#d62728','UNKNOWN':'#7550a3'}
SAFETY_COLORS = {'NOMINAL':'#208443','FALLBACK_FTG':'#e67e22','EMERGENCY_STOP':'#c62828',
                 'REVERSE':'#8c564b','UNKNOWN':'#7550a3'}


def held_record(records, time, gap):
    """Last actually received record; never future-state or stale interpolation."""
    if not records:
        return None
    index=int(np.searchsorted([row['receive_sec'] for row in records],time,side='right')-1)
    return records[index] if index>=0 and time-records[index]['receive_sec']<=gap else None


def exposure(records, quality, duration, gap):
    """Integrate freshness-bounded observed states, retaining UNKNOWN intervals."""
    boundaries={0.,float(duration)}
    for stream in (records,quality):
        for row in stream:
            boundaries.update(t for t in (row['receive_sec'],row['receive_sec']+gap) if 0<t<duration)
    edges=sorted(boundaries);durations={};joint={};segments=[]
    for left,right in zip(edges[:-1],edges[1:]):
        mode=held_record(records,(left+right)/2,gap)
        health=held_record(quality,(left+right)/2,gap)
        mm=mode['state'] if mode else 'UNKNOWN';hh=health['state'] if health else 'UNKNOWN'
        durations[mm]=durations.get(mm,0.)+right-left
        joint.setdefault(mm,{})[hh]=joint.get(mm,{}).get(hh,0.)+right-left
        segments.append(dict(start_sec=left,end_sec=right,state=mm,quality=hh))
    return dict(duration_sec=durations,joint_quality_sec=joint,segments=segments)


def observed_changes(records, pose, duration, gap):
    changes=[]
    for previous,current in zip(records[:-1],records[1:]):
        time=current['receive_sec']
        if not 0<=time<duration or time-previous['receive_sec']>gap or previous['state']==current['state']:
            continue
        event=dict(current,previous_state=previous['state'],position=None)
        index=int(np.searchsorted(pose[:,4],time,side='right')-1)
        if index>=0 and time-pose[index,4]<=gap:
            event['position']=dict(x_m=float(pose[index,1]),y_m=float(pose[index,2]),age_sec=float(time-pose[index,4]))
        changes.append(event)
    return changes


def coalesce_states(segments):
    """Draw one solid span for adjacent identical states; preserve input evidence."""
    result=[]
    for segment in segments:
        if result and result[-1]['state']==segment['state'] and abs(result[-1]['end_sec']-segment['start_sec'])<1e-9:
            result[-1]['end_sec']=segment['end_sec']
        else:
            result.append(dict(segment))
    return result


def read_status(root, roles, role, origin):
    import analyze_localization_run as A
    if role not in roles:
        return []
    spec=roles[role];records=A.read_bag(root,{role:spec})[role];rows=[]
    for received,msg in records:
        fields=A.diagnostic(msg,spec)
        rows.append(dict(receive_sec=(received-origin)*1e-9,source_sec=(A.stamp(msg.header)-origin)*1e-9,
                         state=fields[spec.get('state_key','mode')],reason=fields.get('reason','Not recorded'),fields=fields))
    return rows


def render_run(root, comparison, deviation_vmax=None):
    import matplotlib.pyplot as plt
    from matplotlib.transforms import Affine2D
    import analyze_localization_run as A
    import compare_matched_sections as M
    meta,data,pose,origin,duration=M.load_run(root)
    roles=A.normalize_roles(meta);metrics=json.loads((root/'metrics.json').read_text())
    gap=metrics['settings']['max_status_gap_sec']
    control=read_status(root,roles,'control_status',origin)
    safety=read_status(root,roles,'safety_status',origin)
    quality=[dict(receive_sec=float(t),state=state) for t,state in data.get('localization_health',[])]
    summary=exposure(control,quality,duration,gap)
    changes=observed_changes(control,pose,duration,gap)
    safety_changes=observed_changes(safety,pose,duration,gap)
    deviation,_=A.reference_errors(pose,data['reference_path'])
    import evaluation_particle_spread as P
    spread_evidence=P.render_run(root,comparison,attach=False)
    spread_samples=spread_evidence['pose_samples']
    samples=[]
    for row,error,cloud in zip(pose,deviation,spread_samples):
        sample=dict(particle_spread_m=cloud['spread_m'],source_sec=float(row[0]),receive_sec=float(row[4]),x_m=float(row[1]),y_m=float(row[2]),pf_reference_deviation_m=float(error))
        for name,records in [('control',control),('safety',safety),('quality',quality)]:
            latest=held_record(records,row[4],gap)
            sample[name]=latest['state'] if latest else 'UNKNOWN'
        samples.append(sample)
    conditional={}
    for state in sorted({item['control'] for item in samples}):
        values=[item['pf_reference_deviation_m'] for item in samples if item['control']==state]
        conditional[state]=dict(pose_samples=len(values),p50_m=float(np.median(values)),p95_m=float(np.percentile(values,95)),max_m=float(max(values)))
    conditional_spread={}
    for state in conditional:
        vals=[x['particle_spread_m'] for x in samples if x['control']==state and x['particle_spread_m'] is not None]
        conditional_spread[state]=dict(samples=len(vals),p50_m=float(np.median(vals)) if vals else None,p95_m=float(np.percentile(vals,95)) if vals else None,max_m=float(max(vals)) if vals else None)
    configs=yaml.safe_load((comparison/'sim_section/metadata.yaml').read_text()).get('configs',{})
    background=None
    if all(k in configs for k in ('map_yaml','map_image')):
        paths=[comparison/'sim_section'/configs[k]['copy'] for k in ('map_yaml','map_image')]
        if all(path.is_file() and M.sha256(path)==configs[k]['sha256'] for path,k in zip(paths,('map_yaml','map_image'))):
            background=M.occupancy_background(*paths)
    ref=data['reference_path'];points=np.vstack((ref,pose[:,1:3]));low,high=points.min(axis=0)-1.,points.max(axis=0)+1.
    def context(ax):
        if background is not None:
            shade,res,offset=background
            transform=Affine2D().rotate(offset[2]).translate(offset[0],offset[1])+ax.transData
            ax.imshow(shade,cmap='gray',vmin=0,vmax=1,origin='upper',interpolation='nearest',extent=(0,shade.shape[1]*res,0,shade.shape[0]*res),transform=transform,zorder=0)
        ax.plot(ref[:,0],ref[:,1],'k--',lw=1.2,label='Recorded raceline',zorder=2)
        for start,end in M.direction_segments(ref,(.18,.51,.84)):
            ax.annotate('',xy=end,xytext=start,arrowprops=dict(arrowstyle='-|>',color='black',lw=1.3),zorder=8)
        ax.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),xlabel='Map x [m]',ylabel='Map y [m]')
        ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.12)
    plots=root/'plots';plots.mkdir(exist_ok=True)
    for field,filename,title,palette in [
            ('control','control_mode_map.png','Selected autonomous controller',MODE_COLORS),
            ('safety','lower_safety_mode_map.png','Lower-safety reported state',SAFETY_COLORS),
            ('quality','control_pf_quality_map.png','PF-reported quality at the same estimated positions',QUALITY_COLORS)]:
        fig,ax=plt.subplots(figsize=(10,9));context(ax)
        priority={'RACELINE':0,'NOMINAL':0,'GOOD':0,'REACTIVE':1,'FALLBACK_FTG':1,'DEGRADED':1,'STOP':2,'INVALID':2,'UNKNOWN':3}
        states=sorted({item[field] for item in samples},key=lambda state:(priority.get(state,1),state))
        for state in states:
            group=[item for item in samples if item[field]==state]
            ax.scatter([item['x_m'] for item in group],[item['y_m'] for item in group],s=25 if state!='UNKNOWN' else 45,
                       marker='o' if state!='UNKNOWN' else '+',c=palette.get(state,'#8c564b'),label='{} ({} poses)'.format(state,len(group)),zorder=4)
        if field=='control':
            positioned=[event['position'] for event in changes if event['position']]
            ax.scatter([p['x_m'] for p in positioned],[p['y_m'] for p in positioned],marker='D',s=80,
                       facecolors='none',edgecolors='black',lw=1.5,label='Observed mode change ({} located)'.format(len(positioned)),zorder=7)
        ax.set_title(title,fontsize=15,pad=15);ax.tick_params(labelsize=12)
        ax.legend(loc='upper center',bbox_to_anchor=(.5,-.13),ncol=2,fontsize=12,framealpha=.95)
        fig.text(.5,.02,'Full evaluation; identical PF-estimated positions and map limits across the map views.\n'
                 'Latest received status held <= {:.2f} s. Repeated visits overlap; use the timeline to separate them.'.format(gap),ha='center',fontsize=11)
        fig.tight_layout(rect=(0,.12,1,1));fig.savefig(plots/filename,dpi=160,bbox_inches='tight',pad_inches=.2);plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,9));context(ax)
    scatter=ax.scatter(pose[:,1],pose[:,2],c=deviation,cmap='viridis',vmin=0,vmax=deviation_vmax or max(float(max(deviation)),1e-6),s=15,zorder=4)
    fig.colorbar(scatter,ax=ax,label='PF-estimated distance to raceline [m]',shrink=.8)
    ax.set_title('Distance of PF-estimated position from the raceline',fontsize=15,pad=15);ax.tick_params(labelsize=12)
    ax.legend(fontsize=11)
    fig.text(.5,.02,'Distance to raceline is estimated tracking, not onboard localization error or wall clearance.\nColor scale is shared by this sim/onboard pair; no independent onboard position truth exists.',ha='center',fontsize=11)
    fig.tight_layout(rect=(0,.12,1,1));fig.savefig(plots/'control_reference_deviation_map.png',dpi=160,bbox_inches='tight',pad_inches=.2);plt.close(fig)

    fig,axes=plt.subplots(4,1,sharex=True,figsize=(12,12),gridspec_kw={'height_ratios':[1,1,1,1.5]})
    for ax,records,title,palette in [(axes[0],control,'Selected controller',MODE_COLORS),(axes[1],safety,'Lower safety',SAFETY_COLORS),(axes[2],quality,'PF quality',QUALITY_COLORS)]:
        spans=coalesce_states(exposure(records,[],duration,gap)['segments']);states=sorted({span['state'] for span in spans})
        for span in spans:
            ax.plot([span['start_sec'],span['end_sec']],[states.index(span['state'])]*2,color=palette.get(span['state'],'#8c564b'),lw=5,solid_capstyle='butt')
        ax.set(yticks=range(len(states)),yticklabels=states,ylabel=title,ylim=(-.6,len(states)-.4))
        ax.tick_params(labelsize=11);ax.grid(axis='x',alpha=.15)
    # Preserve visibility of a single transient safety report without stretching its duration.
    safety_states=sorted({span['state'] for span in exposure(safety,[],duration,gap)['segments']})
    for event in safety_changes:
        if event['state'] in safety_states:
            axes[1].scatter(event['receive_sec'],safety_states.index(event['state']),s=35,c='black',marker='D',zorder=6)
    if safety_changes:
        axes[1].set_title('Black diamonds: safety change was reported; marker size is not dwell time',fontsize=11)
    axes[3].plot(pose[:,4],[p['spread_m'] if p['spread_m'] is not None else np.nan for p in spread_samples],color='tab:blue',lw=1.)
    for i in range(1,len(pose)):
        if pose[i,4]-pose[i-1,4]>gap:
            axes[3].axvspan(pose[i-1,4],pose[i,4],color='white',zorder=3)
    axes[3].set(ylabel='PF particle\nspread [m]',xlabel='Elapsed evaluation time — bag receive clock [s]');axes[3].grid(alpha=.15)
    for ax in axes:ax.set_xlim(0,duration)
    fig.suptitle('Control, safety, PF quality and particle spread — same observation clock',fontsize=15)
    fig.tight_layout();fig.savefig(plots/'control_quality_timeline.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,5))
    states=list(summary['duration_sec']);values=[summary['duration_sec'][s] for s in states]
    bars=ax.barh(states,values,color=[MODE_COLORS.get(s,'#8c564b') for s in states])
    for bar,value in zip(bars,values):ax.text(value,bar.get_y()+bar.get_height()/2,' {:.3f} s ({:.1f}%)'.format(value,value/duration*100),va='center',fontsize=12)
    ax.set(xlim=(0,max(values)*1.35 if values else 1),xlabel='Recorded selected-mode exposure [s]',title='How much recorded evidence used each selected mode?')
    ax.tick_params(labelsize=12);ax.grid(axis='x',alpha=.15);fig.tight_layout();fig.savefig(plots/'control_mode_exposure.png',dpi=160);plt.close(fig)
    evidence=dict(method='recorded_control_quality_review_v1',time_basis='bag_receive',status_max_age_sec=gap,
                  duration_sec=duration,control=control,safety=safety,quality=quality,samples=samples,
                  transitions=changes,safety_transitions=safety_changes,exposure=summary,conditional_pf_deviation=conditional,conditional_particle_spread=conditional_spread,
                  source_stamped_exposure={role:A.state_summary(data.get(role,[]),duration,gap) for role in ('control_status','safety_status')},
                  qualification='Reported selection is not mux/actuator authority. Receive-time durations are recorded evidence exposure, not exact physical dwell. No causal inference from co-occurrence.')
    (root/'control_mode_analysis.json').write_text(json.dumps(evidence,indent=2,sort_keys=True,allow_nan=False)+'\n')
    fragment='<a id="control-selection-pf-quality-and-reference-deviation"></a>\n\n#### Control selection, PF quality and particle spread\n\n'
    fragment+=('**What this section answers:** which controller was selected, where changes were reported, and how those observations align with PF quality and believed path tracking. '
               '`RACELINE` selects the primary planning/following chain; `REACTIVE` selects the fallback chain; `STOP` selects stopping; `WAITING` awaits inputs. '
               'Lower safety can independently constrain the result. These are upstream reported states, not proof of onboard mux authority or actual actuator motion.\n\n')
    fragment+='**Observed changes:** {} selected-mode changes and {} lower-safety changes within this evaluation. Initial state acquisition and reconnection after missing evidence are not counted as switches.\n\n'.format(len(changes),len(safety_changes))
    fragment+='| Selected mode | Recorded exposure [s] | Evaluation [%] |\n| --- | --- | --- |\n'
    for state,value in summary['duration_sec'].items():fragment+='| {} | {:.3f} | {:.2f} |\n'.format(state,value,100*value/duration)
    fragment+='\n![Quantity-focused selected-mode exposure](plots/control_mode_exposure.png)\n\n'
    fragment+='**PF quality during each selected mode (recorded exposure, seconds):** these streams are asynchronous. A short overlap does not identify the controller’s instantaneous internal input; inspect the PF status stored in its own diagnostic and its logged reason.\n\n'
    fragment+='| Selected mode | GOOD | DEGRADED | INVALID | UNKNOWN |\n| --- | --- | --- | --- | --- |\n'
    for state,joint in summary['joint_quality_sec'].items():
        fragment+='| {} | {:.3f} | {:.3f} | {:.3f} | {:.3f} |\n'.format(state,*[joint.get(key,0.) for key in ('GOOD','DEGRADED','INVALID','UNKNOWN')])
    triggers={}
    for event in changes:
        if event['state']=='REACTIVE':
            key=event['reason'].split(':')[0]
            triggers[key]=triggers.get(key,0)+1
    if triggers:
        fragment+='\n**Logged reactive-entry reasons:** '+', '.join('{}: {}'.format(key,count) for key,count in triggers.items())+'. These are recorded controller explanations, not causes guessed from map proximity.\n\n'
    fragment+='**Timeline first:** read mode, lower safety, PF quality and particle spread on separate rows with the same time axis. A constant mode in simulation is an observed result, not missing transitions. UNKNOWN denotes missing/stale recorded evidence.\n\n![Control and quality through the evaluation](plots/control_quality_timeline.png)\n\n'
    fragment+='**Then compare the maps:** the following full-size views use identical PF positions, circuit context and axis limits. Each image encodes one quantity. Black hollow diamonds mark located controller changes; repeated visits overlap, so the timeline and event table retain chronology. No missing pose is invented.\n\n'
    for filename,title in [('control_mode_map.png','Selected controller and change positions'),('lower_safety_mode_map.png','Independent lower-safety state'),('control_pf_quality_map.png','PF quality at the same positions'),('particle_spread_map.png','PF internal particle spread at the same estimated positions')]:
        fragment+='![{}](plots/{})\n\n'.format(title,filename)
    fragment+='| Change | Receive time [s] | From → to | PF status inside controller diagnostic | Recorded reason |\n| --- | --- | --- | --- | --- |\n'
    for i,event in enumerate(changes,1):
        reason=event['reason'].replace('|','/').replace('\n',' ')
        short=reason[:120]+'…' if len(reason)>120 else reason
        fragment+='| C{} | {:.3f} | {} → {} | {} | {} |\n'.format(i,event['receive_sec'],event['previous_state'],event['state'],event['fields'].get('pf_state','Not recorded'),short)
    if not changes:fragment+='| None observed | — | No fresh recorded mode changes | — | First observed selection is not a transition |\n'
    if safety_changes:
        fragment+='\n**Lower-safety changes are independent:**\n\n| Receive time [s] | State change | Recorded explanation |\n| --- | --- | --- |\n'
        for event in safety_changes:
            fragment+='| {:.3f} | {} → {} | {} |\n'.format(event['receive_sec'],event['previous_state'],event['state'],event['reason'].replace('|','/'))
        fragment+='\nA single transient sample establishes that state was reported; receive-time spacing does not establish its physical dwell time.\n'
    fragment+='\n| Selected mode at pose receipt | Fresh spread samples | Particle spread median [m] | P95 [m] | Max [m] |\n| --- | --- | --- | --- | --- |\n'
    for state,item in conditional_spread.items():
        values=['{:.4f}'.format(item[k]) if item[k] is not None else 'Unavailable' for k in ('p50_m','p95_m','max_m')]
        fragment+='| {} | {} | {} | {} | {} |\n'.format(state,item['samples'],*values)
    fragment+=('\n**How to interpret the relationship:** DEGRADED PF status (2) is allowed by the inspected arbitration PF gate; INVALID (3), missing or stale health can make the primary chain unavailable. '
               'Reactive selection also depends on planner/guard readiness, lower safety, fallback availability, latching and recovery. '
               'A REACTIVE interval while PF is GOOD can therefore be recovery-in-progress or a separate planning/safety event. '
               'Use the **recorded reason** rather than assigning causality from colors. The current source is mechanism context, not proof of historical deployed configuration. '
               'Different particle spreads by mode are conditional observations at different places/times, not evidence that a controller caused an error. No onboard GT is added.\n\n'
               '**Evidence limits:** all new maps, timeline and exposure use bag receive time to align unstamped PF health. States are held at most {:.2f} s; gaps are UNKNOWN. '
               'Receive-time exposure is not exact controller/actuator dwell, and transient lower-safety samples can be compressed by transport/recording. '
               'Source stamps, complete reasons, conditional sample values and source-stamped exposure are retained in [control analysis JSON](control_mode_analysis.json). '
               'Estimated positions and the captured simulation map do not establish physical onboard trajectory or historically loaded map identity. '
               'Existing metric values, formulas and certified comparison gates remain unchanged.\n\n').format(gap)
    M.attach_report_fragment(root,'control_analysis.md',fragment,'(plots/pf_reference_trajectory.png)')
    return evidence


def render_pair(output, result):
    import compare_matched_sections as M
    if not all((Path(result['inputs'][side]['source_directory'])/'metadata.yaml').is_file() for side in ('sim','onboard')):
        return
    roots={side:Path(result['inputs'][side]['source_directory']) for side in ('sim','onboard')}
    copies=sorted(output.glob('full_onboard_session*'))
    if len(copies)==1:roots['onboard']=copies[0]
    upper=0.
    for root in roots.values():
        _,data,pose,_,_=M.load_run(root)
        upper=max(upper,float(max(M.A.reference_errors(pose,data['reference_path'])[0])))
    evidence={side:render_run(root,output,max(upper,1e-6)) for side,root in roots.items()}
    text='### Full-run control mode, PF quality and tracking review\n\n'
    for side,root in roots.items():
        import os
        link=os.path.relpath(root/'report.md',output)+'#control-selection-pf-quality-and-reference-deviation'
        text+='[{}: full-size maps, timeline and logged transition reasons]({}) · '.format(side,link)
    text+='\n\nThese full-window reviews retain events outside the matched traversal. They do not change section selection, scorecards or certified deltas. Controller selection, lower-safety state and actual onboard mux authority are separate.\n\n'
    M.attach_report_fragment(output,'control_analysis.md',text,'## 4. Trajectories and station-domain plots')
    return evidence
