"""Station alignment is explicit and never substitutes for original metric coordinates."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest

SCRIPT = Path(__file__).resolve().parents[1]/'scripts/compare_matched_sections.py'
spec=importlib.util.spec_from_file_location('matched_sections',SCRIPT)
M=importlib.util.module_from_spec(spec);spec.loader.exec_module(M)


def circle(traversals=2.5):
    angle=np.linspace(0,2*np.pi,801)
    ref=np.c_[4*np.cos(angle),4*np.sin(angle)]
    station=np.arange(.3,traversals*8*np.pi,.05)
    pose=np.c_[station,4*np.cos(station/4),4*np.sin(station/4),station/4+np.pi/2,station]
    return ref,pose


def test_continuous_projection_unwraps_laps_and_selects_first_complete_pass():
    ref,pose=circle()
    rows,length=M.project_continuously(pose,ref,M.defaults())
    assert not any(r['reason'] for r in rows)
    station=np.array([r['s'] for r in rows])
    assert np.all(np.diff(station)>0) and station[-1]>2*length
    selected=M.select_first_pass(rows,length,0.,length,M.defaults())
    import yaml
    yaml.safe_dump(selected)  # Metadata must contain Python scalars, not NumPy scalars.
    assert selected['start_unwrapped_m']==pytest.approx(length)
    assert selected['elapsed_sec']==pytest.approx(length,abs=.02)
    grid,times=M.station_grid(selected,M.defaults())
    assert grid[0]==0 and grid[-1]==pytest.approx(length)
    assert times[-1]-times[0]==pytest.approx(selected['elapsed_sec'])
    assert M.select_first_pass(rows,length,0.,length,M.defaults())==selected


def test_nearby_branches_require_evidence_not_nearest_segment_guess():
    ref=np.array([[-2.,-1.],[2.,1.],[-2.,1.],[2.,-1.],[-2.,-1.]])
    first=np.array([[0.,0.,0.,0.,0.]])
    rows,_=M.project_continuously(first,ref,M.defaults())
    assert rows[0]['reason']=='ambiguous_nearby_branches'
    t=np.arange(0.,2.,.1);x=-1.5+t
    pose=np.c_[t,x,x/2,t*0,t]
    rows,_=M.project_continuously(pose,ref,M.defaults())
    assert all(r['s'] is not None for r in rows)
    assert max(r['s'] for r in rows)-min(r['s'] for r in rows)<3.


def test_source_gap_does_not_invent_lap_or_complete_coverage():
    ref,pose=circle(.9)
    pose[len(pose)//2:,0]+=1.
    rows,length=M.project_continuously(pose,ref,M.defaults())
    assert len({r['segment'] for r in rows})==2
    assert any(r.get('follows_source_gap') for r in rows)
    with pytest.raises(ValueError,match='No continuous complete section'):
        M.select_first_pass(rows,length,0.,length,M.defaults())


def test_missing_station_coverage_and_invalid_reference_fail():
    rows=[dict(t=t,s=s,segment=0) for t,s in [(0.,0.),(.1,.2),(.2,2.),(.3,3.)]]
    with pytest.raises(ValueError,match='missing_station_or_time_coverage'):
        M.select_first_pass(rows,3.,0.,3.,M.defaults())
    with pytest.raises(ValueError,match='closed'):
        M.geometry(np.array([[0.,0.],[1.,0.],[2.,0.],[3.,0.]]))
    with pytest.raises(ValueError,match='Section start'):
        M.select_first_pass(rows,3.,4.,3.,M.defaults())


def test_far_pose_is_excluded_and_health_does_not_select_interval():
    ref,pose=circle()
    rows,length=M.project_continuously(pose,ref,M.defaults())
    original=M.select_first_pass(rows,length,0.,length,M.defaults())
    # No health argument exists: INVALID messages cannot filter a traversal.
    with_health=[dict(r,health='INVALID') for r in rows]
    assert M.select_first_pass(with_health,length,0.,length,M.defaults())==original
    bad=pose[:1].copy();bad[:,1:3]=100.
    rows,_=M.project_continuously(bad,ref,M.defaults())
    assert rows[0]['reason']=='outside_projection_corridor'


def metric(value=2.,evidence='estimated_map_pose_proxy'):
    return dict(status='AVAILABLE',value=value,unit='m',method='nearest_static_segment_v1',
                time_basis='source_wall',evidence=evidence,coverage=1.)


def test_exploratory_is_not_certified_and_unknown_evidence_stays_missing():
    row=M.exploratory(metric(2.),metric(3.),'pf_tracking',.95)
    assert row['exploratory_onboard_minus_sim']==1.
    assert row['classification']=='EXPLORATORY_UNVERIFIED_DEPLOYMENT'
    assert row['certified_delta'] is None
    for change in [dict(status='NOT_APPLICABLE',value=None),dict(status='UNAVAILABLE_DATA',value=None),dict(coverage=.5),dict(evidence='different')]:
        right=metric();right.update(change)
        assert M.exploratory(metric(),right,'measurement',.95)['exploratory_onboard_minus_sim'] is None
    assert M.exploratory(metric(),metric(),'health_time_distribution',.95)['exploratory_onboard_minus_sim'] is None


def test_station_display_interpolation_does_not_bridge_time_gaps():
    samples=np.array([[0.,1.],[1.,3.]])
    result=M.signal_at(samples,1,np.array([0.,.5,1.]),.2)
    assert result[0]==1. and result[2]==3. and np.isnan(result[1])
    assert np.all(np.isnan(M.signal_at(None,1,np.array([0.,1.]),.2)))


def test_existing_consistency_and_oscillation_definitions_are_reused():
    import analyze_localization_run as A
    assert M.A.consistency is A.consistency
    assert M.A.station_oscillation is A.station_oscillation
    # Raceline station is not passed to the shared analyzer or its smoothness method.
    source=SCRIPT.read_text()
    assert 'station_oscillation(' not in source


def test_no_output_overwrite(tmp_path):
    with pytest.raises(ValueError,match='Output exists'):
        M.compare_sections(tmp_path,tmp_path,tmp_path)


def test_section_crop_preserves_originals_identity_and_metric_definitions(tmp_path):
    import yaml
    fixture_spec=importlib.util.spec_from_file_location('measurement_fixtures',Path(__file__).with_name('test_localization_measurements.py'))
    F=importlib.util.module_from_spec(fixture_spec);fixture_spec.loader.exec_module(F)
    source=tmp_path/'source';source.mkdir();F.make_run(source)
    before=(source/'metadata.yaml').read_bytes()
    selected=dict(start_sec=.2,end_sec=.8,elapsed_sec=.6,length_m=.6)
    result=M.create_section(source,tmp_path/'review/section',selected,100000000000)
    assert (source/'metadata.yaml').read_bytes()==before
    assert result['localization']['accuracy']['position_error_m']['value']['p95']==pytest.approx(.2)
    assert result['vehicle']['robustness']['completed']['status']=='UNAVAILABLE_DATA'
    assert result['identity']=={}
    report=(tmp_path/'review/section/report.md').read_text()
    assert '../../../../docs/LOCALIZATION_SIMULATION.md' in report
    assert '../../../scripts/analyze_localization_run.py' in report
    metadata=yaml.safe_load((tmp_path/'review/section/metadata.yaml').read_text())
    assert metadata['source_outcome']['smoke_result']['result']=='PASS'
    assert metadata['interval']==dict(start_wall_ns=100200000000,end_wall_ns=100800000000)


def test_cannot_place_output_inside_a_preserved_run(tmp_path):
    source=tmp_path/'source';source.mkdir()
    with pytest.raises(ValueError,match='separate'):
        M.compare_sections(source,source,source/'comparison')


def test_report_explains_results_before_mechanics_and_renders_deterministically(tmp_path):
    settings=M.defaults()
    names=['pf_position_m','pf_heading_rad','autonomy_speed_mps','autonomy_steering_rad',
           'forward_speed_mps','consistency_position_m','consistency_yaw_rad','health_state','elapsed_sec']
    curve={name:np.zeros(20) for name in names};curve['station_m']=np.linspace(0,10,20)
    item=dict(selected_interval=dict(length_m=10.,start_sec=1.,end_sec=11.,elapsed_sec=10.,non_increasing_station_samples=0),
              section_projection_samples=100,section_projection_coverage=1.,projection_excluded_reasons={},outside_selected_time_samples=10,
              grid_coverage={name:1. for name in names},observed_command_ranges={},source_directory='/preserved/input')
    from copy import deepcopy
    row=M.exploratory(metric(dict(p50=.1,p95=.2,max=.3)),metric(dict(p50=.2,p95=.3,max=.4)),'tracking',.95)
    result=dict(inputs=dict(sim=deepcopy(item),onboard=deepcopy(item)),
                metrics={'vehicle.tracking.pf_reference_position_deviation_m':row,
                         'vehicle.smoothness.autonomy_steering_command_oscillation':M.exploratory(metric(dict(rms=.1,p95_absolute=.2)),metric(dict(rms=.2,p95_absolute=.3)),'steering',.95)},
                elapsed_section_sec=dict(sim=10.,onboard=10.,exploratory_onboard_minus_sim=0.),
                strict_certified_deltas_available=0,settings=settings,limitations=['Historical onboard identity UNKNOWN'])
    result['sim_runtime_parameters']={'values':{'node':{'error':'parameter service unavailable'}}}
    metrics=[dict(analysis_status='PASS'),dict(analysis_status='PASS')]
    M.render(tmp_path,result,metrics,[curve,curve])
    before={str(p.relative_to(tmp_path)):p.read_bytes() for p in tmp_path.rglob('*') if p.is_file()}
    report=(tmp_path/'report.md').read_text()
    assert report.index('Main observations')<report.index('## 3. Evidence')
    assert 'not analyzer failure' in report.replace('**','') and 'raw-odometry cumulative driven distance' in report
    assert 'unavailable post-run service does not qualify runtime settings' in report
    assert 'Software and evidence validation' in report and 'exploratory' in report.lower()
    import json
    result=json.loads(json.dumps(result,sort_keys=True))
    M.render(tmp_path,result,metrics,[curve,curve])
    assert before=={str(p.relative_to(tmp_path)):p.read_bytes() for p in tmp_path.rglob('*') if p.is_file()}


def test_occupancy_display_uses_yaml_thresholds_resolution_and_origin(tmp_path):
    import yaml
    from matplotlib import image
    path=tmp_path/'map.yaml';picture=tmp_path/'map.png'
    image.imsave(str(picture),np.array([[0.,1.],[.5,1.]]),cmap='gray',vmin=0,vmax=1)
    info=dict(resolution=.05,origin=[-1.55,-1.47,.2],negate=0,occupied_thresh=.65,free_thresh=.25)
    path.write_text(yaml.safe_dump(info))
    shade,resolution,origin=M.occupancy_background(path,picture)
    assert shade[0,0]==.32 and shade[0,1]==.99 and shade[1,0]==.84
    assert resolution==.05 and origin==pytest.approx([-1.55,-1.47,.2])
    info['negate']=1;path.write_text(yaml.safe_dump(info))
    shade,_,_=M.occupancy_background(path,picture)
    assert shade[0,0]==.99 and shade[0,1]==.32


@pytest.mark.parametrize('number', [1, 2, 3])
def test_report_identifies_the_actual_onboard_session(number):
    result={'inputs':{'onboard':{'source_directory':'/runs/tempo_shared_metrics_onboard_session{}_motion'.format(number)}}}
    assert M.onboard_name(result)=='Onboard Session '+str(number)


def test_direction_arrows_follow_forward_and_reverse_order_without_changing_points():
    points=np.array([[0.,0.],[0.,0.],[2.,0.]])
    before=points.copy()
    arrows=M.direction_segments(points,[.2,.5,.8])
    assert len(arrows)==3 and all(end[0]>start[0] for start,end in arrows)
    assert all(end[0]<start[0] for start,end in M.direction_segments(points[::-1],[.2,.5,.8]))
    assert np.array_equal(points,before)
    assert M.direction_segments(np.zeros((3,2)),[.5])==[]


def test_health_position_annotations_use_receipt_time_causally_and_do_not_bridge_stale_data():
    pose=np.array([[0.,1.,2.,0.,.05],[0.,1.,2.,0.,.15],[0.,1.,2.,0.,.25],[0.,1.,2.,0.,.8]])
    before=pose.copy()
    states=M.pose_health_states(pose,[(0.,'GOOD'),(.1,'DEGRADED'),(.2,'INVALID'),(.9,'GOOD')],.5)
    assert states.tolist()==['GOOD','DEGRADED','INVALID','UNKNOWN']
    assert np.array_equal(pose,before)
    assert M.pose_health_states(pose,[],.5).tolist()==['UNKNOWN']*4


def test_health_annotation_does_not_use_a_future_health_message():
    pose=np.array([[0.,0.,0.,0.,.05],[0.,0.,0.,0.,.6]])
    assert M.pose_health_states(pose,[(.1,'GOOD')],.5).tolist()==['UNKNOWN','GOOD']


def test_deviation_maps_reuse_existing_errors_and_keep_same_scale(tmp_path, monkeypatch):
    import json
    from copy import deepcopy
    ref,pose=circle(.15)
    outputs=tmp_path/'comparison';(outputs/'plots').mkdir(parents=True)
    roots={side:tmp_path/side for side in ('sim','onboard')}
    source={}
    for side in roots:
        root=roots[side];root.mkdir();(root/'plots').mkdir()
        (root/'metadata.yaml').write_text('{}')
        (root/'report.md').write_text('# Report\n\n![Tracking](plots/pf_reference_trajectory.png)\n')
        section=outputs/(side+'_section');section.mkdir();(section/'metadata.yaml').write_text('{}')
        pp=pose.copy();pp[:,1]+= .1 if side=='onboard' else 0.
        run=({}, {'reference_path':ref},pp,0.,float(pp[-1,0]))
        source[root]=run;source[section]=run
    monkeypatch.setattr(M,'load_run',lambda path:source[path])
    result={'inputs':{side:{'source_directory':str(root)} for side,root in roots.items()}}
    original=deepcopy(source)
    M.deviation_position_plots(outputs,result)
    evidence=json.loads((outputs/'pf_reference_deviation_positions.json').read_text())
    assert evidence['color_range_m'][0]==0 and 'not PF accuracy' in evidence['qualification']
    for side in roots:
        expect=M.A.reference_errors(source[roots[side]][2],ref)[0]
        actual=evidence['views']['full_'+side]['samples']
        assert [x['reference_deviation_m'] for x in actual]==pytest.approx(expect)
        assert 'Where the estimated trajectory deviates' in (roots[side]/'report.md').read_text()
        assert np.array_equal(original[roots[side]][2],source[roots[side]][2])
    before={str(p):p.read_bytes() for p in outputs.rglob('*') if p.is_file()}
    M.deviation_position_plots(outputs,result)
    assert before=={str(p):p.read_bytes() for p in outputs.rglob('*') if p.is_file()}
    assert (roots['sim']/'report.md').read_text().count('Where the estimated trajectory deviates')==1


def test_optional_reference_tangents_preserve_existing_errors_and_ambiguity():
    reference=np.array([[0.,0.],[2.,0.],[2.,2.]])
    pose=np.array([[0.,1.,.1,.2,0.],[1.,2.,0.,0.,1.]])
    before=M.A.reference_errors(pose,reference)
    distance,error,tangent=M.A.reference_errors(pose,reference,return_tangents=True)
    assert np.allclose(before[0],distance) and np.allclose(before[1],error,equal_nan=True)
    assert tangent[0]==0 and np.isnan(tangent[1]) and np.isnan(error[1])


def test_heading_alignment_wraps_angles_and_rejects_gaps_or_wrong_frames():
    reference=np.array([[0.,0.],[2.,0.],[2.,2.],[0.,2.],[0.,0.]])
    pose=np.array([[0.,1.,.1,np.radians(179),0.],[.05,1.,.1,np.radians(179),.05],[.3,1.,.1,0.,.3]])
    truth=np.array([[0.,1.,.1,np.radians(-179)],[.1,1.,.1,np.radians(-179)],[.6,1.,.1,0.]])
    metadata={'platform':'sim','schema_version':2,'roles':{'estimated_pose':{'frame':'map'},'ground_truth_pose':{'frame':'map'}}}
    data={'reference_path':reference,'ground_truth_pose':truth}
    result=M.heading_series(metadata,data,pose,{'max_gt_gap_sec':.2})
    assert abs(result['pf_gt_signed_rad'][0])==pytest.approx(np.radians(2))
    assert np.isnan(result['gt_yaw_rad'][2])
    metadata['roles']['ground_truth_pose']['frame']='different'
    with pytest.raises(ValueError,match='frames differ'):M.heading_series(metadata,data,pose,{'max_gt_gap_sec':.2})
    metadata['platform']='onboard'
    assert 'gt_yaw_rad' not in M.heading_series(metadata,data,pose,{'max_gt_gap_sec':.2})


def test_angle_display_breaks_wrap_and_time_gaps_without_inventing_interpolation():
    times=np.array([0.,.1,1.]);angles=np.radians([179.,-179.,-178.])
    t,y=M.angle_display(times,angles,.5)
    assert sum(np.isnan(y))==2 and t[np.isfinite(t)].tolist()==times.tolist()
    assert y[np.isfinite(y)]==pytest.approx([179.,-179.,-178.])


def test_full_heading_report_uses_existing_metrics_and_is_repeatable(tmp_path):
    import json
    reference=np.array([[0.,0.],[2.,0.],[2.,2.],[0.,2.],[0.,0.]])
    pose=np.array([[0.,.5,.1,.1,0.],[.1,.6,.1,.1,.1],[.2,.7,.1,.1,.2]])
    truth=pose[:,:4].copy();truth[:,3]=.05
    metadata={'platform':'sim','schema_version':2,'roles':{'estimated_pose':{'frame':'map'},'ground_truth_pose':{'frame':'map'}}}
    root=tmp_path;(root/'plots').mkdir();(root/'report.md').write_text('# Report\n\n![Tracking](plots/pf_reference_trajectory.png)\n')
    metric={'status':'AVAILABLE','value':{'p50':.1,'p95':.2,'max':.3}}
    metrics={'vehicle':{'tracking':{'pf_heading_to_reference_rad':metric,'heading_to_reference_rad':metric}},'localization':{'accuracy':{'absolute_heading_error_rad':metric}}}
    path=root/'metrics.json';path.write_text(json.dumps(metrics));original=path.read_bytes()
    run=(metadata,{'reference_path':reference,'ground_truth_pose':truth},pose,0.,.3)
    M.plot_heading_run(root,run,{'max_gt_gap_sec':.2})
    files=[root/'plots/heading_analysis.png',root/'heading_analysis.json',root/'report.md']
    first=[p.read_bytes() for p in files]
    M.plot_heading_run(root,run,{'max_gt_gap_sec':.2})
    assert first==[p.read_bytes() for p in files] and original==path.read_bytes()
    assert 'PF / GT' in (root/'report.md').read_text() and 'not additive' in (root/'report.md').read_text()
