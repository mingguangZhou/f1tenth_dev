"""Run the actual pinned classifier suffix; do not reproduce PF logic in tests."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest
import yaml

BASE=Path(__file__).resolve().parents[2]
SOURCE=BASE/'particle_filter/particle_filter/particle_filter.py'
PROFILE=BASE/'particle_filter/config/localize.yaml'
spec=importlib.util.spec_from_file_location('health_explanation',Path(__file__).resolve().parents[1]/'scripts/explain_pf_health.py')
E=importlib.util.module_from_spec(spec);spec.loader.exec_module(E)


def setup():
    return yaml.safe_load(PROFILE.read_text())['particle_filter']['ros__parameters'],E.classifier(SOURCE)


def samples(values):
    # timestamp, status, match, median error, short-rays, spread, long, lateral, yaw, neff
    return np.array([[i*.04,status,match,.08,0.,.07,0.,lat,0.,.05] for i,(status,match,lat) in enumerate(values)])


def test_degraded_dwell_watchdog_reproduced_with_favorable_entry_scan():
    p,code=setup();rows=samples([(1,.8,0.)]*15+[(1,.5,0.)]*4+[(2,.5,0.)]*49+[(3,.8,0.)])
    trace=E.replay(rows,p,code)
    assert trace[-1]['watchdog'] and not trace[-1]['hard_invalid_evidence']
    assert trace[-1]['good_evidence'] and trace[-1]['predicted']==3
    assert trace[-1]['counter_before']==49 and trace[-1]['counter_after']==50


def test_hard_motion_evidence_and_recovery_are_not_scan_watchdog():
    p,code=setup();trace=E.replay(samples([(1,.8,0.)]*15+[(3,.5,.5)]*8+[(1,.8,0.)]*5),p,code)
    assert trace[-6]['hard_invalid_evidence'] and trace[-6]['predicted']==3
    assert not trace[-6]['watchdog']
    assert trace[-1]['predicted']==1


def test_gap_invalidates_counter_history_until_new_anchor():
    p,code=setup();rows=samples([(1,.8,0.)]*35);rows[20:,0]+=1.
    trace=E.replay(rows,p,code)
    assert not trace[20]['anchored'] and trace[20]['predicted'] is None
    assert trace[-1]['anchored']


def test_diagnostic_only_neff_and_longitudinal_values_do_not_trigger_health():
    p,code=setup();rows=samples([(1,.8,0.)]*20);rows[:,6]=100.;rows[:,9]=.0001
    assert E.replay(rows,p,code)[-1]['predicted']==1


def test_malformed_metric_cannot_be_explained_as_healthy():
    p,code=setup();rows=samples([(1,.8,0.)]*15);rows[-1,2]=np.nan
    with pytest.raises(ValueError,match='finite'):E.replay(rows,p,code)


def test_optional_onboard_addendum_is_near_health_plot_without_changing_metrics(tmp_path):
    import sys
    scripts=Path(__file__).resolve().parents[1]/'scripts'
    sys.path.insert(0,str(scripts))
    import analyze_localization_run as A
    fixture=importlib.util.spec_from_file_location('health_report_fixture',Path(__file__).with_name('test_localization_measurements.py'))
    F=importlib.util.module_from_spec(fixture);fixture.loader.exec_module(F)
    root=F.make_run(tmp_path)
    def onboard(meta):
        meta['roles']=A.normalize_roles(meta)
        meta['schema_version']=2;meta['platform']='onboard'
        for role in ('source_pose','ground_truth_pose','collision_status'):meta['roles'].pop(role,None)
    F.edit_metadata(root,onboard)
    before=A.analyze(root)
    (root/'pf_health_explanation.md').write_text('#### Diagnostic mechanism evidence\n\nREFERENCE PROFILE, NOT DEPLOYMENT PROOF\n')
    after=A.analyze(root)
    assert before['localization']==after['localization'] and before['vehicle']==after['vehicle']
    text=(root/'report.md').read_text()
    assert text.index('### PF health over the evaluation interval')<text.index('REFERENCE PROFILE, NOT DEPLOYMENT PROOF')<text.index('### Localization/odometry consistency')


def explained_rows():
    p,_=setup()
    rows=samples([(1,.8,0.),(2,.5,0.),(2,.5,0.),(3,.8,0.),(3,.8,0.),(2,.8,0.)])
    rows[2,3]=.15
    trace=[dict(reported=int(row[1]),predicted=int(row[1]),anchored=True,
                watchdog=(i==3),good_evidence=(row[2]>=.6 and row[3]<=.1),
                hard_invalid_evidence=False) for i,row in enumerate(rows)]
    pose=np.array([[row[0],float(i),0.,0.,row[0]+.01] for i,row in enumerate(rows)])
    return p,rows,trace,pose


def test_explanatory_modes_separate_instant_scan_from_persistent_invalid_state():
    p,rows,trace,pose=explained_rows()
    result=E.associate_explanations(pose,rows,trace,p)
    assert result[1]['scan_reason']=='match_only'
    assert result[2]['scan_reason']=='both'
    assert result[3]['scan_reason'] is None
    assert result[3]['mechanism']==result[4]['mechanism']=='watchdog'
    assert result[3]['good_evidence_now']
    assert result[5]['mechanism']=='recovery' and result[5]['status']=='DEGRADED'


@pytest.mark.parametrize('match,error,expected',[(.5,.08,'match_only'),(.8,.15,'range_only'),(.5,.15,'both'),(.8,.08,None)])
def test_scan_modes_use_declared_thresholds_without_changing_status(match,error,expected):
    p,rows,trace,pose=explained_rows();rows[1,2]=match;rows[1,3]=error
    item=E.associate_explanations(pose,rows,trace,p)[1]
    assert item['scan_reason']==expected and item['status']=='DEGRADED'


def test_unknown_mode_uses_only_recorded_poses_and_never_future_health():
    p,rows,trace,pose=explained_rows()
    pose=np.array([[-.02,0.,0.,0.,-.01],[.9,2.,3.,0.,.9]])
    rows=rows[:1];trace=trace[:1]
    result=E.associate_explanations(pose,rows,trace,p)
    assert len(result)==2 and all(x['mechanism']=='unknown' for x in result)
    assert all(x['health_index'] is None for x in result)
    assert result[1]['x_m']==2 and result[1]['y_m']==3


def test_unqualified_replay_does_not_invent_watchdog_cause():
    p,rows,trace,pose=explained_rows();trace[3]['anchored']=False
    item=E.associate_explanations(pose,rows,trace,p)[3]
    assert item['mechanism']=='unqualified_replay' and item['status']=='INVALID'


def test_failure_mode_map_is_deterministic_and_retains_sample_evidence(tmp_path):
    import json
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
    p,rows,trace,pose=explained_rows()
    root=tmp_path/'onboard';(root/'plots').mkdir(parents=True)
    group=tmp_path/'comparison';(group/'sim_section').mkdir(parents=True)
    (group/'sim_section/metadata.yaml').write_text('{}')
    data={'reference_path':np.array([[-1.,-1.],[6.,-1.],[6.,1.],[-1.,1.],[-1.,-1.]])}
    evidence=dict(transitions=[],source_sha256='reference-source',profile_sha256='reference-settings')
    first=E.render_failure_modes(root,group,pose,data,rows,trace,p,1.,evidence)
    png=(root/'plots/pf_health_failure_modes.png').read_bytes()
    second=E.render_failure_modes(root,group,pose,data,rows,trace,p,1.,evidence)
    assert first==second and png==(root/'plots/pf_health_failure_modes.png').read_bytes()
    actual=json.loads((root/'pf_health_failure_modes.json').read_text())
    assert len(actual['samples'])==len(pose)
    assert actual['counts']['watchdog']==2 and actual['counts']['recovery']==1
    assert 'omitted' in actual['map_context']


def test_spread_and_quality_are_paired_from_same_health_message_without_ranked_state_codes():
    p,rows,trace,pose=explained_rows()
    associated=E.associate_explanations(pose,rows,trace,p)
    evidence=E.spread_quality_evidence(rows,associated,np.arange(len(pose))*.1,1.,p)
    assert evidence['states']['GOOD']['samples']==1
    assert evidence['states']['INVALID']['samples']==2
    assert all(item['above_good_bound']==0 for item in evidence['states'].values())
    assert evidence['states']['INVALID']['max_m']==pytest.approx(.07)
    assert evidence['spread_vs_match']['value'] is None  # constant spread is not fabricated r=0
    assert all(item['xy_spread_m']==pytest.approx(.07) for item in evidence['samples'])


def test_missing_health_has_no_invented_spread_or_zero_fill():
    p,rows,trace,pose=explained_rows()
    pose=np.array([[-.02,1.,2.,0.,-.01],[.9,2.,3.,0.,.9]])
    associated=E.associate_explanations(pose,rows[:1],trace[:1],p)
    evidence=E.spread_quality_evidence(rows[:1],associated,[.2,.3],1.,p)
    assert evidence['unknown_pose_samples']==2
    assert all(item['xy_spread_m'] is None for item in evidence['samples'])
    assert evidence['spread_vs_pf_reference_deviation']['samples']==0
    assert evidence['spread_vs_pf_reference_deviation']['value'] is None


def test_descriptive_correlation_qualifies_constant_and_missing_pairs():
    assert E.descriptive_correlation([1,2,3],[3,2,1])['value']==pytest.approx(-1.)
    assert E.descriptive_correlation([1,2,3],[1,2,3])['value']==pytest.approx(1.)
    assert E.descriptive_correlation([1,1,1],[1,2,3])['value'] is None
    item=E.descriptive_correlation([1,np.nan,3],[1,2,3])
    assert item['samples']==2 and item['value'] is None


def test_spread_overlay_report_distinguishes_concentration_from_truth_and_state(tmp_path):
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
    p,rows,trace,pose=explained_rows()
    root=tmp_path/'onboard';(root/'plots').mkdir(parents=True)
    group=tmp_path/'comparison';(group/'sim_section').mkdir(parents=True)
    (group/'sim_section/metadata.yaml').write_text('{}')
    data={'reference_path':np.array([[-1.,-1.],[6.,-1.],[6.,1.],[-1.,1.],[-1.,-1.]])}
    before=rows.copy()
    evidence=E.render_spread_quality(root,group,pose,data,rows,trace,p,1.)
    text=E.spread_quality_fragment(evidence)
    assert 'side by side' in text and 'First full-size map' in text and 'Second full-size map' in text and 'Third full-size map' in text
    for term in ['weighted covariance','relative','not truth error','not independent','UNKNOWN','0.5 s','reference']:
        assert term in (text+' '+evidence['qualification']).lower() or term.lower() in text.lower()
    files=[root/'plots/pf_spread_quality_map.png',root/'plots/pf_spread_quality_relationships.png',root/'pf_spread_quality.json']
    initial=[path.read_bytes() for path in files]
    E.render_spread_quality(root,group,pose,data,rows,trace,p,1.)
    assert [path.read_bytes() for path in files]==initial
    assert np.array_equal(rows,before)
