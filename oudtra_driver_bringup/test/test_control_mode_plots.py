"""Offline controller displays must preserve missing evidence and metric meaning."""
import sys
from pathlib import Path
import json
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import evaluation_control_plots as C
import compare_matched_sections as M


def row(time,state,reason='recorded reason'):
    return dict(receive_sec=time,source_sec=time-.01,state=state,reason=reason,fields={'pf_state':'3'})


def test_causal_status_rejects_future_and_stale_samples():
    records=[row(.2,'RACELINE')]
    assert C.held_record(records,.1,.5) is None
    assert C.held_record(records,.3,.5)['state']=='RACELINE'
    assert C.held_record(records,.71,.5) is None
    assert C.held_record([],.3,.5) is None


def test_joint_exposure_conserves_interval_and_keeps_unknown():
    result=C.exposure([row(0,'RACELINE'),row(.2,'REACTIVE')],[row(0,'GOOD'),row(.3,'INVALID')],1.,.5)
    assert sum(result['duration_sec'].values())==pytest.approx(1.)
    assert result['duration_sec']==pytest.approx({'RACELINE':.2,'REACTIVE':.5,'UNKNOWN':.3})
    assert result['joint_quality_sec']['REACTIVE']==pytest.approx({'GOOD':.1,'INVALID':.4})
    assert C.exposure([],[],1.,.5)['duration_sec']=={'UNKNOWN':1.}


def test_changes_do_not_count_reconnection_or_invent_missing_pose():
    records=[row(0,'RACELINE'),row(.2,'REACTIVE'),row(1.,'RACELINE')]
    pose=np.array([[0.,1.,2.,0.,0.]])
    result=C.observed_changes(records,pose,2.,.5)
    assert len(result)==1 and result[0]['reason']=='recorded reason'
    assert result[0]['position']['x_m']==1.
    result=C.observed_changes([row(0,'RACELINE'),row(.2,'STOP')],pose+np.array([-1.,0.,0.,0.,-1.]),2.,.5)
    assert result[0]['position'] is None


def test_control_maps_and_reports_are_deterministic_without_changing_metrics(tmp_path,monkeypatch):
    root=tmp_path/'onboard';(root/'plots').mkdir(parents=True)
    comparison=tmp_path/'comparison';(comparison/'sim_section').mkdir(parents=True)
    (comparison/'sim_section/metadata.yaml').write_text('{}')
    metrics=root/'metrics.json';metrics.write_text(json.dumps({'settings':{'max_status_gap_sec':.5}}))
    original=metrics.read_bytes()
    (root/'report.md').write_text('# Report\n\n![Tracking](plots/pf_reference_trajectory.png)\n')
    t=np.arange(0,1,.05);pose=np.c_[t,t+1,np.full(len(t),.1),np.zeros(len(t)),t]
    data={'reference_path':np.array([[0.,0.],[3.,0.],[3.,2.],[0.,2.],[0.,0.]]),'localization_health':[(0.,'GOOD'),(.6,'INVALID')]}
    metadata={'schema_version':2,'platform':'onboard','roles':{}}
    monkeypatch.setattr(M,'load_run',lambda path:(metadata,data,pose,0,1.))
    monkeypatch.setattr(C,'read_status',lambda root,roles,role,origin:[row(0.,'RACELINE'),row(.4,'REACTIVE','PF_INVALID: recorded')] if role=='control_status' else [])
    output=C.render_run(root,comparison)
    assert output['exposure']['duration_sec']['UNKNOWN']>.09
    assert all(item['safety']=='UNKNOWN' for item in output['samples'])
    text=(root/'report.md').read_text()
    assert 'not proof of onboard mux authority' in text and 'PF_INVALID' in text
    assert (root/'plots/control_pf_quality_map.png').exists()
    files=[root/'report.md',root/'control_mode_analysis.json',*sorted((root/'plots').glob('*.png'))]
    before=[p.read_bytes() for p in files]
    C.render_run(root,comparison)
    assert before==[p.read_bytes() for p in files] and metrics.read_bytes()==original


def test_visual_span_merging_removes_stripes_without_changing_evidence():
    source=[dict(start_sec=0.,end_sec=.1,state='RACELINE'),dict(start_sec=.1,end_sec=.2,state='RACELINE'),dict(start_sec=.2,end_sec=.4,state='REACTIVE')]
    original=json.loads(json.dumps(source))
    result=C.coalesce_states(source)
    assert len(result)==2 and result[0]['end_sec']==.2 and result[1]['state']=='REACTIVE'
    assert source==original
