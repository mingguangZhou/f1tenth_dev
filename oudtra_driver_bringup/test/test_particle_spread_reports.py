from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
import evaluation_particle_spread as P


def test_spread_statistics_are_particle_width_not_tracking_distance():
    result=P.summarize([[0,1,.06],[.1,2,.07],[.2,3,.08]],1.)
    assert result['overall']['p50_m']==pytest.approx(.07)
    assert result['by_quality']['INVALID']['max_m']==pytest.approx(.08)
    assert result['definition']=='sqrt(weighted_var_x + weighted_var_y)'
    assert result==P.summarize([[0,1,.06],[.1,2,.07],[.2,3,.08]],1.)


def test_missing_invalid_and_stale_spread_are_not_zero_filled():
    assert P.summarize([],1.)['status']=='UNAVAILABLE_DATA'
    assert P.summarize([[0,1,-1],[.1,2,np.nan]],1.)['invalid_message_count']==2
    joined=P.associate([[.1,1,.06],[.3,2,np.nan]],[0,.2,.4,1.])
    assert joined[0]['spread_m'] is None and joined[1]['spread_m']==pytest.approx(.06)
    assert joined[2]['spread_m'] is None and joined[3]['quality']=='UNKNOWN'


def test_weighted_cloud_example():
    prior=np.array([[-.02,0],[.02,0],[0,-.02],[0,.02]])
    predicted=np.array([[.03,0],[.17,0],[.10,-.06],[.10,.06]])
    for points,weights,expected in [(prior,np.full(4,.25),.02),(predicted,np.full(4,.25),np.sqrt(.00425)),(predicted,np.array([.1,.6,.15,.15]),np.sqrt(.003285))]:
        mu=np.sum(points*weights[:,None],axis=0)
        spread=np.sqrt(np.sum(weights*np.sum((points-mu)**2,axis=1)))
        assert spread==pytest.approx(expected)


def test_addendum_does_not_recreate_redundant_backups(tmp_path):
    import compare_matched_sections as M
    (tmp_path/'report.md').write_text('# R\n\n## Section\n')
    M.attach_report_fragment(tmp_path,'example.md','Explanation\n','## Section')
    M.attach_report_fragment(tmp_path,'example.md','Better explanation\n','## Section')
    assert 'Better explanation' in (tmp_path/'report.md').read_text()
    assert not list(tmp_path.glob('report_before*'))
