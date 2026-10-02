from dataclasses import replace
import pytest
from coverage_mission.config import Config
from coverage_mission.replay import run


@pytest.mark.parametrize('zones',[
    [],[(4,6,-.5,2)],[(3,4.5,-.5,1),(6,8,3,4.5)],[(8,9.5,0,1.2)],
],ids=['clear','central','multiple','row_end_open'])
def test_camera_to_complete_synthetic_regression(zones):
    cfg=replace(Config(),n_max=10.4,e_max=7.8)
    report,_,_=run(cfg,zones,max_seconds=200)
    assert report['state']=='COMPLETE'
    assert report['incursion_samples']==report['outside_samples']==0
    assert report['pending']==report['unseen_cells']==0


def test_no_checked_escape_is_not_success():
    cfg=replace(Config(),n_max=10.4,e_max=7.8)
    report,_,_=run(cfg,[(-2,11,-3,9)],max_seconds=50)
    assert report['state']=='BLOCKED'
    assert report['pending']>0
    assert any(e['event']=='DEADLINE_FAILURE' for e in report['residence_events'])


def test_tight_row_end_fine_grid_completes_without_smaller_margins():
    # Preserve the original narrow layout: .4 m body + .3 m uncertainty plus
    # the old .2 m grid inflation closed its side passage. Do not weaken margins
    # or delete the still-unobserved corner to make the mission claim COMPLETE.
    cfg=replace(Config(),n_max=10.4,e_max=7.8)
    report,_,_=run(cfg,[(8,9.5,-.4,1.2)],max_seconds=200)
    assert report['state']=='COMPLETE'
    assert report['incursion_samples']==report['outside_samples']==0
    assert report['unseen_cells']==0
