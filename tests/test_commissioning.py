from dataclasses import replace
from types import SimpleNamespace
import time
import math
import numpy as np
import pytest
from coverage_mission.config import Config, Camera
from coverage_mission.geometry import Pose, Projector, red_regions, UnusableImage
from coverage_mission.planning import GroundMap
from coverage_mission.engine import Engine
from coverage_mission.runtime import RoundTripClock, clock_diagnostics
from coverage_mission.faults import Faults
from coverage_mission.fixture import build
from coverage_mission.planning import astar
import xml.etree.ElementTree as ET
import json


def test_round_trip_offset_and_slow_response_rejection():
    c=RoundTripClock()
    for i in range(30):
        c.pending[i]=(5+i*.05,time.monotonic())
        c.reply(SimpleNamespace(ts1=i,tc1=round((i*.05+.005)*1e9)),5+i*.05+.01)
    assert c.ready and c.offset==pytest.approx(5)
    c.pending[100]=(7,time.monotonic())
    c.reply(SimpleNamespace(ts1=100,tc1=1),7.2)
    assert c.offset==pytest.approx(5)
    c.pending[101]=(7,time.monotonic())
    c.reply(SimpleNamespace(ts1=101,tc1=100000000),7.01)
    assert c.ready and c.offset==pytest.approx(5)
    c.last_reply-=3
    assert not c.ready


def test_pause_spanning_clock_exchange_cannot_poison_new_samples():
    c=RoundTripClock()
    for i in range(30):
        c.pending[i]=(5+i*.05,time.monotonic())
        c.reply(SimpleNamespace(ts1=i,tc1=round((i*.05+.005)*1e9)),5+i*.05+.01)
    c.pending[100]=(7,time.monotonic()-3)
    c.reply(SimpleNamespace(ts1=100,tc1=1965000000),7.01)
    assert c.ready and c.offset==pytest.approx(5)
    c.pending[101]=(7,time.monotonic())
    c.reply(SimpleNamespace(ts1=101,tc1=1965000000),7.01)
    assert not c.ready
    assert json.loads(json.dumps(clock_diagnostics(c),allow_nan=False))['clock_error'] is None
    c.pending[102]=(7.05,time.monotonic())
    c.reply(SimpleNamespace(ts1=102,tc1=2055000000),7.06)
    assert c.ready and c.offset==pytest.approx(5)


def test_camera_intrinsic_asymmetry_and_principal_point():
    camera=Camera(calibrated_fx=800,calibrated_fy=900,calibrated_cx=300,calibrated_cy=250)
    cfg=replace(Config(),camera=camera)
    xy=Projector(cfg).project([[300,250],[380,250],[300,340]],Pose(0,0,0,10))
    assert xy==pytest.approx(np.array([[0,0],[0,1],[-1,0]]))
    with pytest.raises(ValueError): replace(cfg,camera=replace(camera,calibrated_fx=-1))


def test_home_height_registration_changes_projection_not_control_altitude():
    cfg=replace(Config(),ground_above_home=-.192)
    p=Pose(0,0,0,10)
    assert Projector(cfg).footprint(p)==pytest.approx(Projector(Config()).footprint(p)*1.0192)
    assert cfg.altitude==p.alt==10  # Do not climb higher to correct the ground datum.


def test_confirmed_map_and_transient_red_reobservation():
    g=GroundMap(Config()); foot=[[-1,-2],[5,-2],[5,5],[-1,5]]; red=[[1,0],[2,0],[2,1],[1,1]]
    g.observe(foot,[red],1)
    assert g.red.any() and not g.confirmed.any()
    assert not g.observe(foot,[red],1)  # Cached frame cannot provide confirmation.
    for t in [2,3,4]: g.observe(foot,[],t)
    assert not g.red.any()
    g.observe(foot,[red],5); g.observe(foot,[red],6)
    assert g.confirmed.any()
    for t in range(7,12): g.observe(foot,[],t)
    assert g.red.any()


@pytest.mark.parametrize('level',[0,255])
def test_dead_or_overexposed_camera_is_not_free_space(level):
    with pytest.raises(UnusableImage): red_regions(np.full((480,640,3),level,np.uint8),Config())


def test_estimator_position_and_yaw_reset_abort():
    for p in [Pose(1.05,.5,0,10),Pose(1.05,0,0,10,yaw=math.pi/2)]:
        e=Engine(Config()); e.step(Pose(1,0,0,10))
        assert e.step(p).state=='ABORTED'


def test_fault_injection_requires_explicit_harness(tmp_path,monkeypatch):
    schedule=tmp_path/'faults.json'; schedule.write_text('[]')
    monkeypatch.delenv('COVERAGE_TEST_INJECTION',raising=False)
    with pytest.raises(ValueError): Faults(schedule)
    monkeypatch.setenv('COVERAGE_TEST_INJECTION','1')
    assert Faults(schedule).active(0)=={}
    schedule.write_text('[{"kind":"unknown","start":0,"duration":1}]')
    with pytest.raises(ValueError): Faults(schedule)


def test_fixture_pins_own_model_and_isolates_ports(tmp_path):
    cfg=tmp_path/'config.json'; cfg.write_text(json.dumps(Config().as_dict()))
    build(cfg,tmp_path/'fixture',instance=1)
    world=ET.parse(tmp_path/'fixture/coverage.sdf').getroot()
    uri=world.find('world/include/uri').text
    model=ET.parse(uri+'/model.sdf').getroot()
    assert model.find('model/plugin/fdm_port_in').text=='9012'
    assert uri==str(tmp_path/'fixture/models/iris_coverage')
    cfg.write_text(json.dumps(replace(Config(),camera=Camera(calibrated_fy=900)).as_dict()))
    with pytest.raises(ValueError): build(cfg,tmp_path/'unsupported')


def test_astar_prefers_tracking_room_without_changing_free_space():
    mask=np.ones((9,9),bool); mask[2:7,4]=False
    clearance=np.ones((9,9)); clearance[:2,:]=.05
    path=astar(mask,(4,1),(4,7),clearance)
    assert path and max(p[0] for p in path)>=7
    assert all(mask[p] for p in path)


def test_sharp_connector_turn_brakes_until_measured_settle():
    e=Engine(replace(Config(),n_max=10.4,e_max=7.8))
    e.ground.observe([[-10,-10],[20,-10],[20,20],[-10,20]],[],1)
    e.path=[np.array([5.,1.])]; e.last_plan=1
    e.last_velocity=np.array([0.,.6])
    d=e.step(Pose(1,1,1,10,ve=.6))
    assert d.state=='TURN_BRAKE' and d.vn==d.ve==0
    e.ground.last_t=1.1
    d=e.step(Pose(1.1,1,1.03,10,ve=.15))
    assert d.state=='TURN_BRAKE' and d.vn==d.ve==0
    e.ground.last_t=1.2
    d=e.step(Pose(1.2,1,1.04,10,ve=.03))
    assert d.state=='TURN_BRAKE'
    e.ground.last_t=1.55; e.last_plan=1.55
    d=e.step(Pose(1.55,1,1.04,10,ve=.03))
    assert d.state=='SWEEP' and d.vn>0
