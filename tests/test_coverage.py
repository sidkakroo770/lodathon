import math
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from coverage_mission.config import Config, Camera
from coverage_mission.geometry import Pose, PoseHistory, Projector, decode_image, red_regions, validated_hsv, UnusableImage
from coverage_mission.planning import GroundMap, CoveragePlan, astar
from coverage_mission.engine import Engine, Residence


def test_projection_known_offsets_and_heading():
    cfg=Config(); proj=Projector(cfg); c=cfg.camera
    p=Pose(0,12,7,10)
    pixels=[[320,240],[420,240],[320,340]]
    out=proj.project(pixels,p)
    assert out[0]==pytest.approx([12,7])
    assert out[1]==pytest.approx([12,7+1000/c.fx])
    assert out[2]==pytest.approx([12-1000/c.fx,7])
    rotated=proj.project(pixels,replace(p,yaw=math.pi/2))
    assert rotated[1]==pytest.approx([12-1000/c.fx,7])


def test_projection_roll_pitch_not_ignored():
    p=Pose(0,0,0,10,pitch=math.radians(5))
    out=Projector(Config()).project([[320,240]],p)
    assert out[0,0]==pytest.approx(10*math.tan(math.radians(5)))


def test_interpolation_yaw_wrap_and_missing_data():
    h=PoseHistory()
    h.add(Pose(1,0,0,10,yaw=math.radians(179)))
    h.add(Pose(1.1,1,0,10,yaw=math.radians(-179)))
    p=h.at(1.05)
    assert p.n==pytest.approx(.5)
    assert abs(p.yaw)==pytest.approx(math.pi)
    assert h.at(1.2) is None
    assert h.at(1.05,.01) is None
    with pytest.raises(ValueError): h.add(Pose(0,0,0,10))


def test_multi_zone_includes_smaller_centre_and_clipped_region():
    frame=np.zeros((480,640,3),np.uint8)
    frame[20:200,0:120]=(0,0,255)
    frame[200:260,280:340]=(0,0,255)
    _,polys=red_regions(frame,Config())
    assert len(polys)==2


def test_camera_health_and_mapping_share_unusable_image_rejection():
    cfg=Config()
    for value in (0,255):
        frame=np.full((480,640,3),value,np.uint8)
        with pytest.raises(UnusableImage): validated_hsv(frame,cfg)
        with pytest.raises(UnusableImage): red_regions(frame,cfg)
    frame=np.full((480,640,3),100,np.uint8)
    assert validated_hsv(frame,cfg).shape==frame.shape


def test_decoder_stride_and_reject_grey():
    rgb=np.array([[255,0,0,0,255,0,99,99]],np.uint8)
    msg=SimpleNamespace(width=2,height=1,step=8,data=rgb.tobytes(),pixel_format_type=3)
    assert decode_image(msg).tolist()==[[[0,0,255],[0,255,0]]]
    msg.pixel_format_type=1
    with pytest.raises(ValueError): decode_image(msg)


def test_spacing_lens_change_and_lane_end_obligations():
    cfg=Config(); p=CoveragePlan(cfg)
    assert p.spacing<3.52
    narrow=CoveragePlan(replace(cfg,camera=replace(cfg.camera,hfov=math.radians(20))))
    assert len(narrow.points)>len(p.points)
    g=GroundMap(cfg); g.observed[:]=True; g.refresh()
    # Old FIND_NEXT would skip all later rows just beyond the first row end.
    end=p.points[np.flatnonzero(np.array(p.lanes)==0)[-1]]
    p.update(Pose(1,end[0]+.1,end[1],10),g)
    later=np.array(p.lanes)>0
    assert not np.any(p.done[later])
    assert len(p.pending())>100


def test_red_persists_after_leaving_image_and_duplicate_ignored():
    g=GroundMap(Config())
    foot=np.array([[-1,-1],[4,-1],[4,3],[-1,3]])
    red=np.array([[1,0],[2,0],[2,1],[1,1]])
    assert g.observe(foot,[red],1)
    before=g.red.copy()
    assert not g.observe(foot,[],1)
    g.observe(foot,[],2)
    assert np.array_equal(g.red,before)


def test_astar_no_corner_cutting_and_detours():
    mask=np.ones((10,10),bool); mask[1:9,5]=False
    path=astar(mask,(4,2),(4,8))
    assert path and all(mask[p] for p in path)
    assert any(p[0] in (0,9) for p in path)
    mask=np.array([[True,False],[False,True]])
    assert astar(mask,(0,0),(1,1))==[]


def test_unknown_and_field_edges_block_motion():
    g=GroundMap(Config()); g.refresh()
    assert not g.line_clear([0,0],[1,0])
    g.observed[:]=True; g.refresh()
    assert g.line_clear([0,0],[1,0])
    assert not g.line_clear([0,0],[-10,0])


def test_motion_rejection_grace_needs_live_camera_and_never_credits_coverage():
    def prepared():
        engine=Engine(Config())
        engine.ground.observed[:]=True
        engine.ground.last_t=1.
        engine.ground.refresh()
        return engine
    engine=prepared()
    before=engine.plan.done.copy()
    decision=engine.step(Pose(1.5,0,0,10),camera_t=1.45)
    assert decision.reason!='Stale/unmatched camera frame'
    assert np.array_equal(engine.plan.done,before)
    assert engine.ground.last_t==1.
    assert prepared().step(Pose(1.5,0,0,10)).reason=='Stale/unmatched camera frame'
    assert prepared().step(Pose(1.5,0,0,10),camera_t=1.1).state=='HOLD'
    assert prepared().step(Pose(2.01,0,0,10),camera_t=2.).state=='HOLD'


def test_repair_retains_viewpoint_then_requires_settled_new_observation(monkeypatch):
    import coverage_mission.engine as module
    engine=Engine(Config())
    engine.ground.observed[:]=True; engine.ground.observed[-1,-1]=False
    engine.ground.refresh(); engine.ground.last_t=1.
    engine.plan.done[:]=True
    engine.path=[np.array([1.,0.])]; engine.repair_viewpoint=True
    calls=[]
    def new_route(*args):
        calls.append(True)
        return [np.array([2.,0.])]
    monkeypatch.setattr(module,'route',new_route)
    engine.step(Pose(1,0,0,10),camera_t=1.)
    assert not calls and engine.path[-1].tolist()==[1.,0.]
    # Isolate arrival/observation behavior from the separate origin-jump guard.
    engine.last_pose=None; engine.path=[]
    engine.ground.last_t=1.1
    assert engine.step(Pose(1.1,1,0,10),camera_t=1.1).state=='OBSERVE'
    engine.ground.last_t=1.4
    assert engine.step(Pose(1.7,1,0,10),camera_t=1.7).state=='OBSERVE'
    assert not calls  # Fresh pixels alone do not replace an admitted projection.
    engine.ground.last_t=1.8
    engine.step(Pose(1.8,1,0,10),camera_t=1.8)
    assert calls


def test_residence_warning_deadline_late_detection_and_invalid_data():
    r=Residence()
    r.update(2,True,0)
    r.update(4.99,True)
    assert not r.warned
    assert r.update(5,True)==5
    assert r.warned
    r.update(8,None)
    assert r.entered==0
    r.update(10,True)
    assert r.failed
    r.update(11,False)
    assert r.failed and r.entered is None
    late=Residence(); late.update(8,True,1)
    assert late.warned and late.remaining(8)==3
    with pytest.raises(ValueError): late.update(2,True)


def test_early_exit_and_exact_deadline():
    r=Residence(); r.update(0,True); r.update(3,False)
    assert not r.warned and not r.failed
    r=Residence(); r.update(0,True); r.update(10,False)
    assert not r.failed
    assert not any(e['event']=='AUTONOMOUS_WARNING' for e in r.events)
    late=Residence(); late.update(0,True); late.update(10.1,False)
    assert late.failed and any(e['event']=='DEADLINE_FAILURE' for e in late.events)


def test_stale_frame_brakes_and_no_false_completion():
    e=Engine(Config()); p=Pose(1,0,0,10)
    frame=np.full((480,640,3),100,np.uint8)
    e.observe(frame,p); e.step(p)
    d=e.step(replace(p,t=1.5))
    assert (d.vn,d.ve)==(0,0)
    assert d.state=='HOLD' and e.terminal is None


def test_fence_and_origin_reset_abort():
    e=Engine(Config()); e.step(Pose(1,0,0,10))
    assert e.step(Pose(1.1,20,0,10)).state=='ABORTED'


def test_escape_and_late_entry_backdating():
    cfg=replace(Config(),n_max=10,e_max=8)
    e=Engine(cfg); g=e.ground
    g.observed[:]=True
    for t in np.arange(0,7,.1):
        e.history.append(Pose(float(t),float(t)/4,0,10))
    zone=np.array([[.5,-1],[2.5,-1],[2.5,1],[.5,1]])
    g.red|=g.raster(zone).astype(bool); g.refresh(); g.last_t=7
    e.turn_braking=True
    d=e.step(Pose(7,1.75,0,10))
    assert d.state=='ESCAPE'
    assert not e.turn_braking  # Residence recovery takes priority over lane turning.
    assert d.entered<2 and d.warned
    assert e.path and g.free[g.cell(e.path[-1])]


def test_escape_endpoint_tolerance_does_not_claim_clearance():
    cfg=replace(Config(),n_max=10.4,e_max=7.8)
    e=Engine(cfg); g=e.ground; g.observed[:]=True
    g.red|=g.raster([[4,0],[6,0],[6,2],[4,2]]).astype(bool); g.refresh(); g.last_t=1
    # Find an unsafe cell adjacent to free space, then approach its safe edge.
    for c in np.argwhere(g.inflated & g.inset):
        nxt=tuple(c+[0,1])
        if g.contains(nxt) and g.free[nxt]:
            xy=g.point(c)+[0,cfg.resolution/2-.004]
            path=e._escape(xy)
            if len(path)!=1 or np.linalg.norm(path[0]-xy)>=.15: continue
            p=Pose(1,*xy,10)
            d=e.step(p)
            assert d.state=='ESCAPE' and math.hypot(d.vn,d.ve)>0
            return
    pytest.fail('No test boundary found')


def test_camera_loss_keeps_known_incursion_recovery_and_timer_alive():
    cfg=replace(Config(),n_max=10.4,e_max=7.8)
    e=Engine(cfg); g=e.ground; g.observed[:]=True
    g.red|=g.raster([[1,-1],[2,-1],[2,1],[1,1]]).astype(bool); g.refresh(); g.last_t=0
    d=e.step(Pose(1,1.5,0,10))
    assert d.state=='ESCAPE' and math.hypot(d.vn,d.ve)>0
    d=e.step(Pose(6,1.5,0,10))
    assert d.warned and d.state=='ESCAPE'
    d=e.step(Pose(11,1.5,0,10))
    assert d.failed and d.state=='ESCAPE'  # Still recover, never claim success.
    assert not e.plan.done.any()  # Stale data never earns coverage.
