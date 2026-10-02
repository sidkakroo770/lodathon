from types import SimpleNamespace
import math
import pytest
from coverage_mission.config import Config
from coverage_mission.geometry import Pose, PoseHistory
from coverage_mission.runtime import TelemetryHistory, SimClock, matched_frame, current_pose, projection_motion_valid, job_due


def message(kind,t,**values):
    return SimpleNamespace(get_type=lambda:kind,time_boot_ms=round(t*1000),**values)


def test_join_uses_source_time_and_waits_for_brackets():
    h=TelemetryHistory()
    for t,alt,yaw in ((1,9000,math.radians(179)),(1.1,11000,math.radians(-179))):
        h.add(message('ATTITUDE',t,roll=0,pitch=0,yaw=yaw))
        h.add(message('GLOBAL_POSITION_INT',t,relative_alt=alt))
    h.add(message('LOCAL_POSITION_NED',1.05,x=4,y=5,vx=.2,vy=0,vz=0))
    h.add(message('LOCAL_POSITION_NED',1.15,x=6,y=5,vx=.2,vy=0,vz=0))
    h.assemble(10,.15)
    assert len(h.poses.samples)==1
    p=h.poses.samples[0]
    assert p.t==pytest.approx(11.05) and p.alt==pytest.approx(10)
    assert abs(p.yaw)==pytest.approx(math.pi)
    assert len(h.pending)==1  # Never extrapolate the unmatched sample.


def test_latest_unbracketed_frame_does_not_starve_processing():
    h=PoseHistory(); h.add(Pose(1,0,0,10)); h.add(Pose(1.1,1,0,10))
    frames=[(1.05,10,None,1),(1.12,10,None,2)]
    frame,pose=matched_frame(frames,h,1.14,Config())
    assert frame[3]==1 and pose.n==pytest.approx(.5)
    assert matched_frame(frames,h,2,Config())==(None,None)


def test_current_pose_never_has_a_future_timestamp():
    h=PoseHistory(); h.add(Pose(1,0,0,10)); h.add(Pose(1.1,1,0,10))
    p=current_pose(h,1.09,.15)
    assert p.t==1.09 and p.n==pytest.approx(.9)
    assert current_pose(h,.9,.15) is None
    assert current_pose(h,1.2,.15).t==1.1  # No forward extrapolation either.


def test_projection_motion_gate_needs_two_sided_slow_attitude():
    h=PoseHistory()
    for t,pitch in [(1,0),(1.05,.01),(1.1,0)]: h.add(Pose(t,0,0,10,pitch=pitch))
    assert not projection_motion_valid(h,h.at(1.05),Config())
    h=PoseHistory()
    for t in (1,1.05,1.1): h.add(Pose(t,0,0,10,yaw=math.radians(2)))
    assert projection_motion_valid(h,h.at(1.05),Config())  # Bias not erased or detected by this gate.
    assert not projection_motion_valid(h,h.at(1),Config())  # Missing past bracket.


def test_motion_admission_cache_waits_for_future_pose_and_stays_bounded():
    h=PoseHistory(); h.add(Pose(1,0,0,10)); h.add(Pose(1.05,0,0,10))
    frames=[(1.05,0,None,1)]; cache={}
    assert matched_frame(frames,h,1.08,Config(),cache)==(None,None)
    assert not cache
    h.add(Pose(1.1,0,0,10))
    assert matched_frame(frames,h,1.12,Config(),cache)[0][3]==1
    assert len(cache)==1
    assert matched_frame([],h,1.2,Config(),cache)==(None,None) and not cache


def test_projection_rejects_combined_tilt_even_when_angular_rate_is_zero():
    h=PoseHistory()
    for t in (1,1.05,1.1):
        h.add(Pose(t,0,0,10,roll=math.radians(4),pitch=math.radians(4)))
    assert not projection_motion_valid(h,h.at(1.05),Config())
    h=PoseHistory()
    for t in (1,1.05,1.1): h.add(Pose(t,0,0,10,pitch=math.radians(3)))
    assert projection_motion_valid(h,h.at(1.05),Config())


def test_worker_schedule_preserves_source_rate_with_bounded_wall_work():
    cfg=Config()
    assert job_due(.05,0,.05,0,cfg)  # Real time / Pi.
    assert job_due(.025,0,.05,0,cfg)  # Gazebo 2x.
    assert not job_due(.025,0,.025,0,cfg)
    assert not job_due(.01,0,.1,0,cfg)  # CPU bound at extreme acceleration.
    assert not job_due(5,0,0,0,cfg)  # Pause cannot generate fake progress ticks.


def test_clock_offset_validation_and_reset():
    clock=SimClock()
    for i in range(60): clock.add(i*.02,5+i*.02)
    assert clock.ready and clock.offset==pytest.approx(5)
    clock.add(1.2,6.4)
    assert not clock.ready  # An abrupt offset change is not followed.
    with pytest.raises(ValueError): clock.add(.1,5.1)


def test_clock_tracks_small_rate_drift_but_fixed_offset_does_not():
    clock=SimClock(); fixed=SimClock(5)
    for i in range(15000):
        boot=i*.02; sim=5+boot*1.0003
        clock.add(boot,sim); fixed.add(boot,sim)
    assert clock.ready and clock.error<.002
    assert not fixed.ready
