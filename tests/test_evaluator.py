import json
import math
from dataclasses import replace
from coverage_mission.config import Config
from coverage_mission.evaluate_run import evaluate_run


def test_gazebo_axis_oracle_and_false_completion_detection(tmp_path):
    cfg=replace(Config(),n_max=10.4,e_max=7.8)
    truth=tmp_path/'truth.jsonl'; mission=tmp_path/'mission.jsonl'
    fixture={'config':cfg.as_dict(),'zones':[]}
    truth.with_suffix('.fixture.json').write_text(json.dumps(fixture))
    poses=[{'t':i*.02,'n':0,'e':0,'z':10.003,
            'quaternion_xyzw':[0,0,math.sin(math.pi/4),math.cos(math.pi/4)]} for i in range(11)]
    truth.write_text('\n'.join(json.dumps(p) for p in poses))
    rows=[{'source_t':t,'frame_t':t,'observation_pose':{'t':t,'n':0,'e':0,'alt':10}}
          for t in (.04,.08,.12,.16)]
    mission.write_text('\n'.join(json.dumps(r) for r in rows))
    mission.with_suffix('.result.json').write_text(json.dumps({'state':'COMPLETE'}))
    r=evaluate_run(mission,truth)
    assert r['sampled_flight_safety_passed']
    assert r['max_projected_corner_error']<1e-10
    assert not r['sampled_geometry_coverage_passed']
    assert r['independently_unseen_permissible_cells']>0
