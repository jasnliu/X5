"""Explicit saved-input independent proof; no camera or CAN. Run in runtime.Job."""
import json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
import numpy as np
from drum_planner.common import atomic_json,endpoint,digest,read_state
from drum_planner.experiment import motion_timing,rgbd
from drum_planner.experiment.config import no_can
from drum_planner.read_only import ALLOWED
from cymbal_destination import solver,aim,inputs
no_can();base=Path(__file__).resolve().parent
report=dict(motor_commands=0,camera_access=False,can_access=False,plans=[],endpoints=[])
for name in ('fresh_right','native_ui_final/right','checkpoints'):
    session=base/name;context=json.loads((session/'context.json').read_text())
    proof=context['dt1_generation'];target,green,normal=inputs.target(
        Path(proof['resolved'][-1]).parents[1]/'vision/target.json',context['calibration'])
    assert proof['identity']['scene_sha256']==context['scene_sha256']
    scene=rgbd.scene(context['scene'],context['calibration'])
    assert scene['source_scene_sha256']==proof['identity']['scene_sha256']
    for goal in proof['goals']:
        q=endpoint(goal['path'],side='right')
        req=json.loads((Path(goal['path']).parent/'backend_request.json').read_text())
        kin=solver.Kinematics(req['urdf'],'right')
        geometry=solver.geometry(kin,q,green,normal,aim.reference(target,normal))
        assert solver.acceptable(geometry)
        assert req['scene_robot_start']==read_state(context['acquired_state'])['right_rad']
        assert req['left']==read_state(context['acquired_state'])['left_rad']
        np.testing.assert_array_equal(json.loads(Path(goal['path']).read_text())['target_world_m'],green)
        f=kin.fk(q,jac=True);direction=f['axis']-normal*(f['axis']@normal)
        direction/=np.linalg.norm(direction);radial=aim.reference(target,normal)
        angle=float(np.degrees(np.arccos(np.clip(direction@radial,-1,1))))
        assert direction@radial>0 and angle<=20.+1e-6
        report['endpoints'].append(dict(session=name,geometry=geometry,independent_yaw_deg=angle))
    # Verify the final/current selected scene. Prior scene plans remain intact.
    for plan_path in sorted((session/'plans').rglob('plan.json')):
        plan=json.loads(plan_path.read_text())
        if plan['status']!='planned' or plan.get('checkpoint_sequence'):continue
        if plan['provenance']['scene']!=context['scene']:continue
        req=json.loads((plan_path.parent/'backend_request.json').read_text())
        source=dict(path_waypoints=plan['path_waypoints'],timing=plan['source_timing'],request=req)
        out=base/'independent'/name.replace('/','_')/plan_path.parent.name
        proof2=motion_timing.validate(plan['timing'],source,out)
        report['plans'].append(dict(path=str(plan_path),duration_s=plan['duration_outward_s'],verification=proof2))
acq=base/'fresh_right/acquisition';a=json.loads((acq/'acquisition_report.json').read_text())
assert a['passed'] and a['camera_released'] and a['telemetry']['thread_stopped']
packets=[json.loads(line) for line in (acq/'can_query_audit.jsonl').read_text().splitlines()]
sent=[p for p in packets if p['event']=='sent']
assert sent and all(bytes.fromhex(p['packet_hex']) in ALLOWED for p in packets)
assert {p['operation'] for p in packets}=={'state_request_only'}
report['query_audit']=dict(sent=len(sent),all_packets_exact_query_allowlist=True,observer_closed=a['telemetry']['observer']['closed'])
assert len(report['plans'])==5 and len(report['endpoints'])==3
report['passed']=True;atomic_json(base/'independent_verification.json',report)
print(json.dumps(dict(passed=True,plans=len(report['plans']),endpoints=len(report['endpoints']),query_audit=report['query_audit']),indent=2))
