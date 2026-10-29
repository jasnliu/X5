"""Independent root/1-ms native checks of saved real-camera experiment results."""
import json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.common import digest,read_state
from drum_planner.experiment.config import no_can
from drum_planner.experiment.runtime import Oracle
from drum_planner.experiment.algorithms import dense_check
from drum_planner.experiment import workspace_recovery as recovery
from drum_planner.timing import core,motion_quality
from drum_planner.timing.benchmark import check_geometry
D=Path(__file__).resolve().parent;no_can();output=D/'audit';output.mkdir(exist_ok=True);rows=[]
for root in ('final_primary','final_extra'):
 for case in json.loads((D/root/'summary.json').read_text()):
  assert case['status']=='planned',case
  p=Path(case['plan']);paths=sorted(p.parent.rglob('plan.json'))
  for path in paths:
   plan=json.loads(path.read_text())
   if plan.get('checkpoint_sequence'):continue
   assert plan['status']=='planned',path
   req=json.loads((path.parent/'backend_request.json').read_text())
   ctx=json.loads((path.parent/'context.json').read_text())
   acquired=read_state(ctx.get('acquired_state',ctx['state']))
   side=plan['moving_side'];passive='left' if side=='right' else 'right'
   assert req['scene_robot_start']==acquired[side+'_rad']
   assert req[passive]==acquired[passive+'_rad']
   assert req['scene']['synthetic'] is False and not req['execute']
   assert plan['motor_commands']==0 and not plan['can_access']
   assert not plan['experiment']['result_cache_used'] and not plan['experiment']['cpu_reference_used']
   law=plan['timing'];source=dict(request=req,timing=plan['source_timing'],path_waypoints=plan['path_waypoints'])
   geometry=check_geometry(law,source)
   pp=core.decode(law['path']);h=core.decode(law['density']);limits=np.array(law['limits'])
   peaks,_=core.density_peaks(pp,h)  # original independent, non-batched roots
   assert (peaks<=limits*(1+1e-9)).all()
   np.testing.assert_allclose(peaks,law['peaks'],atol=1e-8,rtol=1e-8)
   quality=motion_quality.law_metrics(law)
   assert quality['internal_stops']==quality['deep_slowdowns']==0
   folder=output/(root+'_'+case['case']+'_'+path.parent.name);folder.mkdir(exist_ok=True)
   oracle=Oracle(path.parent/'backend_request.json',folder)
   try:
    assert recovery.position_bounds(pp,oracle.lower,oracle.upper)
    endpoints=oracle.check([req['start'],req['goal']]);assert all(endpoints['valid'])
    fk=oracle.call('fk',q=[req['start'],req['goal']])
    t=np.unique(np.r_[np.arange(0,law['duration_s'],.001),core.event_times(law),law['duration_s']])
    q,v,a,j=core.evaluate(law,t)
    np.testing.assert_allclose(q[[0,-1]],[req['start'],req['goal']],atol=1e-10)
    assert np.max(abs(v[[0,-1]]))<1e-8 and np.max(abs(a[[0,-1]]))<1e-7
    ok,proof=dense_check(oracle,q);assert ok,proof
    row=dict(case=case['case'],plan=str(path),passed=True,scene_sha256=digest(ctx['scene']),
       state_sha256=digest(ctx.get('acquired_state',ctx['state'])),calibration_sha256=digest(ctx['calibration']),
       request_sha256=digest(path.parent/'backend_request.json'),geometry=geometry,quality=quality,
       limits=law['limits'],peaks=peaks.tolist(),maximum_limit_fraction=float((peaks/limits).max()),
       check_spacing_s=.001,collision_zone=proof,endpoints=endpoints,endpoint_fk=fk,
       exact_endpoints=True,continuous_position_extrema=True,original_camera_mask=True,
       opposite_arm_unchanged=True,hardware_access=False)
    rows.append(row);(output/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
    print('PASS',case['case'],path.parent.name,'duration',law['duration_s'],'samples',len(q),flush=True)
   finally:oracle.close()
# Formerly successful paths AND all timing coefficients must be retained.
regression=[]
for target in ('R1','R2'):
 name='reference_right_default_'+target
 a=next((D/'baseline'/name/'plans').glob('*/plan.json'));b=next((D/'final_primary'/name/'plans').glob('*/plan.json'))
 old=json.loads(a.read_text());new=json.loads(b.read_text())
 for key in ('path_waypoints','source_timing'):
  assert old[key]==new[key],(target,key)
 for key in ('path','density','duration_s','peaks'):
  assert old['timing'][key]==new['timing'][key],(target,key)
 regression.append(dict(case=name,path_and_clock_bit_identical=True))
(output/'reference_preservation.json').write_text(json.dumps(regression,indent=2)+'\n')
print('INDEPENDENT AUDIT PASSED',len(rows),flush=True)
