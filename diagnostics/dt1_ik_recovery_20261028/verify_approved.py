"""Offline proof readback on the two exact camera captures. No devices."""
import json
from pathlib import Path
import numpy as np
from camera_playback.generated_planning import load
from cymbal_destination import inputs,aim,solver,goal_limits,output
from drum_planner.common import ROOT,digest,atomic_json
from drum_planner.experiment.config import no_can
no_can()

def verify(folder,old_target):
 m,curves=load(folder/'mission.json');d=folder/'right/dt1'
 r=json.loads((d/'result.json').read_text());goal=json.loads(Path(r['resolved'][0]).read_text());output.current_convention(goal)
 point=goal['target_world_m'];target=json.loads((d/'vision/target.json').read_text());old=json.loads(old_target.read_text())
 assert point==old['above_target']['world_m']==target['above_target']['world_m']
 assert target['flat_disc']==old['flat_disc'] and target['target']==old['target']
 req=json.loads((folder/'right/backend_request.json').read_text());kin=solver.Kinematics(req['urdf'],'right')
 _,p,n=inputs.target(d/'vision/target.json',m['calibration']);u=aim.reference(target,n);q=np.array(goal['samples'][0]['positions_rad'])
 proof=solver.geometry(kin,q,p,n,u);assert solver.acceptable(proof);reserve=goal_limits.require(q,kin.lower,kin.upper)
 assert q[1]>=np.radians(1)-1e-10
 for side,c in curves.items():
  assert m['proofs'][side]['all_valid'] and m['return_plan']['proofs'][side]['all_valid']
  np.testing.assert_allclose(c.at(0),m['original'][side+'_rad'],atol=1e-12,rtol=0)
 assert m['order']==['left','right'] and m['return_plan']['order']==['right','left'] and m['motor_commands']==0
 np.testing.assert_allclose(curves['right'].at(curves['right'].duration),q,atol=1e-12,rtol=0)
 return dict(mission=str(folder/'mission.json'),ready=m['status'],duration_s=m['duration_s'],calculation_s=m['calculation_s'],
  unchanged_green_and_disc=True,geometry=proof,reserve=reserve,outward_checks=m['proofs'],return_checks=m['return_plan']['proofs'])

if __name__=='__main__':
 root=ROOT/'planner_runs/dt1-ik-recovery-20261028'
 cases=[('approved-saved',ROOT/'planner_runs/generated-beat-20261028T083720-5ac2c7a8/plan-dd96e4f6/right/dt1/vision/target.json'),
        ('approved-second-capture',ROOT/'planner_runs/generated-prepare-20261028T085139-7a80836a/plan/right/dt1/vision/target.json')]
 report={name:verify(root/name,old) for name,old in cases}
 baseline=json.loads((ROOT/'diagnostics/dt1_ik_recovery_20261028/approved_before_hashes.json').read_text())
 allowed={'cymbal_destination/config.py','cymbal_destination/output.py'}
 changed=[p for p,h in baseline.items() if digest(ROOT/p)!=h]
 assert set(changed)==allowed,changed
 report['preservation']=dict(checked=len(baseline),changed=changed,other_files_unchanged=True)
 atomic_json(ROOT/'diagnostics/dt1_ik_recovery_20261028/approved_verification.json',report)
 for name in dict(cases):
  r=report[name];print(name,'PASS',r['duration_s'],'calculation_s',r['calculation_s'],
      'tip_error_m',r['geometry']['tip_error_m'],'yaw_deg',r['geometry']['inward_yaw_deg'])
 print(report['preservation'])
