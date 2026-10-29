"""Diagnostic only: no destination export and no native/hardware connection."""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from cymbal_destination import solver,goal_limits,aim
from drum_planner.fast_mode.kinematics import Kinematics
root=Path('planner_runs/generated-beat-20261028T083720-5ac2c7a8/plan-dd96e4f6/right/dt1/checkpoint_1')
r=json.loads((root/'backend_request.json').read_text());s=json.loads((root/'search.json').read_text());kin=Kinematics(r['urdf'],'right')
p=np.array(s['target_m']);n=np.array(s['disc_normal']);u=np.array(s['inward_reference_world']);v=np.cross(n,u)
q=np.array(json.loads(Path('diagnostics/dt1_ik_recovery_20261028/initial_probe.json').read_text())[49]['q'])
t=solver.Task(kin,p,n,None,u);lo,hi=goal_limits.bounds(kin.lower,kin.upper);inds=[0,1,2,4]
# With all joint reserves, exact green tip, exact J7 roll and shaft-above,
# find nearest attainable signed inward yaw. Report only; NOT a permissible pose.
def yaw(x):
 f=kin.fk(x,jac=True);d=f['axis'];dx=d@u;dy=d@v
 return np.arctan2(dy,dx),(dx*(v@f['Jaxis'])-dy*(u@f['Jaxis']))/(dx*dx+dy*dy)
rows=[]
for seed in solver.seeds(kin,np.array(r['start'])):
 f=minimize(lambda x:yaw(x)[0]**2,seed,jac=lambda x:2*yaw(x)[0]*yaw(x)[1],bounds=list(zip(lo,hi)),method='SLSQP',constraints=[dict(type='eq',fun=lambda x:t.fun(x)[inds],jac=lambda x:t.jac(x)[inds]),dict(type='ineq',fun=t.above,jac=t.above_jac)],options=dict(maxiter=180,ftol=1e-11))
 g=solver.geometry(kin,f.x,p,n,u)
 if f.success and np.linalg.norm(t.fun(f.x)[inds])<1e-6 and g['negative_j7_toward_disc_cosine']>0:
  rows.append(dict(q=f.x.tolist(),geometry=g))
rows.sort(key=lambda r:abs(r['geometry']['inward_yaw_deg']))
print('minimum yaw diagnostic',json.dumps(rows[:1],indent=2));Path('diagnostics/dt1_ik_recovery_20261028/sensitivity.json').write_text(json.dumps(dict(diagnostic_only=True,not_a_destination=True,feasible_with_other_constraints=rows),indent=2))
