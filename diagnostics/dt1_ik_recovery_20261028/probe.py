"""Offline diagnostic of the exact failed model. No device access."""
import json,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.optimize._numdiff import approx_derivative
from cymbal_destination import solver,goal_limits
from drum_planner.fast_mode.kinematics import Kinematics
root=Path('planner_runs/generated-beat-20261028T083720-5ac2c7a8/plan-dd96e4f6/right/dt1/checkpoint_1')
req=json.loads((root/'backend_request.json').read_text());search=json.loads((root/'search.json').read_text())
kin=Kinematics(req['urdf'],'right');start=np.array(req['start']);point=np.array(search['target_m']);normal=np.array(search['disc_normal']);inward=np.array(search['inward_reference_world'])
print('BOUNDS_DEG',np.degrees(goal_limits.bounds(kin.lower,kin.upper)),flush=True)
rows=[];begin=time.monotonic()
for tilt in (0.,None):
 t=solver.Task(kin,point,normal,tilt,inward)
 for i,q in enumerate(solver.seeds(kin,start)):
  if i==0:print('JAC ERR',np.max(abs(t.jac(q)-approx_derivative(t.fun,q))),flush=True)
  fit=least_squares(t.fun,q,jac=t.jac,bounds=goal_limits.bounds(kin.lower,kin.upper),max_nfev=100,ftol=1e-10,xtol=1e-10,gtol=1e-10)
  g=solver.geometry(kin,fit.x,point,normal,inward)
  rows.append(dict(tilt=tilt,i=i,cost=fit.cost,nfev=fit.nfev,status=fit.status,optimality=fit.optimality,q=fit.x.tolist(),residual=t.fun(fit.x).tolist(),proof=g,acceptable=solver.acceptable(g)))
print('elapsed',time.monotonic()-begin)
for row in sorted(rows,key=lambda r:r['cost'])[:8]:print(json.dumps(row))
Path('diagnostics/dt1_ik_recovery_20261028/initial_probe.json').write_text(json.dumps(rows,indent=2))
