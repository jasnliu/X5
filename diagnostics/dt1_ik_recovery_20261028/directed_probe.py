"""Offline directed orientation feasibility experiment; constraints unchanged."""
import json,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.stats import qmc
from cymbal_destination import solver,goal_limits
from drum_planner.fast_mode.kinematics import Kinematics
root=Path('planner_runs/generated-beat-20261028T083720-5ac2c7a8/plan-dd96e4f6/right/dt1/checkpoint_1')
req=json.loads((root/'backend_request.json').read_text());s=json.loads((root/'search.json').read_text())
kin=Kinematics(req['urdf'],'right');start=np.array(req['start']);p=np.array(s['target_m']);n=np.array(s['disc_normal']);u=np.array(s['inward_reference_world']);v=np.cross(n,u)
lo,hi=goal_limits.bounds(kin.lower,kin.upper);low=np.r_[lo,np.radians(-20),0];high=np.r_[hi,np.radians(20),np.pi/2]
class Task:
 def calc(self,x):
  q=x[:7];yaw,elev=x[7:];f=kin.fk(q,jac=True)
  t=np.cos(yaw)*u+np.sin(yaw)*v;ty=-np.sin(yaw)*u+np.cos(yaw)*v
  d=np.cos(elev)*t-np.sin(elev)*n;a=np.cross(t,n)
  ad=f['Jrotation'][:,6];ja=np.cross(f['Jrotation'].T,ad).T;ja[:,6]=0
  residual=np.r_[(f['tip']-p)/.3,f['axis']-d,ad-a]
  jac=np.zeros((9,9));jac[:3,:7]=f['Jtip']/.3;jac[3:6,:7]=f['Jaxis'];jac[6:,:7]=ja
  jac[3:6,7]=-np.cos(elev)*ty;jac[3:6,8]=np.sin(elev)*t+np.cos(elev)*n;jac[6:,7]=-t
  return residual,jac
 def fun(self,x):return self.calc(x)[0]
 def jac(self,x):return self.calc(x)[1]
task=Task();rows=[];begin=time.monotonic()
seeds=solver.seeds(kin,start)+list(lo+qmc.Sobol(7,scramble=True,seed=73).random_base2(8)*(hi-lo))
for i,q in enumerate(seeds):
 f=kin.fk(q);yaw=np.clip(np.arctan2(f['axis']@v,f['axis']@u),low[7],high[7]);elev=np.clip(np.arcsin(np.clip(-f['axis']@n,-1,1)),0,np.pi/2)
 fit=least_squares(task.fun,np.r_[q,yaw,elev],jac=task.jac,bounds=(low,high),max_nfev=200,ftol=1e-11,xtol=1e-11,gtol=1e-11)
 proof=solver.geometry(kin,fit.x[:7],p,n,u)
 row=dict(i=i,cost=fit.cost,nfev=fit.nfev,q=fit.x[:7].tolist(),angles=np.degrees(fit.x[7:]).tolist(),proof=proof,acceptable=solver.acceptable(proof))
 rows.append(row)
 if row['acceptable']:
  print('FOUND',json.dumps(row),flush=True)
  if sum(r['acceptable'] for r in rows)>=5:break
print('ELAPSED',time.monotonic()-begin,'ATTEMPTS',len(rows),'FOUND',sum(r['acceptable'] for r in rows))
for row in sorted(rows,key=lambda r:r['cost'])[:2]:print(json.dumps(row))
Path('diagnostics/dt1_ik_recovery_20261028/directed_probe.json').write_text(json.dumps(rows,indent=2))
