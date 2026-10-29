import sys,json
from pathlib import Path
import numpy as np
from scipy.integrate import quad
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from drum_planner.timing import core
from drum_planner.common import atomic_json
base=Path(__file__).resolve().parent;rows=[]
for f in sorted(Path('diagnostics/fast_gpu_20261024/evidence').glob('*.json')):
 if f.name=='index.json':continue
 obj=json.loads(f.read_text());src=obj['source'];path=core.original(src);law=obj['law'];e=core.Evaluator(law)
 vel_lb=quad(lambda t:max(abs(path(t,nu=1)))/core.LIMITS[0],0,1,points=np.unique(path.x),epsabs=1e-8,limit=500)[0]
 distances=abs(path(1)-path(0));single_lb=max(core.scurve(float(d))[1][-1] for d in distances if d>1e-10)
 ts=np.linspace(0,e.duration,10001);q,v,a,j=e(ts)
 fraction={key:float(np.mean(np.max(abs(val),axis=1)>=lim*.98)) for key,val,lim in zip(('velocity','acceleration','jerk'),(v,a,j),core.LIMITS)}
 row=dict(name=f.stem,duration_s=e.duration,velocity_lb=vel_lb,joint_displacement_jerk_lb=single_lb,best_lb=max(vel_lb,single_lb),near_limit_fraction=fraction)
 rows.append(row);print(row,flush=True)
atomic_json(base/'bounds_initial.json',rows)
