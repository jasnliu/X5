import diagnose as d
import json,numpy as np
from scipy.optimize import least_squares
OUT=d.OUT; rows=[];qt=[];qv=[];centers=[]
for s in d.training:
 batches=s['stationary_evidence']['encoder_batches'];qt.append(np.array([(b['start']+b['end'])/2 for b in batches]));qv.append(np.array([[b['joints'][k] for k in d.core.JOINTS] for b in batches]));centers.append(s['frame_header']['received_monotonic'])
low=max(t[0]-c for t,c in zip(qt,centers));high=min(t[-1]-c for t,c in zip(qt,centers))
for lag in np.linspace(low+.0001,high-.0001,41):
 q=np.array([[np.interp(c+lag,t,v[:,j]) for j in range(7)] for c,t,v in zip(centers,qt,qv)]);aa=d.fk(q)
 def fun(p):return d.rigid_residual(d.core._unpack(p[:6]),d.core._unpack(p[6:]),aa,d.B)
 f=least_squares(fun,d.seed,loss='soft_l1',max_nfev=150);c,x=d.core._unpack(f.x[:6]),d.core._unpack(f.x[6:]);m,_=d.summary(aa,d.B,c,x)
 rows.append(dict(lag_seconds=float(lag),metrics=m,cost=float(f.cost)))
best=min(rows,key=lambda x:x['cost']);print('Best grid timing hypothesis',json.dumps(best))
(OUT/'timing_grid.json').write_text(json.dumps(dict(best=best,all=rows),indent=2)+'\n')
