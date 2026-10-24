"""Explain remaining proposal failures, without changing the production planner."""
import json,time,sys
from pathlib import Path
import numpy as np
from scipy.interpolate import PPoly
from drum_planner import common,planning
from drum_planner.experiment import config,runtime,workspace_curve as wc,gesture
config.no_can()
D=Path(__file__).resolve().parent
folder=D/'remaining';folder.mkdir(exist_ok=True)
p=D/'full_calculate/plan/result/checkpoint_1/backend_request.json';req=json.loads(p.read_text())
oracle=runtime.Oracle(p,folder);rows=[]
try:
    ends=np.array(gesture.fk(oracle,np.array([req['start'],req['goal']]))['wrist_xyz_m'])
    for weight in wc.OPTIONS['prior_weights']:
      for offset in wc.arc_offsets(*ends,count=wc.OPTIONS['proposal_count']):
        q,fit=wc.CurveFit(oracle,req,offset,dict(wc.OPTIONS,prior_weight=weight)).solve()
        outward,fine,law=planning.smooth_samples(q);bad=[]
        for j,(c,b) in enumerate(zip(law['coefficients'],law['polynomial_breaks_unscaled_s'])):
            pp=PPoly(np.asarray(c),np.asarray(b));roots=pp.derivative().roots(extrapolate=False)
            roots=roots[np.isfinite(roots)&(roots>=pp.x[0])&(roots<=pp.x[-1])]
            val=pp(np.r_[pp.x,roots]);lo,hi=oracle.lower[j],oracle.upper[j]
            if val.min()<lo-1e-12 or val.max()>hi+1e-12:bad.append(dict(joint=j+1,min=float(val.min()),max=float(val.max()),lower=lo,upper=hi))
        row=dict(index=len(rows)+1,bounds=bad)
        if not bad:
            proof=oracle.check(fine,edges=True,stop=True)
            n=len(proof['valid']);row.update(first_rejection=proof.get('details'),sample_index=n-1,
                fraction=(n-1)/(len(fine)-1),position=fine[n-1].tolist())
            check=dict(req,check_states=[fine[n-1].tolist()]);result=planning.backend(check,folder/str(row['index']))
            row['native_details']=result['checks']
        rows.append(row);print(row,flush=True)
finally:oracle.close()
common.atomic_json(folder/'report.json',dict(software_only=True,motor_commands=0,candidates=rows))
