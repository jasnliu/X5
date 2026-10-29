"""Bounded native corridor escape when task-minimum/workspace curves are blocked."""
import time
import numpy as np
from .. import planning
from ..common import atomic_json
from ..experiment.algorithms import thin
from ..timing import rounding


def candidates(oracle,req,folder,log,seconds=8.):
    deadline=time.monotonic()+seconds;attempts=[]
    for trial,width in enumerate([.15,.35,None]):
        remaining=deadline-time.monotonic()
        if remaining<=0:break
        args=dict(start=req['start'],goal=req['goal'],seconds=min(2.,remaining))
        if width is not None:args['corridor_rad']=width
        log(f'Native clearance search {trial+1}/3; '+('full joint bounds' if width is None else f'corridor ±{np.degrees(width):.1f}°'))
        result=oracle.call('plan',**args)
        attempts.append(dict(corridor_rad=width,status=result['status'],raw=result))
        atomic_json(folder/'clearance_search.json',attempts)
        if result['status']!='planned':continue
        q=thin(np.array(result['path']),epsilon=1e-10)
        try:
            _,_,timing=planning.smooth_samples(q,False)
            yield 'native_corridor_quintic',dict(path_waypoints=q.tolist(),timing=timing,request=dict(req))
        except ValueError as exc:attempts[-1]['quintic_rejected']=str(exc)
        _,_,timing=planning.smooth_samples(q,True)
        raw=dict(path_waypoints=q.tolist(),timing=timing,request=dict(req))
        # Reuse established C3 blends and their explicit 1-degree / 1-cm
        # geometric deviation checks. Never play the raw corner-stop polyline.
        for fraction in [.45,.2,.08]:
            if time.monotonic()>deadline:break
            try:
                curve,_=rounding.propose(raw,'local_c3',fraction=fraction)
                valid,proof=rounding.inspect(curve,raw,oracle)
                attempts[-1].setdefault('blends',[]).append(dict(fraction=fraction,proof=proof))
                atomic_json(folder/'clearance_search.json',attempts)
                if valid:yield 'native_corridor_c3',rounding.as_source(curve,raw)
            except ValueError as exc:attempts[-1].setdefault('blend_errors',[]).append(str(exc))
