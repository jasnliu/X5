"""One shared, from-scratch, nominal candidate pool for all five rankings."""
import hashlib,json,time
from pathlib import Path
import numpy as np
from ..common import ROOT,atomic_json,digest
from ..planning import smooth_samples
from .. import style
from .config import SCHEMA
from .fluidity import resample,measure,rank


def thin(q,epsilon=.006):
    """RDP in joint space only; any changed curve is independently rechecked."""
    q=np.asarray(q,dtype=float)
    if len(q)<=2:return q
    d=q[-1]-q[0];u=np.clip((q-q[0])@d/max(d@d,1e-20),0,1)
    distance=np.linalg.norm(q-(q[0]+u[:,None]*d),axis=1);k=int(np.argmax(distance))
    if distance[k]<=epsilon:return q[[0,-1]]
    return np.vstack((thin(q[:k+1],epsilon)[:-1],thin(q[k:],epsilon)))


def dense_check(oracle,q,**kwargs):
    count=0
    for start in range(0,len(q),4000):
        part=q[max(0,start-1):start+4000]
        result=oracle.check(part,edges=True,stop=True,**kwargs);count+=len(result['valid'])
        if not all(result['valid']) or len(result['valid'])!=len(part):return False,dict(samples_checked=count,details=result.get('details'))
    return True,dict(samples_checked=count,edge_joint_l1_step_rad=.003,max_time_step_s=.002)


def timed_candidate(oracle,q,req,index,log,params,stop=False):
    q=np.asarray(q,dtype=float)
    if q.ndim!=2 or q.shape[1]!=7 or len(q)<2 or not np.isfinite(q).all():return None
    if np.max(abs(q[0]-req['start']))>2e-7 or np.max(abs(q[-1]-req['goal']))>2e-7:return None
    q=q.copy();q[0]=req['start'];q[-1]=req['goal']
    try:points,fine,timing=smooth_samples(q,stop)
    except ValueError as exc:log(f'candidate {index}: timing rejected: {exc}');return None
    from scipy.interpolate import PPoly
    for j,(coeff,breaks) in enumerate(zip(timing['coefficients'],timing['polynomial_breaks_unscaled_s'])):
        pp=PPoly(np.asarray(coeff),np.asarray(breaks),extrapolate=False)
        roots=pp.derivative().roots(extrapolate=False)
        roots=roots[np.isfinite(roots)&(roots>=pp.x[0])&(roots<=pp.x[-1])]
        values=pp(np.r_[pp.x,roots])
        if values.min()<oracle.lower[j]-1e-12 or values.max()>oracle.upper[j]+1e-12:
            log(f'candidate {index}: rejected by continuous joint bounds');return None
    ok,proof=dense_check(oracle,fine)
    if not ok:
        log(f'candidate {index}: rejected by final zone/collision check: {proof.get("details")}');return None
    samples=style.sample_path(fine)
    features=oracle.call('style',paths=[samples.tolist()])['rows'][0]
    metrics=measure(samples,features,params)
    log(f'candidate {index}: VALID; {timing["duration_s"]:.2f}s; reversals={metrics["reversals"]["count"]}; '
        f'coordination={metrics["values"]["coordination"]:.3f}; spatial={metrics["values"]["spatial"]:.3f}')
    return dict(index=index,path=q.tolist(),outward=points,timing=timing,proof=proof,metrics=metrics,
                duration_s=timing['duration_s'])


def pool_key(context,req,params,seconds):
    # Include algorithm/model/scene contents, not temporary per-method paths.
    sources=['experiment/algorithms.py','experiment/fluidity.py','experiment/config.py',
             'experiment/worker.py','planning.py','style.py','model.py','right_zone.py','common.py',
             'cpp/experiment_oracle.cpp','cpp/moveit_plan.cpp','cpp/CMakeLists.txt']
    data=dict(schema=SCHEMA,scene=context['scene_sha256'],state=context['state_sha256'],
              calibration=digest(context['calibration']),urdf=digest(req['urdf']),srdf=digest(req['srdf']),
              goal=req['goal'],zone=req['right_zone']['constraint_sha256'],params=params,budget_s=seconds,
              collision_policy=req['collision_policy'],sources={n:digest(ROOT/'drum_planner'/n) for n in sources})
    return hashlib.sha256(json.dumps(data,sort_keys=True,allow_nan=False).encode()).hexdigest()


def read_pool(folder,key):
    path=Path(folder)/'pool.json'
    if not path.exists():return None
    obj=json.loads(path.read_text())
    if obj['schema']!=SCHEMA or obj['key']!=key or not obj['complete']:raise ValueError('Stale/incomplete candidate pool')
    for row in obj['candidates']:
        p=(Path(folder)/row['file']).resolve()
        if p.parent!=Path(folder).resolve() or digest(p)!=row['sha256']:raise ValueError('Candidate cache hash mismatch')
    return obj


def build_pool(oracle,req,folder,key,params,seconds,log):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    started=time.monotonic();deadline=started+seconds;rows=[];attempts=[];counter=0
    endpoints=oracle.check([req['start'],req['goal']])
    result=dict(schema=SCHEMA,key=key,complete=True,candidates=rows,attempts=attempts,
                budget_s=seconds,joint_limit_check=oracle.meta['joint_limit_check'],
                generation='Independent RRTConnect proposals; exact endpoints only; identical nominal collision/zone rules',
                duration_ratio_filter=None)
    if not all(endpoints['valid']):result.update(reason='nominal_endpoint_invalid',details=endpoints)
    else:
        for trial in range(params['candidates']):
            remaining=deadline-time.monotonic()
            if remaining<.25:break
            allocation=min(8.,max(.25,remaining/(params['candidates']-trial)))
            log(f'SHARED POOL search {trial+1}/{params["candidates"]}; allocation {allocation:.1f}s; all five methods will reuse these candidates')
            out=oracle.call('plan',seconds=allocation)
            attempts.append(dict(search=trial+1,status=out['status'],allocation_s=allocation))
            if out['status']!='planned':continue
            original=np.asarray(out['path']);compressed=thin(original)
            atomic_json(folder/f'raw_{trial+1:02d}.json',dict(path=original.tolist()))
            variants=[(compressed,False),(compressed,True)]
            if not np.array_equal(compressed,original):variants.append((original,True))
            accepted=False
            for i,(q,stop) in enumerate(variants):
                # The original corner-stop route is a fallback only. Always
                # finish validation of a started candidate even at the deadline.
                if i==2 and accepted:break
                if time.monotonic()>=deadline and accepted:break
                counter+=1
                c=timed_candidate(oracle,q,req,counter,log,params,stop)
                if c is None:continue
                name=f'candidate_{counter:02d}.json';atomic_json(folder/name,c)
                rows.append(dict(index=counter,file=name,sha256=digest(folder/name),metrics=c['metrics'],
                                 duration_s=c['duration_s'],search=trial+1,stop_at_corners=stop))
                del c;accepted=True
        if not rows:result['reason']='no_valid_route_within_budget'
    result['elapsed_s']=time.monotonic()-started
    # Only a fully built pool is published; Cancel leaves no reusable partial pool.
    atomic_json(folder/'pool.json',result)
    log(f'SHARED POOL COMPLETE: {len(rows)} validated candidates, {len(attempts)} searches, {result["elapsed_s"]:.1f}s; no 2x duration cutoff')
    return result


def run(method,req,context,pool_root,request_path,params,seconds,log):
    from .runtime import Oracle
    key=pool_key(context,req,params,seconds);folder=Path(pool_root)/key
    folder.mkdir(parents=True,exist_ok=True)
    # Serialize across separate invocations as well as the single GUI queue.
    import fcntl
    with (folder/'pool.lock').open('w') as lock:
        end=time.monotonic()+seconds+20
        while True:
            try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);break
            except BlockingIOError:
                if time.monotonic()>end:raise TimeoutError('Shared candidate pool is still being generated by another worker')
                time.sleep(.1)
        try:pool=read_pool(folder,key)
        except (OSError,ValueError,KeyError,TypeError) as exc:
            log(f'Ignoring damaged/stale pool; rebuilding: {exc}');pool=None
        reused=pool is not None
        if pool is None:
            oracle=Oracle(request_path,folder)
            try:pool=build_pool(oracle,req,folder,key,params,seconds,log)
            finally:oracle.close()
        else:log(f'REUSING SHARED POOL: {len(pool["candidates"])} validated candidates; no repeat search or FCL world loading')
        report=dict(method=method,pool_key=key,pool_file=str(folder/'pool.json'),pool_reused=reused,
                    candidates=[],selected=None,searches=pool['attempts'],pool_elapsed_s=pool['elapsed_s'],
                    budget_s=seconds,joint_limit_check=pool['joint_limit_check'],duration_ratio_filter=None)
        if not pool['candidates']:
            report.update(reason=pool.get('reason','no_valid_route_within_budget'),details=pool.get('details',{}))
            return None,report
        ranked,normalization=rank(pool['candidates'],method)
        report.update(candidates=ranked,selected=ranked[0]['index'],normalization=normalization)
        for row in ranked:
            log(f'{method}: candidate {row["index"]}, score={row["score"]:.4f}, duration={row["duration_s"]:.2f}s')
        path=folder/ranked[0]['file'];winner=json.loads(path.read_text())
        log(f'SELECTED candidate {winner["index"]}: lowest {method} score; {len(ranked)} equally checked candidates')
        return winner,report
