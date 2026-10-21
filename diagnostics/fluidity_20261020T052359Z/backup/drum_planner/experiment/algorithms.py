"""From-scratch route generation. Recording interiors are never read here."""
import json,time
from pathlib import Path
import numpy as np
from ..common import atomic_json,digest,DEFAULT_RECORDING
from ..planning import smooth_samples,cycle
from .. import style
from .config import DESCRIPTIONS
from .risk import robust_envelope,evaluate_many


def resample(q,n):
    q=np.asarray(q,dtype=float);keep=np.r_[True,np.linalg.norm(np.diff(q,axis=0),axis=1)>1e-12];q=q[keep]
    if len(q)<2:return np.repeat(q,n,axis=0)
    s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(q,axis=0),axis=1))]
    return np.column_stack([np.interp(np.linspace(0,s[-1],n),s,q[:,j]) for j in range(7)])


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


def timed_candidate(oracle,q,req,index,log,extra=0.,bounds_margin=0.,self_margin=0.):
    q=np.asarray(q,dtype=float)
    if q.ndim!=2 or q.shape[1]!=7 or len(q)<2 or not np.isfinite(q).all():return None
    if np.max(abs(q[0]-req['start']))>2e-7 or np.max(abs(q[-1]-req['goal']))>2e-7:return None
    q=q.copy();q[0]=req['start'];q[-1]=req['goal']
    original=q.copy();compressed=thin(q)
    variants=[(compressed,False),(compressed,True)]
    if not np.array_equal(compressed,original):variants.append((original,True))
    for q,stop in variants:
        try:points,fine,timing=smooth_samples(q,stop)
        except ValueError as exc:
            log(f"timing rejected ({'corner-stop' if stop else 'smooth'}): {exc}");continue
        # Continuous polynomial position extrema, not only sampled bounds.
        from scipy.interpolate import PPoly
        bounded=True
        for j,(coeff,breaks) in enumerate(zip(timing['coefficients'],timing['polynomial_breaks_unscaled_s'])):
            pp=PPoly(np.asarray(coeff),np.asarray(breaks),extrapolate=False)
            roots=pp.derivative().roots(extrapolate=False)
            roots=roots[np.isfinite(roots)&(roots>=pp.x[0])&(roots<=pp.x[-1])]
            values=pp(np.r_[pp.x,roots])
            if values.min()<oracle.lower[j]+bounds_margin-1e-12 or values.max()>oracle.upper[j]-bounds_margin+1e-12:
                bounded=False;break
        if not bounded:
            log('candidate rejected by continuous polynomial joint bounds');continue
        ok,proof=dense_check(oracle,fine,extra_padding=extra,self_margin=self_margin)
        if not ok:log(f"hard constraint rejected ({'corner-stop' if stop else 'smooth'}): {proof.get('details')}")
        if ok:
            samples=resample(fine,101);features=oracle.features(samples)
            clearance=np.array([f['clearance_m'] for f in features]);wrist=np.array([f['wrist'] for f in features]);tip=np.array([f['tip'] for f in features])
            distance=np.minimum(np.linalg.norm(samples-samples[0],axis=1),np.linalg.norm(samples-samples[-1],axis=1))
            transit=distance>=.35
            if not np.any(transit):transit=np.arange(len(samples))==len(samples)//2
            length=float(np.linalg.norm(np.diff(wrist,axis=0),axis=1).sum()+np.linalg.norm(np.diff(tip,axis=0),axis=1).sum())
            metrics=dict(transit_min_clearance_m=float(clearance[transit].min()),
                         transit_low_decile_clearance_m=float(np.quantile(clearance[transit],.1)),
                         all_path_min_clearance_m=float(clearance.min()),
                         transit_proximity=float(np.mean(np.maximum(0,1-clearance[transit]/.1)**2)),
                         workspace_length_m=length,joint_length_rad=float(np.linalg.norm(np.diff(samples,axis=0),axis=1).sum()),
                         bending_deg=float(style.broad_turning(wrist)+style.broad_turning(tip)))
            log(f'candidate {index}: valid, {timing["duration_s"]:.2f}s outward, transit clearance {metrics["transit_min_clearance_m"]*1000:.1f} mm')
            return dict(index=index,path=q,outward=points,fine=fine,samples=samples,timing=timing,proof=proof,metrics=metrics,features=features)
    log(f'candidate {index}: rejected by final bounds/zone/collision checks')
    return None


def clearance_key(c):
    m=c['metrics']
    # Millimetre-scale FCL padding bands; neither duration nor average cost can
    # buy its way through a worse bottleneck tier.
    return (-m['transit_min_clearance_m'],-m['transit_low_decile_clearance_m'],
            m['transit_proximity'],m['workspace_length_m'],c['timing']['duration_s'])


def run(method,oracle,req,folder,params,budget,log):
    folder=Path(folder);began=time.monotonic();deadline=began+budget
    candidates=[];attempts=[];special={};env=None;counter=0
    def remaining():return max(0.,deadline-time.monotonic())
    def search(a=None,b=None,margin=0.,taper=False,seconds=3.):
        if remaining()<1:return None
        command=dict(seconds=min(seconds,max(.2,remaining()*.35)),margin=margin,taper=taper)
        if a is not None:command['start']=np.asarray(a).tolist()
        if b is not None:command['goal']=np.asarray(b).tolist()
        log(f'search {len(attempts)+1}: {command["seconds"]:.1f}s allocation, added clearance {margin*1000:.1f} mm')
        out=oracle.call('plan',**command);attempts.append(dict(parameters=command,status=out['status']))
        return np.asarray(out['path']) if out['status']=='planned' else None
    def add(q,extra=0.,bounds=0.,self_margin=0.):
        nonlocal counter
        if q is None:return None
        counter+=1
        atomic_json(folder/f'raw_candidate_{counter:02d}.json',dict(path=np.asarray(q).tolist()))
        c=timed_candidate(oracle,q,req,counter,log,extra,bounds,self_margin)
        if c:
            # Bound in-memory resident fine arrays; only the current winner needs
            # them, and all complete paths are also archived independently.
            atomic_json(folder/f'candidate_{counter:02d}.json',dict(path=c['path'].tolist(),metrics=c['metrics'],timing=c['timing']))
            c.pop('fine');candidates.append(c)
        return c
    if method=='robust':
        env=robust_envelope(params);special['envelope']=env
        log(f'robust envelope: +{env["world_m"]*1000:.1f} mm world, {env["self_m"]*1000:.1f} mm self bound')
        lower=np.asarray(oracle.lower)+env['joint_rad'];upper=np.asarray(oracle.upper)-env['joint_rad']
        if any(np.any(np.asarray(req[k])<lower) or np.any(np.asarray(req[k])>upper) for k in ('start','goal')):
            return None,dict(reason='endpoint_outside_bounded_joint_envelope',**special)
        if env['world_m']>.25 or env['self_m']>.3:return None,dict(reason='declared_error_envelope_exceeds_search_range',**special)
        endpoint_check=oracle.check(np.array([req['start'],req['goal']]),extra_padding=env['world_m'])
        if not all(endpoint_check['valid']):return None,dict(reason='endpoint_blocked_by_error_envelope',details=endpoint_check,**special)
        # Self collision distance and joint perturbations are checked independently
        # below. World padding alone is NOT called whole-body robustness.
        for _ in range(5):
            if remaining()<max(3,budget*.18):break
            add(search(margin=env['world_m']),env['world_m'],env['joint_rad'],env['self_m'])
    elif method in ('chomp','stomp'):
        for trial in range(6):
            if remaining()<3:break
            seed=search(seconds=3.)
            if seed is None:continue
            log(f'{method.upper()}: native MoveIt optimization, initialization {trial+1} (new geometric path)')
            optimized=oracle.call('native',method=method,seconds=min(20.,max(1.,remaining()*.6)),
                                  seed_path=resample(seed,40).tolist(),timeout=max(40.,remaining()+5))
            attempts.append(dict(native=method,status=optimized['status'],code=optimized.get('moveit_error_code')))
            if optimized['status']=='planned':add(np.asarray(optimized['path']))
            if candidates and remaining()<budget*.35:break
    elif method=='iris_gcs':
        from .regions import generate
        seeds=[]
        for _ in range(2):
            q=search(seconds=3.)
            if q is not None:seeds.append(q)
        if seeds and remaining()>budget*.7:
            log('IRIS seeding: adding a fresh native-CHOMP broad-route proposal')
            proposal=oracle.call('native',method='chomp',seconds=min(8.,remaining()*.2),seed_path=resample(seeds[-1],40).tolist())
            if proposal['status']=='planned':
                q=np.asarray(proposal['path']);q[0]=req['start'];q[-1]=req['goal']
                ok,_=dense_check(oracle,resample(q,801))
                if ok:seeds[-1]=q
        if seeds:
            q,special=generate(oracle,seeds,req,params,deadline,log,folder)
            add(q)
    elif method=='clearance':
        for margin in (0.,0.,.005,.010,.020,.040):
            if remaining()<max(3,budget*.18):break
            add(search(margin=margin,taper=True,seconds=5.))
    else:
        # Common fresh candidates for risk comparisons; geometry, not a taught
        # trajectory, supplies the start/goal/scene. Diversified modes additionally
        # search C-space waypoints and increased-clearance routes.
        limit=2 if method in ('chance','cvar') else 6
        for i in range(limit):
            if remaining()<max(3,budget*(.65 if method in ('chance','cvar') else .2)):break
            add(search(margin=(.010 if i%2 and method not in ('chance','cvar') else 0.),taper=True))
        if method in ('chance','cvar') and candidates and remaining()>budget*.6:
            # MCMP-style risk selection can wrap a deterministic planner. Add a
            # genuinely different broad-route proposal rather than only scoring
            # variants of the same narrow RRT corridor. Still generated solely
            # from current geometry/endpoints; never a demonstrated trajectory.
            seed=min(candidates,key=lambda c:c['timing']['duration_s'])['path']
            log('risk candidate pool: native CHOMP broad-route proposal from a newly planned seed')
            proposal=oracle.call('native',method='chomp',seconds=min(8.,remaining()*.2),seed_path=resample(seed,40).tolist())
            attempts.append(dict(proposal='native_chomp',status=proposal['status']))
            if proposal['status']=='planned':add(np.asarray(proposal['path']))
        if method=='families':
            rng=np.random.default_rng(params['seed']);lower=np.asarray(oracle.lower);upper=np.asarray(oracle.upper)
            seeds=[c['samples'][50] for c in candidates] or [(np.asarray(req['start'])+req['goal'])/2]
            for i in range(8):
                if remaining()<max(4,budget*.2):break
                waypoint=np.clip(seeds[i%len(seeds)]+rng.normal(0,.5,7),lower+1e-6,upper-1e-6)
                if not oracle.check(waypoint[None,:])['valid'][0]:continue
                a=search(b=waypoint,seconds=2.)
                if a is None:continue
                b=search(a=waypoint,seconds=2.)
                if b is not None:add(np.vstack((a[:-1],b)))
            families=[]
            for c in candidates:
                x=resample(c['path'],31)/(upper-lower)
                k=next((j for j,p in enumerate(families) if np.sqrt(np.mean((x-p)**2))<.045),len(families))
                if k==len(families):families.append(x)
                c['family']=k+1
            special.update(route_families=len(families),family_distance='RMS normalized joint-arc path; threshold 0.045; heuristic, not homotopy proof')
    if not candidates:return None,dict(attempts=attempts,**({'reason':'no_valid_route_within_budget'}|special))
    if method in ('chance','cvar'):
        assessed=[];origin=np.array(req['scene']['T_base_depth'])[:3,3]
        candidates.sort(key=clearance_key)
        log(f'evaluating {params["scenarios"]} common uncertainty scenarios across {len(candidates)} candidates')
        try:
            reports=evaluate_many(oracle,[c['samples'] for c in candidates],params,origin,log,deadline=deadline)
        except TimeoutError:
            log('budget reached; partial scenario estimates discarded');reports=[]
        for c,risk_report in zip(candidates,reports):
            c['risk']=risk_report
            if method=='cvar' or c['risk']['accepted']:assessed.append(c)
            else:log(f'candidate {c["index"]} rejected: scenario upper bound {c["risk"]["wilson_one_sided_95_upper"]:.3f} > {params["chance_limit"]:.3f}')
        if not assessed:return None,dict(reason=('chance_bound_exceeded_for_all_candidates' if reports else 'scenario_budget_exhausted'),attempts=attempts,
                                         candidates=[summary(c) for c in candidates],**special)
        candidates.sort(key=lambda c: (c.get('risk',{}).get('cvar',float('inf')),clearance_key(c)))
        winner=min(assessed,key=lambda c:((c['risk']['cvar'] if method=='cvar' else c['risk']['wilson_one_sided_95_upper']),clearance_key(c)))
    else:winner=min(candidates,key=clearance_key)
    report=dict(method=method,description=DESCRIPTIONS[method],attempts=attempts,candidates=[summary(c) for c in candidates],
                selected=winner['index'],elapsed_s=time.monotonic()-began,budget_s=budget,**special)
    return winner,report


def summary(c):
    return {key:c[key] for key in ('index','metrics','risk','family','score','style_metrics') if key in c}|dict(duration_s=c['timing']['duration_s'])
