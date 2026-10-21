"""Experimental uncertainty models; no claims about real collision probability."""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def robust_envelope(p,stick_length=.4094):
    # A deliberately conservative displacement bound: 1.5 m lever for EACH
    # moving joint, 5 m camera range, isotropic surface/tool/scene bounds.
    joint=1.5*7*2*np.sin(np.radians(p['joint_deg'])/2)
    camera=5.*2*np.sin(np.radians(p['camera_rotation_deg'])/2)
    tool=p['tool_mm']/1000+stick_length*2*np.sin(np.radians(p['tool_angle_deg'])/2)
    world=(p['depth_mm']+p['camera_translation_mm']+p['scene_motion_mm'])/1000+camera+tool+joint
    return dict(world_m=float(world),self_m=float(2*joint+tool),joint_rad=float(np.radians(p['joint_deg'])),
                lever_arm_bound_m=1.5,camera_range_bound_m=5.,additional_to_original_padding=True)


def wilson_upper(failures,n,z=1.6448536269514722):
    if n<=0 or not 0<=failures<=n:raise ValueError('Invalid scenario counts')
    rate=failures/n;z2=z*z
    return (rate+z2/(2*n)+z*math.sqrt(rate*(1-rate)/n+z2/(4*n*n)))/(1+z2/n)


def cvar(losses,alpha):
    a=np.asarray(losses,dtype=float)
    if a.ndim!=1 or len(a)==0 or not np.isfinite(a).all() or not 0<alpha<1:raise ValueError('Invalid CVaR data')
    # Exact empirical upper-tail mass, including a fractional boundary sample.
    x=np.sort(a)[::-1];mass=(1-alpha)*len(x);n=int(np.floor(mass));frac=mass-n
    return float((x[:n].sum()+(frac*x[n] if n<len(x) else 0.))/mass)


def scenarios(p,npoints,camera_origin):
    rng=np.random.default_rng(p['seed']+1447);t=np.linspace(0,1,npoints)
    def ball(radius):
        v=rng.normal(size=3);return v/max(np.linalg.norm(v),1e-12)*rng.uniform(0,1)**(1/3)*radius
    for _ in range(p['scenarios']):
        shift=ball(p['camera_translation_mm']/1000)
        rot=Rotation.from_rotvec(ball(np.radians(p['camera_rotation_deg']))).as_matrix()
        T=np.eye(4);T[:3,:3]=rot;origin=np.asarray(camera_origin)
        T[:3,3]=origin-rot@origin+shift
        # One calibration/joint bias for the whole path + smooth tracking term;
        # not independent per frame. Components stay within the declared bound.
        bias=rng.uniform(-1,1,7);wave=rng.uniform(-1,1,(3,7))
        noise=.5*bias+.5*(np.sin(np.pi*t[:,None]*np.arange(1,4))@wave)/3
        delta=noise*np.radians(p['joint_deg'])
        extra=rng.uniform(0,1)*(p['depth_mm']+p['scene_motion_mm'])/1000
        tool=rng.uniform(0,1)*(p['tool_mm']/1000+.4094*2*np.sin(np.radians(p['tool_angle_deg'])/2))
        yield delta,dict(world_transform=T.tolist(),extra_padding=float(extra),tool_padding=float(tool))


def evaluate_many(oracle,paths,p,camera_origin,progress=lambda *_:None,deadline=None):
    import time
    from scipy.stats import norm
    paths=[np.asarray(q) for q in paths]
    if not 1<=len(paths)<=4 or len({q.shape for q in paths})!=1:raise ValueError('Need 1–4 equal-size risk paths')
    npoints=len(paths[0]);rows=[[] for _ in paths];batch=[]
    for i,(delta,kwargs) in enumerate(scenarios(p,npoints,camera_origin)):
        batch.append(dict(kwargs,paths=[(q+delta).tolist() for q in paths]))
        if len(batch)==8 or i+1==p['scenarios']:
            if deadline is not None and time.monotonic()>deadline:
                raise TimeoutError('Scenario budget exhausted; incomplete estimates cannot pass')
            if hasattr(oracle,'call'):
                results=oracle.call('scenarios',scenarios=batch)['rows']
                if len(results)!=len(batch) or any(len(r)!=len(paths) for r in results):
                    raise RuntimeError('Incomplete native scenario batch')
                for result in results:
                    for dest,row in zip(rows,result):dest.append(row)
            else:
                for item in batch:
                    perturb=item.copy();qs=perturb.pop('paths')
                    for dest,q in zip(rows,qs):
                        q=np.asarray(q);failed=not all(oracle.check(q,**perturb)['valid'])
                        near=dict(perturb);near['extra_padding']+=.02
                        hits=1-np.asarray(oracle.check(q,**near)['valid'],dtype=float)
                        dest.append(dict(failed=failed,loss=10*failed+float(hits.mean())))
            batch=[];progress(f'scenarios {i+1}/{p["scenarios"]} × {len(paths)} candidates (shared scene work)')
    reports=[]
    # Conservative multiple-candidate adjustment. Still an approximate interval
    # for this ASSUMED scenario distribution, not a hardware guarantee.
    z=float(norm.ppf(1-.05/len(paths)))
    for row in rows:
        failed=sum(r['failed'] for r in row);losses=[r['loss'] for r in row]
        upper=wilson_upper(failed,len(row),z)
        reports.append(dict(scenarios=len(row),failed_scenarios=failed,failure_fraction=failed/len(row),
            endpoint_failed_scenarios=sum(r.get("endpoint_failed",False) for r in row),
            wilson_one_sided_95_upper=upper,accepted=upper<=p['chance_limit'],
            confidence_candidates=len(paths),confidence_adjustment='Bonferroni; Wilson interval remains approximate',
            cvar=cvar(losses,p['cvar_alpha']),alpha=p['cvar_alpha'],mean_loss=float(np.mean(losses)),
            losses=losses,whole_path_samples=npoints,temporal_model='constant bias plus bounded smooth tracking modes',
            distribution='experimental bounded uniform scene and joint perturbations; not fitted from hardware',
            real_world_probability_claim=False,continuous_collision_probability=False))
    return reports


def evaluate(oracle,q,p,camera_origin,progress=lambda *_:None,deadline=None):
    return evaluate_many(oracle,[q],p,camera_origin,progress,deadline)[0]
