"""Endpoint-only gesture search. No recording reader or learned demonstration.

The objectives describe geometry, not a physiological model of human motion.
All proposed curves still require the production full-body/zone checks AFTER
timing. Cheap FK and coarse tests here are proposal filters only.
"""
import time
import numpy as np
from .. import style
from .fluidity import resample, reversals


def geometry(q, features):
    q=np.asarray(q,dtype=float)
    total=float(np.abs(np.diff(q,axis=0)).sum())
    direct=float(np.abs(q[-1]-q[0]).sum())
    excess=max(0.,total-direct)
    xyz={};overshoots=[];detours=[];bends=[]
    for name in ('wrist','tip'):
        x=np.asarray(features[name+'_xyz_m'],dtype=float)
        length=float(np.linalg.norm(np.diff(x,axis=0),axis=1).sum())
        straight=float(np.linalg.norm(x[-1]-x[0]))
        lo=np.minimum(x[0],x[-1]);hi=np.maximum(x[0],x[-1])
        outside=np.maximum(np.maximum(lo-x,x-hi),0.)
        over=float(np.linalg.norm(outside,axis=1).max())
        bend=style.broad_turning(x)
        xyz[name]=dict(length_m=length,straight_m=straight,detour_ratio=length/max(straight,.02),
                       broad_turning_deg=bend,endpoint_box_overshoot_m=over,
                       positive_x_goal_overshoot_m=max(0.,float(x[:,0].max()-x[-1,0])))
        overshoots.append(over/max(straight,.1))
        detours.append(max(0.,length/max(straight,.02)-1.))
        bends.append(np.radians(bend))
    # Endpoint-relative excursion, normalized with a floor so near-zero joint
    # tasks cannot dominate due to division by a tiny net angle.
    delta=q[-1]-q[0]
    progress=(q-q[0])*np.sign(delta)/np.maximum(abs(delta),.15)
    active=abs(delta)>.035
    disagreement=float(np.sqrt(np.mean(np.var(progress[:,active],axis=1)))) if active.sum()>1 else 0.
    return dict(joint_excess_deg=float(np.degrees(excess)),joint_total_deg=float(np.degrees(total)),
                joint_excess_ratio=excess/max(direct,.5),workspace_detour=float(np.mean(detours)),
                workspace_overshoot=float(np.mean(overshoots)),
                broad_turning_rad=float(.65*bends[0]+.35*bends[1]),
                progress_disagreement=disagreement,reversal_count=reversals(q,np.radians(2.))['count'],
                **xyz)


def objective(g, mode='economy', proximity=0., params=None):
    b=g['joint_excess_ratio'];d=g['workspace_detour'];o=g['workspace_overshoot']
    turn=g['broad_turning_rad'];sync=g['progress_disagreement']
    params=params or {}
    wb=params.get('backtrack_weight',4.);wo=params.get('overshoot_weight',2.)
    if mode=='economy':return wb*b+2*d+wo*o+.10*turn+.5*sync
    if mode=='arc':return .375*wb*b+2*d+wo*o+.35*turn+.25*sync
    if mode=='comfort':return objective(g,'arc',params=params)+params.get('clearance_weight',3.)*proximity
    raise ValueError('Unknown gesture objective')


def fk(oracle,q):
    return oracle.call('fk',q=np.asarray(q,dtype=float).tolist())


def quick_score(oracle,q,mode='economy',params=None):
    q=resample(q,81)
    return objective(geometry(q,fk(oracle,q)),mode,params=params)


def curves(start,goal,parameters,family,n=49):
    """One smooth gesture with exact endpoints and low dimensional controls."""
    a=np.asarray(start);b=np.asarray(goal);s=np.linspace(0.,1.,n)[None,:,None]
    z=np.asarray(parameters).reshape(-1,7 if family=='phase' else 14)
    if family=='phase':
        # Positive exponent preserves monotonicity of every motor. Zero means
        # simultaneous joint progress; +/- values create early/late progress.
        u=s**np.exp(z[:,None,:])
        q=a+(b-a)*u
    elif family=='arch':
        q=a+s*(b-a)+np.sin(np.pi*s)*z[:,None,:7]+np.sin(2*np.pi*s)*z[:,None,7:]
    else:raise ValueError('Unknown smooth family')
    q[:,0]=a;q[:,-1]=b
    return q


def parametric_search(oracle,req,family,rng,deadline,log,rounds=5,population=40,seed_paths=(),params=None):
    """Bounded cross-entropy proposal search; feasibility dominates soft cost.

    Does not use record3, cached baseline paths, or previous-task solutions.
    Stagnant infeasible runs stop early. Valid proposals are later densely
    checked again after quintic interpolation.
    """
    dims=7 if family=='phase' else 14
    mean=np.zeros(dims);std=np.full(dims,.65 if family=='phase' else .22)
    if family=='arch':std[7:]=.09
    limits=np.full(dims,1.3 if family=='phase' else .55)
    if family=='arch':limits[7:]=.22
    initial=[]
    if family=='arch':
        s=np.linspace(0,1,49);basis=np.column_stack([np.sin(np.pi*s),np.sin(2*np.pi*s)])
        chord=(1-s[:,None])*np.asarray(req['start'])+s[:,None]*np.asarray(req['goal'])
        for path in seed_paths:
            coef=np.linalg.lstsq(basis,resample(path,49)-chord,rcond=None)[0]
            initial.append(np.clip(coef.ravel(),-limits,limits))
        if initial:mean=np.asarray(initial[0]);std*=.6
    survivors=[];best_bad=10**9;stagnant=0
    for generation in range(rounds):
        if time.monotonic()>=deadline:break
        z=np.clip(rng.normal(mean,std,(population,dims)),-limits,limits)
        z[0]=mean
        if generation==0:z[1]=0.
        if initial:
            for i,value in enumerate(initial[:population//4]):z[2+i]=value
        # Keep some global exploration instead of collapsing immediately on a
        # near-feasible local minimum of the discrete collision-count filter.
        z[-population//5:]=np.clip(rng.normal(0,.25,(population//5,dims)),-limits,limits)
        paths=curves(req['start'],req['goal'],z,family)
        valid=np.asarray(oracle.check(paths.reshape(-1,7))['valid']).reshape(population,-1)
        bad=(~valid).sum(axis=1)
        # FK is inexpensive and uses the identical calibrated model.
        features=fk(oracle,paths.reshape(-1,7))
        features={k:np.asarray(v).reshape(population,paths.shape[1],3) for k,v in features.items()
                  if k in ('wrist_xyz_m','tip_xyz_m')}
        costs=[]
        for i,q in enumerate(paths):
            g=geometry(q,{k:v[i] for k,v in features.items()})
            cost=objective(g,'arc' if family=='arch' else 'economy',params=params)
            costs.append(1000.*bad[i]+cost)
            if bad[i]==0:survivors.append((cost,z[i].copy(),q.copy()))
        order=np.argsort(costs);elite=z[order[:max(6,population//5)]]
        mean=.25*mean+.75*elite.mean(axis=0)
        std=np.maximum(.025,.25*std+.75*elite.std(axis=0))
        improvement=int(bad.min())<best_bad
        best_bad=min(best_bad,int(bad.min()));stagnant=0 if improvement else stagnant+1
        log(f'{family} search {generation+1}/{rounds}: {int((bad==0).sum())}/{population} coarse-feasible; best blocked samples {best_bad}')
        if not survivors and stagnant>=5:break
    survivors.sort(key=lambda row:row[0])
    selected=[]
    for cost,z,q in survivors:
        if any(np.max(abs(q-other[1]))<.025 for other in selected):continue
        selected.append((cost,q))
        if len(selected)>=3:break
    # A few interpolation knots reduce tiny, unhelpful spline segments.
    return [q[np.linspace(0,len(q)-1,9).astype(int)] for _,q in selected]


def with_vertices(path,n):
    """Densify without cutting across original polyline corners."""
    q=np.asarray(path,dtype=float)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(q,axis=0),axis=1))]
    keep=np.r_[True,np.diff(arc)>1e-12]
    if arc[-1]<1e-12:return q[[0,-1]]
    grid=np.unique(np.r_[arc[keep],np.linspace(0,arc[-1],n)])
    return np.column_stack([np.interp(grid,arc[keep],q[keep,j]) for j in range(7)])


def shortcut(oracle,path,rng,deadline,log,mode='economy',attempts=100,params=None):
    """Partial joint shortcuts, allowing joints to improve independently.

    Unlike straight-line all-joint shortcuts, this can remove a large J5
    reversal while preserving the other joints' obstacle-avoidance excursion.
    """
    q=with_vertices(path,25);score=quick_score(oracle,q,mode,params);accepted=0
    # First explicitly try entire-curve single-joint progress, then subsets.
    for iteration in range(attempts):
        if time.monotonic()>=deadline:break
        if iteration<7:i,j=0,len(q)-1;indices=[iteration]
        else:
            i,j=sorted(rng.choice(len(q),2,replace=False))
            if j-i<3:continue
            indices=rng.choice(7,int(rng.integers(1,4)),replace=False)
        trial=q.copy()
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(q[i:j+1],axis=0),axis=1))]
        u=arc/max(arc[-1],1e-12)
        for k in indices:trial[i:j+1,k]=q[i,k]+u*(q[j,k]-q[i,k])
        new=quick_score(oracle,trial,mode,params)
        if new>=score-1e-7:continue
        checked=oracle.check(trial,edges=True,stop=True)
        if len(checked['valid'])!=len(trial) or not all(checked['valid']):continue
        q=trial;score=new;accepted+=1
    log(f'partial shortcut ({mode}): {accepted} accepted changes; objective {score:.4f}')
    return q


def elastic(oracle,path,rng,deadline,log,attempts=80,params=None):
    """Collision-constrained low-frequency path deformation (not CHOMP).

    Endpoint-preserving sine displacements and gentle Laplacian proposals
    reduce broad detours. Each accepted update is checked, and final timing is
    separately verified. No finite-difference collision gradients are assumed.
    """
    q=with_vertices(path,17);score=quick_score(oracle,q,'arc',params);accepted=0
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(q,axis=0),axis=1))]
    s=arc/max(arc[-1],1e-12);scales=(.12,.06,.03)
    for iteration in range(attempts):
        if time.monotonic()>=deadline:break
        trial=q.copy()
        if iteration%4==0:
            u=(s[1:-1]-s[:-2])/np.maximum(s[2:]-s[:-2],1e-12)
            target=(1-u[:,None])*q[:-2]+u[:,None]*q[2:]
            trial[1:-1]+=.44*(target-q[1:-1])
        else:
            delta=rng.normal(0,scales[min(2,iteration*3//attempts)],7)
            trial+=np.sin(np.pi*s)[:,None]*delta
        trial[[0,-1]]=q[[0,-1]]
        new=quick_score(oracle,trial,'arc',params)
        if new>=score-1e-7:continue
        check=oracle.check(trial,edges=True,stop=True)
        if len(check['valid'])!=len(trial) or not all(check['valid']):continue
        q=trial;score=new;accepted+=1
    log(f'elastic deformation: {accepted} accepted updates; objective {score:.4f}')
    return q
