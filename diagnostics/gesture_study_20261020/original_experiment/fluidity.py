"""Geometric preference metrics, NOT validators or human-motion certificates.

All metrics use outward path progress. Neither slowing down nor adding a pause
can improve them. Fixed angular/spatial scales suppress sampling-rate artifacts.
"""
import numpy as np
from .. import style


def resample(q,n):
    q=np.asarray(q,dtype=float)
    if q.ndim!=2 or len(q)<2 or not np.isfinite(q).all():raise ValueError('Expected a finite curve')
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(q,axis=0),axis=1))]
    keep=np.r_[True,np.diff(arc)>1e-12]
    if arc[-1]<1e-12:return np.repeat(q[:1],n,axis=0)
    return np.column_stack([np.interp(np.linspace(0,arc[-1],n),arc[keep],q[keep,j]) for j in range(q.shape[1])])


def reversals(q,threshold):
    counts=[];excess=[]
    for column in q.T:
        direction=0;extreme=float(column[0]);turns=[extreme];count=0
        for value in column[1:]:
            value=float(value)
            if direction==0:
                if abs(value-extreme)>=threshold:direction=1 if value>extreme else -1;extreme=value
            elif direction*(value-extreme)>=0:extreme=value
            elif direction*(extreme-value)>=threshold:
                turns.append(extreme);count+=1;direction=-direction;extreme=value
        turns.append(extreme)
        counts.append(count)
        excess.append(max(0.,float(np.abs(np.diff(turns)).sum())-abs(float(column[-1]-column[0]))))
    backtracking=float(sum(excess))
    return dict(count=int(sum(counts)),per_joint_count=counts,excess_travel_rad=backtracking,
                per_joint_excess_rad=excess,score=float(sum(counts)+backtracking/(np.pi/6)))


def curve_shape(x,spacing):
    x=np.asarray(x,dtype=float)
    length=float(np.linalg.norm(np.diff(x,axis=0),axis=1).sum())
    if length<1e-9:return dict(length=length,total_turn_rad=0.,bending_energy=0.,curvature_variation=0.)
    # Bound work without dropping collision/render geometry; these are SOFT
    # scoring samples only. 801 source samples already bound practical detail.
    n=min(4097,max(3,int(np.ceil(length/spacing))+1))
    curve=resample(x,n);d=np.diff(curve,axis=0);ds=np.linalg.norm(d,axis=1)
    d=d[ds>1e-10];ds=ds[ds>1e-10]
    if len(d)<2:return dict(length=length,total_turn_rad=0.,bending_energy=0.,curvature_variation=0.)
    u=d/ds[:,None];mid=(ds[:-1]+ds[1:])/2
    theta=np.arccos(np.clip(np.sum(u[:-1]*u[1:],axis=1),-1.,1.))
    curvature=np.diff(u,axis=0)/mid[:,None]
    energy=length*float(np.sum(theta**2/mid))
    variation=length*float(np.linalg.norm(np.diff(curvature,axis=0),axis=1).sum())
    return dict(length=length,total_turn_rad=float(theta.sum()),bending_energy=energy,curvature_variation=variation)


def measure(q,features,params):
    q=np.asarray(q,dtype=float)
    original=style.metrics(q,features)
    reverse=reversals(q,np.radians(params['reversal_threshold_deg']))
    joints=curve_shape(q,params['joint_spacing_rad'])
    coordination=joints['total_turn_rad']+np.sqrt(joints['bending_energy'])
    spatial={};space_score=0.
    for name in ('wrist','tip'):
        x=np.asarray(features[name+'_xyz_m']);m=curve_shape(x,params['spatial_spacing_m'])
        # Normalized excess travel prevents tiny-radius detours from looking
        # cheap. No time, recording interiors or learned examples enter here.
        m['excess_length_ratio']=max(0.,m['length']/max(float(np.linalg.norm(x[-1]-x[0])),.01)-1.)
        m['score']=m['bending_energy']+.25*m['curvature_variation']+m['excess_length_ratio']
        spatial[name]=m;space_score+=m['score']
    values=dict(reversals=reverse['score'],coordination=float(coordination),spatial=float(space_score))
    if not all(np.isfinite(v) and v>=0 for v in values.values()):raise ValueError('Invalid fluidity metric')
    return dict(values=values,reversals=reverse,coordination=joints,spatial=spatial,baseline=original)


def rank(rows,method):
    """Rank identical fully checked candidates; duration NEVER filters the pool."""
    if not rows:raise ValueError('Cannot rank an empty candidate pool')
    from .config import METHODS
    if method not in METHODS:raise ValueError('Unknown fluidity method')
    fastest=min(rows,key=lambda c:(c['duration_s'],c['index']))
    baseline_scales=style.normalizers(fastest['metrics']['baseline'])
    base={c['index']:style.score(c['metrics']['baseline'],baseline_scales) for c in rows}
    baseline=min(rows,key=lambda c:(base[c['index']]['total'],c['duration_s'],c['index']))
    scales={k:max(1.,float(np.median([c['metrics']['values'][k] for c in rows])))
            for k in ('reversals','coordination','spatial')}
    scored=[]
    for c in rows:
        components={k:100.*v/scales[k] for k,v in c['metrics']['values'].items()}
        total=base[c['index']]['total'] if method=='baseline' else (
            sum(components.values())/3 if method=='combined' else components[method])
        scored.append(dict(c,score=float(total),normalized_fluidity=components,baseline_score=base[c['index']]))
    scored.sort(key=lambda c:(c['score'],c['duration_s'] if method=='baseline' else c['index'],c['index']))
    return scored,dict(baseline_reference_index=baseline['index'],fastest_reference_index=fastest['index'],
                       baseline_normalizers=baseline_scales,fluidity_normalizers=scales,
                       duration_ratio_filter=None,lower_is_better=True,fluidity_reference='per-component pool median, floored at 1',
                       baseline_uses_fastest_reference=True,fluidity_scoring_uses_elapsed_time=False)
