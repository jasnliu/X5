"""Version 2 geometry proposals. Version 1 timing modules remain untouched.

Small local C2/C3 blends or a clamped quintic interpolant; all are proposals,
never presumed collision-free. No recording reader or actuator dependencies.
"""
import numpy as np
from numpy.polynomial import Polynomial as Poly
from scipy.interpolate import BPoly,PPoly,make_interp_spline
from ..experiment.algorithms import thin,dense_check
from . import core


def vertices(source,tolerance=1e-12):
    q=np.asarray(source['path_waypoints'],float)
    if not source['timing']['stop_at_corners']:
        raise ValueError('Rounding requires a corner-stop source; retain continuous paths')
    core.straight_blocks(source)  # Do not silently replace an arbitrary curve by chords.
    return thin(q,epsilon=tolerance)


def propose(source,kind='local_c3',fraction=.45,simplify=1e-12):
    q=vertices(source,simplify)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(q,axis=0),axis=1))];arc/=arc[-1]
    if kind=='overlap_c3':return overlapping(q,arc,fraction),q
    if kind=='quintic':
        curves=[PPoly.from_spline(make_interp_spline(arc,q[:,j],k=5,
            bc_type=([(1,0.),(2,0.)],[(1,0.),(2,0.)]))) for j in range(7)]
        return PPoly(np.stack([p.c for p in curves],axis=-1),curves[0].x),q
    if kind not in ('local_c2','local_c3'):raise ValueError('Unknown corner blend')
    if not 0<fraction<.5:raise ValueError('Local blend fraction must be between 0 and 0.5')
    continuity=2 if kind=='local_c2' else 3
    d=np.diff(arc);velocity=np.diff(q,axis=0)/d[:,None]
    radii=fraction*np.minimum(d[:-1],d[1:]);pieces=[]
    def straight(lo,hi,pos,vel):
        if hi-lo<=1e-13:return
        pieces.append(PPoly(np.stack([vel,pos])[:,None,:],[lo,hi]))
    def endpoint(lo,hi,pos,vel,starting):
        # Construct a unit scalar ramp first. Fitting seven nearly-linear high
        # degree polynomials on tiny intervals amplifies cancellation noise.
        a=[0.,0. if starting else 1.,*[0.]*(continuity-1)]
        b=[1.,1. if starting else 0.,*[0.]*(continuity-1)]
        scalar=PPoly.from_bernstein_basis(BPoly.from_derivatives([0.,1.],[a,b]))
        powers=np.arange(scalar.c.shape[0]-1,-1,-1)
        c=scalar.c[:,:,None]*(hi-lo)*vel/(hi-lo)**powers[:,None,None]
        c[-1,0]+=pos;pieces.append(PPoly(c,[lo,hi]))
    def bend(lo,hi,pos,before,after):
        # Integrate a smooth tangent transition: cubic smoothstep for C2,
        # quintic smootherstep for C3. No spurious degree-7 coefficient.
        integral=np.array([0,0,0,1.,-.5]) if continuity==2 else np.array([0,0,0,0,2.5,-3.,1.])
        c=integral[:,None]*(after-before)*(hi-lo);c[0]=pos;c[1]+=(hi-lo)*before
        c=(c/(hi-lo)**np.arange(len(c))[:,None])[::-1,None,:]
        pieces.append(PPoly(c,[lo,hi]))
    first=fraction*d[0];last=1-fraction*d[-1]
    endpoint(0,first,q[0],velocity[0],True)
    at=first;position=q[0]+first*velocity[0]
    for i,r in enumerate(radii,1):
        lo,hi=arc[i]-r,arc[i]+r;a=q[i]-r*velocity[i-1];b=q[i]+r*velocity[i]
        straight(at,lo,position,velocity[i-1])
        bend(lo,hi,a,velocity[i-1],velocity[i])
        at=hi;position=b
    a=q[-1]-(1-last)*velocity[-1]
    straight(at,last,position,velocity[-1])
    endpoint(last,1,a,velocity[-1],False)
    degree=max(p.c.shape[0] for p in pieces)
    c=np.concatenate([np.pad(p.c,((degree-p.c.shape[0],0),(0,0),(0,0))) for p in pieces],axis=1)
    return PPoly(c,np.r_[pieces[0].x[0],[p.x[-1] for p in pieces]]),q


def overlapping(q,arc,width):
    """C3 compact-kernel smoothing of slope changes, permitting blend overlap.

    Each original hinge max(s-k,0) becomes the integral of a quintic smoothstep.
    Unlike disjoint fillets, a tiny intermediate edge cannot force a tiny blend
    radius. Endpoint padding is linear; end caps change only progression along
    the existing first/last straight segment. No global curve refit.
    """
    if not 0<width<.25:raise ValueError('Kernel halfwidth must be in (0,0.25)')
    velocities=np.diff(q,axis=0)/np.diff(arc)[:,None];delta=np.diff(velocities,axis=0)
    knots=arc[1:-1];r=np.minimum(width,np.minimum(knots,1-knots)*.8)
    starts=knots-r;ends=knots+r
    first=(starts[0] if len(starts) else 1.)*.45
    last=1-(1-(ends[-1] if len(ends) else 0.))*.45
    bounds=np.unique(np.r_[0,first,starts,ends,last,1.]);bounds=bounds[np.r_[True,np.diff(bounds)>1e-12]]
    pieces=[];integral=Poly([0,0,0,0,2.5,-3.,1.])
    for lo,hi in zip(bounds[:-1],bounds[1:]):
        span=hi-lo;u=Poly([lo,span]);mid=(lo+hi)/2
        components=[Poly([q[0,j]])+velocities[0,j]*u for j in range(7)]
        for i,k in enumerate(knots):
            if mid<starts[i]:continue
            hinge=(u-k) if mid>ends[i] else 2*r[i]*integral((u-starts[i])/(2*r[i]))
            for j in range(7):components[j]+=delta[i,j]*hinge
        if hi<=first+1e-13 or lo>=last-1e-13:
            starting=hi<=first+1e-13;vel=velocities[0] if starting else velocities[-1]
            pos=q[0] if starting else q[-1]-(1-last)*vel
            a=[0.,0. if starting else 1.,0.,0.];b=[1.,1. if starting else 0.,0.,0.]
            scalar=PPoly.from_bernstein_basis(BPoly.from_derivatives([0.,1.],[a,b]))
            c=scalar.c[:,:,None]*span*vel/span**np.arange(7,-1,-1)[:,None,None];c[-1,0]+=pos
        else:
            c=np.zeros((8,1,7))
            for j,p in enumerate(components):
                c[-len(p.coef):,0,j]=(p.coef/span**np.arange(len(p.coef)))[::-1]
        pieces.append(c)
    return PPoly(np.concatenate(pieces,axis=1),bounds)


def as_source(pp,source):
    peaks,jumps=core.polynomial_peaks(pp)
    if np.any(jumps>[1e-9,1e-8,1e-7]):raise ValueError('Rounded path is not C2')
    if np.max(abs(pp(0)-source['request']['start']))>1e-10 or np.max(abs(pp(1)-source['request']['goal']))>1e-10:
        raise ValueError('Rounded endpoints changed')
    if np.max(abs(pp([0,1],nu=1)))>1e-8 or np.max(abs(pp([0,1],nu=2)))>1e-7:
        raise ValueError('Rounded endpoints not at rest')
    scale=max(1.,core.scale_needed(peaks))*core.RESERVE
    x=np.unique(pp.x);x=x[(x>=0)&(x<=1)]
    return dict(path_waypoints=pp(x).tolist(),request=source['request'],
        timing=dict(coefficients=np.moveaxis(pp.c,-1,0).tolist(),
            polynomial_breaks_unscaled_s=np.broadcast_to(pp.x,(7,len(pp.x))).tolist(),
            knots_unscaled_s=x.tolist(),stop_at_corners=False,time_scale=scale,duration_s=scale))


def closest(q,reference):
    """Ordered reference-polyline projections; no point cloud/nearest waypoint shortcut."""
    a=np.asarray(reference[:-1]);d=np.diff(reference,axis=0);length=np.linalg.norm(d,axis=1)
    u=np.clip(np.einsum('nsj,sj->ns',q[:,None,:]-a,d)/(length**2),0,1)
    projected=a+u[:,:,None]*d;dist=np.linalg.norm(q[:,None,:]-projected,axis=2)
    indices=np.argmin(dist,axis=1);rows=np.arange(len(q))
    progress=(np.r_[0.,np.cumsum(length)][indices]+u[rows,indices]*length[indices])/length.sum()
    return projected[rows,indices],dist[rows,indices],progress


def inspect(pp,source,oracle,*,joint_tube_rad=np.radians(1.),workspace_limit_m=.01):
    """Bound the path change and check the actual new curve, not just vertices.

    Distance to a fixed set is 1-Lipschitz: maximum sampled L2 distance plus
    half the sampling interval times a continuous derivative bound encloses
    the entire curve in a joint-space tube around the ORIGINAL polyline.
    Cartesian deviations are separate dense measurements, not a safety proof.
    """
    grid=np.unique(np.r_[np.linspace(0,1,2001),pp.x]);grid=grid[(grid>=0)&(grid<=1)]
    q=pp(grid);ref,dist,progress=closest(q,np.asarray(source['path_waypoints']))
    derivative,_=core.polynomial_peaks(pp)
    bound=float(dist.max()+np.linalg.norm(derivative[0])*np.diff(grid).max()/2)
    report=dict(max_joint_l2_deviation_rad=float(dist.max()),certified_joint_tube_rad=bound,
                joint_tube_limit_rad=float(joint_tube_rad),max_joint_component_deviation_rad=float(abs(q-ref).max()),
                original_progress_backtrack=float(max(0.,-np.diff(progress).min())))
    if bound>joint_tube_rad:return False,dict(report,reason='deviation_bound')
    if report['original_progress_backtrack']>1e-5:return False,dict(report,reason='rounded_path_backtracks')
    # Continuous position extrema, not merely sampled joint limits.
    for j in range(7):
        p=PPoly(pp.c[:,:,j],pp.x);roots=p.derivative().roots(extrapolate=False)
        gridj=np.r_[0.,1.,roots[np.isfinite(roots)&(roots>0)&(roots<1)]];v=p(gridj)
        if v.min()<oracle.lower[j]-1e-10 or v.max()>oracle.upper[j]+1e-10:
            return False,dict(report,reason='continuous_joint_limits')
    features=oracle.call('fk',q=np.r_[q,ref].tolist());n=len(q)
    for name in ('wrist','tip'):
        x=np.asarray(features[name+'_xyz_m']);report[name+'_deviation_m']=float(np.linalg.norm(x[:n]-x[n:],axis=1).max())
    if max(report['wrist_deviation_m'],report['tip_deviation_m'])>workspace_limit_m:
        return False,dict(report,reason='workspace_deviation')
    ok,collision=dense_check(oracle,q)
    collision.pop('max_time_step_s',None)
    collision['max_parameter_step']=float(np.diff(grid).max())
    report.update(collision_zone=collision,workspace_limit_m=workspace_limit_m,geometry_samples=len(q))
    if not ok:return False,dict(report,reason='collision_or_zone')
    return True,dict(report,valid=True)
