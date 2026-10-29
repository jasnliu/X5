"""Exact-polynomial proof for departure from a measured J2 rest inside the margin.

Only a prefix leaving the ORIGINAL rest and a suffix returning to it may be
negative. Below zero these are monotone, never below the acquired rest. This
does not grant a new negative workspace or alter any measured/commanded pose.
"""
import numpy as np
from .joint_position_limits import policy,command_lower
from .timing import core

TOL=1e-10  # polynomial roundoff, not a tracking/encoder allowance


def verify(path,req):
    ctx=req.get('joint_position_context')
    if req.get('moving_side','right')!='right' or not ctx:return None  # historical recovery remains readable
    if ctx.get('policy')!=policy():raise ValueError('Obsolete J2 rest-transition policy; Calculate again')
    reference=ctx.get('rest_reference_rad');lower=command_lower(reference)
    start=np.asarray(req['start'],float);goal=np.asarray(req['goal'],float)
    x=np.unique(path.x);lo=float(x[0]);hi=float(x[-1])
    if (not np.isfinite(path.c).all() or not np.isfinite(x).all() or
            np.max(abs(path(lo)-start))>TOL or np.max(abs(path(hi)-goal))>TOL):
        raise ValueError('J2 transition curve/endpoints mismatch')
    if reference is None:reference=np.full(7,np.nan)
    leaves=np.allclose(start,reference,rtol=0,atol=TOL)
    returns=np.allclose(goal,reference,rtol=0,atol=TOL)
    if (start[1]<-TOL and not leaves) or (goal[1]<-TOL and not returns):
        raise ValueError('A below-zero J2 endpoint must be the exact original acquired rest, not another destination')
    intervals=[];minimum=float(min(start[1],goal[1]))
    for a,b in zip(x[:-1],x[1:]):
        if b<=a:continue
        p=core.local(path,a,b,j=1);v=p.deriv()
        cuts=np.unique([0.,1.,*core.unit_roots(p),*core.unit_roots(v)])
        values=p(cuts);minimum=min(minimum,float(values.min()))
        for left,right in zip(cuts[:-1],cuts[1:]):
            mid=(left+right)/2
            intervals.append((float(p(mid))<-TOL,float(v(mid))))
    if minimum<lower-TOL:raise ValueError('J2 path goes below acquired rest/operating floor')
    # Roots divide each segment into constant-sign position/derivative pieces.
    # Thus the checks cover the continuous polynomial, not just a sample grid.
    negative=[i for i,(below,_) in enumerate(intervals) if below]
    prefix=0
    while prefix<len(intervals) and intervals[prefix][0]:prefix+=1
    suffix=len(intervals)
    while suffix>0 and intervals[suffix-1][0]:suffix-=1
    for i in negative:
        slope=intervals[i][1]
        outward=leaves and i<prefix and slope>=-TOL
        inward=returns and i>=suffix and slope<=TOL
        if not (outward or inward):
            raise ValueError('J2 below-zero motion is allowed only monotonically away from/to the exact original rest')
    return dict(policy=policy(),minimum_j2_deg=float(np.degrees(minimum)),
                rest_j2_deg=None if not np.isfinite(reference[1]) else float(np.degrees(reference[1])),
                continuous_polynomial_checked=True,negative_intervals=len(negative))


def verify_curve(curve,req):
    if not req.get('joint_position_context'):return
    if curve.parts:
        for part in curve.parts:
            child=part['curve']
            verify_curve(child,dict(req,start=child.at(0).tolist(),goal=child.at(child.duration).tolist()))
    elif curve.retimed is not None:
        p=curve.retimed.path if curve.plan['timing']['kind']=='time_density' else curve.retimed.p
        verify(p,req)
    else:
        verify(core.original(curve.plan),req)
