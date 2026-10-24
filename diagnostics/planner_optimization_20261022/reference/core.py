"""Serializable exact-path timing laws and continuous polynomial limit checks.

No device dependencies. Geometry is either the ORIGINAL spline composed with
a monotone clock, or the exact straight segments of a corner-stop path. Jerk
may jump (bounded one-sided values); position, velocity and acceleration may not.
"""
import hashlib
import json
import math

import numpy as np
from numpy.polynomial import Polynomial as Poly
from numpy.polynomial.polynomial import polyadd,polymul
from scipy.interpolate import PPoly
from scipy.optimize import brentq

LIMITS=np.array([.6,1.2,6.])
RESERVE=1.001  # Same 0.1% duration reserve as the existing implementation.
PATH_TOL=1e-11


def identity(source):
    t=source['timing']
    data={k:t[k] for k in ('coefficients','polynomial_breaks_unscaled_s','stop_at_corners')}
    data['waypoints']=source.get('path_waypoints',source.get('path'))
    return hashlib.sha256(json.dumps(data,sort_keys=True,allow_nan=False).encode()).hexdigest()


def original(source):
    t=source['timing'];breaks=np.asarray(t['polynomial_breaks_unscaled_s'])
    if not np.array_equal(breaks,np.broadcast_to(breaks[0],breaks.shape)):
        raise ValueError('Mismatched joint spline partitions')
    c=np.moveaxis(np.asarray(t['coefficients'],float),0,-1)
    if not np.isfinite(c).all() or not np.isfinite(breaks).all():raise ValueError('Nonfinite source curve')
    return PPoly(c,breaks[0],extrapolate=False)


def encode(pp):return dict(c=pp.c.tolist(),x=pp.x.tolist())
def decode(data):return PPoly(np.asarray(data['c']),np.asarray(data['x']),extrapolate=False)


def local(pp,lo,hi,j=None,derivative=0):
    """Polynomial on u=[0,1], evaluated from original coefficients (no refit)."""
    return _local_derivative(pp.derivative(derivative),lo,hi,j)


def _local_derivative(pp,lo,hi,j=None):
    """Same polynomial operations, with an already constructed derivative."""
    k=int(np.clip(np.searchsorted(pp.x,(lo+hi)/2,side='right')-1,0,len(pp.x)-2))
    c=pp.c[:,k] if j is None else pp.c[:,k,j]
    if c.ndim!=1:raise ValueError('Coefficient array is not 1-d')
    # The same affine Horner composition as Polynomial.__call__, without
    # constructing a Polynomial/domain/window wrapper for every multiply/add.
    # Keep operand order and NumPy polynomial operations, not fast-math/FMA.
    affine=np.array([lo-pp.x[k],hi-lo])
    result=np.atleast_1d(c[0])
    for coefficient in c[1:]:result=polyadd(coefficient,polymul(result,affine))
    return Poly(result)


def unit_roots(poly):
    roots=poly.trim(tol=1e-15).roots()
    return [float(r.real) for r in roots if abs(r.imag)<1e-7 and 0<r.real<1]


def polynomial_peaks(pp):
    peaks=np.zeros((3,7));jumps=np.zeros(3)
    bounds=np.unique(pp.x)
    derivatives=[pp.derivative(d) for d in range(1,4)]
    for lo,hi in zip(bounds[:-1],bounds[1:]):
        for j in range(7):
            for d in range(1,4):
                p=_local_derivative(derivatives[d-1],lo,hi,j)
                peaks[d-1,j]=max(peaks[d-1,j],np.max(abs(p(np.array([0,1,*unit_roots(p.deriv())])))))
    for x in bounds[1:-1]:
        left=np.nextafter(x,-np.inf)
        for d in range(3):
            jumps[d]=max(jumps[d],float(np.max(abs(pp(left,nu=d)-pp(x,nu=d)))))
    return peaks,jumps


def density_peaks(path,h):
    """Extrema of rational v/a/j, using roots of N' h - power*N*h'.

    Checks both sides of every original spline/density knot; not just a grid.
    All root polynomials are scaled to [0,1] to avoid tiny-interval conditioning.
    """
    boundaries=np.unique(np.r_[path.x,h.x]);boundaries=boundaries[(boundaries>=0)&(boundaries<=1)]
    peaks=np.zeros((3,7));locations=np.zeros((3,7))
    # These derivatives are invariant across every joint and interval. Hoisting
    # their construction preserves coefficient arithmetic and every root check.
    h_derivatives=[h.derivative(d) for d in range(3)]
    path_derivatives=[path.derivative(d) for d in (1,2,3)]
    for lo,hi in zip(boundaries[:-1],boundaries[1:]):
        H,H1,H2=[_local_derivative(p,lo,hi) for p in h_derivatives]
        if min(H(np.array([0,1,*unit_roots(H.deriv())])))<=0:raise ValueError('Nonmonotone time density')
        for j in range(7):
            q1,q2,q3=[_local_derivative(p,lo,hi,j) for p in path_derivatives]
            numerators=[q1,q2*H-q1*H1,q3*H*H-3*q2*H*H1-q1*H*H2+3*q1*H1*H1]
            for k,(N,power) in enumerate(zip(numerators,(1,3,5))):
                roots=unit_roots(N.deriv()*H-power*N*H.deriv())
                at=np.array([0,1,*roots]);values=abs(N(at)/H(at)**power);index=int(np.argmax(values))
                if values[index]>peaks[k,j]:
                    peaks[k,j]=values[index];locations[k,j]=lo+(hi-lo)*at[index]
    return peaks,locations


def scale_needed(peaks):
    return max(float((np.max(peaks[k])/LIMITS[k])**(1/(k+1))) for k in range(3))


def pack_poly(pp,source,method,details=None):
    peaks,jumps=polynomial_peaks(pp)
    if np.any(peaks>LIMITS[:,None]*(1+1e-8)) or np.any(jumps>[1e-9,1e-8,1e-7]):
        raise ValueError('Timing violates continuous limits or C2 continuity')
    return dict(schema='x5-fixed-path-timing-v1',method=method,kind='joint_polynomial',
        duration_s=float(pp.x[-1]),curve=encode(pp),source_path_sha256=identity(source),
        limits=LIMITS.tolist(),peaks=peaks.tolist(),continuity_jumps=jumps.tolist(),
        geometry_preserved=True,details=details or {})


def pack_density(path,h,source,method,details=None):
    peaks,locations=density_peaks(path,h)
    if np.any(peaks>LIMITS[:,None]*(1+1e-8)):raise ValueError('Density law exceeds motion limits')
    integral=h.antiderivative();duration=float(integral(1)-integral(0))
    return dict(schema='x5-fixed-path-timing-v1',method=method,kind='time_density',duration_s=duration,
        path=encode(path),density=encode(h),source_path_sha256=identity(source),limits=LIMITS.tolist(),
        peaks=peaks.tolist(),extrema_s=locations.tolist(),geometry_preserved=True,details=details or {})


class Evaluator:
    """Decode once for playback; never reconstruct geometry or spline per frame."""
    def __init__(self,law):
        self.law=law;self.duration=float(law['duration_s'])
        if law['kind']=='joint_polynomial':self.p=decode(law['curve'])
        elif law['kind']=='time_density':
            self.path=decode(law['path']);self.h=decode(law['density']);self.H=self.h.antiderivative()
            self.offset=self.H(0)
            s=np.unique(np.r_[np.linspace(0,1,1001),self.h.x,self.path.x]);s=s[(s>=0)&(s<=1)]
            # Independently computed knots can differ by one ULP. Do not let
            # an unresolvable time interval look like a nonmonotone clock.
            self.s=s[np.r_[True,np.diff(s)>1e-13]];self.s[0]=0.;self.s[-1]=1.
            self.t=self.H(self.s)-self.offset
            if np.any(np.diff(self.t)<=0):raise ValueError('Nonmonotone saved timing law')
        else:raise ValueError('Unknown timing law')

    def __call__(self,times):
        t=np.clip(np.asarray(times,float),0,self.duration)
        if self.law['kind']=='joint_polynomial':return tuple(self.p(t,nu=k) for k in range(4))
        path,h,H=self.path,self.h,self.H
        s=np.interp(t,self.t,self.s)
        for _ in range(8):s=np.clip(s-(H(s)-self.offset-t)/h(s),0,1)
        if np.max(abs(H(s)-self.offset-t),initial=0)>1e-9:raise ValueError('Time inversion did not converge')
        a,b,c=h(s)[...,None],h(s,nu=1)[...,None],h(s,nu=2)[...,None]
        q,q1,q2,q3=[path(s,nu=k) for k in range(4)]
        return q,q1/a,q2/a**2-q1*b/a**3,q3/a**3-3*q2*b/a**4-q1*c/a**4+3*q1*b*b/a**5


def evaluate(law,times):return Evaluator(law)(times)


def event_times(law):
    if law['kind']=='joint_polynomial':return np.unique(law['curve']['x'])
    h=decode(law['density']);H=h.antiderivative()
    s=np.unique(np.r_[law['path']['x'],h.x]);s=s[(s>=0)&(s<=1)]
    return H(s)-H(0)


def samples(law,source,step=.01):
    grid=np.unique(np.r_[np.arange(0,law['duration_s'],step),event_times(law),law['duration_s']])
    q,v,a,_=evaluate(law,grid);old=original(source)
    q[0]=old(0);q[-1]=old(1);v[[0,-1]]=0;a[[0,-1]]=0
    return [dict(time_s=float(t),positions_rad=x.tolist(),velocities_rad_s=y.tolist(),accelerations_rad_s2=z.tolist())
            for t,x,y,z in zip(grid,q,v,a)]


def global_law(source):
    path=original(source);scale=float(source['timing']['time_scale'])
    degree=path.c.shape[0]-1;c=path.c/scale**np.arange(degree,-1,-1)[:,None,None]
    return pack_poly(PPoly(c,path.x*scale),source,'global',dict(original_timing=True))


def straight_blocks(source,pass_collinear=True):
    """Keep every waypoint; group ONLY numerically identical straight segments.

    Grouping permits crossing redundant collinear waypoints without stopping.
    No corner is rounded, no point is discarded, and no path is refitted.
    """
    if not source['timing']['stop_at_corners']:return None
    q=np.asarray(source.get('path_waypoints',source.get('path')),float)
    knots=np.asarray(source['timing']['knots_unscaled_s']);p=original(source)
    if len(knots)!=len(q):raise ValueError('Waypoint/curve mismatch')
    for i,(lo,hi) in enumerate(zip(knots[:-1],knots[1:])):
        for j in range(7):
            expected=Poly([q[i,j],0,0,10*(q[i+1,j]-q[i,j]),-15*(q[i+1,j]-q[i,j]),6*(q[i+1,j]-q[i,j])])
            if np.max(abs((local(p,lo,hi,j)-expected).coef))>1e-9:
                raise ValueError('Corner-stop flag does not describe the supplied exact path')
    groups=[];a=0
    while a<len(q)-1:
        b=a+1
        while pass_collinear and b<len(q)-1:
            delta=q[b+1]-q[a];length2=delta@delta
            if length2<1e-24:break
            u=(q[a:b+2]-q[a])@delta/length2
            residual=q[a:b+2]-(q[a]+u[:,None]*delta)
            if np.max(abs(residual))>1e-13 or np.any(np.diff(u)<=0):break
            b+=1
        groups.append((a,b));a=b
    return q,groups


def scurve(distance):
    """Closed-form time-optimal symmetric rest-to-rest scalar S-curve.

    Constant-jerk phases: +J,0,-J,0,-J,0,+J. Handles triangular jerk,
    acceleration plateaus, and velocity cruise without an iterative solver.
    """
    V,A,J=LIMITS
    if not np.isfinite(distance) or distance<=0:raise ValueError('Positive segment distance required')
    tj=min(A/J,math.sqrt(V/J));ta=max(0.,V/(J*tj)-tj)
    if distance>=V*(2*tj+ta):tv=distance/V-(2*tj+ta)
    else:
        tv=0.
        if distance>=2*A**3/J**2:
            tj=A/J;ta=max(0.,(-3*tj+math.sqrt(tj*tj+4*distance/A))/2)
        else:tj=(distance/(2*J))**(1/3);ta=0.
    durations=[tj,ta,tj,tv,tj,ta,tj];jerks=[J,0,-J,0,-J,0,J]
    x=v=a=0.;breaks=[0.];coeff=[]
    for dt,j in zip(durations,jerks):
        if dt<=1e-15:continue
        coeff.append([j/6,a/2,v,x]);breaks.append(breaks[-1]+dt)
        x+=v*dt+a*dt**2/2+j*dt**3/6;v+=a*dt+j*dt**2/2;a+=j*dt
    if abs(x-distance)>1e-10 or abs(v)>1e-10 or abs(a)>1e-10:raise ValueError('S-curve endpoint error')
    return np.array(coeff).T,np.array(breaks)


def local_law(source,method,pass_collinear=True):
    blocks=straight_blocks(source,pass_collinear)
    if blocks is None:
        law=global_law(source);law['method']=method
        law['details']['curved_path_policy']='Keep original continuous law; no new stops or geometric approximation'
        return law
    q,groups=blocks;parts=[];breaks=[0.];waypoint_times=np.zeros(len(q));block_report=[]
    for first,last in groups:
        delta=q[last]-q[first];D=float(np.max(abs(delta)))
        if method=='local_quintic':
            duration=max(1.875*D/LIMITS[0],math.sqrt((10/math.sqrt(3))*D/LIMITS[1]),(60*D/LIMITS[2])**(1/3))*RESERVE
            c=np.stack([6*delta/duration**5,-15*delta/duration**4,10*delta/duration**3,np.zeros(7),np.zeros(7),q[first]])[:,None,:]
            x=np.array([0.,duration])
        else:
            scalar,x=scurve(D);x*=RESERVE
            scalar=scalar/RESERVE**np.arange(3,-1,-1)[:,None]
            c=scalar[:,:,None]*(delta/D);c[-1]+=q[first]
            duration=x[-1]
        part=PPoly(c,x)
        for index in range(first,last+1):
            if index==first:at=0.
            elif index==last:at=duration
            else:
                j=int(np.argmax(abs(delta)));target=q[index,j]
                at=brentq(lambda t:part(t)[j]-target,0,duration,xtol=1e-13)
            waypoint_times[index]=breaks[-1]+at
        block_report.append(dict(first=first,last=last,duration_s=float(duration)))
        parts.extend(np.moveaxis(c,1,0));breaks.extend((breaks[-1]+x[1:]).tolist())
    law=pack_poly(PPoly(np.stack(parts,axis=1),np.array(breaks)),source,method,
        dict(blocks=block_report,waypoint_times_s=waypoint_times.tolist(),all_original_waypoints_retained=True,
             pass_collinear=pass_collinear,geometric_tolerance_rad=1e-13,
             solution='closed-form optimum in rest-to-rest quintic family' if method=='local_quintic' else
                      'closed-form time-optimal jerk-limited rest-to-rest straight blocks'))
    if np.max(abs(evaluate(law,waypoint_times)[0]-q))>PATH_TOL:raise ValueError('Lost an original waypoint')
    return law
