"""Device-free exact-curve timing and checked-prefix recovery construction."""
import numpy as np
from scipy.optimize import minimize_scalar
from ..execution import TRACK_L1
from .curve import Curve


def speed(value):
    value=float(value)
    if not np.isfinite(value) or not .25<=value<=1.:
        raise ValueError('Hardware speed must be 0.25–1.00 (25–100% of validated limits)')
    return value


def blend(s):return 10*s**3-15*s**4+6*s**5


def peaks(curve):
    if getattr(curve,'retimed',None) is not None:return curve._peaks.copy()
    from ..planning import extrema
    return np.array([[extrema(p,k)/curve.scale**k for p in curve.curves] for k in (1,2,3)])


def validate_timing(curve):
    p=peaks(curve)
    if np.any(p>np.array([.6,1.2,6.])[:,None]+1e-8):
        raise ValueError('Curve exceeds validated velocity/acceleration/jerk limits')
    if getattr(curve,'retimed',None) is not None:
        _,v,a,_=curve.retimed([0,curve.duration])
        if max(np.max(abs(v)),np.max(abs(a)))>1e-7:
            raise ValueError('Curve must start/end at rest with zero acceleration')
        return
    for ppoly in curve.curves:
        for k in (1,2):
            if np.max(abs(ppoly.derivative(k)([0,curve.duration/curve.scale])))/curve.scale**k>1e-7:
                raise ValueError('Curve must start/end at rest with zero acceleration')


def recovery(curve,held,tau,rate):
    """Retrace only the already-commanded prefix, never go on toward the goal.

    A tracking offset needs a SMALL independently collision-checked connector
    to the nearest earlier point. No arbitrary replan, center or endpoint jump.
    Time-warp derivative bounds cover the composed curve, including jerk.
    """
    held=np.asarray(held,dtype=float);rate=min(.5,speed(rate))
    if held.shape!=(7,) or not np.isfinite(held).all() or not np.isfinite(tau) or not 0<=tau<=curve.duration:
        raise ValueError('Invalid held pose/progress')
    # Search only near the last executed progress, not another branch where
    # the arm happened to visit a similar pose earlier in the cycle.
    lower=max(0.,tau-2.)
    grid=np.linspace(lower,tau,max(2,int((tau-lower)/.005)+1))
    costs=np.array([np.sum((curve.at(float(t))-held)**2) for t in grid]);i=int(costs.argmin())
    a,b=grid[max(0,i-1)],grid[min(len(grid)-1,i+1)]
    candidates=[float(grid[i]),float(tau),float(lower)]
    if b>a:
        fit=minimize_scalar(lambda t:float(np.sum((curve.at(t)-held)**2)),bounds=(a,b),method='bounded',options={'xatol':1e-12})
        candidates.append(float(fit.x))
    projected=min(candidates,key=lambda t:float(np.sum((curve.at(t)-held)**2)))
    anchor=curve.at(projected);offset=float(np.sum(abs(anchor-held)))
    if offset>TRACK_L1:
        raise ValueError('Held pose is too far from the traveled path; no blind recovery or relaxation')
    d=float(np.max(abs(anchor-held)))
    bridge=max(.35,1.875*d/(.6*rate),np.sqrt(5.774*d/(1.2*rate**2)),np.cbrt(60*d/(6*rate**3)))
    v,a,j=peaks(curve).max(axis=1);u=1.875*projected;w=5.774*projected;z=60*projected
    duration=max(.35,u*v/(.6*rate),np.sqrt((a*u*u+v*w)/(1.2*rate**2)),
                 np.cbrt((j*u**3+3*a*u*w+v*z)/(6*rate**3)))
    return dict(schema='x5-baseline-prefix-return-v1',held=held.tolist(),tau=projected,
                last_commanded_tau=float(tau),connector_l1_rad=offset,bridge_s=bridge,return_s=duration,
                recovery_rate=rate,original_curve_duration_s=curve.duration)


def recovery_at(curve,spec,elapsed):
    b=spec['bridge_s'];duration=spec['return_s'];elapsed=float(np.clip(elapsed,0,b+duration))
    if elapsed<b:
        s=blend(elapsed/b)
        return np.array(spec['held'])*(1-s)+curve.at(spec['tau'])*s,spec['tau'],'RETURN_CONNECTOR'
    tau=float(np.clip(spec['tau']*(1-blend((elapsed-b)/duration)),0,spec['tau']))
    return curve.at(tau),tau,'RETURN_PREFIX'


def recovery_samples(curve,spec):
    duration=spec['bridge_s']+spec['return_s']
    times=np.unique(np.r_[np.arange(0,duration,.002),spec['bridge_s'],duration])
    return np.array([recovery_at(curve,spec,t)[0] for t in times])
