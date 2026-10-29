"""Diagnostic-only per-joint limits, reusing the unchanged experiment optimizer.

No device imports, persisted settings, application edits or execution export.
Temporary overrides are confined to one fresh research process and restored.
The only numerical extension is replacing scalar-per-derivative bounds by a
3 by 7 matrix. Uniform matrices must reproduce the original implementation.
"""
from contextlib import contextmanager
from pathlib import Path
import hashlib

import numpy as np
from scipy.interpolate import PPoly
from scipy.sparse import csc_matrix

from drum_planner.timing import core
from drum_planner.experiment import fast_timing as fast


def checked_limits(limits):
    limits=np.asarray(limits,float)
    if limits.shape!=(3,7) or not np.isfinite(limits).all() or np.any(limits<=0):
        raise ValueError('Expected positive finite velocity/acceleration/jerk limits for all seven joints')
    return limits.copy()


def required_scale(peaks,limits):
    return float(np.max((np.asarray(peaks)/limits)**(1/np.arange(1,4)[:,None])))


def certify(path,density,source,limits,*,reserved=False,geometry=None,log=lambda _:None):
    h=PPoly(density.c.copy(),density.x.copy(),extrapolate=False)
    if not np.isfinite(h.c).all() or not np.isfinite(h.x).all():raise ValueError('Nonfinite density')
    cuts=np.unique(h.x);cuts=cuts[(cuts>0)&(cuts<1)]
    for d in (0,1):
        if not np.allclose(h(np.nextafter(cuts,-np.inf),nu=d),h(cuts,nu=d),rtol=1e-9,atol=1e-8):
            raise ValueError('Discontinuous density or acceleration')
    try:peaks,where=fast.density_peaks(path,h);checker='batched polynomial roots'
    except np.linalg.LinAlgError:
        peaks,where=core.density_peaks(path,h,geometry=geometry);checker='original polynomial roots'
        log('CONTINUOUS CHECK FALLBACK')
    if not np.isfinite(peaks).all():raise ValueError('Nonfinite extrema')
    needed=required_scale(peaks,limits)
    scale=max(1.,needed*core.RESERVE) if reserved else max(1.,needed)*core.RESERVE
    h.c*=scale;peaks/=scale**np.arange(1,4)[:,None];duration=float(h.integrate(0,1))
    if not np.isfinite(duration) or duration<=0 or np.any(peaks>limits):raise ValueError('Per-joint verification failed')
    return dict(schema='x5-fixed-path-timing-v1',method='diagnostic_per_joint_sparse_density',
        kind='time_density',duration_s=duration,path=core.encode(path),density=core.encode(h),
        source_path_sha256=core.identity(source),limits=limits.tolist(),peaks=peaks.tolist(),
        extrema_s=where.tolist(),geometry_preserved=True,
        details=dict(correction_scale=scale,continuous_checker=checker),
        diagnostic_only=True,motion_authorized=False,hardware_execution_available=False)


def sparse_solve(B,q,weights,initial,limits):
    import casadi as ca
    z=ca.MX.sym('clock',len(initial))
    H,H1,H2=[ca.DM(csc_matrix(b))@z for b in B]
    h,h1,h2=[ca.repmat(v,1,7) for v in (H,H1,H2)]
    q1,q2,q3=map(ca.DM,q);m=len(q[0])
    acceleration=q2/h**2-q1*h1/h**3
    jerk=q3/h**3-3*q2*h1/h**4-q1*h2/h**4+3*q1*h1**2/h**5
    bound=lambda i:ca.repmat(ca.DM(limits[i].reshape(1,7)),m,1)
    g=ca.vertcat(H-np.max(abs(q[0])/limits[0],axis=1),
                 ca.vec(acceleration/bound(1)),ca.vec(jerk/bound(2)))
    solver=ca.nlpsol('research_joint_clock','ipopt',dict(x=z,f=ca.dot(weights,z),g=g),
        {'print_time':False,'ipopt.print_level':0,'ipopt.sb':'yes','ipopt.tol':1e-7,
         'ipopt.max_iter':fast.MAX_ITERATIONS,'ipopt.max_wall_time':fast.SOLVE_WALL_SECONDS,
         'ipopt.acceptable_iter':0,'ipopt.mu_strategy':'adaptive','ipopt.bound_relax_factor':0.,
         'ipopt.linear_solver':'mumps'})
    result=solver(x0=initial,lbx=.005,ubx=5.,lbg=np.r_[np.zeros(m),-np.ones(14*m)],
                  ubg=np.r_[np.full(m,np.inf),np.ones(14*m)])
    stats=solver.stats()
    return np.asarray(result['x']).ravel(),dict(iterations=int(stats['iter_count']),
        status=stats['return_status'],converged=bool(stats['success']),casadi_version=ca.__version__)


@contextmanager
def policy(limits,*,refined=False):
    limits=checked_limits(limits)
    original=(core.LIMITS,fast.certify,fast.sparse_solve,fast.CONTROLS,fast.MAX_ITERATIONS,fast.SOLVE_WALL_SECONDS)
    try:
        # The existing vectorized CPU seed already broadcasts a per-joint row.
        core.LIMITS=limits
        fast.certify=lambda p,h,s,**kw:certify(p,h,s,limits,**kw)
        fast.sparse_solve=lambda B,q,w,x:sparse_solve(B,q,w,x,limits)
        if refined:fast.CONTROLS=192;fast.MAX_ITERATIONS=300;fast.SOLVE_WALL_SECONDS=3.
        yield
    finally:
        core.LIMITS,fast.certify,fast.sparse_solve,fast.CONTROLS,fast.MAX_ITERATIONS,fast.SOLVE_WALL_SECONDS=original


def optimize(source,limits,*,refined=False,log=lambda _:None):
    with policy(limits,refined=refined):law=fast.optimize(source,log)
    law['details']['research_extension_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    law['details']['per_joint_limits_research_only']=True
    return law
