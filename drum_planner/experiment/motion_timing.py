"""Per-joint, anytime fixed-path timing, confined to software experiments.

Explicit arguments replace diagnostic global overrides. Original hardware,
legacy fast_timing and --fast policies are untouched. Every retained clock is
continuously checked; additional work may improve it but cannot replace it with
a slower/invalid candidate. The geometric polynomial is never changed.
"""
import hashlib
from pathlib import Path
import time

import numpy as np
from scipy.interpolate import BSpline, PPoly
from scipy.optimize import minimize
from scipy.sparse import csc_matrix

from ..common import atomic_json
from ..timing import core
from ..timing.benchmark import check_geometry
from ..timing.vulkan.optimizer import arrays
from .fast_timing import grid_for
from .timing_extrema import density_peaks
from .motion_settings import checked_limits, summary

# Work is nested: each quality retains the best fully checked lower-quality
# incumbent. Higher-resolution work is optional, never an accuracy guarantee.
POLICIES = {
    'fast': [(96, 90, 1.)],
    'balanced': [(96, 90, 1.), (96, 400, 4.)],
    'thorough': [(96, 90, 1.), (96, 400, 4.), (192, 700, 8.)],
}


def required_scale(peaks, limits):
    return float(np.max((np.asarray(peaks)/limits)**(1/np.arange(1, 4)[:, None])))


class SeedProblem:
    """Original vectorized 24-control seed, with explicit per-joint constraints."""
    def __init__(self, B, q, weights, limits):
        self.B=B;self.q=q;self.weights=weights;self.limits=limits
        self.lam=np.zeros((len(B[0]), 3, 7));self.rho=1.

    def calc(self, x, update=False):
        H,H1,H2=[(b@x)[:, None] for b in self.B];q1,q2,q3=self.q
        vals=[q1/H, q2/H**2-q1*H1/H**3,
              q3/H**3-3*q2*H1/H**4-q1*H2/H**4+3*q1*H1**2/H**5]
        parts=[(-q1/H**2,None,None),
               (-2*q2/H**3+3*q1*H1/H**4,-q1/H**3,None),
               (-3*q3/H**4+12*q2*H1/H**5+4*q1*H2/H**5-15*q1*H1**2/H**6,
                -3*q2/H**4+6*q1*H1/H**5,-q1/H**4)]
        g=np.stack([abs(v)/self.limits[i]-1 for i,v in enumerate(vals)], axis=1)
        z=np.maximum(0, self.lam+self.rho*g)
        cost=self.weights@x+np.sum((z*z-self.lam*self.lam)/(2*self.rho));grad=self.weights.copy()
        for d,partials in enumerate(parts):
            factor=z[:, d, :]*np.sign(vals[d])/self.limits[d]
            for part,b in zip(partials,self.B):
                if part is not None:grad+=b.T@np.sum(factor*part,axis=1)
        if update:self.lam=z
        return cost,grad,float(np.max(g))


def seed_solve(B, q, weights, limits):
    x=np.ones(len(weights));problem=SeedProblem(B,q,weights,limits);iterations=0
    for outer in range(7):
        result=minimize(lambda z:problem.calc(z)[:2],x,method='L-BFGS-B',jac=True,
            bounds=[(.005,5.)]*len(x),options=dict(maxiter=300,ftol=1e-11,gtol=1e-6,maxls=25,maxcor=8))
        x=result.x;iterations+=result.nit;_,_,violation=problem.calc(x,True)
        if outer>=3 and violation<1e-5:break
        problem.rho*=5.
    if not np.isfinite(x).all():raise ValueError('Nonfinite CPU seed')
    return x,iterations


def sparse_solve(B, q, weights, initial, limits, max_iterations, wall_seconds):
    import casadi as ca
    z=ca.MX.sym('clock',len(initial));m=len(q[0])
    H,H1,H2=[ca.DM(csc_matrix(b))@z for b in B]
    h,h1,h2=[ca.repmat(v,1,7) for v in (H,H1,H2)]
    q1,q2,q3=map(ca.DM,q)
    acc=q2/h**2-q1*h1/h**3
    jerk=q3/h**3-3*q2*h1/h**4-q1*h2/h**4+3*q1*h1**2/h**5
    bound=lambda d:ca.repmat(ca.DM(limits[d].reshape(1,7)),m,1)
    g=ca.vertcat(H-np.max(abs(q[0])/limits[0],axis=1),ca.vec(acc/bound(1)),ca.vec(jerk/bound(2)))
    solver=ca.nlpsol('experiment_joint_clock','ipopt',dict(x=z,f=ca.dot(weights,z),g=g),
        {'print_time':False,'ipopt.print_level':0,'ipopt.sb':'yes','ipopt.tol':1e-7,
         'ipopt.max_iter':max_iterations,'ipopt.max_wall_time':wall_seconds,
         'ipopt.acceptable_iter':0,'ipopt.mu_strategy':'adaptive','ipopt.bound_relax_factor':0.,
         'ipopt.linear_solver':'mumps'})
    result=solver(x0=initial,lbx=.005,ubx=5.,lbg=np.r_[np.zeros(m),-np.ones(14*m)],
                  ubg=np.r_[np.full(m,np.inf),np.ones(14*m)])
    stats=solver.stats()
    return np.asarray(result['x']).ravel(),dict(iterations=int(stats['iter_count']),
        status=stats['return_status'],converged=bool(stats['success']),casadi_version=ca.__version__)


def certify(path, density, source, limits, *, log=lambda _:None):
    h=PPoly(density.c.copy(),density.x.copy(),extrapolate=False)
    if not np.isfinite(h.c).all() or not np.isfinite(h.x).all():raise ValueError('Nonfinite time density')
    cuts=np.unique(h.x);cuts=cuts[(cuts>0)&(cuts<1)]
    for d in (0,1):
        if not np.allclose(h(np.nextafter(cuts,-np.inf),nu=d),h(cuts,nu=d),rtol=1e-9,atol=1e-8):
            raise ValueError('Discontinuous time density or acceleration')
    checker='batched polynomial roots'
    try:peaks,where=density_peaks(path,h)
    except np.linalg.LinAlgError:
        log('CONTINUOUS CHECK FALLBACK — original polynomial roots')
        peaks,where=core.density_peaks(path,h);checker='original polynomial roots'
    if not np.isfinite(peaks).all():raise ValueError('Nonfinite continuous extrema')
    # Normalize even an over-conservative candidate to its limiting constraint.
    # This is a uniform clock change, not a path change. Keep the original reserve.
    scale=required_scale(peaks,limits)*core.RESERVE
    if not np.isfinite(scale) or scale<=0:raise ValueError('Nonmoving/invalid density candidate')
    h.c*=scale;peaks/=scale**np.arange(1,4)[:,None]
    duration=float(h.integrate(0,1))
    if not np.isfinite(duration) or duration<=0 or np.any(peaks>limits):
        raise ValueError('Continuous per-joint timing verification failed')
    return dict(schema='x5-fixed-path-timing-v1',method='experiment_per_joint_sparse_density',
        kind='time_density',duration_s=duration,path=core.encode(path),density=core.encode(h),
        source_path_sha256=core.identity(source),limits=limits.tolist(),peaks=peaks.tolist(),
        extrema_s=where.tolist(),geometry_preserved=True,
        details=dict(correction_scale=scale,continuous_checker=checker))


def optimize(source, log=lambda _:None, *, limits, quality='balanced'):
    limits=checked_limits(limits)
    if quality not in POLICIES:raise ValueError('Unknown timing quality')
    if source['timing']['stop_at_corners']:
        raise ValueError('Per-joint timing requires the continuous Workspace curve')
    began=time.monotonic();path=core.original(source);history=[];warnings=[]
    log(f'CPU SPARSE TIMING — intentional CPU backend; {quality}; 21 explicit joint limits; no result cache')
    # A fresh, exactly checked uniform clock is also a robust normalization for
    # very small/large custom limits. The source's old timing is not a speed cap.
    best=certify(path,PPoly(np.ones((1,1)),[0.,1.]),source,limits,log=log)
    uniform_s=best['duration_s'];uniform_base=uniform_s/core.RESERVE;selected='uniform'
    source_base=float(source['timing']['time_scale'])/core.RESERVE
    if not np.isfinite(source_base) or source_base<=0:raise ValueError('Invalid source clock')
    # Retain the proven source-clock conditioning where reasonable. This is
    # just a coefficient unit, not a speed bound; rescale extreme custom inputs.
    base=max(uniform_base,min(source_base,8*uniform_base))
    fresh_seed_density=core.decode(best['density'])
    knots,basis,_,B,q,weights=arrays(path,base,24)
    try:
        seed,iterations=seed_solve(B,q,weights,limits)
        fresh_seed_density=PPoly.from_spline(BSpline(knots,seed*base,3))
        law=certify(path,fresh_seed_density,source,limits,log=log)
        if law['duration_s']<best['duration_s']:best=law;selected='seed'
        history.append(dict(stage='fresh_seed',iterations=iterations,duration_s=law['duration_s']))
    except (ValueError,RuntimeError,FloatingPointError) as exc:
        warnings.append('CPU seed failed: '+str(exc));log('CPU SEED FALLBACK — checked uniform clock: '+str(exc))
    last_converged=False;last_correction=np.inf;last_n=None;last_grid=None
    for stage,(n,iterations,wall_seconds) in enumerate(POLICIES[quality]):
        # Continuing an already converged same-resolution solve adds no value.
        if n==last_n and last_converged and last_correction<=1.0015:
            history.append(dict(stage='refinement_skipped',reason='same-resolution converged',controls=n));continue
        before=best['duration_s']
        knots=np.r_[np.zeros(4),np.linspace(0,1,n-2)[1:-1],np.ones(4)]
        basis=BSpline(knots,np.eye(n),3,extrapolate=False)
        points=np.linspace(0,1,max(201,n*4))
        # A globally stretched certified seed can be a poor nonlinear starting
        # point. Keep its fresh raw coefficients as an initialization only,
        # while the independently certified incumbent remains the output guard.
        initial_density=(fresh_seed_density if n==96 and not last_converged else core.decode(best['density']))
        x=np.clip(np.linalg.lstsq(basis(points),initial_density(points)/base,rcond=None)[0],.005,5.)
        weights=basis.antiderivative()(1)-basis.antiderivative()(0)
        grid=grid_for(knots,path,() if last_grid is None else last_grid)
        for attempt in range(2):
            t=time.monotonic();B=np.array([basis(grid,nu=d)*base for d in range(3)])
            q=np.array([path(grid,nu=d) for d in (1,2,3)])
            try:
                x,stats=sparse_solve(B,q,weights,x,limits,iterations,wall_seconds)
                if not np.isfinite(x).all() or np.any(x<=0):raise ValueError('Invalid sparse coefficients')
                law=certify(path,PPoly.from_spline(BSpline(knots,x*base,3)),source,limits,log=log)
                accepted=law['duration_s']<best['duration_s']
                if accepted:best=law;selected='sparse'
                last_converged=stats['converged'];last_correction=law['details']['correction_scale']
                history.append(dict(stage='sparse',quality_stage=stage,controls=n,attempt=attempt+1,
                    grid_points=len(grid),duration_s=law['duration_s'],accepted=accepted,
                    correction_scale=last_correction,elapsed_s=time.monotonic()-t,**stats))
                log(f'TIMING {n} controls: {law["duration_s"]:.6f}s movement; {stats["status"]}; '
                    +('new checked best' if accepted else 'keeping faster checked incumbent'))
                if not stats['converged']:
                    warnings.append('Sparse bounded stop: '+stats['status']);break
                if last_correction<=1.004 or attempt==1:break
                where=np.asarray(law['extrema_s']).ravel()
                grid=grid_for(knots,path,np.r_[grid,where,np.nextafter(where,-np.inf)])
                log('CPU SPARSE: bounded extrema refinement')
            except (ImportError,OSError,RuntimeError,ValueError,FloatingPointError) as exc:
                warnings.append('Sparse refinement failed: '+str(exc))
                log('CPU TIMING FALLBACK — retaining checked incumbent: '+str(exc))
                last_converged=False
                break
        last_grid=grid;last_n=n
        history.append(dict(stage='quality_stage_complete',controls=n,
                            improvement_s=before-best['duration_s'],best_duration_s=best['duration_s']))
        if warnings and warnings[-1].startswith('Sparse refinement failed:'):break
    fallback=selected!='sparse'
    label=('CPU TIMING FALLBACK — checked '+selected if fallback else 'CPU sparse timing — '+quality)
    if fallback:log(label+'; sparse refinement supplied no faster accepted clock')
    best['details'].update(actual_backend='cpu',requested_backend='cpu_sparse',backend_label=label,
        selected_stage=selected,fallback_used=fallback,fallback_reason='; '.join(warnings) if warnings else
        ('Sparse refinement did not improve incumbent' if fallback else None),warnings=warnings,history=history,
        calculation_seconds=time.monotonic()-began,uniform_duration_s=uniform_s,
        timing_quality=quality,precision='float64',cpu_reference_used=False,result_cache_used=False,
        global_optimum_claimed=False,gpu_native_optimizer=False,per_joint_limits=True,
        constraint_summary=summary([best],limits),
        implementation_sha256=hashlib.sha256(Path(__file__).read_bytes()+
            Path(__file__).with_name('timing_extrema.py').read_bytes()).hexdigest())
    log(f'TIMING READY: {best["duration_s"]:.6f}s movement, {best["details"]["calculation_seconds"]:.3f}s calculation')
    return best


def validate(law, source, folder, collision=True, *, oracle=None):
    """Unchanged dense/native final proof, using the exact requested 21 limits."""
    limits=checked_limits(law['limits'])
    from ..rest_paths import verify
    evaluator=core.Evaluator(law)
    transition=verify(evaluator.path if law['kind']=='time_density' else evaluator.p,source['request'])
    if law.get('stationary'):
        old=core.original(source);new=core.decode(law['curve'])
        if (law['duration_s']!=0 or law['source_path_sha256']!=core.identity(source) or
            np.any(old.c[:-1]!=0) or np.any(new.c[:-1]!=0) or
            not np.array_equal(old(0),old(1)) or not np.array_equal(new(0),old(0))):
            raise ValueError('Invalid stationary trajectory')
        geometry=dict(type='identical stationary pose; zero movement duration',max_error_rad=0.)
        grid=np.array([0.])
    else:
        geometry=check_geometry(law,source)
        grid=np.unique(np.r_[np.arange(0,law['duration_s'],.002),core.event_times(law),law['duration_s']])
    q,v,a,j=core.evaluate(law,grid)
    peaks=np.array([np.max(abs(x),axis=0) for x in (v,a,j)])
    if np.any(peaks>limits*(1+1e-7)):raise ValueError('Dense per-joint motion limits exceeded')
    if np.max(abs(v[[0,-1]]))>1e-8 or np.max(abs(a[[0,-1]]))>1e-7:raise ValueError('Non-rest endpoints')
    result=dict(geometry=geometry,dense_samples=len(grid),dense_peaks=peaks.tolist(),
                limits=limits.tolist(),motor_commands=0)
    if transition is not None:result['rest_transition']=transition
    if collision:
        from .runtime import Oracle
        from .algorithms import dense_check
        folder=Path(folder);folder.mkdir(parents=True,exist_ok=True);atomic_json(folder/'request.json',source['request'])
        owned=oracle is None
        if owned:oracle=Oracle(folder/'request.json',folder)
        try:
            if np.any(q<np.array(oracle.lower)-1e-10) or np.any(q>np.array(oracle.upper)+1e-10):raise ValueError('Joint bounds')
            ok,proof=dense_check(oracle,q)
            if not ok:raise ValueError('Retimed curve collision/zone failure: '+str(proof))
            result['collision_zone']=proof
        finally:
            if owned:oracle.close()
    return result
