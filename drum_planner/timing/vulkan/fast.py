"""Bounded GPU-native timing. NO reference solve, SLSQP, result cache or training.

One low-dimensional clock is optimized on-device. Continuous rational extrema
then determine an exact homogeneous scale correction, with the unchanged reserve.
The original CPU policies and hardware dispatch are not modified.
"""
import time
import hashlib
import fcntl
from pathlib import Path
import numpy as np
from scipy.interpolate import BSpline,PPoly
from .. import core
from .device import Device,GPUUnavailable,GPUExecutionError,fingerprint,BUILD
from .optimizer import arrays

CONTROLS=24

def optimize(source,log=lambda _:None,*,controls=CONTROLS,first_blocks=64,later_blocks=32):
    began=time.monotonic();path=core.original(source)
    base=float(source['timing']['time_scale'])/core.RESERVE
    knots,basis,grid,B,q,weights=arrays(path,base,controls)
    x=np.ones(controls);backend='vulkan';device_name=None;device_identity=None;fallback_reason=None;iterations=0
    try:gpu=Device(B,q,weights,x)
    except GPUUnavailable as exc:
        backend='cpu';fallback_reason=str(exc)
        log('*** GPU UNAVAILABLE — FAST CPU TIMING FALLBACK *** '+fallback_reason)
        x,iterations=cpu_solve(B,q,weights,x)
    else:
        with gpu, (BUILD/'solve.lock').open('a') as lock:
            log('GPU COMPUTE: '+gpu.name+' (FP64); waiting for exclusive timing slot')
            fcntl.flock(lock,fcntl.LOCK_EX)
            solve_began=time.monotonic()
            device_name=gpu.name;device_identity=gpu.identity;rho=1.
            for outer in range(7):
                for block in range(first_blocks if outer==0 else later_blocks):
                    x,state=gpu.step();iterations+=32
                    if time.monotonic()-solve_began>8.:
                        raise GPUExecutionError('Fast GPU timing exceeded its bounded solve time')
                    if state[7]>=64 and state[4]<1e-6:break
                if outer>=3 and state[8]<1e-5:break
                rho*=5.;gpu.dual(rho)
    solve_seconds=time.monotonic()-began
    h=PPoly.from_spline(BSpline(knots,x*base,3))
    peaks,where=core.density_peaks(path,h)
    scale=max(1.,core.scale_needed(peaks))*core.RESERVE
    h.c*=scale;peaks/=scale**np.arange(1,4)[:,None]
    duration=float(h.integrate(0,1))
    if not np.isfinite(duration) or duration<=0 or np.any(peaks>core.LIMITS[:,None]):
        raise ValueError('Fast timing continuous verification failed')
    uniform_selected=False
    if duration>float(source['timing']['duration_s']):
        # A bounded optimizer is allowed not to converge, never to make the
        # simple feasible global clock worse. Recheck this exact replacement.
        h=PPoly(np.array([[float(source['timing']['time_scale'])]]),[0.,1.])
        peaks,where=core.density_peaks(path,h)
        scale=max(1.,core.scale_needed(peaks)*core.RESERVE)
        h.c*=scale;peaks/=scale**np.arange(1,4)[:,None]
        duration=float(h.integrate(0,1));uniform_selected=True
        log('FAST TIMING: bounded optimizer did not improve the feasible uniform clock; using checked uniform clock')
    if not np.isfinite(duration) or duration<=0 or not np.isfinite(peaks).all() or np.any(peaks>core.LIMITS[:,None]):
        raise ValueError('Fast timing final continuous verification failed')
    law=dict(schema='x5-fixed-path-timing-v1',method='fast_vulkan_density' if backend=='vulkan' else 'fast_cpu_density',kind='time_density',
        duration_s=duration,path=core.encode(path),density=core.encode(h),source_path_sha256=core.identity(source),
        limits=core.LIMITS.tolist(),peaks=peaks.tolist(),extrema_s=where.tolist(),geometry_preserved=True,
        details=dict(gpu_native_optimizer=backend=='vulkan',actual_backend=backend,device=device_name,
            device_identity=device_identity,fallback_reason=fallback_reason,uniform_clock_selected=uniform_selected,
            implementation_sha256=hashlib.sha256(Path(__file__).read_bytes()+fingerprint().encode()).hexdigest(),
            precision='float64',controls=controls,iterations=iterations,correction_scale=scale,
            optimizer_s=solve_seconds,continuous_check_s=time.monotonic()-began-solve_seconds,
            calculation_seconds=time.monotonic()-began,cpu_reference_used=False,result_cache_used=False,
            global_optimum_claimed=False))
    log(f'FAST {backend.upper()} TIMING: {duration:.4f}s movement; {time.monotonic()-began:.3f}s calculation; '
        f'{controls} controls; no CPU reference / no result cache')
    return law


class Problem:
    """Vectorized bounded fallback, not the original full-refinement CPU solver."""
    def __init__(self,B,q,w):
        self.B=B;self.q=q;self.w=w;self.lam=np.zeros((len(B[0]),3,7));self.rho=1.
    def calc(self,x,update=False):
        H,H1,H2=[(b@x)[:,None] for b in self.B];q1,q2,q3=self.q
        vals=[q1/H,q2/H**2-q1*H1/H**3,q3/H**3-3*q2*H1/H**4-q1*H2/H**4+3*q1*H1**2/H**5]
        parts=[(-q1/H**2,None,None),(-2*q2/H**3+3*q1*H1/H**4,-q1/H**3,None),
            (-3*q3/H**4+12*q2*H1/H**5+4*q1*H2/H**5-15*q1*H1**2/H**6,
             -3*q2/H**4+6*q1*H1/H**5,-q1/H**4)]
        g=np.stack([abs(v)/core.LIMITS[i]-1 for i,v in enumerate(vals)],axis=1)
        z=np.maximum(0,self.lam+self.rho*g);cost=self.w@x+np.sum((z*z-self.lam*self.lam)/(2*self.rho));grad=self.w.copy()
        for d,partials in enumerate(parts):
            factor=z[:,d,:]*np.sign(vals[d])/core.LIMITS[d]
            for part,b in zip(partials,self.B):
                if part is not None:grad+=b.T@np.sum(factor*part,axis=1)
        if update:self.lam=z
        return cost,grad,float(np.max(g))


def cpu_solve(B,q,w,x):
    from scipy.optimize import minimize
    problem=Problem(B,q,w);iterations=0
    for outer in range(7):
        result=minimize(lambda z:problem.calc(z)[:2],x,method='L-BFGS-B',jac=True,
            bounds=[(.005,5.)]*len(x),options=dict(maxiter=300,ftol=1e-11,gtol=1e-6,maxls=25,maxcor=8))
        x=result.x;iterations+=result.nit;_,_,violation=problem.calc(x,True)
        if outer>=3 and violation<1e-5:break
        problem.rho*=5.
    if not np.isfinite(x).all():raise ValueError('Nonfinite fast CPU fallback')
    return x,iterations
