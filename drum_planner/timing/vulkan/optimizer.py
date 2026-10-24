"""GPU-native constrained fixed-path timing, with a strict CPU-reference gate.

Finite GPU iterations are not an optimality certificate. Continuous extrema
remain independently checked, and the accepted movement duration must be at
most 1% above an exact-input, fully refined original CPU reference.
"""
import time
import numpy as np
from scipy.interpolate import BSpline,PPoly

from .. import core,motion_quality
from .device import Device,GPUExecutionError

LEVELS=(8,12,20,32,48,64,96,128,192)
MAX_DURATION_RATIO=1.01


def policy():
    return dict(schema='x5-vulkan-timing-v1',method='vulkan_al_lbfgs',levels=list(LEVELS),
        max_duration_ratio=MAX_DURATION_RATIO,precision='float64',line_search_candidates=24,
        history=8,outer_iterations=7,first_inner_iterations=2048,later_inner_iterations=1024,
        max_refinements=3,timeout_s=90.,limits=core.LIMITS.tolist(),reserve=core.RESERVE)


def arrays(path,base,n,grid=None):
    knots=np.r_[np.zeros(4),np.linspace(0,1,n-2)[1:-1],np.ones(4)]
    basis=BSpline(knots,np.eye(n),3,extrapolate=False)
    if grid is None:
        grid=np.unique(np.r_[np.linspace(0,1,max(129,n*8+1)),path.x])
        grid=grid[(grid>=0)&(grid<=1)]
    B=np.array([basis(grid,nu=d)*base for d in range(3)])
    q=np.array([path(grid,nu=d) for d in (1,2,3)])
    weights=basis.antiderivative()(1)-basis.antiderivative()(0)
    return knots,basis,grid,B,q,weights


def reference_check(source,reference):
    if reference['kind']!='time_density' or reference['source_path_sha256']!=core.identity(source):
        raise ValueError('GPU reference must match the exact fixed-path timing input')
    if reference['limits']!=core.LIMITS.tolist():
        raise ValueError('GPU reference motion limits differ')
    path=core.original(source)
    saved=core.decode(reference['path'])
    if not np.array_equal(path.c,saved.c) or not np.array_equal(path.x,saved.x):
        raise ValueError('GPU reference geometry differs')


def quality_gate(law,reference,reference_quality=None):
    if law['duration_s']>MAX_DURATION_RATIO*reference['duration_s']:
        return False,None
    before=reference_quality or motion_quality.law_metrics(reference)
    after=motion_quality.law_metrics(law)
    return (after['internal_stops']<=before['internal_stops'] and
            after['deep_slowdowns']<=before['deep_slowdowns']),after


def optimize(source,reference,log=lambda _:None,*,levels=LEVELS,timeout_s=90.):
    reference_check(source,reference)
    began=time.monotonic();path=core.original(source)
    base=float(source['timing']['time_scale'])/core.RESERVE
    geometry=core.DensityGeometry(path);previous=None;history=[];best=None
    reference_quality=motion_quality.law_metrics(reference)
    for n in levels:
        started=time.monotonic()
        knots,basis,grid,B,q,weights=arrays(path,base,n)
        if previous is None:x=np.ones(n)
        else:
            s=np.linspace(0,1,max(201,n*4))
            x=np.clip(np.linalg.lstsq(basis(s),previous(s)/base,rcond=None)[0],.005,5.)
        for refinement in range(3):
            with Device(B,q,weights,x) as gpu:
                rho=1.;iterations=0
                for outer in range(7):
                    for block in range(64 if outer==0 else 32):
                        if time.monotonic()-began>timeout_s:
                            raise GPUExecutionError('GPU timing reached its bounded solve time without meeting the 1% quality gate')
                        x,state=gpu.step();iterations+=32
                        # A stationary AL subproblem can stop; feasibility alone
                        # is not convergence and never ends the first outer pass.
                        if state[7]>=64 and state[4]<1e-6:break
                    if outer>=3 and state[8]<1e-5:break
                    rho*=5.;gpu.dual(rho)
                device=gpu.name
            h=PPoly.from_spline(BSpline(knots,x*base,3))
            peaks,where=core.density_peaks(path,h,geometry=geometry)
            factor=core.scale_needed(peaks)
            h.c*=max(1.,factor)*core.RESERVE
            candidate=core.pack_density(path,h,source,'vulkan_al_lbfgs',geometry=geometry)
            if best is None or candidate['duration_s']<best['duration_s']:
                best=candidate;previous=h
            accepted,quality=quality_gate(candidate,reference,reference_quality)
            row=dict(controls=n,refinement=refinement,iterations=iterations,
                     duration_s=candidate['duration_s'],constraint_scale=factor,
                     duration_ratio=candidate['duration_s']/reference['duration_s'],
                     elapsed_s=time.monotonic()-started,accepted=accepted)
            history.append(row)
            log(f'GPU VULKAN {n} controls / pass {refinement+1}: {candidate["duration_s"]:.6f}s movement, '
                f'{100*(row["duration_ratio"]-1):+.3f}% vs CPU reference; '
                f'{time.monotonic()-began:.2f}s GPU optimization + continuous checking')
            if accepted:
                candidate['details'].update(device=device,precision='float64',gpu_native_optimizer=True,
                    global_optimum_claimed=False,reference_duration_s=reference['duration_s'],
                    max_duration_ratio=MAX_DURATION_RATIO,duration_ratio=row['duration_ratio'],
                    quality_before=reference_quality,quality_after=quality,tuning=history,
                    calculation_seconds=time.monotonic()-began,
                    limits_checked='independent continuous polynomial extrema; dense scene verification still required')
                return candidate
            # Do not spend three exact-extrema passes polishing an insufficient
            # low-dimensional clock. Refine between-grid violations only when
            # the unscaled optimum could pass the user-approved 1% duration gate.
            if factor<=1.00002 or float(weights@x)*base*core.RESERVE>reference['duration_s']*MAX_DURATION_RATIO:
                break
            grid=np.unique(np.r_[grid,where.ravel()])
            knots,basis,grid,B,q,weights=arrays(path,base,n,grid)
    raise GPUExecutionError('GPU optimizer did not meet the 1% duration / no-added-stops gate; '
        f'best {best["duration_s"]:.6f}s, CPU reference {reference["duration_s"]:.6f}s. '
        'No trajectory was accepted and no silent CPU fallback was performed.')
