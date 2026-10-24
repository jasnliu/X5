"""Experiment-only bounded sparse timing on the UNCHANGED geometric curve.

A freshly computed 24-control CPU seed initializes one 96-control sparse solve.
All seven joints constrain every derivative; no active-joint approximation.
Polynomial root checks certify candidates continuously, including knot sides.
Retain a checked incumbent on failure; never import this from hardware dispatch.
"""
import hashlib
from pathlib import Path
import time

import numpy as np
from scipy.interpolate import BSpline, PPoly
from scipy.sparse import csc_matrix

from ..timing import core
from .timing_extrema import density_peaks
from ..timing.vulkan.fast import cpu_solve
from ..timing.vulkan.optimizer import arrays

CONTROLS = 96
MAX_ITERATIONS = 90
SOLVE_WALL_SECONDS = 1.0
# Refine only a meaningful between-sample violation, not a negligible gain.
REFINEMENT_SCALE = 1.004


def grid_for(knots, path, extra=()):
    cuts = np.unique(knots)
    # Four subintervals per cubic, rather than overlapping unrelated grids.
    # h is C2; q's knots also get an explicit left-side sample.
    grid = np.unique(np.r_[(cuts[:-1, None] + np.diff(cuts)[:, None] *
                           np.linspace(0, 1, 5)).ravel(), path.x,
                          np.nextafter(path.x[1:-1], -np.inf), extra])
    return grid[np.isfinite(grid) & (grid >= 0) & (grid <= 1)]


def sparse_solve(B, q, weights, initial):
    import casadi as ca  # Optional: missing/broken dependency has a checked fallback.
    z = ca.MX.sym('clock', len(initial))
    H, H1, H2 = [ca.DM(csc_matrix(b)) @ z for b in B]
    h, h1, h2 = [ca.repmat(v, 1, 7) for v in (H, H1, H2)]
    q1, q2, q3 = map(ca.DM, q)
    acc = q2 / h**2 - q1 * h1 / h**3
    jerk = q3 / h**3 - 3*q2*h1/h**4 - q1*h2/h**4 + 3*q1*h1**2/h**5
    # Exact equivalent of all signed velocity constraints, since H > 0.
    g = ca.vertcat(H - np.max(abs(q[0]), axis=1) / core.LIMITS[0],
                   ca.vec(acc/core.LIMITS[1]), ca.vec(jerk/core.LIMITS[2]))
    m = len(q[0])
    solver = ca.nlpsol('experiment_clock', 'ipopt',
        {'x': z, 'f': ca.dot(weights, z), 'g': g},
        {'print_time': False, 'ipopt.print_level': 0, 'ipopt.sb': 'yes',
         'ipopt.tol': 1e-7, 'ipopt.max_iter': MAX_ITERATIONS,
         'ipopt.max_wall_time': SOLVE_WALL_SECONDS, 'ipopt.acceptable_iter': 0,
         'ipopt.mu_strategy': 'adaptive', 'ipopt.bound_relax_factor': 0.,
         'ipopt.linear_solver': 'mumps'})
    result = solver(x0=initial, lbx=.005, ubx=5.,
                    lbg=np.r_[np.zeros(m), -np.ones(14*m)],
                    ubg=np.r_[np.full(m, np.inf), np.ones(14*m)])
    stats = solver.stats()
    return np.asarray(result['x']).ravel(), dict(
        iterations=int(stats['iter_count']), status=stats['return_status'],
        converged=bool(stats['success']), casadi_version=ca.__version__)


def certify(path, density, source, *, reserved=False, geometry=None, log=lambda _: None):
    """Check original rational extrema, then apply the exact homogeneous scale.

    Multiplying h by k divides velocity/acceleration/jerk by k/k²/k³.
    The same unchanged 0.1% reserve covers roundoff. No sampled-only acceptance.
    """
    h = PPoly(density.c.copy(), density.x.copy(), extrapolate=False)
    if not np.isfinite(h.c).all() or not np.isfinite(h.x).all():
        raise ValueError('Nonfinite time density')
    cuts = np.unique(h.x); cuts = cuts[(cuts > 0) & (cuts < 1)]
    for derivative in (0, 1):
        left = h(np.nextafter(cuts, -np.inf), nu=derivative)
        right = h(cuts, nu=derivative)
        if not np.allclose(left, right, rtol=1e-9, atol=1e-8):
            raise ValueError('Discontinuous time density or acceleration')
    checker = 'batched polynomial roots'
    try:
        peaks, where = density_peaks(path, h)
    except np.linalg.LinAlgError:
        checker = 'original polynomial roots (eigensolver fallback)'
        log('CONTINUOUS CHECK FALLBACK — using original polynomial root checker')
        peaks, where = core.density_peaks(path, h, geometry=geometry)
    if not np.isfinite(peaks).all():
        raise ValueError('Nonfinite continuous extrema')
    needed = core.scale_needed(peaks)
    scale = max(1., needed * core.RESERVE) if reserved else max(1., needed) * core.RESERVE
    h.c *= scale
    peaks = peaks / scale**np.arange(1, 4)[:, None]
    duration = float(h.integrate(0, 1))
    if not np.isfinite(duration) or duration <= 0 or np.any(peaks > core.LIMITS[:, None]):
        raise ValueError('Continuous timing verification failed')
    return dict(schema='x5-fixed-path-timing-v1', method='experiment_sparse_density',
        kind='time_density', duration_s=duration, path=core.encode(path), density=core.encode(h),
        source_path_sha256=core.identity(source), limits=core.LIMITS.tolist(), peaks=peaks.tolist(),
        extrema_s=where.tolist(), geometry_preserved=True, details={'correction_scale': scale, 'continuous_checker': checker})


def optimize(source, log=lambda _: None):
    began = time.monotonic()
    if source['timing']['stop_at_corners']:
        raise ValueError('Experiment sparse timing requires the continuous Workspace curve')
    path = core.original(source)
    base = float(source['timing']['time_scale']) / core.RESERVE
    if not np.isfinite(base) or base <= 0:
        raise ValueError('Invalid source clock')
    log('CPU SPARSE TIMING — intentional CPU backend; GPU not requested; fresh solve, no result cache')
    uniform = PPoly(np.array([[base*core.RESERVE]]), [0., 1.])
    best = certify(path, uniform, source, reserved=True, log=log)
    selected = 'uniform'; history = []; warnings = []
    knots, basis, _, B, q, weights = arrays(path, base, 24)
    seed = np.ones(24)
    try:
        seed, iterations = cpu_solve(B, q, weights, seed)
        law = certify(path, PPoly.from_spline(BSpline(knots, seed*base, 3)), source, log=log)
        if law['duration_s'] < best['duration_s']:
            best = law; selected = 'seed'
        history.append(dict(stage='fresh_seed', iterations=iterations, duration_s=law['duration_s']))
    except (ValueError, RuntimeError, FloatingPointError) as exc:
        warnings.append('CPU seed failed: ' + str(exc)); seed = np.ones(24)
        log('*** CPU SEED FALLBACK — using checked uniform initialization *** ' + str(exc))
    # Warm initialization comes ONLY from this fresh source, never another run.
    seed_basis = basis
    knots = np.r_[np.zeros(4), np.linspace(0, 1, CONTROLS-2)[1:-1], np.ones(4)]
    basis = BSpline(knots, np.eye(CONTROLS), 3, extrapolate=False)
    s = np.linspace(0, 1, max(201, CONTROLS*4))
    x = np.clip(np.linalg.lstsq(basis(s), seed_basis(s) @ seed, rcond=None)[0], .005, 5.)
    weights = basis.antiderivative()(1) - basis.antiderivative()(0)
    grid = grid_for(knots, path); geometry = core.DensityGeometry(path)
    for attempt in range(2):
        started = time.monotonic()
        B = np.array([basis(grid, nu=d)*base for d in range(3)])
        q = np.array([path(grid, nu=d) for d in (1, 2, 3)])
        try:
            x, stats = sparse_solve(B, q, weights, x)
            if not np.isfinite(x).all() or np.any(x <= 0):
                raise ValueError('Invalid sparse optimizer coefficients')
            law = certify(path, PPoly.from_spline(BSpline(knots, x*base, 3)), source, geometry=geometry, log=log)
            accepted = law['duration_s'] < best['duration_s']
            if accepted:
                best = law; selected = 'sparse'
            history.append(dict(stage='sparse', attempt=attempt+1, grid_points=len(grid),
                duration_s=law['duration_s'], accepted=accepted,
                correction_scale=law['details']['correction_scale'],
                elapsed_s=time.monotonic()-started, **stats))
            if not stats['converged']:
                warnings.append('Sparse solve stopped: ' + stats['status'])
                log('CPU SPARSE bounded stop: ' + stats['status'] + '; retaining best continuously checked clock')
                break
            if law['details']['correction_scale'] <= REFINEMENT_SCALE or attempt == 1:
                break
            where = np.asarray(law['extrema_s']).ravel()
            grid = grid_for(knots, path, np.r_[grid, where, np.nextafter(where, -np.inf)])
            log('CPU SPARSE: one bounded extrema refinement')
        except (ImportError, OSError, RuntimeError, ValueError, FloatingPointError) as exc:
            warnings.append('Sparse refinement failed: ' + str(exc))
            log('*** CPU TIMING FALLBACK — sparse refinement failed; retaining checked clock *** ' + str(exc))
            break
    fallback = selected != 'sparse'
    if fallback:
        log('*** CPU TIMING FALLBACK — checked ' + selected + ' clock selected; sparse refinement gave no improvement ***')
    label = 'CPU sparse timing — intentional CPU backend' if not fallback else 'CPU TIMING FALLBACK — checked ' + selected
    best['details'].update(actual_backend='cpu', requested_backend='cpu_sparse', gpu_native_optimizer=False,
        backend_label=label, selected_stage=selected, fallback_used=fallback,
        fallback_reason='; '.join(warnings) if warnings else ('Sparse refinement did not improve incumbent' if fallback else None),
        warnings=warnings, controls=CONTROLS, precision='float64', history=history,
        calculation_seconds=time.monotonic()-began, cpu_reference_used=False, result_cache_used=False,
        implementation_sha256=hashlib.sha256((Path(__file__).read_bytes()+Path(__file__).with_name("timing_extrema.py").read_bytes())).hexdigest(),
        global_optimum_claimed=False)
    log(f'CPU TIMING READY: {best["duration_s"]:.4f}s movement; '
        f'{best["details"]["calculation_seconds"]:.3f}s calculation; {selected}; continuous limits verified')
    return best
