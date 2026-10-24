"""Opt-in sparse IPOPT fork of adaptive fixed-path timing.

Same positive cubic dt/ds parameterization, levels, limits and exact extrema
checks. Only the nonlinear solver changes. Signed per-joint inequalities are
equivalent to the old maximum-absolute-joint inequalities, without their kinks.
CasADi supplies sparse derivatives; no generated code, GPU or new parallelism.
Imported only in the separate planner environment, never by a motor owner.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np
from scipy.interpolate import BSpline, PPoly
from scipy.sparse import csc_matrix

from . import core


def problem(path, basis, grid, base, ftol, maxiter):
    import casadi as ca
    n=basis.c.shape[0]
    x=ca.MX.sym('density',n)
    # Structural zeros, not dense matrices containing zero-valued entries.
    B=[ca.DM(csc_matrix(basis.derivative(d)(grid)*base)) for d in range(3)]
    H,H1,H2=[ca.repmat(b@x,1,7) for b in B]
    q1,q2,q3=[ca.DM(path(grid,nu=d)) for d in (1,2,3)]
    vals=[q1/H,q2/H**2-q1*H1/H**3,
          q3/H**3-3*q2*H1/H**4-q1*H2/H**4+3*q1*H1**2/H**5]
    g=ca.vertcat(*[ca.vec(v/core.LIMITS[d]) for d,v in enumerate(vals)])
    weights=basis.antiderivative()(1)-basis.antiderivative()(0)
    f=ca.dot(ca.DM(weights),x)
    options={'print_time':False,'ipopt.print_level':0,'ipopt.sb':'yes',
             'ipopt.tol':ftol,'ipopt.max_iter':maxiter,'ipopt.acceptable_iter':0,
             'ipopt.mu_strategy':'adaptive','ipopt.bound_relax_factor':0.,
             'ipopt.linear_solver':'mumps'}
    solver=ca.nlpsol('retime','ipopt',{'x':x,'f':f,'g':g},options)
    sparsity=ca.jacobian(g,x).sparsity()
    return solver,dict(variables=n,constraints=int(g.numel()),jacobian_nnz=sparsity.nnz(),
                       dense_jacobian_entries=int(g.numel())*n)


def optimize(source,*,levels,ftol,maxiter,log=lambda _:None):
    import casadi as ca
    if source['timing']['stop_at_corners']:
        raise ValueError('Sparse hardware timing requires the continuous Workspace curve path')
    path=core.original(source);base=float(source['timing']['time_scale'])/core.RESERVE
    geometry=core.DensityGeometry(path)
    previous=None;best=core.global_law(source);best['method']='sparse_ipopt'
    history=[];began=time.monotonic()
    for n in levels:
        started=time.monotonic()
        log(f'SPARSE IPOPT {n} controls: building locally supported constraints')
        knots=np.r_[np.zeros(4),np.linspace(0,1,n-2)[1:-1],np.ones(4)]
        basis=BSpline(knots,np.eye(n),3,extrapolate=False)
        if previous is None:z=np.ones(n)
        else:
            s=np.linspace(0,1,max(201,n*4))
            z=np.clip(np.linalg.lstsq(basis(s),previous(s)/base,rcond=None)[0],.005,5.)
        grid=np.unique(np.r_[np.linspace(0,1,max(129,n*8+1)),path.x])
        grid=grid[(grid>=0)&(grid<=1)];attempts=[]
        for refinement in range(3):
            solver,structure=problem(path,basis,grid,base,ftol,maxiter)
            result=solver(x0=z,lbx=.005,ubx=5.,lbg=-1.,ubg=1.)
            stats=solver.stats();z=np.asarray(result['x']).ravel()
            if not np.all(np.isfinite(z)) or np.any(z<=0):
                raise ValueError('Sparse optimizer returned an invalid time density')
            h=PPoly.from_spline(BSpline(knots,z*base,3))
            peaks,where=core.density_peaks(path,h,geometry=geometry)
            factor=core.scale_needed(peaks)
            attempts.append(dict(iterations=int(stats['iter_count']),success=bool(stats['success']),
                message=stats['return_status'],constraint_scale=float(factor),sample_count=len(grid),**structure))
            # Free the solver graph before building a refined graph. Memory is
            # bounded by one level, inside the existing 3 GiB calculation group.
            del result,solver
            if factor<=1.00002:break
            grid=np.unique(np.r_[grid,where.ravel()])
            log(f'  {n} controls: refining {factor:.6f} limit ratio; {attempts[-1]["message"]}')
        h.c*=max(1.,factor)*core.RESERVE
        candidate=core.pack_density(path,h,source,'sparse_ipopt',geometry=geometry)
        old=best['duration_s'];improved=candidate['duration_s']<old
        if improved:best=candidate;previous=h
        row=dict(controls=n,duration_s=candidate['duration_s'],best_duration_s=best['duration_s'],
                 improvement_fraction=max(0.,1-best['duration_s']/old),elapsed_s=time.monotonic()-started,
                 ftol=ftol,maxiter=maxiter,attempts=attempts,accepted=improved)
        history.append(row)
        log(f'SPARSE IPOPT {n}: best {best["duration_s"]:.6f}s; {row["elapsed_s"]:.1f}s computation; '
            f'{structure["jacobian_nnz"]}/{structure["dense_jacobian_entries"]} Jacobian entries')
    best['details'].update(branch='sparse IPOPT positive cubic dt/ds on original curve',tuning=history,
        calculation_seconds=time.monotonic()-began,global_optimum_claimed=False,casadi_version=ca.__version__,
        limits_checked='all polynomial intervals; stationary roots of rational v/a/j, plus one-sided knot values',
        fallback_used=not any(r['accepted'] for r in history))
    return best


def main():
    from ..common import atomic_json
    from ..experiment.config import no_can
    from .integration import policy
    no_can()
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();source=json.loads(a.input.read_text());cfg=policy()
    law=optimize(source,levels=tuple(cfg['levels']),ftol=cfg['ftol'],maxiter=cfg['maxiter'],log=lambda s:print(s,flush=True))
    atomic_json(a.output,law)


if __name__=='__main__':main()
