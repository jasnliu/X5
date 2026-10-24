"""Jerk-constrained fixed-path retiming using a positive cubic time density.

Optimize h(s)=dt/ds, NOT q(s). The original joint spline is never refitted.
SLSQP uses analytic Jacobians; independent rational-polynomial extrema checks
certify the final limits between grid points. No global-optimality claim for
curved paths. Exact straight blocks use the closed-form S-curve solution.
"""
import time
import numpy as np
from scipy.interpolate import BSpline,PPoly
from scipy.optimize import minimize

from .core import (LIMITS,RESERVE,original,decode,encode,global_law,local_law,
                   density_peaks,scale_needed,pack_density,DensityGeometry)

DEFAULT_LEVELS=(8,12,20,32,48,64,96)


def optimize(source,*,levels=DEFAULT_LEVELS,ftol=1e-9,maxiter=400,log=lambda _:None):
    if source['timing']['stop_at_corners']:
        law=local_law(source,'scurve');law['method']='adaptive'
        law['details']['branch']='analytic straight-block optimum; numerical optimization unnecessary'
        return law
    path=original(source);base=float(source['timing']['time_scale'])/RESERVE
    geometry=DensityGeometry(path)
    # An original global clock is an always-feasible incumbent.
    previous=None;best=global_law(source);best['method']='adaptive';history=[];began=time.monotonic()
    for n in levels:
        started=time.monotonic()
        knots=np.r_[np.zeros(4),np.linspace(0,1,n-2)[1:-1],np.ones(4)]
        basis=BSpline(knots,np.eye(n),3,extrapolate=False)
        weights=basis.antiderivative()(1)-basis.antiderivative()(0)
        if previous is None:z=np.ones(n)
        else:
            s=np.linspace(0,1,max(201,n*4))
            z=np.clip(np.linalg.lstsq(basis(s),previous(s)/base,rcond=None)[0],.005,5.)
        grid=np.unique(np.r_[np.linspace(0,1,max(129,n*8+1)),path.x])
        grid=grid[(grid>=0)&(grid<=1)]
        attempts=[]
        basis_derivatives=[basis.derivative(d) for d in range(3)]
        for refinement in range(3):
            B,B1,B2=[p(grid)*base for p in basis_derivatives]
            q1,q2,q3=[path(grid,nu=d) for d in (1,2,3)]
            memo={}
            def calculate(x,need_jac=False):
                # SLSQP evaluates many line-search trials without requesting
                # their Jacobian. Do not build a dense Jacobian for those trials.
                # Expressions/order, grid, bounds and solver policy are unchanged.
                if 'x' not in memo or not np.array_equal(x,memo['x']):
                    H=(B@x)[:,None];H1=(B1@x)[:,None];H2=(B2@x)[:,None]
                    vals=[q1/H,q2/H**2-q1*H1/H**3,
                          q3/H**3-3*q2*H1/H**4-q1*H2/H**4+3*q1*H1**2/H**5]
                    rows=np.arange(len(grid));g=[];active=[]
                    for d,v in enumerate(vals):
                        joint=np.argmax(abs(v),axis=1);sign=np.sign(v[rows,joint]);peak=abs(v[rows,joint])
                        g.append(1-peak/LIMITS[d]);active.append((joint,sign))
                    memo.update(x=x.copy(),g=np.concatenate(g),J=None,H=H,H1=H1,H2=H2,active=active,rows=rows)
                if not need_jac or memo['J'] is not None:return memo['g'],memo['J']
                H,H1,H2=memo['H'],memo['H1'],memo['H2']
                partials=[(-q1/H**2,None,None),
                    (-2*q2/H**3+3*q1*H1/H**4,-q1/H**3,None),
                    (-3*q3/H**4+12*q2*H1/H**5+4*q1*H2/H**5-15*q1*H1**2/H**6,
                     -3*q2/H**4+6*q1*H1/H**5,-q1/H**4)]
                jac=[];rows=memo['rows']
                for d,parts in enumerate(partials):
                    joint,sign=memo['active'][d]
                    gradient=np.zeros_like(B)
                    for part,b in zip(parts,(B,B1,B2)):
                        if part is not None:gradient+=part[rows,joint,None]*b
                    jac.append(-sign[:,None]*gradient/LIMITS[d])
                memo['J']=np.concatenate(jac);return memo['g'],memo['J']
            result=minimize(lambda x:float(weights@x),z,jac=lambda x:weights,
                method='SLSQP',bounds=[(.005,5.)]*n,
                constraints=[dict(type='ineq',fun=lambda x:calculate(x)[0],jac=lambda x:calculate(x,True)[1])],
                options=dict(ftol=ftol,maxiter=maxiter,disp=False))
            z=result.x
            h=PPoly.from_spline(BSpline(knots,z*base,3))
            peaks,where=density_peaks(path,h,geometry=geometry);factor=scale_needed(peaks)
            attempts.append(dict(iterations=int(result.nit),success=bool(result.success),message=str(result.message),
                constraint_scale=float(factor),sample_count=len(grid)))
            # Resolve discretization errors at their actual between-grid extrema.
            if factor<=1.00002:break
            grid=np.unique(np.r_[grid,where.ravel()]);log(f'  {n} controls: refining {factor:.6f} limit ratio')
        h.c*=max(1.,factor)*RESERVE
        candidate=pack_density(path,h,source,'adaptive',geometry=geometry)
        old=best['duration_s']
        improved=candidate['duration_s']<old
        if improved:best=candidate;previous=h
        row=dict(controls=n,duration_s=candidate['duration_s'],best_duration_s=best['duration_s'],
                 improvement_fraction=max(0.,1-best['duration_s']/old),elapsed_s=time.monotonic()-started,
                 ftol=ftol,maxiter=maxiter,attempts=attempts,accepted=improved)
        history.append(row)
        log(f'ADAPTIVE {n} controls: {candidate["duration_s"]:.6f}s; best {best["duration_s"]:.6f}s; '
            f'{row["elapsed_s"]:.1f}s computation')
    best['details'].update(branch='positive cubic dt/ds optimization on original curve',tuning=history,
        calculation_seconds=time.monotonic()-began,global_optimum_claimed=False,
        limits_checked='all polynomial intervals; stationary roots of rational v/a/j, plus one-sided knot values')
    return best
