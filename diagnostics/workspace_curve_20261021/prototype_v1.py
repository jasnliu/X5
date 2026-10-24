"""Experimental Cartesian-first broad curves, isolated from the four survivors.

A cubic Bezier wrist curve and a continuous joint spline are fitted together.
Only the endpoint joint poses, calibrated FK/model and frozen world are inputs.
The stick tip and joint motion regularize the IK solution; this is not seven
independent waypoint IK calls. Proposal fitting is NOT a collision certificate.
Every retained curve uses the existing continuous bounds and dense FCL/zone
validator, followed by the unchanged arm-specific timing dispatch.
"""
import hashlib
import json
import time
from pathlib import Path

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

from ..common import atomic_json, digest
from .. import style
from . import gesture
from .fluidity import curve_shape

VERSION = 'workspace-bezier-v1'
# Shared across arms/destinations; no target-specific or recorded-route tuning.
OPTIONS = dict(knots=9, samples=33, max_nfev=45, fit_weight=35.,
               length_weight=.45, bend_weight=.08, tip_weight=.025,
               joint_weight=.012, posture_weight=.015, prior_weight=.5,
               coherence_weight=.35, max_fit_m=.025, proposal_count=7)


def bezier_basis(s):
    s=np.asarray(s,dtype=float)
    return np.column_stack((3*(1-s)**2*s,3*(1-s)*s*s))


def arc_offsets(a,b,count=7):
    """Task-relative broad route alternatives, not hard-coded drum coordinates."""
    delta=np.asarray(b)-a;distance=float(np.linalg.norm(delta))
    direction=delta/max(distance,1e-12)
    if distance<1e-8:direction=np.array([1.,0.,0.])
    up=np.array([0.,0.,1.]);up-=direction*(up@direction)
    if np.linalg.norm(up)<.1:
        up=np.array([0.,1.,0.]);up-=direction*(up@direction)
    up/=np.linalg.norm(up);side=np.cross(direction,up)
    amount=np.clip(.55*distance,.12,.32)
    vectors=[np.zeros(3),up,-up,side,-side,(up+side)/np.sqrt(2),(up-side)/np.sqrt(2)]
    return [np.tile(amount*v,(2,1)) for v in vectors[:count]]


class BudgetExpired(Exception):
    pass


class CurveFit:
    """Small joint/Cartesian least-squares problem with batched FK Jacobians."""
    def __init__(self,oracle,req,offset,options=None,deadline=float('inf')):
        self.oracle=oracle;self.cfg=dict(OPTIONS,**(options or {}));self.deadline=deadline
        self.start=np.asarray(req['start'],float);self.goal=np.asarray(req['goal'],float)
        self.n=int(self.cfg['knots']);self.s=np.linspace(0,1,int(self.cfg['samples']))
        knots=np.linspace(0,1,self.n)
        self.B=make_interp_spline(knots,np.eye(self.n),k=3)(self.s)
        self.Bi=self.B[:,1:-1];self.C=bezier_basis(self.s)
        self.nq=(self.n-2)*7
        ends=gesture.fk(oracle,np.array([self.start,self.goal]))
        self.a,self.b=np.asarray(ends['wrist_xyz_m'])
        self.chord=self.a+(self.b-self.a)*self.s[:,None]
        self.offset=np.asarray(offset,float)
        self.qline=self.start+(self.goal-self.start)*self.s[:,None]
        self.QJ=np.kron(self.Bi,np.eye(7)).reshape(len(self.s),7,self.nq)
        self.CJ=np.kron(self.C,np.eye(3)).reshape(len(self.s),3,6)
        self.last_x=None;self.last_values=None;self.last_jac=None;self.best=None
        self.calls=0

    def initial(self):
        return np.r_[np.linspace(self.start,self.goal,self.n)[1:-1].ravel(),self.offset.ravel()]

    def bounds(self):
        # Bounds apply to interior fit knots; the final continuous polynomial
        # still goes through the production extrema and collision validator.
        span=max(.3,float(np.linalg.norm(self.b-self.a)))
        return (np.r_[np.tile(self.oracle.lower,self.n-2),np.full(6,-span)],
                np.r_[np.tile(self.oracle.upper,self.n-2),np.full(6,span)])

    def nodes(self,x):
        return np.vstack((self.start,np.asarray(x[:self.nq]).reshape(-1,7),self.goal))

    def features(self,x):
        if self.last_x is not None and np.array_equal(x,self.last_x):return self.last_values
        if time.monotonic()>self.deadline:raise BudgetExpired()
        q=self.B@self.nodes(x)
        f=gesture.fk(self.oracle,q)
        p=np.asarray(f['wrist_xyz_m']);tip=np.asarray(f['tip_xyz_m'])
        curve=self.chord+self.C@x[self.nq:].reshape(2,3)
        self.last_x=x.copy();self.last_values=(q,p,tip,curve);self.last_jac=None;self.calls+=1
        return self.last_values

    def residual_parts(self,x,q,p,tip,curve):
        cfg=self.cfg;n=len(self.s);h=1/(n-1)
        return [cfg['fit_weight']*(p-curve)/np.sqrt(n),
                cfg['length_weight']*np.diff(curve,axis=0)/h/np.sqrt(n-1),
                cfg['bend_weight']*np.diff(curve,n=2,axis=0)/h**2/np.sqrt(n-2),
                cfg['tip_weight']*np.diff(tip,n=2,axis=0)/h**2/np.sqrt(n-2),
                cfg['joint_weight']*np.diff(q,n=2,axis=0)/h**2/np.sqrt(n-2),
                cfg['posture_weight']*(q-self.qline)/np.sqrt(n),
                cfg['prior_weight']*(x[self.nq:].reshape(2,3)-self.offset),
                cfg['coherence_weight']*np.diff(x[self.nq:].reshape(2,3),axis=0)]

    def fun(self,x):
        result=np.concatenate([a.ravel() for a in self.residual_parts(x,*self.features(x))])
        cost=float(result@result)
        if self.best is None or cost<self.best[0]:self.best=(cost,x.copy())
        return result

    def jac(self,x):
        q,p,tip,curve=self.features(x)
        if self.last_jac is not None:return self.last_jac
        n=len(q);step=1e-5
        # Seven simultaneous per-state central differences, one bounded IPC
        # call. Chain through the spline basis instead of 49 separate solves.
        perturbed=np.tile(q,(14,1,1))
        for j in range(7):
            perturbed[2*j,:,j]+=step;perturbed[2*j+1,:,j]-=step
        f=gesture.fk(self.oracle,perturbed.reshape(-1,7))
        J=[]
        for key in ('wrist_xyz_m','tip_xyz_m'):
            a=np.asarray(f[key]).reshape(14,n,3)
            local=((a[::2]-a[1::2])/(2*step)).transpose(1,2,0)
            J.append(np.einsum('saj,sk->sakj',local,self.Bi).reshape(n,3,self.nq))
        zeros=lambda n,d:np.zeros((n,d,6))
        P=np.concatenate((J[0],zeros(n,3)),axis=2)
        T=np.concatenate((J[1],zeros(n,3)),axis=2)
        Q=np.concatenate((self.QJ,zeros(n,7)),axis=2)
        C=np.concatenate((np.zeros((n,3,self.nq)),self.CJ),axis=2)
        V=np.concatenate((np.zeros((6,self.nq)),np.eye(6)),axis=1).reshape(2,3,-1)
        cfg=self.cfg;h=1/(n-1)
        parts=[cfg['fit_weight']*(P-C)/np.sqrt(n),
               cfg['length_weight']*np.diff(C,axis=0)/h/np.sqrt(n-1),
               cfg['bend_weight']*np.diff(C,n=2,axis=0)/h**2/np.sqrt(n-2),
               cfg['tip_weight']*np.diff(T,n=2,axis=0)/h**2/np.sqrt(n-2),
               cfg['joint_weight']*np.diff(Q,n=2,axis=0)/h**2/np.sqrt(n-2),
               cfg['posture_weight']*Q/np.sqrt(n),
               cfg['prior_weight']*V,cfg['coherence_weight']*np.diff(V,axis=0)]
        self.last_jac=np.concatenate([a.reshape(-1,self.nq+6) for a in parts])
        return self.last_jac

    def solve(self):
        try:
            result=least_squares(self.fun,self.initial(),jac=self.jac,bounds=self.bounds(),
                max_nfev=self.cfg['max_nfev'],ftol=1e-5,xtol=1e-5,gtol=1e-5)
            x=result.x;message=result.message;success=bool(result.success)
        except BudgetExpired:
            if self.best is None:return None,dict(reason='fit_budget_exhausted')
            x=self.best[1];message='Bounded fit budget reached; validate best proposal';success=False
        # Do not call features here: timeout checking intentionally stops the
        # optimizer, not the required diagnostics or final validation.
        q=self.nodes(x);f=gesture.fk(self.oracle,self.B@q)
        curve=self.chord+self.C@x[self.nq:].reshape(2,3)
        errors=np.linalg.norm(np.asarray(f['wrist_xyz_m'])-curve,axis=1)
        return q,dict(optimizer_converged=success,message=message,fk_evaluations=self.calls,
                      fit_rms_m=float(np.sqrt(np.mean(errors**2))),fit_max_m=float(errors.max()),
                      control_offsets_m=x[self.nq:].reshape(2,3).tolist(),
                      requested_offsets_m=self.offset.tolist(),cost=float(self.fun_without_budget(x)))

    def fun_without_budget(self,x):
        previous=self.deadline;self.deadline=float('inf')
        try:r=self.fun(x);return r@r
        finally:self.deadline=previous


def path_metrics(q,features):
    g=gesture.geometry(q,features)
    shapes={name:curve_shape(np.asarray(features[name+'_xyz_m']),.05) for name in ('wrist','tip')}
    return dict(gesture=g,shapes=shapes)


def score(metrics,proximity,params):
    """Geometry first; no playback duration, recording similarity or target ID."""
    g=metrics['gesture'];shapes=metrics['shapes']
    # Length prevents an arbitrarily large, low-curvature detour. Total turn
    # discourages multiple steering corrections; curvature variation separates
    # a coherent bend from an equally turning but unevenly bending route.
    variation=sum(w*shapes[n]['curvature_variation'] for n,w in (('wrist',.8),('tip',.2)))
    return (g['broad_turning_rad']+1.2*g['workspace_detour']+.08*variation
            +.15*g['joint_excess_ratio']+.1*g['progress_disagreement']
            +float(params.get('clearance_weight',3.))*proximity)


def build(oracle,req,folder,params,seconds,log,options=None):
    from .algorithms import timed_candidate
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    cfg=dict(OPTIONS,**(options or {}));started=time.monotonic();deadline=started+seconds
    attempts=[];rows=[]
    endpoints=oracle.check([req['start'],req['goal']])
    if not all(endpoints['valid']):
        return dict(candidates=[],attempts=[],elapsed_s=time.monotonic()-started,
                    reason='nominal_endpoint_invalid',details=endpoints,options=cfg)
    ends=gesture.fk(oracle,np.array([req['start'],req['goal']]))['wrist_xyz_m']
    offsets=arc_offsets(*np.array(ends),count=cfg['proposal_count'])
    for i,offset in enumerate(offsets,1):
        if time.monotonic()>=deadline:break
        allocation=min(12.,max(.1,(deadline-time.monotonic())/(len(offsets)-i+1)))
        log(f'WORKSPACE CURVE {i}/{len(offsets)}: joint + Cartesian Bezier fit, {allocation:.1f}s fit allowance')
        solver=CurveFit(oracle,req,offset,cfg,time.monotonic()+allocation)
        q,detail=solver.solve();detail.update(index=i,accepted=False)
        attempts.append(detail)
        if q is None:continue
        log(f'workspace {i}: fit max {detail["fit_max_m"]*1000:.1f} mm, {detail["fk_evaluations"]} evaluations')
        if detail['fit_max_m']>cfg['max_fit_m']:
            detail['reason']='cartesian_curve_fit_too_far';continue
        # No segmented/corner-stop or legacy-route fallback. Failure remains a
        # visible experiment result rather than another method under this name.
        candidate=timed_candidate(oracle,q,req,i,log,
            dict(reversal_threshold_deg=2.,joint_spacing_rad=.05,spatial_spacing_m=.05),stop=False)
        if candidate is None:detail['reason']='continuous_curve_bounds_collision_or_zone';continue
        samples=style.sample_path(np.array([p['positions_rad'] for p in candidate['outward']]))
        f=gesture.fk(oracle,samples);metrics=path_metrics(samples,f)
        # Interpolation can change the fitted geometry. Check the played curve,
        # not only the IK samples, against the proposed Cartesian curve too.
        s=np.linspace(0,1,1001)
        target=np.array(ends[0])+(np.array(ends[1])-ends[0])*s[:,None]+bezier_basis(s)@np.array(detail['control_offsets_m'])
        error=float(cKDTree(target).query(np.array(f['wrist_xyz_m']))[0].max())
        detail['played_curve_fit_max_m']=error
        if error>cfg['max_fit_m']:
            detail['reason']='played_curve_fit_too_far';continue
        proximity=candidate['metrics']['baseline']['components']['obstacle_proximity']
        value=score(metrics,proximity,params)
        candidate.update(source='workspace_curve',gesture=metrics['gesture'],workspace=metrics,
                         workspace_fit=detail.copy(),score=value,stop_at_corners=False)
        name=f'candidate_{i:03d}.json';atomic_json(folder/name,candidate)
        row=dict(index=i,file=name,sha256=digest(folder/name),source='workspace_curve',
                 gesture=metrics['gesture'],workspace=metrics,metrics=candidate['metrics'],
                 score=value,duration_s=candidate['duration_s'],stop_at_corners=False)
        rows.append(row);detail['accepted']=True;detail['score']=value
        log(f'WORKSPACE VALID {i}: turn={np.degrees(metrics["gesture"]["broad_turning_rad"]):.1f}deg, score={value:.4f}')
    return dict(candidates=rows,attempts=attempts,elapsed_s=time.monotonic()-started,options=cfg,
                **({} if rows else {'reason':'no_valid_workspace_curve_within_budget'}))


def run(req,context,pool_root,request_path,params,seconds,log):
    """Separate pool/cache: never changes the original four methods' search."""
    from .algorithms import pool_key,read_pool
    from .config import SCHEMA
    from .runtime import Oracle
    from . import cache
    import fcntl
    key=hashlib.sha256((pool_key(context,req,params,seconds)+digest(__file__)+VERSION).encode()).hexdigest()
    folder=Path(pool_root)/key;folder.mkdir(parents=True,exist_ok=True)
    persistent_key=cache.identity(context,req,'workspace_curve',key)
    with (folder/'pool.lock').open('w') as lock:
        end=time.monotonic()+seconds+20
        while True:
            try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);break
            except BlockingIOError:
                if time.monotonic()>end:raise TimeoutError('Workspace curve pool is still being generated')
                time.sleep(.1)
        try:pool=read_pool(folder,key)
        except (OSError,ValueError,KeyError,TypeError):pool=None
        reused=pool is not None
        if pool is None and cache.restore(persistent_key,folder,log):
            try:pool=read_pool(folder,key)
            except (OSError,ValueError,KeyError,TypeError):pool=None
            reused=pool is not None
        if pool is None:
            oracle=Oracle(request_path,folder)
            try:
                pool=build(oracle,req,folder,params,seconds,log)
                pool.update(schema=SCHEMA,key=key,complete=True,budget_s=seconds,
                            joint_limit_check=oracle.meta['joint_limit_check'])
                atomic_json(folder/'pool.json',pool)
            finally:oracle.close()
            cache.publish(persistent_key,folder,['pool.json']+[r['file'] for r in pool['candidates']])
        else:log(f'WORKSPACE CACHE: {len(pool["candidates"])} checked Cartesian curve candidates')
    ranked=sorted(pool['candidates'],key=lambda r:(round(r['score'],8),r['index']))
    report=dict(method='workspace_curve',pool_key=key,pool_file=str(folder/'pool.json'),pool_reused=reused,
                candidates=ranked,selected=ranked[0]['index'] if ranked else None,searches=pool['attempts'],
                pool_elapsed_s=pool['elapsed_s'],budget_s=seconds,options=pool['options'],
                joint_limit_check=pool['joint_limit_check'],duration_ratio_filter=None,
                normalization=dict(selection_policy=VERSION,uses_recording_similarity=False,uses_elapsed_time=False,
                                   fallback_used=False,score_round_digits=8))
    if not ranked:
        report.update(reason=pool.get('reason','no_valid_workspace_curve_within_budget'),details=pool.get('details',{}))
        return None,report
    winner=json.loads((folder/ranked[0]['file']).read_text())
    log(f'WORKSPACE SELECTED {winner["index"]}: geometry + clearance; {len(ranked)} fully checked candidates; no legacy fallback')
    return winner,report
