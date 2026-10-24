"""Reproducible, hardware-blocked timing benchmark on four frozen input paths."""
import argparse,json,time
from pathlib import Path
import numpy as np
from ..common import ROOT,atomic_json,digest
from ..experiment.config import no_can
from . import core,adaptive

METHODS={'global':'Current global scaling','local_quintic':'Local quintic',
         'scurve':'Local time-optimal S-curve','adaptive':'Adaptive fixed-path retiming'}
INPUTS=ROOT/'diagnostics/timing_experiment_20261021/inputs.json'


def check_geometry(law,source):
    if law['source_path_sha256']!=core.identity(source):raise ValueError('Source geometry changed')
    old=core.original(source)
    if law['kind']=='time_density':
        new=core.decode(law['path'])
        if not np.array_equal(new.c,old.c) or not np.array_equal(new.x,old.x):
            raise ValueError('Retimer refitted the original curve')
        return dict(type='original spline coefficients bit-identical; positive monotone time density',max_error_rad=0.)
    times=np.unique(np.r_[np.linspace(0,law['duration_s'],10001),core.event_times(law)])
    q=core.evaluate(law,times)[0]
    if law['details'].get('original_timing'):
        error=float(np.max(abs(q-old(times/source['timing']['time_scale']))))
        if error>core.PATH_TOL:raise ValueError('Original timing changed geometry')
        return dict(type='same original spline and global clock',max_error_rad=error)
    vertices=np.asarray(source['path_waypoints']);events=np.asarray(law['details']['waypoint_times_s'])
    # Evaluate against the proper ORIGINAL segment, not the nearest arbitrary
    # segment. This also checks order and passage through every waypoint.
    indices=np.clip(np.searchsorted(events,times,side='right')-1,0,len(vertices)-2)
    a=vertices[indices];d=vertices[indices+1]-a
    u=((q-a)*d).sum(1)/(d*d).sum(1)
    error=float(np.max(abs(q-(a+np.clip(u,0,1)[:,None]*d))))
    waypoint_error=float(np.max(abs(core.evaluate(law,events)[0]-vertices)))
    if max(error,waypoint_error)>core.PATH_TOL:raise ValueError('Path/waypoints changed')
    return dict(type='same ordered straight segments; all original waypoints retained',max_error_rad=error,
                max_waypoint_error_rad=waypoint_error)


def validate(law,source,folder,collision=True,*,oracle=None):
    geometry=check_geometry(law,source)
    # Independent dense derivative check supplements, never replaces, exact extrema.
    grid=np.unique(np.r_[np.arange(0,law['duration_s'],.002),core.event_times(law),law['duration_s']])
    q,v,a,j=core.evaluate(law,grid)
    peaks=np.array([np.max(abs(x),axis=0) for x in (v,a,j)])
    if np.any(peaks>core.LIMITS[:,None]*(1+1e-7)):raise ValueError('Dense motion limits exceeded')
    if np.max(abs(v[[0,-1]]))>1e-8 or np.max(abs(a[[0,-1]]))>1e-7:raise ValueError('Non-rest endpoints')
    result=dict(geometry=geometry,dense_samples=len(grid),dense_peaks=peaks.tolist(),motor_commands=0)
    if collision:
        from ..experiment.runtime import Oracle
        from ..experiment.algorithms import dense_check
        folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
        atomic_json(folder/'request.json',source['request'])
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


def benchmark_request(source):
    """Use curated byte-identical models, not a disposable historical run."""
    req=dict(source['request'])
    for kind,key in [('urdf','calibrated_urdf_sha256'),('srdf','srdf_sha256')]:
        expected=source['model'][key]
        path=INPUTS.parent/'inputs/models'/f'{expected}.{kind}'
        if digest(path)!=expected:raise ValueError('Frozen benchmark model changed')
        req[kind]=str(path.resolve())
    return req


def run(folder,levels,ftol,maxiter,collision=True):
    no_can();folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    inputs=json.loads(INPUTS.read_text());rows={}
    for case,spec in inputs.items():
        path=ROOT/spec['file']
        if digest(path)!=spec['sha256']:raise ValueError('Frozen benchmark input changed')
        source=json.loads(path.read_text());source['request']=benchmark_request(source);rows[case]={}
        for method in METHODS:
            start=time.monotonic();print(f'\n=== {case} / {METHODS[method]} ===',flush=True)
            if method=='global':law=core.global_law(source)
            elif method=='adaptive':law=adaptive.optimize(source,levels=levels,ftol=ftol,maxiter=maxiter,log=lambda s:print(s,flush=True))
            else:
                # Test retaining every stop vs passing exactly collinear points.
                candidates=[core.local_law(source,method,pass_collinear=x) for x in (False,True)]
                law=min(candidates,key=lambda c:c['duration_s'])
                law['details']['tuning']=[dict(pass_collinear=x,duration_s=c['duration_s']) for x,c in zip((False,True),candidates)]
            target=folder/case/method;target.mkdir(parents=True,exist_ok=True)
            proof=validate(law,source,target,collision)
            atomic_json(target/'law.json',law)
            row=dict(duration_s=law['duration_s'],cycle_s=2*law['duration_s']+2,proof=proof,
                     elapsed_compute_and_check_s=time.monotonic()-start,law=str((target/'law.json').resolve()),
                     source_path_sha256=law['source_path_sha256'])
            rows[case][method]=row
            print(f'PASS {case} {method}: {law["duration_s"]:.6f}s outward',flush=True)
            atomic_json(folder/'partial.json',rows)
    averages={m:float(np.mean([r[m]['duration_s'] for r in rows.values()])) for m in METHODS}
    winner=min(averages,key=averages.get)
    result=dict(schema='x5-timing-benchmark-v1',limits=core.LIMITS.tolist(),inputs=inputs,rows=rows,
                mean_outward_s=averages,winner=winner,software_only=True,motor_commands=0,
                levels=list(levels),ftol=ftol,maxiter=maxiter)
    atomic_json(folder/'report.json',result);print(json.dumps(averages,indent=2),flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True)
    p.add_argument('--levels',default='8,12,20,32,48,64,96');p.add_argument('--ftol',type=float,default=1e-9)
    p.add_argument('--maxiter',type=int,default=400);p.add_argument('--math-only',action='store_true')
    a=p.parse_args();run(a.output,tuple(map(int,a.levels.split(','))),a.ftol,a.maxiter,not a.math_only)
