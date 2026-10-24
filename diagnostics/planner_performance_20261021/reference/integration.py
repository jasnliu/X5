"""Apply the single benchmark winner AFTER path generation and ranking.

The geometric pool, its eligibility checks, scores and selected index are not
modified. Cache timing separately, including code, limits and input identities.
Only the software experiment imports this module; hardware modes are unchanged.
"""
import hashlib,json,time
from pathlib import Path
import numpy as np
from ..common import atomic_json,digest
from . import core,adaptive
from .benchmark import validate


def policy():
    obj=json.loads(Path(__file__).with_name('selected.json').read_text())
    if obj['schema']!='x5-experiment-retiming-policy-v1' or not np.array_equal(obj['limits'],core.LIMITS):
        raise ValueError('Unexpected experiment timing policy/limits')
    return obj


def fingerprint():
    folder=Path(__file__).parent
    return hashlib.sha256('\n'.join(digest(folder/p) for p in (
        'selected.json','core.py','adaptive.py','integration.py','benchmark.py')).encode()).hexdigest()


def retime(winner,report,req,context,folder,log):
    cfg=policy();source=dict(path_waypoints=winner['path'],timing=winner['timing'],request=req)
    source_id=core.identity(source);signature=fingerprint()
    key=hashlib.sha256((source_id+signature+report['pool_key']).encode()).hexdigest()
    cache=Path(context['pool_root'])/'timing'/key;cache.mkdir(parents=True,exist_ok=True)
    lawfile=cache/'law.json';checkfile=cache/'verification.json';began=time.monotonic();reused=False
    try:
        checked=json.loads(checkfile.read_text());law=json.loads(lawfile.read_text())
        if (checked['law_sha256']!=digest(lawfile) or checked['fingerprint']!=signature
                or checked['pool_key']!=report['pool_key'] or law['source_path_sha256']!=source_id):
            raise ValueError('Stale or damaged timing cache')
        proof=checked['proof'];reused=True
        log(f'TIMING cache: {cfg["label"]}, {law["duration_s"]:.3f}s; same exact selected path')
    except (OSError,ValueError,KeyError,TypeError):
        log(f'TIMING: {cfg["label"]}; reference {winner["duration_s"]:.3f}s; path and ranking locked')
        if cfg['method']=='adaptive':
            law=adaptive.optimize(source,levels=tuple(cfg['levels']),ftol=cfg['ftol'],maxiter=cfg['maxiter'],log=log)
        elif cfg['method']=='global':law=core.global_law(source)
        elif cfg['method'] in ('local_quintic','scurve'):law=core.local_law(source,cfg['method'])
        else:raise ValueError('Unknown selected timing method')
        log('TIMING VERIFY: continuous velocity/acceleration/jerk extrema + exact geometry + dense zone/collision check')
        proof=validate(law,source,Path(folder)/'retiming_check')
        atomic_json(lawfile,law)
        atomic_json(checkfile,dict(law_sha256=digest(lawfile),fingerprint=signature,pool_key=report['pool_key'],proof=proof))
    points=core.samples(law,source)
    points[0]['positions_rad']=list(req['start']);points[-1]['positions_rad']=list(req['goal'])
    meta=dict(method=cfg['method'],label=cfg['label'],fingerprint=signature,source_path_sha256=source_id,
              original_duration_s=winner['duration_s'],duration_s=law['duration_s'],cached=reused,
              elapsed_s=time.monotonic()-began,geometry_preserved=True,limits=core.LIMITS.tolist(),
              exact_extrema_peaks=law['peaks'],verification=proof)
    log(f'TIMING READY: {winner["duration_s"]:.3f}s → {law["duration_s"]:.3f}s outward; limits unchanged')
    return points,law,proof['collision_zone'],meta
