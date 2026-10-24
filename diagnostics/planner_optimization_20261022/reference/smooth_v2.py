"""Left-only bounded smoothing, separately versioned from the v1 policy.

Select the broadest independently validated local C3 kernel blend. Endpoint
timing then uses v1's mathematical engine on that NEW curve. No demonstration
is an input. All failed proposals are discarded, not executed or hidden.
"""
import hashlib,json,time,warnings
from pathlib import Path
import numpy as np
from ..common import atomic_json,digest
from ..experiment.runtime import Oracle
from . import core,adaptive,rounding,motion_quality,benchmark,integration
from ..experiment import cache as persistent_cache


def policy():
    cfg=json.loads(Path(__file__).with_name('selected_v2.json').read_text())
    if cfg['schema']!='x5-left-corner-smoothing-v2' or not np.array_equal(cfg['limits'],core.LIMITS):
        raise ValueError('Unexpected v2 policy or changed motion limits')
    return cfg


def fingerprint():
    files=['selected_v2.json','smooth_v2.py','rounding.py','motion_quality.py']
    return hashlib.sha256((integration.fingerprint()+''.join(digest(Path(__file__).with_name(p)) for p in files)).encode()).hexdigest()


def acceptable(quality,baseline,cfg):
    return (quality['internal_stops']==0 and quality['deep_slowdowns']==0
        and quality['interior_speed_cv']<=cfg['max_speed_cv']
        and quality['interior_speed_p10_ratio']>=cfg['min_speed_p10_ratio']
        and quality['duration_s']<=baseline['duration_s']*cfg['max_duration_ratio'])


def optimize_rounded(source,cfg,log):
    """Report SLSQP trial-density bound handling, not as a robot-limit fault.

    This one SciPy diagnostic concerns temporary optimizer variables only.
    Actual path/derivative violations still fail independent validation.
    All other warnings are re-emitted, including if optimization raises.
    """
    adjustments=0;caught=[]
    try:
        with warnings.catch_warnings(record=True) as caught:
            warnings.filterwarnings('always',message='Values in x were outside bounds during a minimize step, clipping to bounds',
                                    category=RuntimeWarning,module=r'scipy\.optimize\..*')
            law=adaptive.optimize(source,levels=tuple(cfg['levels']),ftol=cfg['ftol'],maxiter=cfg['maxiter'],log=log)
    finally:
        for w in caught:
            if (w.category is RuntimeWarning and 'scipy/optimize/' in w.filename.replace('\\','/') and
                str(w.message)=='Values in x were outside bounds during a minimize step, clipping to bounds'):
                adjustments+=1
            else:warnings.warn_explicit(w.message,w.category,w.filename,w.lineno)
        if adjustments:
            log(f'SMOOTH V2 solver: {adjustments} trial time-density bound adjustments; no robot joints clipped')
    return law,adjustments


def choose(source,folder,log,cfg=None):
    cfg=cfg or policy();folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    baseline=core.local_law(source,'scurve');before=motion_quality.law_metrics(baseline)
    attempts=[];timed=0;atomic_json(folder/'request.json',source['request'])
    oracle=Oracle(folder/'request.json',folder)
    try:
        for width in cfg['widths']:
            params=dict(kind=cfg['method'],fraction=width,simplify=cfg['simplify_rad'])
            row=dict(params=params);attempts.append(row)
            log(f'SMOOTH V2: C3 halfwidth {width:.3f}; checking bounded deviation, joint limits, full robot/stick and zone')
            try:
                pp,_=rounding.propose(source,**params)
                ok,geometry=rounding.inspect(pp,source,oracle,joint_tube_rad=cfg['joint_tube_rad'],
                    workspace_limit_m=cfg['workspace_limit_m'])
                row['geometry']=geometry
                if not ok:
                    row.update(status='rejected_geometry',reason=geometry.get('reason'))
                    log(f'SMOOTH V2 rejected: {row["reason"]}; trying a smaller blend');continue
                if timed>=cfg['max_timed_candidates']:
                    row.update(status='not_timed',reason='bounded postprocessing budget');break
                timed+=1
                rounded=rounding.as_source(pp,source)
                law,row['solver_trial_bound_adjustments']=optimize_rounded(rounded,cfg,log)
                quality=motion_quality.law_metrics(law);row['quality']=quality
                if not acceptable(quality,before,cfg):
                    row.update(status='rejected_quality',reason='stop/slowdown/variation/duration gate')
                    log('SMOOTH V2: still too segmented or too slow; trying another blend');continue
                log('SMOOTH V2: independent final timed-curve collision/zone + derivative checks')
                proof=benchmark.validate(law,rounded,folder/f'final_{len(attempts)}')
                law.update(method='adaptive_c3_v2',geometry_preserved=False,retiming_geometry_preserved=True,
                    original_path_sha256=core.identity(source),geometry_reference='new bounded C3-rounded curve')
                row['status']='selected'
                details=dict(before=before,after=quality,rounding=geometry,params=params,attempts=attempts,
                    continuous_geometry='C3 compact-kernel blend, exact start/goal')
                return rounded,law,proof,details
            except ValueError as exc:
                row.update(status='rejected_curve',reason=str(exc));log(f'SMOOTH V2 rejected curve: {exc}')
    finally:oracle.close()
    return None,None,None,dict(before=before,attempts=attempts,reason='No validated satisfactory blend within bounds')


def retime(winner,report,req,context,folder,log):
    if req.get('moving_side','right')!='left':raise ValueError('V2 smoothing is left-only')
    if not winner['timing']['stop_at_corners']:
        log('SMOOTH V2: selected left path already continuous; retain original v1 retiming')
        return integration.retime(winner,report,req,context,folder,log)
    cfg=policy();source=dict(path_waypoints=winner['path'],timing=winner['timing'],request=req)
    signature=fingerprint();source_id=core.identity(source);began=time.monotonic();reused=False
    key=hashlib.sha256((signature+source_id+report['pool_key']).encode()).hexdigest()
    cache=Path(context['pool_root'])/'timing_v2'/key;cache.mkdir(parents=True,exist_ok=True)
    files={name:cache/(name+'.json') for name in ('law','source','verification')}
    persistent_key=persistent_cache.identity(context,req,'timing_v2',key)
    if not files['verification'].exists():persistent_cache.restore(persistent_key,cache,log)
    try:
        checked=json.loads(files['verification'].read_text())
        if (checked['fingerprint']!=signature or checked['original_path_sha256']!=source_id or
                checked['pool_key']!=report['pool_key'] or any(digest(files[k])!=checked[k+'_sha256'] for k in ('law','source'))):
            raise ValueError('Stale or damaged v2 cache')
        law=json.loads(files['law'].read_text());rounded=json.loads(files['source'].read_text())
        if law['source_path_sha256']!=core.identity(rounded) or law['original_path_sha256']!=source_id:
            raise ValueError('V2 cache geometry mismatch')
        rounded['request']=req;proof=checked['proof'];details=checked['details'];reused=True
        log(f'SMOOTH V2 cache: {law["duration_s"]:.3f}s outward; same validated rounded curve')
    except (OSError,ValueError,KeyError,TypeError):
        rounded,law,proof,details=choose(source,Path(folder)/'smoothing_v2',log,cfg)
        if law is None:
            log('SMOOTH V2: no acceptable safe blend; explicitly retaining original v1, without changing margins')
            points,law,proof,meta=integration.retime(winner,report,req,context,folder,log)
            meta.update(smoothing_applied=False,smoothing_attempts=details,
                label='V1 fallback — no valid fluid blend within deviation bounds')
            return points,law,proof,meta
        atomic_json(files['law'],law);atomic_json(files['source'],rounded)
        atomic_json(files['verification'],dict(fingerprint=signature,original_path_sha256=source_id,
            pool_key=report['pool_key'],law_sha256=digest(files['law']),source_sha256=digest(files['source']),proof=proof,details=details))
    persistent_cache.publish(persistent_key,cache,[p.name for p in files.values()])
    points=core.samples(law,rounded)
    # Tiny duplicated event times can round to identical timestamps on return.
    # Removing only sub-nanosecond duplicate samples does not change the law.
    clean=[points[0]]
    for point in points[1:-1]:
        if point['time_s']-clean[-1]['time_s']>1e-10 and points[-1]['time_s']-point['time_s']>1e-10:clean.append(point)
    points=clean+[points[-1]]
    points[0]['positions_rad']=list(req['start']);points[-1]['positions_rad']=list(req['goal'])
    meta=dict(method='adaptive_c3_v2',label=cfg['label'],version=2,fingerprint=signature,cached=reused,
        elapsed_s=time.monotonic()-began,geometry_preserved=False,smoothing_applied=True,
        original_duration_s=details['before']['duration_s'],raw_global_duration_s=winner['duration_s'],
        duration_s=law['duration_s'],source_path_sha256=source_id,smoothed_path_sha256=core.identity(rounded),
        smoothed_path_waypoints=rounded['path_waypoints'],limits=core.LIMITS.tolist(),
        exact_extrema_peaks=law['peaks'],verification=proof,**details)
    log(f'SMOOTH V2 READY: {details["before"]["internal_stops"]} → {details["after"]["internal_stops"]} internal stops; '
        f'{details["before"]["duration_s"]:.3f}s → {law["duration_s"]:.3f}s; tip deviation {details["rounding"]["tip_deviation_m"]*1000:.2f} mm')
    return points,law,proof['collision_zone'],meta
