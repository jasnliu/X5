"""Saved-input experiments only: GPU timing with an explicit 1% reference gate.

The original CPU implementation remains untouched and supplies the quality
reference. Its existing exact-input caches avoid re-solving qualified paths.
A previously unseen curve may need a slow first CPU qualification; that time
is separately logged and NEVER counted as a GPU speedup.
"""
import fcntl
import hashlib
import json
from pathlib import Path
import time

from ..common import atomic_json
from . import core,integration,smooth_v2,rounding,math_cache,motion_quality
from .benchmark import validate
from .vulkan import device,optimizer


def cpu_retime(winner,report,req,context,folder,log):
    selected=smooth_v2.retime if req.get('moving_side','right')=='left' else integration.retime
    return selected(winner,report,req,context,folder,log)


def reference_source(winner,req,law):
    source=dict(path_waypoints=winner['path'],timing=winner['timing'],request=req)
    if law.get('retiming_geometry_preserved'):
        source=rounding.as_source(core.decode(law['path']),source)
    optimizer.reference_check(source,law)
    return source


def retime(winner,report,req,context,folder,log):
    if not context.get('experiment_gpu_timing') or req.get('execute') is not False:
        raise ValueError('Vulkan timing is authorized only in saved-input experiment mode')
    began=time.monotonic();folder=Path(folder)
    try:
        gpu=device.probe()
    except device.GPUUnavailable as exc:
        log('*** GPU UNAVAILABLE — CPU TIMING FALLBACK *** '+str(exc))
        points,law,proof,meta=cpu_retime(winner,report,req,context,folder,log)
        meta=dict(meta,requested_backend='vulkan',actual_backend='cpu',gpu_used=False,
                  fallback_reason=str(exc),label='CPU FALLBACK — GPU unavailable; '+meta['label'])
        log('CPU FALLBACK READY: this trajectory was NOT optimized on the GPU.')
        return points,law,proof,meta
    log(f'GPU AVAILABLE: {gpu["name"]}, FP64. Preparing exact-input CPU quality reference.')
    log('CPU REFERENCE QUALIFICATION: a new curve can require a full CPU solve once; '
        'reference/cache time is separate from GPU time. Hardware modes are unchanged.')
    started=time.monotonic()
    _,reference,reference_proof,reference_meta=cpu_retime(winner,report,req,context,folder,
        lambda message:log('CPU REFERENCE | '+message))
    reference_seconds=time.monotonic()-started
    reference_cached=bool(reference_meta.get('cached') or (reference_meta.get('math_cache') or {}).get('cached'))
    log(f'CPU REFERENCE READY: {reference["duration_s"]:.6f}s movement; '
        f'{reference_seconds:.2f}s reference lookup/qualification; cached={reference_cached}. '
        f'GPU acceptance ceiling {reference["duration_s"]*optimizer.MAX_DURATION_RATIO:.6f}s.')
    if reference['kind']!='time_density':
        # Exact straight-block S-curves do not use SLSQP or require optimization.
        # Preserve that analytic branch rather than changing path geometry.
        points=core.samples(reference,dict(path_waypoints=winner['path'],timing=winner['timing'],request=req))
        points[0]['positions_rad']=list(req['start']);points[-1]['positions_rad']=list(req['goal'])
        meta=dict(reference_meta,requested_backend='vulkan',actual_backend='analytic_cpu',gpu_used=False,
                  label='Analytic CPU timing — no nonlinear optimizer needed',cpu_reference_elapsed_s=reference_seconds)
        if 'smoothing_attempts' in reference_meta:
            meta['label']+='; '+reference_meta['label']
        log('ANALYTIC TIMING: retained exact straight-block solution; no GPU optimization necessary.')
        return points,reference,reference_proof,meta
    source=reference_source(winner,req,reference)
    cfg=optimizer.policy()
    ref_identity={k:v for k,v in reference.items() if k!='details'}
    cfg.update(gpu_implementation=device.fingerprint(),device_identity=gpu['identity'],
        moving_side=req.get('moving_side','right'),
        reference_sha256=hashlib.sha256(json.dumps(ref_identity,sort_keys=True,allow_nan=False).encode()).hexdigest(),
        integration_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    def calculate():
        # Two existing timing children may qualify references concurrently, but
        # one GPU solver owns the small integrated GPU at a time. Lock lifetime
        # is tied to the disposable worker; cancellation releases it automatically.
        lockfile=device.BUILD/'solve.lock'
        with lockfile.open('a') as lock:
            log('GPU QUEUE: waiting for the bounded Vulkan timing slot')
            wait_start=time.monotonic();fcntl.flock(lock,fcntl.LOCK_EX)
            queue_seconds=time.monotonic()-wait_start
            log('GPU OPTIMIZATION START: optimizer state/updates on device; no CPU SLSQP in this phase')
            law=optimizer.optimize(source,reference,log)
            return dict(law=law,queue_seconds=queue_seconds)
    started=time.monotonic()
    payload,reuse=math_cache.solve(source,cfg,calculate,log);law=payload['law']
    gpu_seconds=time.monotonic()-started
    if law.get('method')!='vulkan_al_lbfgs' or not law.get('details',{}).get('gpu_native_optimizer'):
        raise ValueError('GPU cache did not contain a GPU-native optimizer result')
    optimizer.reference_check(source,law)
    passed,quality=optimizer.quality_gate(law,reference)
    if not passed:raise ValueError('GPU result failed the 1% duration/no-added-stops gate')
    if reference_meta.get('smoothing_applied'):
        if not smooth_v2.acceptable(quality,reference_meta['before'],smooth_v2.policy()):
            raise ValueError('GPU timing did not retain the existing left-arm fluidity requirements')
    # Cache hits still receive continuous extrema and a fresh scene-bound check.
    started=time.monotonic()
    path=core.decode(law['path']);h=core.decode(law['density'])
    peaks,_=core.density_peaks(path,h)
    if core.scale_needed(peaks)>1+1e-8:
        raise ValueError('GPU continuous velocity/acceleration/jerk verification failed')
    log('GPU VERIFY: continuous extrema, exact curve, joint limits, rest endpoints, dense collision and zone')
    proof=validate(law,source,folder/'gpu_retiming_check')
    verification_seconds=time.monotonic()-started
    for name in ('original_path_sha256','geometry_reference','retiming_geometry_preserved'):
        if name in reference:law[name]=reference[name]
    if reference_meta.get('smoothing_applied'):law['geometry_preserved']=False
    atomic_json(folder/'gpu_timing_law.json',law)
    points=core.samples(law,source)
    clean=[points[0]]
    for point in points[1:-1]:
        if point['time_s']-clean[-1]['time_s']>1e-10 and points[-1]['time_s']-point['time_s']>1e-10:clean.append(point)
    points=clean+[points[-1]]
    points[0]['positions_rad']=list(req['start']);points[-1]['positions_rad']=list(req['goal'])
    meta=dict(reference_meta,method='vulkan_al_lbfgs',requested_backend='vulkan',actual_backend='vulkan',
        label='GPU Vulkan FP64 timing'+(' (cached)' if reuse['cached'] else ''),gpu_used=True,
        gpu_device=gpu,implementation_sha256=cfg['gpu_implementation'],cached=reuse['cached'],math_cache=reuse,
        cpu_reference_elapsed_s=reference_seconds,cpu_reference_cached=reference_cached,
        cpu_reference_duration_s=reference['duration_s'],duration_s=law['duration_s'],
        duration_ratio=law['duration_s']/reference['duration_s'],max_duration_ratio=optimizer.MAX_DURATION_RATIO,
        gpu_phase_elapsed_s=gpu_seconds,verification_elapsed_s=verification_seconds,
        elapsed_s=time.monotonic()-began,exact_extrema_peaks=law['peaks'],verification=proof,
        gpu_quality=quality,reference_quality=motion_quality.law_metrics(reference))
    if reference_meta.get('smoothing_applied'):
        meta['label']+='; retained checked C3 curve'
    atomic_json(folder/'gpu_timing_report.json',meta)
    log(f'GPU TIMING READY: {law["duration_s"]:.6f}s movement '
        f'({100*(meta["duration_ratio"]-1):+.3f}% vs CPU); GPU phase {gpu_seconds:.2f}s, '
        f'final checks {verification_seconds:.2f}s, CPU qualification {reference_seconds:.2f}s; '
        f'GPU result cached={reuse["cached"]}.')
    return points,law,proof['collision_zone'],meta
