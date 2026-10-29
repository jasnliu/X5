"""Fresh exact-path timing forecasts; NEVER promote them to hardware-ready plans."""
from pathlib import Path
import time

import numpy as np

from drum_planner.timing import core
from drum_planner.experiment import motion_timing,motion_settings
from .schema import read,write,file_hash,identity,VERSION


def estimate(plan_path,profile_path,folder,quality='balanced'):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    plan_path=Path(plan_path).resolve();plan=read(plan_path)
    if plan.get('status')!='planned' or plan.get('checkpoint_sequence'):
        raise ValueError('Choose one successful single-leg plan; forecast each checkpoint leg separately')
    side=plan['moving_side'];profile=motion_settings.load(profile_path,side)
    request=read(plan_path.parent/'backend_request.json')
    if request.get('execute') is not False:raise ValueError('Only non-executable experiment paths are accepted')
    source=dict(timing=plan['source_timing'],path_waypoints=plan['path_waypoints'],request=request)
    if plan['timing']['source_path_sha256']!=core.identity(source):raise ValueError('Saved source geometry identity changed')
    for key,field in [('urdf','calibrated_urdf_sha256'),('srdf','srdf_sha256')]:
        if file_hash(request[key])!=plan['model'][field]:raise ValueError('Saved model changed')
    limits=motion_settings.matrix(profile)
    started=time.monotonic()
    law=motion_timing.optimize(source,print,limits=limits,quality=quality)
    if not np.isfinite(law['duration_s']) or law['duration_s']/.002>100000:
        raise ValueError('Forecast exceeds the bounded 100,000-sample validation budget; no unchecked result exported')
    proof=motion_timing.validate(law,source,folder/'check')
    # Independent original root checker, not only optimizer metadata.
    peaks,_=core.density_peaks(core.decode(law['path']),core.decode(law['density']))
    if np.any(peaks>limits*(1+1e-8)):raise ValueError('Independent limit check failed')
    if law['path']!=core.encode(core.original(source)):raise ValueError('Path changed')
    evidence_path=Path(profile_path).with_suffix('.evidence.json');evidence=None
    if evidence_path.exists():
        evidence=read(evidence_path)
        if evidence.get('profile_sha256')!=motion_settings.identity(profile):raise ValueError('Profile evidence hash mismatch')
    report=dict(schema=VERSION,kind='fixed_path_forecast',side=side,
        source_plan=str(plan_path),source_plan_sha256=file_hash(plan_path),
        source_geometry_sha256=core.identity(source),profile=profile,profile_sha256=motion_settings.identity(profile),
        profile_evidence=evidence,quality=quality,previous_movement_s=plan['duration_outward_s'],
        predicted_movement_s=law['duration_s'],calculation_s=time.monotonic()-started,
        geometric_path_unchanged=True,verification=proof,limits=limits.tolist(),peaks=peaks.tolist(),
        hardware_execution_available=False,motion_authorized=False,global_fastest_claimed=False,
        result_cache_used=False,physical_route_validation_required=True,
        note='Fastest checked clock found at these candidate limits on this exact saved path; NOT measured route performance or an absolute optimum.')
    write(folder/'timing.json',law);write(folder/'report.json',report)
    (folder/'REPORT.md').write_text('# Fixed-path timing forecast\n\n'+report['note']+'\n\n'+
        f"Previous: {plan['duration_outward_s']:.6f} s. Candidate: {law['duration_s']:.6f} s.\n"+
        f"Fresh timing and validation: {report['calculation_s']:.3f} s (path generation/acquisition excluded).\n")
    return report
