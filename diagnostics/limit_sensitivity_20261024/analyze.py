"""Saved-data analysis, target-time bounds and plots; never opens hardware."""
import csv
import json
from pathlib import Path
import sys

import numpy as np
from scipy.signal import savgol_filter

from study import HERE, ROOT, read, save, sha, guard
from drum_planner.timing import core


def record_reference():
    file=ROOT/'recordings/record4.json';record=read(file)
    t=np.array([s['time_s'] for s in record['samples']])
    q=np.array([s['positions_rad'] for s in record['samples']])
    assert np.all(np.diff(t)>0) and np.isfinite(q).all()
    step=.02;grid=np.arange(t[0],t[-1],step)
    uniform=np.column_stack([np.interp(grid,t,q[:,j]) for j in range(7)])
    estimates=[]
    for window in (7,11,15):
        derivatives=[savgol_filter(uniform,window,3,deriv=d,delta=step,axis=0)
                     [window//2:-(window//2)] for d in (1,2,3)]
        estimates.append(dict(window_samples=window,window_span_s=(window-1)*step,
            peaks_per_joint=[np.max(abs(x),axis=0).tolist() for x in derivatives],
            peak_any_joint=[float(np.max(abs(x))) for x in derivatives],
            p95_any_joint=[float(np.percentile(np.max(abs(x),axis=1),95)) for x in derivatives]))
    windows=[]
    for duration in (.5,.25,1/6):
        starts=grid[grid+duration<=t[-1]]
        first=np.column_stack([np.interp(starts,t,q[:,j]) for j in range(7)])
        last=np.column_stack([np.interp(starts+duration,t,q[:,j]) for j in range(7)])
        windows.append(dict(interval_s=duration,max_net_joint_change_deg=np.degrees(np.max(abs(last-first),axis=0)).tolist()))
    source=read(ROOT/'diagnostics/fast_gpu_20261024/evidence/checkpoint_2.json')['source']
    path=core.original(source)
    result=dict(file=str(file),sha256=sha(file),source=record['source'],duration_s=record['duration_s'],
        sample_count=len(t),dt_range_s=[float(np.diff(t).min()),float(np.diff(t).max())],
        source_sample_positions_unchanged=True,method='linear resample at 50 Hz; cubic Savitzky-Golay; trim half-window ends',
        joint_order=record['joint_order'],estimates=estimates,beat_windows=windows,
        position_range_deg=np.degrees(np.ptp(q,axis=0)).tolist(),
        closest_to_frozen_endpoints_max_joint_error_deg=[float(np.degrees(np.min(np.max(abs(q-e),axis=1)))) for e in path([0,1])],
        caveat='Derivative estimates depend on smoothing, especially acceleration and jerk. Disabled-motor demonstration, not powered capability or approved limits. Not the same endpoints as the benchmark route.')
    save(HERE/'record4_analysis.json',result)
    return result,t,q


def main():
    rows=read(HERE/'primary2/results.json');by={r['name']:r for r in rows}
    base=by['baseline'];T0=base['movement_s'];limits=np.array(base['limits'])
    max_displacement=max(base['joint_displacement_rad'])
    baseline_law=read(HERE/'primary2/baseline.law.json')
    targets=[]
    for target in (1.,.5,.25,1/6):
        factor=T0/target
        peak=np.array(base['original_checker_peaks'])*factor**np.arange(1,4)[:,None]
        density=core.decode(baseline_law['density']);density.c/=factor
        verified,_=core.density_peaks(core.decode(baseline_law['path']),density)
        np.testing.assert_allclose(peak,verified,rtol=1e-7,atol=1e-6)
        assert abs(density.integrate(0,1)-target)<1e-10
        targets.append(dict(target_s=target,speed_factor=factor,
            sufficient_limits_same_clock=(limits*factor**np.arange(1,4)).tolist(),
            sufficient_multipliers=(factor**np.arange(1,4)).tolist(),
            actual_same_clock_peaks=np.max(verified,axis=1).tolist(),
            necessary_individual_lower_limits=[base['path_velocity_integral_rad']/target,
                4*max_displacement/target**2,32*max_displacement/target**3],
            necessary_limits_caveat='Separate necessary bounds only, not jointly sufficient. Acceleration/jerk assume rest-to-rest with zero endpoint acceleration; no torque model.',
            sufficient_limits_caveat='Verified feasible kinematically on the frozen path, not minimal limits or hardware-approved settings. No strike/rebound/settle time included.'))
    save(HERE/'target_times.json',targets)
    reference,t,q=record_reference()
    checks=read(HERE/'checks/results.json') if (HERE/'checks/results.json').is_file() else []
    preserved=read(HERE/'before_hashes.json')
    changes=[name for name,value in preserved.items() if sha(ROOT/name)!=value]
    assert not changes,changes
    assert np.array_equal(core.LIMITS,[.6,1.2,6.])
    # Explicit comparison against archived source provenance, not the new camera.
    origin=read(ROOT/'diagnostics/fast_gpu_20261024/evidence/checkpoint_2.json')
    endpoint=core.original(origin['source'])([0,1])
    current_destinations={}
    for name in ('R1','R4'):
        file=ROOT/f'destinations/right/destination{name}.json'
        current_destinations[name]=dict(present=file.exists())
        if file.exists():
            value=np.array(read(file)['samples'][-1]['positions_rad'])
            current_destinations[name]['max_difference_from_frozen_deg']=float(np.degrees(np.max(abs(value-endpoint[0 if name=='R1' else 1]))))
    summary=dict(baseline_s=T0,main_cases=len(rows),followup_cases=len(checks),
        main_all_converged=all(not r['details']['warnings'] for r in rows),
        unchanged_source_and_input_files=len(preserved),changed_files=changes,
        application_limits_unchanged=core.LIMITS.tolist(),
        device_attempts=[x for r in rows+checks for x in r['denied_device_attempts']],
        hardware_access=False,source_geometry_sha256=base['source_path_sha256'],
        source_file_sha256=base['source_file_sha256'],source_origin=origin['origin'],
        current_destination_comparison=current_destinations,
        baseline_scalar_lower_bound_s=base['combined_lower_bound_s'],
        baseline_maximum_possible_improvement_fraction=1-base['combined_lower_bound_s']/T0,
        runtime_s=dict(min=min(r['optimizer_s'] for r in rows),median=float(np.median([r['optimizer_s'] for r in rows])),max=max(r['optimizer_s'] for r in rows)),
        targets=targets,record4=reference,
        statements=['Frozen pre-camera-move route; not new live-scene collision approval.',
                    'No geometry, positional limits, endpoint stops or holds changed. Holds excluded from transfer durations.',
                    'No timing/trajectory cache used for optimizing any case. Saved fixed geometry is an explicit controlled input.',
                    'Same motor limits left installed. No movement, camera or encoder access.'])
    save(HERE/'summary.json',summary)
    with (HERE/'results.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['case','V_rad_s','A_rad_s2','J_rad_s3','motion_s','saving_percent','optimizer_s','lower_bound_s'])
        for r in rows:writer.writerow([r['name'],*r['limits'],r['movement_s'],100*(1-r['movement_s']/T0),r['optimizer_s'],r['combined_lower_bound_s']])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    labels=['Current limits','Velocity ×2','Acceleration ×2','Jerk ×2','All ×2','All ×4','All ×8','All ×16','V×8, A×64, J×512']
    names=['baseline','velocity_x2','acceleration_x2','jerk_x2','all_x2','all_x4','all_x8','all_x16','time_compression_x8']
    fig,ax=plt.subplots(figsize=(10,5.5),layout='constrained')
    values=[by[n]['movement_s'] for n in names]
    ax.barh(labels[::-1],values[::-1],color=['#c76b40']+['#4878a8']*8)
    for i,v in enumerate(values[::-1]):ax.text(v+.035,i,f'{v:.3f} s',va='center',fontsize=10)
    ax.axvline(.5,color='#3a8739',linestyle='--',label='120 BPM quarter-note interval')
    ax.set_xlim(0,3.95);ax.set_xlabel('R1 → R4 movement time (seconds), holds excluded')
    ax.set_title('Same archived path, different hypothetical timing limits\nSoftware only — NOT hardware-approved settings')
    ax.legend(loc='lower right');fig.savefig(HERE/'sensitivity.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True,layout='constrained')
    for j in range(4):axes[0].plot(t,np.degrees(q[:,j]),label=f'J{j+1}')
    for j in range(4,7):axes[1].plot(t,np.degrees(q[:,j]),label=f'J{j+1}')
    for a in axes:a.legend(ncol=4);a.set_ylabel('Recorded angle (degrees)');a.grid(alpha=.25)
    axes[0].set_title('record4: slow large-arm positioning and much faster distal-joint strokes\nDisabled-motor demonstration, not powered tracking evidence')
    axes[1].set_xlabel('Recording time (seconds)');fig.savefig(HERE/'record4_joints.png',dpi=150);plt.close(fig)
    print(json.dumps({k:summary[k] for k in ['baseline_s','main_cases','followup_cases','changed_files','runtime_s','current_destination_comparison']},indent=2))


if __name__=='__main__':main()
