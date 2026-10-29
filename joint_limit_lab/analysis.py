"""Score timestamped evidence; absent or failed evidence never becomes a limit."""
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter

from drum_planner.experiment.motion_settings import settings,FIELDS,identity as profile_identity
from . import trajectories
from .schema import array,identity,read,write,VERSION

ARRAYS=('time_s','command_time_s','command_q_rad','q_rad','velocity_rad_s','current_a',
        'temperature_c','fault_bits','mode','feedback_age_s','feedback_sequence','current_age_s','torque_nm')


def load_trace(path):
    path=Path(path)
    if path.stat().st_size>32*1024**2:raise ValueError('Trace file too large')
    import zipfile
    with zipfile.ZipFile(path) as z:
        if sum(i.file_size for i in z.infolist())>64*1024**2:raise ValueError('Expanded trace too large')
    with np.load(path,allow_pickle=False) as npz:
        if set(npz.files)!={'metadata',*ARRAYS}:raise ValueError('Missing/unknown telemetry fields')
        import json
        metadata=json.loads(str(npz['metadata']))
        arrays={k:npz[k].copy() for k in ARRAYS}
    return metadata,arrays


def score(campaign,trial,metadata,data):
    config=campaign['config'];a=config['acceptance'];law=trial['law'];errors=[]
    expected=dict(campaign_sha256=campaign['sha256'],trial_sha256=trial['sha256'],
        trial_id=trial['id'],side=config['side'],conditions=config['conditions'])
    if any(metadata.get(k)!=v for k,v in expected.items()):raise ValueError('Telemetry belongs to a different trial/arm/condition')
    if metadata.get('origin') not in ('synthetic','physical_recording'):raise ValueError('Unknown telemetry origin')
    t=np.asarray(data['time_s'],float);n=len(t)
    if not 10<=n<=100000:raise ValueError('Invalid sample count')
    t=array(t,(n,),'time_s');ct=array(data['command_time_s'],(n,),'command_time_s')
    if np.any(np.diff(t)<=0) or t[0]<0:raise ValueError('Telemetry times must strictly increase')
    if np.any(np.diff(ct)<0) or np.any(ct>t+1e-9) or np.any(ct<0):raise ValueError('Invalid command clock')
    values={k:array(data[k],(n,7),k) for k in ARRAYS if k not in ('time_s','command_time_s')}
    if 'position_bounds_rad' in campaign:
        bounds=array(campaign['position_bounds_rad'],(7,2),'position_bounds_rad')
        if np.any(values['q_rad']<bounds[:,0]) or np.any(values['q_rad']>bounds[:,1]):errors.append('measured_position_bounds')
    if t[0]>config['sample_period_s'] or t[-1]<law['duration_s']-1e-6:errors.append('incomplete_trace')
    if t[-1]>law['duration_s']+5:errors.append('unexpected_extra_motion_time')
    gap=float(np.max(np.diff(t)))
    if gap>a['max_feedback_gap_s']:errors.append('feedback_gap')
    if np.max(t-ct)>a['max_command_gap_s']:errors.append('command_gap')
    if np.any(values['feedback_age_s']<0) or np.max(values['feedback_age_s'])>a['max_feedback_gap_s']:errors.append('stale_feedback')
    if np.any(values['current_age_s']<0) or np.max(values['current_age_s'])>a['max_current_age_s']:errors.append('stale_current_feedback')
    seq=values['feedback_sequence']
    if np.any(seq<0) or np.any(seq!=np.floor(seq)) or np.any(np.diff(seq,axis=0)<0):errors.append('invalid_feedback_sequence')
    for j in range(7):
        last=0
        for i in range(1,n):
            if seq[i,j]!=seq[last,j]:last=i
            if t[i]-t[last]>a['max_feedback_gap_s']+1e-9:
                errors.append('repeated_stale_feedback');break
    if np.any(values['fault_bits']!=0):errors.append('motor_fault')
    if np.any(values['mode']!=2):errors.append('motor_not_running')
    if np.max(values['temperature_c'])>=a['max_temperature_c']:errors.append('temperature')
    rise=np.max(values['temperature_c']-values['temperature_c'][0],axis=0)
    if np.max(rise)>a['max_temperature_rise_c']:errors.append('temperature_rise')
    current=np.max(abs(values['current_a']),axis=0)
    if np.any(current>np.array(a['current_limits_a'])*a['max_current_fraction']):errors.append('current_headroom')
    commanded=trajectories.evaluate(law,np.minimum(ct,law['duration_s']))[0]
    if np.max(abs(commanded-values['command_q_rad']))>1e-7:errors.append('commanded_curve_mismatch')
    desired,velocity,_,_=trajectories.evaluate(law,np.minimum(t,law['duration_s']))
    error=values['q_rad']-desired
    maximum=np.max(abs(error),axis=0);rms=np.sqrt(np.mean(error**2,axis=0))
    if np.max(maximum)>a['max_tracking_rad']:errors.append('tracking_error')
    if np.max(rms)>a['rms_tracking_rad']:errors.append('tracking_rms')
    j=trial['joint']-1;held=[k for k in range(7) if k!=j]
    drift=np.max(abs(values['q_rad'][:,held]-np.array(law['anchor'])[held]))
    if drift>a['max_held_joint_drift_rad']:errors.append('held_joint_drift')
    verror=np.max(abs(values['velocity_rad_s']-velocity),axis=0)
    if np.max(verror)>a['max_velocity_error_rad_s']:errors.append('velocity_tracking_error')
    measured_peak=np.max(abs(values['velocity_rad_s']),axis=0)
    ratio=float(measured_peak[j]/law['peaks'][0][j])
    if ratio<a['minimum_peak_tracking_fraction']:errors.append('requested_speed_not_achieved')
    if np.count_nonzero(np.linalg.norm(velocity,axis=1)>1e-6)<a['minimum_motion_samples']:errors.append('under_sampled_motion')
    end=t>=law['duration_s']-min(.2,config['settle_s']/2)
    if np.count_nonzero(end)<5:errors.append('inadequate_end_settling_samples')
    if np.max(abs(values['q_rad'][end]-np.array(law['anchor'])))>a['end_position_error_rad']:errors.append('end_position_not_settled')
    if np.max(abs(values['velocity_rad_s'][end]))>a['end_velocity_rad_s']:errors.append('end_velocity_not_settled')
    # Derivatives from quantized measurements are diagnostics, never the limit
    # estimator. Use two time windows and expose their disagreement explicitly.
    dt=float(np.median(np.diff(t)));grid=np.arange(t[0],t[-1]+dt*.1,dt)
    vg=np.column_stack([np.interp(grid,t,values['velocity_rad_s'][:,k]) for k in range(7)])
    derivative_estimates=[]
    for seconds in (.05,.09):
        window=max(5,int(round(seconds/dt))|1);window=min(window,len(grid)-(1-len(grid)%2))
        if window>=5:
            derivative_estimates.append(dict(window_s=window*dt,
                acceleration_peak_rad_s2=np.max(abs(savgol_filter(vg,window,3,deriv=1,delta=dt,axis=0)),axis=0).tolist(),
                jerk_peak_rad_s3=np.max(abs(savgol_filter(vg,window,3,deriv=2,delta=dt,axis=0)),axis=0).tolist()))
    requested=np.array(law['limits'])[:,j];achieved=np.array(law['peaks'])[:,j]
    return dict(trial_id=trial['id'],trial_sha256=trial['sha256'],origin=metadata['origin'],
        passed=not errors,reasons=sorted(set(errors)),samples=n,max_gap_s=gap,
        max_tracking_rad=maximum.tolist(),rms_tracking_rad=rms.tolist(),
        measured_velocity_peak_rad_s=measured_peak.tolist(),peak_tracking_fraction=ratio,
        peak_current_a=current.tolist(),peak_reported_torque_nm=np.max(abs(values['torque_nm']),axis=0).tolist(),
        peak_temperature_c=np.max(values['temperature_c'],axis=0).tolist(),
        temperature_rise_c=rise.tolist(),commanded_peaks=law['peaks'],
        requested_limit_excitation_fraction=(achieved/requested).tolist(),
        measured_derivatives_diagnostic_only=derivative_estimates,
        interpretation='Passing demonstrates this commanded motion only. Measured derivative noise is not a hardware limit.')


def analyze(campaign,trace_folder,output,*,allow_synthetic=False):
    traces=sorted(Path(trace_folder).glob('*.npz'))
    if len(traces)>2500:raise ValueError('Too many traces')
    lookup={t['id']:t for t in campaign['trials']};scores={};invalid=[];duplicate=set()
    for path in traces:
        try:
            meta,arrays=load_trace(path);key=meta.get('trial_id')
            if key not in lookup:raise ValueError('Unknown trial ID')
            if key in scores:duplicate.add(key);raise ValueError('Duplicate trial record cannot count as a repeat')
            scores[key]=score(campaign,lookup[key],meta,arrays)
        except Exception as exc:invalid.append(dict(file=str(path),reason=str(exc)))
    for key in duplicate:scores[key].update(passed=False,reasons=['duplicate_record'])
    config=campaign['config'];groups={};selected=[];coverage=[];directions=(-1,1)
    for trial in campaign['trials']:groups.setdefault(trial['group'],[]).append(trial)
    for name,items in groups.items():
        accepted=None;stop=None
        for li,level in enumerate(config['test_levels']):
            batch=[t for t in items if t['level_index']==li]
            evidence=[scores.get(t['id']) for t in batch]
            # Higher levels cannot substitute for missing/failed lower levels.
            if len(batch)!=config['repeats'] or any(e is None or not e['passed'] for e in evidence):
                stop=dict(level=level,reason='missing_or_failed_repeat',
                    missing=[t['id'] for t,e in zip(batch,evidence) if e is None],
                    failed={t['id']:e['reasons'] for t,e in zip(batch,evidence) if e and not e['passed']});break
            origins={e['origin'] for e in evidence}
            if len(origins)!=1:stop=dict(level=level,reason='mixed_evidence_origins');break
            accepted=dict(group=name,trial=batch[0],scores=evidence,origin=origins.pop())
        if accepted:selected.append(accepted)
        coverage.append(dict(group=name,highest_complete_level=None if accepted is None else accepted['trial']['level'],
            stopped=stop,ceiling_reached=stop is None))
    incomplete=[];limits=np.zeros((3,7));origins=set()
    for joint in range(1,8):
        cohort=[]
        for pi in range(len(campaign['anchors'])):
            for direction in directions:
                good=[s for s in selected if s['trial']['joint']==joint and s['trial']['pose_index']==pi and s['trial']['direction']==direction]
                if not good:incomplete.append(dict(joint=joint,pose_index=pi,direction=direction));continue
                cohort.append(np.max([np.array(s['trial']['law']['peaks'])[:,joint-1] for s in good],axis=0))
                origins.update(s['origin'] for s in good)
        if cohort:limits[:,joint-1]=np.min(cohort,axis=0)
    synthetic=origins!={'physical_recording'}
    conditions=config['conditions'];condition_gaps=[k for k in ('payload_id','controller_id') if conditions[k]=='UNSPECIFIED']
    if conditions['supply_voltage_v'] is None:condition_gaps.append('supply_voltage_v')
    all_origins={s['origin'] for s in scores.values()}
    complete=not incomplete and not invalid and not condition_gaps and len(campaign['anchors'])>=2 and len(origins)==1 and len(all_origins)==1
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    profile_path=None
    if complete and (not synthetic or allow_synthetic):
        fraction=config['operating_speed_fraction'];operating=limits*fraction**np.arange(1,4)[:,None]
        profile=settings(config['side'],dict(name=('SYNTHETIC DEMO ONLY' if synthetic else 'Measured-motion candidate; route validation required'),
            **dict(zip(FIELDS,operating.tolist()))))
        profile_path=output/(('SYNTHETIC_NOT_FOR_HARDWARE_' if synthetic else 'candidate_')+config['side']+'.json')
        write(profile_path,profile)
        write(profile_path.with_suffix('.evidence.json'),dict(profile_sha256=profile_identity(profile),
            campaign_sha256=campaign['sha256'],synthetic=synthetic,conditions=conditions,
            hardware_limit_certified=False,combined_route_physically_validated=False,
            operating_speed_fraction=fraction,selected_trials=[s['trial']['id'] for s in selected],
            interpretation='Conservative commanded-motion candidate from tested probes. Component maxima may not be jointly achievable; validate complete routes.'))
    report=dict(schema=VERSION,kind='analysis',campaign_sha256=campaign['sha256'],side=config['side'],
        scores=list(scores.values()),invalid_records=invalid,groups=coverage,missing_coverage=incomplete,
        condition_gaps=condition_gaps,at_least_two_poses=len(campaign['anchors'])>=2,evidence_origins=sorted(all_origins),
        complete_probe_coverage=complete,synthetic=synthetic,hardware_limits_known=False,
        observed_commanded_envelope=limits.tolist(),profile=None if profile_path is None else str(profile_path.resolve()),
        combined_route_validation_required=True,
        conclusion='Synthetic results do not establish hardware capability.' if synthetic else
        'Successfully tested operating envelope only; untested maxima and combined-route capability remain unknown.')
    write(output/'report.json',report)
    lines=['# Joint-limit study result','',report['conclusion'],'',
        f"Scored {len(scores)} traces; {sum(s['passed'] for s in scores.values())} passed; {len(invalid)} invalid records.",
        f"Missing joint/pose/direction cohorts: {len(incomplete)}. Conditions missing: {condition_gaps}.",
        'Profile: '+(str(profile_path) if profile_path else 'NOT EXPORTED — incomplete evidence or synthetic export not requested.'),'',
        'The exported V/A/J box is a candidate for planning, not a certified simultaneous hardware envelope.',
        'Long/repeated drum routes, duty-cycle heating and actual stopping behavior require separate physical validation.']
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    import csv
    with (output/'joint_envelope.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['joint','demonstrated_command_velocity_rad_s',
            'demonstrated_command_acceleration_rad_s2','demonstrated_command_jerk_rad_s3','missing_cohorts','synthetic'])
        for j in range(7):writer.writerow([j+1,*limits[:,j],sum(x['joint']==j+1 for x in incomplete),synthetic])
    return report
