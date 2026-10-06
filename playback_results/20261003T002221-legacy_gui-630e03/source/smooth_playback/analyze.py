"""Offline analysis of physical motor measurements (never commanded-only scores)."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.signal import butter,sosfiltfilt,savgol_filter


def analyze(directory):
    directory=Path(directory)
    summary=json.loads((directory/'summary.json').read_text())
    with open(directory/'feedback.csv') as f:feedback=list(csv.DictReader(f))
    with open(directory/'trace.csv') as f:trace=list(csv.DictReader(f))
    events=[json.loads(s) for s in (directory/'events.jsonl').read_text().splitlines()]
    plays=[e for e in events if e['phase']=='PLAYBACK']
    if not plays:return dict(path=str(directory),success=False,error='No playback')
    start=plays[0]['t'];duration=summary['trajectory']['duration_s'];end=start+duration
    grid=np.arange(start+.25,end-.25,.01)
    if len(grid)<100:return dict(path=str(directory),success=False,error='Incomplete playback')
    q=[];firmware_velocity=[];gaps=[];counts=[]
    for i in range(1,8):
        rows=[r for r in feedback if r['side']=='right' and int(r['motor'])==i and start<=float(r['monotonic_s'])<=end]
        ts=np.array([float(r['monotonic_s']) for r in rows]);vs=np.array([float(r['position_rad']) for r in rows])
        if len(ts)<100:raise ValueError('Insufficient physical feedback')
        q.append(np.interp(grid,ts,vs));gaps.append(float(np.max(np.diff(ts))));counts.append(len(ts))
        # Installed SDK: RS04 J1/J2 ±15, RS03 J3/J4 ±20,
        # RS00 J5/J6/J7 ±33 rad/s. No numerical differentiation here.
        vmax=[15.,15.,20.,20.,33.,33.,33.][i-1]
        velocity=-(np.array([float(r['velocity_raw']) for r in rows])/65535*2-1)*vmax
        firmware_velocity.append(np.interp(grid,ts,velocity))
    q=np.array(q).T
    band=sosfiltfilt(butter(3,[3,15],btype='bandpass',fs=100,output='sos'),q,axis=0)
    ripple=np.degrees(np.sqrt(np.mean(band**2,axis=0)))
    firmware_velocity=np.array(firmware_velocity).T
    speed_band=sosfiltfilt(butter(3,[3,15],btype='bandpass',fs=100,output='sos'),firmware_velocity,axis=0)
    speed_ripple=np.sqrt(np.mean(speed_band**2,axis=0))
    velocity=savgol_filter(q,11,3,deriv=1,delta=.01,axis=0)
    acceleration=savgol_filter(q,15,3,deriv=2,delta=.01,axis=0)
    commands=[r for r in trace if r['phase']=='PLAYBACK']
    ct=np.array([float(r['monotonic_s']) for r in commands])
    cq=np.array([[float(r[f'target{i}']) for i in range(1,8)] for r in commands])
    reference=np.column_stack([np.interp(grid,ct,cq[:,j]) for j in range(7)])
    errors=np.degrees(q-reference)
    late=np.array([float(r['lateness_s']) for r in commands])
    result=dict(path=str(directory),method=summary['method'],success=summary['success'],
        duration_s=duration,measured_position_ripple_3_15hz_rms_deg=ripple.tolist(),
        measured_position_ripple_3_15hz_vector_rms_deg=float(np.linalg.norm(ripple)),
        firmware_velocity_ripple_3_15hz_rms_rad_s=speed_ripple.tolist(),
        firmware_velocity_ripple_vector_rms_rad_s=float(np.linalg.norm(speed_ripple)),
        measured_acceleration_rms_rad_s2=np.sqrt(np.mean(acceleration**2,axis=0)).tolist(),
        measured_velocity_peak_rad_s=np.max(np.abs(velocity),axis=0).tolist(),
        tracking_rms_deg=np.sqrt(np.mean(errors**2,axis=0)).tolist(),
        feedback_max_gap_s=gaps,physical_feedback_samples=counts,
        max_scheduler_lateness_s=float(late.max()),p95_scheduler_lateness_s=float(np.percentile(late,95)),
        endpoint_error_deg=None if summary['endpoint'] is None else summary['endpoint']['max_error_deg'],
        center_error_deg=None if summary['center'] is None else summary['center']['max_error_deg'],
        relaxed_verified=summary['relaxed_verified'],
        note='3-15 Hz bandpass of actual encoder position, 100 Hz resampling, 0.25 s trim each end; motor-side motion proxy, not an accelerometer. Same-duration matched comparator isolates spatial smoothing from slowdown.')
    (directory/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('sessions',nargs='+');a=p.parse_args()
    results=[analyze(s) for s in a.sessions];print(json.dumps(results,indent=2))

if __name__=='__main__':main()
