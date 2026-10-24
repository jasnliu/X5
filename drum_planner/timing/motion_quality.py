"""Scale-independent stop/go diagnostics; recording access is study-only."""
import numpy as np
from scipy.signal import find_peaks,savgol_filter
from . import core


def measure(t,q,v):
    t=np.asarray(t);q=np.asarray(q);speed=np.linalg.norm(v,axis=1)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(q,axis=0),axis=1))]
    if arc[-1]<1e-9:raise ValueError('No motion')
    progress=arc/arc[-1];interior=(progress>.05)&(progress<.95)
    cruise=float(np.percentile(speed[interior],90));normalized=speed/max(cruise,1e-12)
    # Prominence prevents tiny encoder variations being counted as restarts.
    valleys,_=find_peaks(-normalized,prominence=.15)
    # Count actual mid-route stops even near the goal, while excluding the
    # deliberate endpoint acceleration/deceleration from speed-variation stats.
    valleys=valleys[(progress[valleys]>.005)&(progress[valleys]<.995)]
    stops=valleys[normalized[valleys]<.05];slow=valleys[normalized[valleys]<.25]
    inside=normalized[interior]
    return dict(duration_s=float(t[-1]-t[0]),internal_stops=int(len(stops)),deep_slowdowns=int(len(slow)),
        stop_times_s=t[stops].tolist(),slowdown_times_s=t[slow].tolist(),
        interior_speed_min_ratio=float(inside.min()),interior_speed_p10_ratio=float(np.percentile(inside,10)),
        interior_speed_cv=float(np.std(inside)/np.mean(inside)),
        peak_joint_speed_rad_s=float(np.max(abs(v))),cruise_joint_norm_rad_s=cruise,
        path_length_rad=float(arc[-1]),endpoint_exclusion='speed statistics: 5%; stop counting: 0.5% joint arc',
        definition='joint-speed norm valleys; stops <5%, deep slowdowns <25% of interior 90th percentile; prominence >=15%')


def law_metrics(law):
    t=np.unique(np.r_[np.linspace(0,law['duration_s'],5001),core.event_times(law)])
    q,v,_,_=core.evaluate(law,t);return measure(t,q,v)


def recording_metrics(obj,window_s=.2):
    samples=obj['samples'];t=np.array([s['time_s'] for s in samples]);q=np.array([s['positions_rad'] for s in samples])
    grid=np.linspace(t[0],t[-1],max(101,round((t[-1]-t[0])/.005)+1));dt=grid[1]-grid[0]
    q=np.column_stack([np.interp(grid,t,q[:,j]) for j in range(7)])
    n=max(5,int(round(window_s/dt))|1);n=min(n,(len(grid)-1)|1)
    filtered=savgol_filter(q,n,3,axis=0);v=savgol_filter(q,n,3,deriv=1,delta=dt,axis=0)
    return dict(measure(grid,filtered,v),encoder_filter_window_s=window_s,
        reference_only=True,not_a_safety_certificate=True)
