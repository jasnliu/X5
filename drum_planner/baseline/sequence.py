"""Exact stopped-leg mission law: checkpoints, fixed holds, planned return.

Hardware normal playback advances through this complete mission. Emergency
recovery reverses only its traveled prefix using the existing recovery code.
"""
import numpy as np
from ..experiment.checkpoints import SCHEMA,RETURN_POLICY,HOLD_SECONDS,MAX_CHECKPOINTS,ExperimentPlanDisplay


def initialize(curve,plan):
    from .curve import Curve
    sequence=plan['checkpoint_sequence'];segments=sequence['segments']
    if sequence.get('schema')!=SCHEMA or sequence.get('return_policy')!=RETURN_POLICY or not 2<=len(segments)<=MAX_CHECKPOINTS:
        raise ValueError('Invalid hardware checkpoint sequence')
    start=plan['outward'][0]['positions_rad'];previous=start;offset=0.;parts=[]
    for i,segment in enumerate(segments+[sequence['return_leg']]):
        returning=i==len(segments)
        samples=[dict(time_s=0.,positions_rad=segment['start'])]
        if segment['timing']['duration_s']:
            samples.append(dict(time_s=segment['timing']['duration_s'],positions_rad=segment['goal']))
        elif segment['start']!=segment['goal']:raise ValueError('Stationary checkpoint changes position')
        child=Curve(dict(timing=segment['timing'],outward=samples))
        if (not np.array_equal(previous,segment['start']) or
            abs(segment['start_time_s']-offset)>1e-9 or
            abs(segment['arrival_time_s']-offset-child.duration)>1e-9 or
            (not returning and segment['hold_s']!=HOLD_SECONDS)):
            raise ValueError('Discontinuous checkpoint mission')
        _,v,a,_=child.retimed([0.,child.duration])
        if max(np.max(abs(v)),np.max(abs(a)))>1e-7:raise ValueError('Checkpoint joins must be at rest')
        parts.append(dict(start=offset,arrival=offset+child.duration,curve=child,
                          name='original start' if returning else segment['name'],returning=returning))
        previous=segment['goal'];offset+=child.duration+(0. if returning else HOLD_SECONDS)
    if (not np.array_equal(previous,start) or
        abs(parts[-2]['arrival']-plan['duration_outward_s'])>1e-9 or
        abs(offset-plan['cycle'][-1]['time_s'])>1e-9):
        raise ValueError('Mission must finish at its original start')
    curve.parts=parts;curve.duration=offset
    curve._peaks=np.max([p['curve']._peaks for p in parts],axis=0)
    curve.retimed=Evaluator(parts,offset)
    # Validate the embedded full-cycle samples, not only endpoint metadata.
    times=np.array([p['time_s'] for p in plan['cycle']],float)
    q=np.array([p['positions_rad'] for p in plan['cycle']],float)
    if (times[0]!=0 or np.any(np.diff(times)<=0) or not np.isfinite(times).all() or
        q.shape!=(len(times),7) or not np.allclose(curve.at_many(times),q,atol=1e-9,rtol=0)):
        raise ValueError('Checkpoint law differs from checked mission samples')


class Evaluator:
    def __init__(self,parts,duration):self.parts=parts;self.duration=duration
    def __call__(self,times):
        values=np.asarray(times,float);flat=values.reshape(-1)
        arrays=[np.zeros((len(flat),7)) for _ in range(4)]
        for i,p in enumerate(self.parts):
            upper=self.parts[i+1]['start'] if i+1<len(self.parts) else self.duration
            mask=(flat>=p['start']) & (flat<=upper if i+1==len(self.parts) else flat<upper)
            if not np.any(mask):continue
            local=np.clip(flat[mask]-p['start'],0,p['curve'].duration)
            evaluated=p['curve'].retimed(local)
            for k in range(4):arrays[k][mask]=evaluated[k]
            held=mask & (flat>p['arrival'])
            for k in (1,2,3):arrays[k][held]=0.
        return tuple(a.reshape(values.shape+(7,)) for a in arrays)


class PlanDisplay(ExperimentPlanDisplay):
    """Hardware preview speed scales movement, not the two-second holds."""
    def __init__(self,path):
        super().__init__(path);self.source_duration=self.duration;self.display_parts=[]
        if self.segments:self.set_preview_rate(1.)

    def set_preview_rate(self,rate):
        offset=0.;self.display_parts=[]
        for i,(segment,ev) in enumerate(self.segments+[self.return_leg]):
            arrival=offset+ev.duration/rate
            self.display_parts.append((offset,arrival,segment['start_time_s'],segment['arrival_time_s']))
            offset=arrival+(HOLD_SECONDS if i<len(self.segments) else 0.)
        self.preview_rate=rate;self.duration=offset

    def source_time(self,t):
        if t<=0:return 0.
        if t>=self.duration:return self.source_duration
        for i,(start,arrival,source_start,source_arrival) in enumerate(self.display_parts):
            if t<=arrival:return source_start+(t-start)*self.preview_rate
            if i<len(self.segments) and t<arrival+HOLD_SECONDS:return source_arrival+t-arrival
        return self.source_duration

    def positions(self,t):
        if not self.segments:return super().positions(t)
        if t<=0 or t>=self.duration:return np.array(self.req['start'])
        return super().positions(self.source_time(t))

    def phase(self,t):
        if not self.segments:return super().phase(t)
        if t>=self.duration:return 'RETURN COMPLETE'
        return super().phase(self.source_time(t))
