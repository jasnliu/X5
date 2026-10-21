"""Finite, balanced exploration. Parameter updates happen BETWEEN complete strokes."""
from dataclasses import dataclass
import random
from .methods import METHODS
from .scoring import identity

# Comparable search budget per mode, not an exhaustive Cartesian product.
TUNING_KEYS = {
    'baseline': ('fall_kd','brake_accel','return_accel','command_latency'),
    'gravity': ('fall_kd','brake','return_time','latency'),
    'variable_damping': ('end_kd','ramp_start','brake','return_time'),
    'powered': ('down_time','up_time','curvature','inertia'),
    'cosine': ('duration','kp','kd','inertia'),
    'torque': ('down_time','up_time','curvature','inertia'),
    'hybrid': ('kick_torque','kick_time','brake','return_time'),
    'impedance': ('kp','return_kp','down_time','up_time'),
    'optimized': ('velocity_shape','knot_fraction','curvature','up_time'),
}


@dataclass(frozen=True)
class Plan:
    screen_repeats: int = 3
    refine_repeats: int = 20
    validate_repeats: int = 100
    refine_rounds: int = 2
    endurance_seconds: float = 300.
    seed: int = 20261001

    def __post_init__(self):
        if any(type(v) is not int for v in (self.screen_repeats,self.refine_repeats,self.validate_repeats,self.refine_rounds,self.seed)):
            raise ValueError('Campaign counts and seed must be integers')
        if min(self.screen_repeats,self.refine_repeats,self.validate_repeats)<1:
            raise ValueError('Campaign repetition counts must be positive')
        if not 0<=self.refine_rounds<=10 or not 0<=self.endurance_seconds<=3600:
            raise ValueError('Invalid campaign budget')


def neighbors(method,base=None,fraction=.15):
    d=METHODS[method];base=d.parameters(base)
    choices=[base]
    for k in TUNING_KEYS[method]:
        lo,hi=d.bounds[k]
        step=max(abs(base[k])*fraction,(hi-lo)*fraction*.1)
        for sign in (-1,1):
            p=dict(base);p[k]=max(lo,min(hi,base[k]+sign*step))
            if identity(p) not in {identity(c) for c in choices}:choices.append(p)
    return choices


def choose(batch):
    """No successful-subset bias. Accuracy first, then time during calibration.

    Depth isn't a free parameter. Reject configs with any failed trial; among
    valid configs first favor closeness to the selected goal until depths match.
    The final report separately checks depth matching before naming a winner.
    """
    import statistics
    groups={}
    for t in batch:groups.setdefault(t['metadata']['parameters_id'],[]).append(t)
    valid=[g for g in groups.values() if all(t['score']['passed'] for t in g)]
    if not valid:return None
    def key(g):
        depth=statistics.median(t['score']['peak_depth_deg'] for t in g)
        target=g[0]['score'].get('target_deg',10.)
        return (round(abs(depth-target)/.05),statistics.median(t['score']['lower_zone_ms'] for t in g))
    return min(valid,key=key)[0]


def run_campaign(engine, method_ids, plan=Plan(), stage='all', pause=None, seeds=None):
    rng=random.Random(plan.seed)
    outcomes={};best={}
    def before():
        while pause and pause.is_set():engine.idle()
    def run_candidates(mid,candidates,reps,label):
        jobs=[p for p in candidates for _ in range(reps)];rng.shuffle(jobs)
        results=[]
        for p in jobs:
            before();results.append(engine.trial(mid,p,label))
            # Equal explicit rest between screening strokes. Not scored as cycle.
            until=engine.backend.now()+.25
            while engine.backend.now()<until:engine.idle()
        return results
    order=list(method_ids);rng.shuffle(order)
    for mid in order:
        outcomes[mid]=run_candidates(mid,neighbors(mid,(seeds or {}).get(mid)),plan.screen_repeats,'screen')
        selected=choose(outcomes[mid])
        if selected:best[mid]=selected
    if stage=='screen':return outcomes
    for round_index in range(plan.refine_rounds):
        order=list(best);rng.shuffle(order)
        for mid in order:
            selected=best[mid]
            batch=run_candidates(mid,neighbors(mid,selected['metadata']['parameters'],.08/(round_index+1)),
                                 plan.refine_repeats,f'refine_{round_index+1}')
            outcomes[mid]+=batch
            improved=choose(batch)
            if improved:best[mid]=improved
    if stage=='refine':return outcomes
    validated={}
    # Fixed parameters throughout validation, separate from training trials.
    for mid,selected in best.items():
        batch=run_candidates(mid,[selected['metadata']['parameters']],plan.validate_repeats,'validate')
        outcomes[mid]+=batch
        if all(t['score']['passed'] for t in batch):validated[mid]=selected
    if stage=='validate':return outcomes
    # All validated families, not only a lucky global winner, get the same
    # mixed intervals. Periods are start-to-start; late readiness is recorded.
    intervals=(2.,.8,.5,.5,1.5,.65,.5,2.5)
    for mid,selected in validated.items():
        start=engine.backend.now();next_start=start;i=0
        while engine.backend.now()-start<plan.endurance_seconds:
            before()
            while engine.backend.now()<next_start:
                engine.idle(min(.01,max(.0005,next_start-engine.backend.now())))
            actual=engine.backend.now()
            lateness=max(0.,actual-next_start)
            result=engine.trial(mid,selected['metadata']['parameters'],'endurance',selected['id'],
                                dict(requested_start=next_start,actual_start=actual,lateness_ms=lateness*1000))
            engine.publish(dict(phase='ENDURANCE TIMING',method=mid,lateness_ms=lateness*1000))
            outcomes[mid].append(result)
            # Schedule from this actual release, so lateness is measured next
            # time without compressing later intervals into catch-up bursts.
            next_start=result['score'].get('release_at',actual)+intervals[i%len(intervals)];i+=1
    return outcomes
