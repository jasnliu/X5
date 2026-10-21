"""Shared endpoint-only proposal factory for the surviving gesture families.

Memory-bounded: one oracle, candidate payloads on disk, short metadata in RAM.
Search source and fallback provenance remain visible in every selection report.
"""
import json,time
from pathlib import Path
import numpy as np
from ..common import atomic_json,digest
from .. import style
from . import gesture

# Initially screened together. The public registry may retain only a subset.
FAMILIES=('compact','progress','sweep','single_arc','clearance')
DEFAULTS=dict(backtrack_weight=4.,overshoot_weight=2.,clearance_weight=3.,
              corridor_deg=7.,candidates=6,seed=19)


def rank(rows,method,params=None):
    if method not in FAMILIES:raise ValueError('Unknown gesture method')
    if not rows:raise ValueError('Cannot rank an empty pool')
    sources={'compact':('corridor','fallback'),'progress':('corridor','fallback','progress'),
             'sweep':('corridor','fallback','sweep'),'single_arc':('corridor','fallback','single_arc'),
             'clearance':('corridor','fallback','progress','sweep','single_arc')}
    eligible=[row for row in rows if row['source'] in sources[method]]
    mode='economy' if method in ('compact','progress') else 'comfort' if method=='clearance' else 'arc'
    ranked=[]
    for row in eligible:
        value=gesture.objective(row['gesture'],mode,row['metrics']['baseline']['components']['obstacle_proximity'],params)
        ranked.append(dict(row,score=float(value)))
    # Match the production scorer's numerical precision convention. A
    # resampled copy of the SAME geometry can differ by ~1e-9 in its score;
    # that must not displace the earlier route merely due to floating error.
    # Timing still does not enter the preference or the tie-break.
    ranked.sort(key=lambda r:(round(r['score'],8),r['index']))
    return ranked,dict(objective=mode,parameters=dict(DEFAULTS,**(params or {})),
                       duration_ratio_filter=None,lower_is_better=True,
                       uses_recording_similarity=False,uses_elapsed_time=False,
                       score_round_digits=8,tie_break='earlier candidate index')


def build(oracle,req,folder,params,seconds,log,enabled=FAMILIES):
    from .algorithms import timed_candidate,thin
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    start=time.monotonic();deadline=start+seconds
    rows=[];attempts=[];counter=0;paths={};rng=np.random.default_rng(params['seed'])
    endpoints=oracle.check([req['start'],req['goal']])
    if not all(endpoints['valid']):
        return dict(candidates=[],attempts=[],reason='nominal_endpoint_invalid',details=endpoints,elapsed_s=time.monotonic()-start)

    def add(q,source,origin=None):
        nonlocal counter
        original=np.asarray(q,dtype=float);q=thin(original)
        variants=[(q,False),(q,True)]
        if not np.array_equal(q,original):variants.append((original,True))
        for attempt,(path,stop) in enumerate(variants):
            counter+=1
            # Retain the existing diagnostic fluidity features, but do not
            # confuse their old preference formulas with the new objectives.
            metric_params=dict(reversal_threshold_deg=2.,joint_spacing_rad=.05,spatial_spacing_m=.05)
            c=timed_candidate(oracle,path,req,counter,log,metric_params,stop)
            attempts.append(dict(index=counter,source=source,origin=origin,accepted=c is not None,stop_at_corners=stop))
            if c is None:continue
            samples=style.sample_path(np.array([x['positions_rad'] for x in c['outward']]))
            c.update(source=source,origin=origin,gesture=gesture.geometry(samples,gesture.fk(oracle,samples)))
            name=f'candidate_{counter:03d}.json';atomic_json(folder/name,c)
            rows.append(dict(index=counter,file=name,sha256=digest(folder/name),metrics=c['metrics'],
                             gesture=c['gesture'],duration_s=c['duration_s'],source=source,origin=origin,
                             stop_at_corners=stop))
            paths[counter]=path
            # Smooth continuous motion preferred for the SAME geometric
            # proposal; a corner-stop is a fallback, not a way to game timing.
            return rows[-1]
        return None

    if all(oracle.check([req['start'],req['goal']],edges=True)['valid']):
        add(np.array([req['start'],req['goal']]),'corridor','direct')
    width=np.radians(params.get('corridor_deg',7.))
    widths=np.array([.65,1.,1.65,1.,2.05,1.65])*width
    for trial in range(params['candidates']):
        if time.monotonic()>=deadline:break
        allocation=min(4.,max(.1,(deadline-time.monotonic())/(params['candidates']-trial+2)))
        w=float(widths[trial%len(widths)])
        log(f'CORRIDOR {trial+1}/{params["candidates"]}: endpoint joint range + {np.degrees(w):.1f}deg; {allocation:.1f}s search')
        out=oracle.call('plan',start=req['start'],goal=req['goal'],seconds=allocation,corridor_rad=w)
        attempts.append(dict(search=trial+1,source='corridor',corridor_rad=w,status=out['status']))
        if out['status']=='planned':add(out['path'],'corridor',trial+1)
    # Never invent an invalid result for an over-constrained corridor. Broad
    # search remains an explicitly labelled fallback using unchanged checks.
    if not rows:
        for trial in range(2):
            if time.monotonic()>=deadline:break
            log('No validated compact route yet; trying unrestricted fallback')
            out=oracle.call('plan',start=req['start'],goal=req['goal'],seconds=min(5.,max(.1,deadline-time.monotonic())))
            attempts.append(dict(source='fallback',status=out['status']))
            if out['status']=='planned':add(out['path'],'fallback',trial+1)
    if rows:
        candidates,_=rank(rows,'compact',params)
        # Starting routes are generated on THIS task from endpoints, never
        # read from baseline playback or demonstration trajectories.
        seeds=[paths[r['index']] for r in candidates[:2]]
        if 'single_arc' in enabled and time.monotonic()<deadline:
            qs=gesture.parametric_search(oracle,req,'arch',rng,min(deadline,time.monotonic()+15),log,
                                         rounds=10,population=60,seed_paths=seeds,params=params)
            for q in qs:
                if time.monotonic()>=deadline:break
                add(q,'single_arc')
        for family in ('progress','sweep'):
            if family not in enabled:continue
            for i,q in enumerate(seeds):
                if time.monotonic()>=deadline:break
                op=gesture.shortcut if family=='progress' else gesture.elastic
                refined=op(oracle,q,rng,min(deadline,time.monotonic()+12),log,params=params)
                add(refined,family,i+1)
    return dict(candidates=rows,attempts=attempts,elapsed_s=time.monotonic()-start,
                **({} if rows else {'reason':'no_valid_route_within_budget'}))
