"""Offline research harness. Demonstrations are read ONLY after generation.

This module is not imported by the interactive planner. A fixed acquisition
request constructs the scene/self-filter once; scenario start/goal overrides
never move that mask or change the observed obstacles.
"""
import argparse,copy,json,time
from pathlib import Path
import numpy as np
from ..common import ROOT,atomic_json,digest
from .. import style,planning
from .config import no_can,settings
from .runtime import Oracle
from .algorithms import timed_candidate,thin
from . import gesture

STUDY=ROOT/'diagnostics/gesture_study_20261020'
FAMILIES=('baseline_control','economy','partial','corridor','phase','arch','elastic','comfort')


def save_candidate(oracle,q,req,folder,index,source,log,params=None,stop=False):
    row=timed_candidate(oracle,q,req,index,log,dict(reversal_threshold_deg=2.,joint_spacing_rad=.05,spatial_spacing_m=.05,**(params or {})),stop)
    if row is None:return None
    samples=style.sample_path(np.array([x['positions_rad'] for x in row['outward']]))
    f=gesture.fk(oracle,samples)
    row.update(source=source,gesture=gesture.geometry(samples,f))
    atomic_json(folder/f'candidate_{index:03d}.json',row)
    return dict(index=index,file=f'candidate_{index:03d}.json',source=source,
                duration_s=row['duration_s'],metrics=row['metrics'],gesture=row['gesture'])


def select(rows,method):
    seed=[r for r in rows if r['source']=='rrt']
    own={'economy':('rrt',),'partial':('rrt','partial'),'corridor':('rrt','corridor'),
         'phase':('rrt','phase'),'arch':('rrt','arch'),'elastic':('rrt','elastic'),
         'comfort':('rrt','partial','corridor','phase','arch','elastic')}
    if method=='baseline_control':
        if not seed:return None
        fastest=min(seed,key=lambda r:r['duration_s'])
        scales=style.normalizers(fastest['metrics']['baseline'])
        eligible=[r for r in seed if r['duration_s']<=2*fastest['duration_s']+1e-9]
        return min(eligible,key=lambda r:(style.score(r['metrics']['baseline'],scales)['total'],r['duration_s']))
    eligible=[r for r in rows if r['source'] in own[method]]
    if not eligible:return None
    mode='arc' if method in ('arch','elastic') else 'comfort' if method=='comfort' else 'economy'
    return min(eligible,key=lambda r:(gesture.objective(r['gesture'],mode,
                r['metrics']['baseline']['components']['obstacle_proximity']),r['index']))


def run_case(root,case,seed,seconds=240.,families=FAMILIES):
    folder=Path(root)/f'{case["id"]}-seed{seed}'
    folder.mkdir(parents=True,exist_ok=True)
    final=folder/'result.json'
    if final.exists():return json.loads(final.read_text())
    inputs=json.loads((STUDY/'inputs.json').read_text())
    original=json.loads((Path(inputs['baseline']).parent/'backend_request.json').read_text())
    original['seed']=int(seed)
    req=copy.deepcopy(original);req['start']=case['start'];req['goal']=case['goal']
    atomic_json(folder/'acquisition_request.json',original)
    atomic_json(folder/'scenario.json',dict(case=case,seed=seed,request=req))
    start=time.monotonic();deadline=start+seconds
    def log(text):print(f'[{case["id"]}/{seed} +{time.monotonic()-start:.1f}s] {text}',flush=True)
    oracle=Oracle(folder/'acquisition_request.json',folder)
    rows=[];raw=[];counter=0;attempts=[]
    rng=np.random.default_rng(seed)
    def add(path,source):
        nonlocal counter
        q=thin(path)
        for stop in (False,True):
            counter+=1
            c=save_candidate(oracle,q,req,folder,counter,source,log,stop=stop)
            attempts.append(dict(index=counter,source=source,accepted=c is not None,stop_at_corners=stop))
            if c:
                rows.append(c)
                # Corner-stop changes geometry, and can be much slower.
                # Only use it as a feasible fallback in the new families.
                if source!='rrt':break
    try:
        endpoints=oracle.check([req['start'],req['goal']])
        if not all(endpoints['valid']):raise ValueError('Invalid study endpoint: '+str(endpoints))
        direct=oracle.check([req['start'],req['goal']],edges=True)
        if all(direct['valid']):add(np.array([req['start'],req['goal']]),'rrt')
        log('Independent endpoint-only RRT control searches')
        for i in range(6):
            out=oracle.call('plan',start=req['start'],goal=req['goal'],seconds=4.)
            log(f'RRT {i+1}/6: {out["status"]}')
            if out['status']=='planned':
                q=np.asarray(out['path']);raw.append(q);add(q,'rrt')
        # Cheap promising sources are explored before local refinements.
        if 'corridor' in families:
            for width in (.04,.12,.25):
                if time.monotonic()>=deadline:break
                out=oracle.call('plan',start=req['start'],goal=req['goal'],seconds=4.,corridor_rad=width)
                log(f'endpoint corridor ±{np.degrees(width):.1f}deg: {out["status"]}')
                if out['status']=='planned':add(np.asarray(out['path']),'corridor')
        for family in ('phase','arch'):
            if family not in families or time.monotonic()>=deadline:continue
            paths=gesture.parametric_search(oracle,req,family,rng,min(deadline,time.monotonic()+35),log)
            for q in paths:add(q,family)
        # Seed with the two best independently generated feasible raw routes,
        # NOT the saved baseline or record3 interiors.
        ranked=sorted(raw,key=lambda q:gesture.quick_score(oracle,q))
        for family in ('partial','elastic'):
            if family not in families:continue
            for path in ranked[:2]:
                if time.monotonic()>=deadline:break
                operation=gesture.shortcut if family=='partial' else gesture.elastic
                q=operation(oracle,path,rng,min(deadline,time.monotonic()+30),log)
                add(q,family)
        selected={method:select(rows,method) for method in families}
        result=dict(case=case,seed=seed,rows=rows,attempts=attempts,selected=selected,
                    elapsed_s=time.monotonic()-start,budget_s=seconds,
                    no_can=True,source_hashes={p.name:digest(p) for p in (
                        Path(__file__),Path(gesture.__file__),ROOT/'drum_planner/cpp/experiment_oracle.cpp')},
                    reference_not_read_during_generation=True,
                    fixed_acquisition_self_filter=True)
        atomic_json(final,result)
        log('DONE: '+', '.join(f'{m}={v["index"] if v else "none"}' for m,v in selected.items()))
        return result
    finally:oracle.close()


def cases():
    folder=STUDY/'cases';folder.mkdir(exist_ok=True)
    inputs=json.loads((STUDY/'inputs.json').read_text())
    req=json.loads((Path(inputs['baseline']).parent/'backend_request.json').read_text())
    atomic_json(folder/'acquisition_request.json',req);o=Oracle(folder/'acquisition_request.json',folder)
    rng=np.random.default_rng(83271);start=np.array(req['start']);goal=np.array(req['goal'])
    out=[dict(id='main',split='development',start=start.tolist(),goal=goal.tolist(),start_max_deg=0.,goal_max_deg=0.)]
    try:
        # Fixed manifest written before seeing any generated trajectories.
        for kind,count in [('start',5),('goal',5)]:
            accepted=0
            for attempt in range(100):
                if accepted==count:break
                ds=rng.uniform(-1.5,1.5,7) if kind=='start' else np.zeros(7)
                dg=rng.uniform(-3.,3.,7) if kind=='goal' else np.zeros(7)
                a=start+np.radians(ds);b=goal+np.radians(dg)
                if not all(o.check(np.array([a,b]))['valid']):continue
                accepted+=1
                f=gesture.fk(o,np.array([start,a,goal,b]))
                tip=np.array(f['tip_xyz_m'])
                out.append(dict(id=f'{kind}{accepted}',split='development' if accepted<=2 else 'heldout',
                    start=a.tolist(),goal=b.tolist(),start_delta_deg=ds.tolist(),goal_delta_deg=dg.tolist(),
                    start_max_deg=float(abs(ds).max()),goal_max_deg=float(abs(dg).max()),
                    start_tip_delta_m=float(np.linalg.norm(tip[1]-tip[0])),
                    goal_tip_delta_m=float(np.linalg.norm(tip[3]-tip[2]))))
            if accepted<count:raise RuntimeError('Insufficient valid small perturbations')
        atomic_json(STUDY/'cases.json',out);return out
    finally:o.close()


def refined_case(root,case,seed,seconds=120.):
    from . import search
    folder=Path(root)/f'{case["id"]}-seed{seed}';folder.mkdir(parents=True,exist_ok=True)
    final=folder/'result.json'
    if final.exists():return json.loads(final.read_text())
    inputs=json.loads((STUDY/'inputs.json').read_text())
    original=json.loads((Path(inputs['baseline']).parent/'backend_request.json').read_text())
    original['seed']=int(seed)
    req=copy.deepcopy(original);req['start']=case['start'];req['goal']=case['goal']
    atomic_json(folder/'acquisition_request.json',original)
    atomic_json(folder/'scenario.json',dict(case=case,seed=seed,request=req))
    oracle=Oracle(folder/'acquisition_request.json',folder);started=time.monotonic()
    params=dict(search.DEFAULTS,seed=seed)
    def log(text):print(f'[{case["id"]}/{seed} +{time.monotonic()-started:.1f}s] {text}',flush=True)
    try:
        result=search.build(oracle,req,folder,params,seconds,log)
        selected={}
        for method in search.FAMILIES:
            ranked,_=search.rank(result['candidates'],method,params) if result['candidates'] else ([],{})
            selected[method]=ranked[0] if ranked else None
        result.update(case=case,seed=seed,rows=result.pop('candidates'),selected=selected,parameters=params,
                      no_can=True,reference_not_read_during_generation=True,fixed_acquisition_self_filter=True,
                      source_hashes={p.name:digest(p) for p in (Path(search.__file__),Path(gesture.__file__),
                          ROOT/'drum_planner/cpp/experiment_oracle.cpp')})
        atomic_json(final,result)
        log('DONE: '+', '.join(f'{m}={v["index"] if v else "none"}' for m,v in selected.items()))
        return result
    finally:oracle.close()


def main():
    no_can();p=argparse.ArgumentParser()
    p.add_argument('action',choices=('cases','case'));p.add_argument('--case',default='main')
    p.add_argument('--seed',type=int,default=19);p.add_argument('--seconds',type=float,default=240.)
    p.add_argument('--root',default=str(STUDY/'development'))
    p.add_argument('--stage',type=int,choices=(1,2),default=1)
    p.add_argument('--families',nargs='+',default=list(FAMILIES));a=p.parse_args()
    if a.action=='cases':cases()
    else:
        case=next(c for c in json.loads((STUDY/'cases.json').read_text()) if c['id']==a.case)
        if a.stage==2:refined_case(a.root,case,a.seed,a.seconds)
        else:run_case(a.root,case,a.seed,a.seconds,a.families)


if __name__=='__main__':main()
