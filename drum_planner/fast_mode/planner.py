"""Task-equivalent postures + minimum-joint-motion curves, isolated from experiment.

All plans are non-executable. Same original motion limits, tools, zones and native
collision checks. An optimized first playing posture is reached by a separately
checked approach; it is never substituted for the measured initial state.
"""
import copy
import json
import time
from pathlib import Path
import numpy as np
from scipy.interpolate import PPoly
from ..common import atomic_json,digest,read_state
from .. import planning
from ..timing import core
from ..timing.benchmark import validate
from ..experiment import inputs,destinations,checkpoints,fast_path,fast_timing
from ..experiment.config import settings
from ..experiment.runtime import Oracle
from ..experiment.worker import base_result
from .kinematics import Kinematics,target,task_proof
from .postures import solve

VERSION='task-wrist-path-v1'
WRIST_PREFERENCE_TIME_ALLOWANCE=1.05


def order_finalists(feasible):
    """Prefer less proximal travel among paths within 5% of the fastest checked.

    A bounded, reported Pareto compromise: never choose a substantially slower
    route merely to improve a joint-travel score. Remaining rows are fallbacks
    only if the preferred playing postures cannot be approached/returned from.
    """
    fastest=min(r[0]['transfer_movement_s'] for r in feasible)
    near=[r for r in feasible if r[0]['transfer_movement_s']<=fastest*WRIST_PREFERENCE_TIME_ALLOWANCE]
    other=[r for r in feasible if r[0]['transfer_movement_s']>fastest*WRIST_PREFERENCE_TIME_ALLOWANCE]
    return (sorted(near,key=lambda r:(r[0]['actual_proximal_travel_rad'],r[0]['transfer_movement_s']))+
            sorted(other,key=lambda r:(r[0]['transfer_movement_s'],r[0]['actual_proximal_travel_rad'])))


def timed(source,log):
    # Process-local budget only. Identical algorithm/budget for reference and new
    # paths; no edits to experiment modules or their normal runtime settings.
    previous=(fast_timing.MAX_ITERATIONS,fast_timing.SOLVE_WALL_SECONDS)
    try:
        fast_timing.MAX_ITERATIONS=600;fast_timing.SOLVE_WALL_SECONDS=8.
        return fast_timing.optimize(source,log)
    finally:fast_timing.MAX_ITERATIONS,fast_timing.SOLVE_WALL_SECONDS=previous


def travel(source):
    path=core.original(source);values=[]
    for j in range(7):
        p=PPoly(path.c[:,:,j],path.x);roots=p.derivative().roots(extrapolate=False)
        cuts=np.unique(np.r_[path.x,roots[np.isfinite(roots)&(roots>=0)&(roots<=1)]])
        values.append(float(abs(np.diff(p(cuts))).sum()))
    return dict(joint_travel_rad=values,proximal_travel_rad=sum(values[:4]),distal_travel_rad=sum(values[4:]),
                net_change_rad=abs(path(1)-path(0)).tolist())


def source_for(q,req):
    _,_,timing=planning.smooth_samples(q,False)
    return dict(path_waypoints=np.asarray(q).tolist(),timing=timing,request=copy.deepcopy(req))


def continuous_joint_bounds(source,lower,upper):
    path=core.original(source)
    for j in range(7):
        p=PPoly(path.c[:,:,j],path.x);roots=p.derivative().roots(extrapolate=False)
        grid=np.r_[path.x,roots[np.isfinite(roots)&(roots>=0)&(roots<=1)]];values=p(grid)
        if values.min()<lower[j]-1e-10 or values.max()>upper[j]+1e-10:
            raise ValueError(f'Continuous path exceeds joint {j+1} position limits')


def route(oracle,req,folder,log,*,reference=False,seconds=12.):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True);began=time.monotonic();attempts=[]
    a,b=np.array(req['start']),np.array(req['goal'])
    if np.max(abs(a-b))<1e-7:raise ValueError('Coincident task endpoints: choose distinct destinations')
    endpoints=oracle.check([a.tolist(),b.tolist()])
    if not all(endpoints['valid']) or len(endpoints['valid'])!=2:
        raise ValueError('Endpoint collision/zone/joint rejection: '+str(endpoints))
    candidates=[]
    if not reference:
        source=source_for([a,b],req)
        screen=oracle.check(core.original(source)(np.linspace(0,1,65)),edges=True,stop=True)
        if len(screen['valid'])==65 and all(screen['valid']):candidates.append(('minimum_joint_curve',source))
        else:attempts.append(dict(generator='minimum_joint_curve',accepted=False,reason='coarse_collision_or_zone',details=screen))
    def evaluate(candidates):
        for generator,source in candidates:
            try:
                continuous_joint_bounds(source,oracle.lower,oracle.upper)
                screen=oracle.check(core.original(source)(np.linspace(0,1,501)),edges=True,stop=True)
                if len(screen['valid'])!=501 or not all(screen['valid']):
                    raise ValueError('Curve collision/zone screen: '+str(screen.get('details')))
            except ValueError as exc:
                attempts.append(dict(generator=generator,accepted=False,reason=str(exc)));continue
            try:
                law=timed(source,log)
                proof=validate(law,source,folder/'final_check',oracle=oracle)
            except ValueError as exc:
                attempts.append(dict(generator=generator,accepted=False,reason=str(exc)));continue
            # The original root checker independently verifies the fast extrema.
            peaks,_=core.density_peaks(core.original(source),core.decode(law['density']))
            if np.any(peaks>core.LIMITS[:,None]*(1+1e-9)):raise ValueError('Independent continuous check failed')
            result=dict(generator=generator,source=source,law=law,proof=proof,travel=travel(source),
                        duration_s=law['duration_s'],calculation_s=time.monotonic()-began,attempts=attempts)
            atomic_json(folder/'route.json',result);return result
        return None
    result=evaluate(candidates)
    if result is not None:return result
    log('Trying unchanged Workspace curve as a collision-avoidance alternative' if not reference else 'Captured-posture Workspace curve reference')
    report={}
    from ..experiment.fast import proposal_stages
    for candidate in proposal_stages(oracle,req,settings(),seconds,log,report):
        source=dict(path_waypoints=candidate['path'],timing=candidate['timing'],request=copy.deepcopy(req))
        result=evaluate([('workspace_reference' if reference else 'workspace_clearance_alternative',source)])
        if result is not None:
            result['workspace_report']=report;atomic_json(folder/'route.json',result);return result
    atomic_json(folder/'workspace_report.json',report)
    if not reference:
        from .clearance import candidates as clearance_candidates
        result=evaluate(clearance_candidates(oracle,req,folder,log,seconds=min(8.,seconds)))
        if result is not None:return result
    atomic_json(folder/'rejected_attempts.json',attempts)
    raise ValueError('No checked continuous route; '+str(report.get('reason','all candidates rejected')))


def leg_result(context,base_req,manifest,original,folder,a,b,route_data,binding):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True);side=context['moving_side']
    state=checkpoints.derived_state(original,side,a,context['state_sha256'])
    atomic_json(folder/'simulated_start.json',state)
    local=dict(context,**binding,state=str((folder/'simulated_start.json').resolve()))
    local.pop('checkpoints',None);local.pop('checkpoint_sha256',None)
    local['state_sha256']=digest(local['state']);req=dict(base_req,start=list(a),goal=list(b))
    if side=='left':req['left']=list(a)
    atomic_json(folder/'backend_request.json',req)
    result=base_result(local,req,manifest,state);law=route_data['law'];source=route_data['source']
    points=core.samples(law,source);points[0]['positions_rad']=list(a);points[-1]['positions_rad']=list(b)
    clean=[points[0]]
    for p in points[1:-1]:
        if p['time_s']-clean[-1]['time_s']>1e-10 and points[-1]['time_s']-p['time_s']>1e-10:clean.append(p)
    points=clean+[points[-1]]
    result.update(status='planned',backend='Separate --fast wrist-biased task planner',outward=points,
        cycle=planning.cycle(points),timing=law,source_timing=source['timing'],path_waypoints=source['path_waypoints'],
        duration_outward_s=law['duration_s'],hold_at_goal_s=2.,reverse_checked_on_same_static_scene=True,
        final_curve_check=route_data['proof']['collision_zone'],
        sampled_interpolated_checks=route_data['proof']['collision_zone']['samples_checked'],
        velocity_limit_rad_s=.6,acceleration_limit_rad_s2=1.2,jerk_limit_rad_s3=6.,
        experiment=dict(method='fast_task_postures',description='Separate --fast mode',budget_s=12.,
            report=dict(candidates=[],selected=None),elapsed_s=route_data['calculation_s'],
            retiming=dict(label=law['details']['backend_label'],**law['details'])),
        fast_mode=dict(version=VERSION,generator=route_data['generator'],travel=route_data['travel']))
    atomic_json(folder/'plan.json',result);return result


def run(context,folder,bindings,*,kind='full-pose',compare=False,seconds=45.):
    if kind not in ('full-pose','tip-axis'):raise ValueError('Unknown task constraint')
    if not 1<=len(bindings)<=3:raise ValueError('Choose one to three destinations')
    folder=Path(folder).resolve();folder.mkdir(parents=True,exist_ok=True);began=time.monotonic()
    log=lambda s:print(f'[--fast +{time.monotonic()-began:6.2f}s] {s}',flush=True)
    side=context['moving_side'];original=read_state(context['state']);start=original[side+'_rad']
    nominal=np.array([destinations.goal(dict(b,moving_side=side))[1] for b in bindings])
    if any(np.max(abs(a-b))<1e-7 for a,b in zip(nominal,nominal[1:])):
        raise ValueError('Consecutive destinations must differ')
    req,manifest=inputs.request(context,folder,original);req['goal']=nominal[-1].tolist()
    req['scene_robot_start']=original[side+'_rad'];req['seed']=19
    if req.get('execute') is not False:raise ValueError('--fast is software-only')
    atomic_json(folder/'backend_request.json',req)
    oracle=None;result=base_result(dict(context,**bindings[-1]),req,manifest,original)
    report=dict(version=VERSION,task_constraint=kind,limits=core.LIMITS.tolist(),motion_authorized=False,
        motor_commands=0,result_cache_used=False,recording_interiors_used=False,nominal_poses=nominal.tolist(),
        original_measured_start=start,candidates=[],reference=[],scene_sha256=digest(context['scene']))
    report['provenance']=dict(state_sha256=digest(context['state']),calibration_sha256=digest(context['calibration']),
        destinations=[dict(source=b['destination_source'],sha256=b['destination_sha256']) for b in bindings],
        code_sha256={p.name:digest(p) for p in Path(__file__).parent.glob('*.py')},
        shared_code_sha256={str(p.relative_to(Path(__file__).parents[1])):digest(p) for p in [
            Path(fast_timing.__file__),Path(fast_path.__file__),Path(core.__file__),Path(planning.__file__)]})
    try:
        oracle=Oracle(folder/'backend_request.json',folder);kin=Kinematics(req['urdf'],side)
        report['kinematics_native_parity']=kin.verify_native(oracle,np.vstack((start,nominal)))
        goals=[target(kin,q) for q in nominal]
        rows,posture_report=solve(kin,nominal,start=start,kind=kind,seconds=min(10.,seconds/3))
        report['posture_search']=posture_report
        # Include the captured-posture control even if alternate IK branches
        # dominate the shortlist. A bounded maximum of five endpoint sets.
        chosen=rows[:4]
        captured=next(r for r in rows if r['source']=='captured_postures')
        if captured not in chosen:chosen.append(captured)
        feasible=[]
        for index,row in enumerate(chosen):
            poses=np.array(row['poses']);entry=dict(row,index=index,accepted=False);report['candidates'].append(entry)
            path=folder/'candidates'/str(index)
            check=oracle.check(poses)
            if len(check['valid'])!=len(poses) or not all(check['valid']):
                entry['reason']='task-equivalent posture collides or leaves zone';continue
            pairs=list(zip(poses[:-1],poses[1:])) if len(poses)>1 else [(np.array(start),poses[0])]
            routes=[]
            try:
                for i,(a,b) in enumerate(pairs):
                    local=dict(req,start=a.tolist(),goal=b.tolist())
                    if side=='left':local['left']=a.tolist()
                    log(f'Posture set {index+1}/{len(chosen)}, transfer {i+1}: {row["source"]}')
                    routes.append(route(oracle,local,path/f'transfer_{i+1}',log,seconds=min(12.,seconds)))
            except ValueError as exc:entry['reason']=str(exc);continue
            entry.update(accepted=True,transfer_movement_s=sum(r['duration_s'] for r in routes),
                         actual_proximal_travel_rad=sum(r['travel']['proximal_travel_rad'] for r in routes),
                         transfer_paths=[str((path/f'transfer_{i+1}'/'route.json').resolve()) for i in range(len(routes))])
            feasible.append((entry,poses,routes))
            atomic_json(folder/'fast_report.json',report)
        if not feasible:raise ValueError('No task-equivalent posture set has a checked route')
        report['fastest_checked_transfer_s']=min(r[0]['transfer_movement_s'] for r in feasible)
        report['selection_policy']='Least proximal travel within 5% of fastest checked transfer; checked approach/return required'
        report['wrist_preference_time_allowance']=WRIST_PREFERENCE_TIME_ALLOWANCE
        feasible=order_finalists(feasible)
        for entry,poses,routes in feasible:
            approach=return_route=None
            try:
                if len(poses)>1:
                    log('Checking measured-start approach and independent return; no pose teleportation')
                    local=dict(req,start=start,goal=poses[0].tolist())
                    if side=='left':local['left']=start
                    approach=route(oracle,local,folder/'candidates'/str(entry['index'])/'approach',log,seconds=min(15.,seconds))
                    local=dict(req,start=poses[-1].tolist(),goal=start)
                    if side=='left':local['left']=poses[-1].tolist()
                    return_route=route(oracle,local,folder/'candidates'/str(entry['index'])/'return',log,seconds=min(15.,seconds))
                break
            except ValueError as exc:
                entry['mission_rejected']=str(exc);log('Posture set rejected: approach/return unavailable; trying next')
        else:raise ValueError('Transfer postures exist, but no complete checked approach/return')
        kin.verify_native(oracle,poses)
        proofs=[task_proof(kin,q,g,kind) for q,g in zip(poses,goals)]
        if not all(p['valid'] for p in proofs):raise ValueError('Final task endpoints changed')
        report.update(selected=entry['index'],optimized_poses=poses.tolist(),task_proofs=proofs,
            transfer_movement_s=entry['transfer_movement_s'],
            approach_movement_s=0. if approach is None else approach['duration_s'],
            return_movement_s=None if return_route is None else return_route['duration_s'])
        if compare:
            pairs=list(zip(nominal[:-1],nominal[1:])) if len(nominal)>1 else [(np.array(start),nominal[0])]
            for i,(a,b) in enumerate(pairs):
                local=dict(req,start=a.tolist(),goal=b.tolist())
                if side=='left':local['left']=a.tolist()
                try:
                    ref=route(oracle,local,folder/'reference'/str(i),log,reference=True,seconds=min(20.,seconds))
                    report['reference'].append(dict(status='planned',duration_s=ref['duration_s'],travel=ref['travel'],
                        route=str((folder/'reference'/str(i)/'route.json').resolve())))
                except ValueError as exc:report['reference'].append(dict(status='no_feasible_route',reason=str(exc)))
        if len(poses)==1:
            result=leg_result(context,req,manifest,original,folder,start,poses[0],routes[0],bindings[0])
        else:
            legs=[];paths=[];current=start
            for i,(pose,binding,rdata) in enumerate(zip(poses,bindings,[approach]+routes)):
                target_folder=folder/f'checkpoint_{i+1}'
                legs.append(leg_result(context,req,manifest,original,target_folder,current,pose,rdata,binding))
                paths.append(target_folder/'plan.json');current=pose.tolist()
            returning=leg_result(context,req,manifest,original,folder/'return_to_start',current,start,return_route,bindings[-1])
            local=dict(context,checkpoint_sha256=checkpoints.signature(bindings))
            result=checkpoints.combine(legs,bindings,paths,local,returning,folder/'return_to_start/plan.json')
            req['goal']=poses[-1].tolist();atomic_json(folder/'backend_request.json',req)
        report['elapsed_s']=time.monotonic()-began
        result['fast_mode']=report
        log(f'READY: {report["transfer_movement_s"]:.3f}s transfer movement; original limits; no motors')
    except Exception as exc:
        result.update(status='no_feasible_route',details=dict(reason=str(exc)));report['error']=str(exc)
        log('No complete route: '+str(exc))
    finally:
        if oracle is not None:oracle.close()
        report['elapsed_s']=time.monotonic()-began
        atomic_json(folder/'fast_report.json',report);atomic_json(folder/'plan.json',result)
    return result
