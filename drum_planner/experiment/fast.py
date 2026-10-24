"""Cold, single-pipeline fixed-path experiment. No result or CPU-reference caches.

One frozen native world lives for one calculation only. All legs are freshly
generated, optimized and checked serially inside the existing bounded job.
Never used by either hardware planner.
"""
import copy
import json
import time
from pathlib import Path
from ..common import atomic_json, digest, read_state
from ..right_zone import metadata as zone_metadata
from ..timing import core
from ..timing.benchmark import validate
from .fast_timing import optimize
from .destinations import goal
from . import inputs, fast_path
from .runtime import Oracle
from .fast_config import LABEL


def world_identity(req):
    """Only selected-arm start/goal may vary in a single frozen world.

    The selected arm's scene mask stays at acquisition pose, including left
    mode's redundant `left` field. Model bytes and every other request field
    (passive arm, scene, tool, zones, margins) must match exactly.
    """
    data=copy.deepcopy(req);side=data.get('moving_side','right')
    snapshot=data.pop('scene_robot_start',data['start'])
    data['start']=snapshot;data.pop('goal',None)
    if side=='left':data['left']=snapshot
    for key in ('urdf','srdf'):data[key]=digest(data[key])
    return json.dumps(data,sort_keys=True,allow_nan=False)


class Session:
    def __init__(self):self.inputs={};self.oracle=None;self.identity=None
    def world(self,req,path,folder):
        identity=world_identity(req)
        if self.oracle is None:
            self.oracle=Oracle(path,folder);self.identity=identity
        elif identity!=self.identity:raise ValueError('Frozen world changed between checkpoint legs')
        return self.oracle
    def close(self):
        if self.oracle is not None:self.oracle.close();self.oracle=None


def proposal_stages(oracle,req,params,seconds,log,report):
    """Lazy second stage: no extra fitting after the consumer accepts a law."""
    remaining=seconds
    for weight in (2.,5.):
        if weight==5. and report.get('expanded_alternatives'):break
        rows,stage=fast_path.candidates(oracle,req,params,remaining,log,prior_weight=weight)
        remaining-=stage.get('pool_elapsed_s',0.)
        if not report:report.update(stage)
        else:
            report['candidates'].extend(stage['candidates']);report['searches'].extend(stage['searches'])
            report['pool_elapsed_s']+=stage.get('pool_elapsed_s',0.)
            for key,value in stage.get('profile',{}).items():report['profile'][key]+=value
        yield from rows
        if stage.get('reason')=='nominal_endpoint_invalid' or remaining<=0:break
        log('No validated moderate-prior route; trying stronger broad-curve alternatives')


def leg(context,folder,method,params,seconds,session):
    from .worker import base_result
    from ..planning import cycle
    from ..workflow import simulate
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True);began=time.monotonic()
    log=lambda s:print(f'[Fast experiment {folder.name} +{time.monotonic()-began:5.2f}s] {s}',flush=True)
    context=dict(context,_sequence_inputs=session.inputs)
    state=read_state(context['state']);side=context.get('moving_side','right')
    req,manifest=inputs.request(context,folder,state);req['seed']=params['seed']
    if req.get('execute') is not False:
        raise ValueError('Fast planner is saved-input experiment only; execution is forbidden')
    goal_path,req['goal']=goal(context)
    if 'scene_robot_start' in context:req['scene_robot_start']=context['scene_robot_start']
    if context.get(side+'_zone')!=zone_metadata(req.get(side+'_zone')):
        raise ValueError('Zone/reference model changed; restart the experiment')
    atomic_json(folder/'backend_request.json',req)
    result=base_result(context,req,manifest,state)
    result['experiment']=dict(method=method,description=LABEL,gesture_settings=params,budget_s=seconds,
        goal_source='Selected endpoint only; no recorded interior motion',destination_sha256=digest(goal_path),
        state_source='saved measurement; no live CAN',scene_sha256=digest(context['scene']),
        cpu_reference_used=False,result_cache_used=False)
    try:
        oracle=session.world(req,folder/'backend_request.json',folder)
        prepare_s=time.monotonic()-began
        report={}
        result['experiment']['report']=report
        for winner in proposal_stages(oracle,req,params,seconds,log,report):
            source=dict(path_waypoints=winner['path'],timing=winner['timing'],request=req)
            law=optimize(source,log)
            t=time.monotonic()
            try:proof=validate(law,source,folder/'final_check',oracle=oracle)
            except ValueError as exc:
                winner['workspace_fit']['reason']='final_validation: '+str(exc)
                log(f'Candidate {winner["index"]} rejected: {exc}; trying next ranked curve');continue
            verify_s=time.monotonic()-t
            points=core.samples(law,source)
            clean=[points[0]]
            for point in points[1:-1]:
                if point['time_s']-clean[-1]['time_s']>1e-10 and points[-1]['time_s']-point['time_s']>1e-10:
                    clean.append(point)
            points=clean+[points[-1]];points[0]['positions_rad']=list(req['start']);points[-1]['positions_rad']=list(req['goal'])
            report['selected']=winner['index'];report.pop('reason',None);winner['workspace_fit']['accepted']=True
            backend=law['details']['actual_backend']
            meta=dict(law['details'],method=law['method'],requested_backend='cpu_sparse',gpu_used=False,
                label=law['details']['backend_label'],
                cached=False,duration_s=law['duration_s'],verification=proof,verification_elapsed_s=verify_s)
            result.update(status='planned',backend=LABEL,outward=points,cycle=cycle(points),timing=law,
                source_timing=winner['timing'],path_waypoints=winner['path'],duration_outward_s=law['duration_s'],
                hold_at_goal_s=2.,reverse_checked_on_same_static_scene=True,final_curve_check=proof['collision_zone'],
                sampled_interpolated_checks=proof['collision_zone']['samples_checked'],
                joint_limit_check=report['joint_limit_check'],velocity_limit_rad_s=.6,
                acceleration_limit_rad_s2=1.2,jerk_limit_rad_s3=6.,collision_edge_l1_step_rad=.003)
            result['experiment'].update(retiming=meta,cycle_test=simulate(result),
                profile=dict(prepare_s=prepare_s,path_s=report['pool_elapsed_s'],
                             timing_s=law['details']['calculation_seconds'],verification_s=verify_s))
            atomic_json(folder/'selected_candidate.json',winner)
            atomic_json(folder/'fast_timing_report.json',meta)
            log(f'READY: candidate {winner["index"]}, {law["duration_s"]:.3f}s movement; '
                'continuous limits + dense collision/zone verified; SIMULATION ONLY')
            break
        else:result.update(status='no_feasible_route',details=dict(report,reason=report.get('reason','all_ranked_curves_failed_validation')))
        atomic_json(folder/'experiment_report.json',report)
    except Exception as exc:
        result.update(status='experiment_error',error=str(exc));raise
    finally:
        result['experiment']['elapsed_s']=time.monotonic()-began
        atomic_json(folder/'plan.json',result)
    return result


def plan(context,folder,method,params,seconds):
    if method!='workspace_curve':raise ValueError('Fast experiment has one workspace pipeline')
    session=Session()
    try:
        if len(context.get('checkpoints',[]))>1:
            from .checkpoints import plan_sequence
            result=plan_sequence(context,folder,method,params,seconds,
                lambda *args:leg(*args,session),parallel_timing=False)
            if result['status']=='planned':
                reports=result['experiment']['checkpoint_reports']+[result['experiment']['return_report']]
                backends=sorted({r['retiming']['actual_backend'] for r in reports})
                result['experiment']['retiming']=dict(actual_backends=backends,requested_backend='cpu_sparse',
                    fallback_used=any(r['retiming']['fallback_used'] for r in reports),
                    label=('CPU TIMING FALLBACK on some legs' if any(r['retiming']['fallback_used'] for r in reports)
                           else 'CPU sparse timing — intentional CPU backend')+'; 2 s at every checkpoint')
                atomic_json(Path(folder)/'plan.json',result)
            return result
        return leg(context,folder,method,params,seconds,session)
    finally:session.close()
