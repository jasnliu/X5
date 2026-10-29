"""Cold, single-pipeline fixed-path experiment. No result or CPU-reference caches.

One frozen native world lives for one calculation only. All legs are freshly
generated, optimized and checked serially inside the existing bounded job.
Also used for current hardware Calculate via baseline.current; no motor authority.
"""
import copy
import json
import time
from pathlib import Path
import numpy as np
from ..common import atomic_json, digest, read_state
from ..right_zone import metadata as zone_metadata
from ..timing import core
from .motion_timing import optimize, validate
from . import motion_settings, stationary
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
    def __init__(self):self.inputs={};self.oracle=None;self.identity=None;self.snapshot_start=None
    def world(self,req,path,folder):
        identity=world_identity(req)
        if self.oracle is None:
            self.oracle=Oracle(path,folder);self.identity=identity
        elif identity!=self.identity:raise ValueError('Frozen world changed between checkpoint legs')
        return self.oracle
    def close(self):
        if self.oracle is not None:self.oracle.close();self.oracle=None


def proposal_stages(oracle,req,params,seconds,log,report):
    """Keep successful legacy curves; recover only after their validation fails."""
    def checked(rows):
        from ..rest_paths import verify
        for row in rows:
            if not req.get('joint_position_context'):
                yield row;continue
            try:row['rest_transition_check']=verify(core.original(dict(timing=row['timing'])),req)
            except ValueError as exc:
                row['workspace_fit']['reason']='rest_transition: '+str(exc)
                log(f'Candidate {row["index"]} rejected: {exc}');continue
            yield row
    def merge(stage):
        report['candidates'].extend(stage['candidates']);report['searches'].extend(stage['searches'])
        report['pool_elapsed_s']+=stage.get('pool_elapsed_s',0.)
        for key,value in stage.get('profile',{}).items():
            report['profile'][key]=report['profile'].get(key,0.)+value
    remaining=seconds
    for weight in (2.,5.):
        if weight==5. and report.get('expanded_alternatives'):break
        rows,stage=fast_path.candidates(oracle,req,params,remaining,log,prior_weight=weight)
        remaining-=stage.get('pool_elapsed_s',0.)
        if not report:report.update(stage)
        else:merge(stage)
        yield from checked(rows)
        if stage.get('reason')=='nominal_endpoint_invalid' or remaining<=0:return
        if weight==2. and not report.get('expanded_alternatives'):
            log('No validated moderate-prior route; trying stronger broad-curve alternatives')
    from . import workspace_recovery
    report['recovery_policy']=workspace_recovery.VERSION
    for rows,stage in workspace_recovery.stages(oracle,req,params,remaining,log):
        merge(stage)
        yield from checked(rows)


def leg(context,folder,method,params,seconds,session):
    from .worker import base_result
    from ..planning import cycle
    from ..workflow import simulate
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True);began=time.monotonic()
    log=lambda s:print(f'[Workspace experiment {folder.name} +{time.monotonic()-began:5.2f}s] {s}',flush=True)
    context=dict(context,_sequence_inputs=session.inputs)
    if session.snapshot_start is not None:
        # Shared checkpoint composition derives subsequent simulated states.
        # Its old rest-based mask must NOT replace the actual acquisition mask
        # when this experiment explicitly starts at a different destination.
        context['scene_robot_start']=session.snapshot_start
        atomic_json(folder/'context.json',{k:v for k,v in context.items() if k!='_sequence_inputs'})
    state=read_state(context['state']);side=context.get('moving_side','right')
    profile=motion_settings.from_context(context);limits=motion_settings.matrix(profile)
    req,manifest=inputs.request(context,folder,state);req['seed']=params['seed']
    if side=='right':log('RIGHT J2: 0° operating minimum; exact acquired rest may be inside the margin, with checked monotone departure/return. No clipping.')
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
        cpu_reference_used=False,result_cache_used=False,motion_settings=profile,
        motion_settings_sha256=motion_settings.identity(profile),
        experiment_start=context.get('experiment_start',{'kind':'default','label':'Default'}))
    result['experiment_start']=result['experiment']['experiment_start']
    if context.get('acquired_state'):
        result['provenance'].update(acquired_state=context['acquired_state'],
            acquired_state_sha256=context['acquired_state_sha256'])
    if result['experiment_start']['kind']!='default':
        result['experiment']['state_source']='Simulated destination spawn; NOT a measurement; no motor commands'
    elif state.get('source','').startswith('Simulated checkpoint'):
        result['experiment']['state_source']='Simulated preceding checkpoint; NOT a measurement; no motor commands'
    try:
        oracle=session.world(req,folder/'backend_request.json',folder)
        prepare_s=time.monotonic()-began
        report={}
        result['experiment']['report']=report
        same=np.array_equal(req['start'],req['goal'])
        choices=(stationary.candidate(req,oracle,report) if same else
                 proposal_stages(oracle,req,params,seconds,log,report))
        for winner in choices:
            source=dict(path_waypoints=winner['path'],timing=winner['timing'],request=req)
            law=(stationary.law(source,limits,profile['timing_quality']) if same else
                 optimize(source,log,limits=limits,quality=profile['timing_quality']))
            t=time.monotonic()
            try:proof=validate(law,source,folder/'final_check',oracle=oracle)
            except ValueError as exc:
                winner['workspace_fit']['reason']='final_validation: '+str(exc)
                log(f'Candidate {winner["index"]} rejected: {exc}; trying next ranked curve');continue
            verify_s=time.monotonic()-t
            points=([dict(time_s=0.,positions_rad=list(req['start']),velocities_rad_s=[0.]*7,
                          accelerations_rad_s2=[0.]*7)] if same else core.samples(law,source))
            clean=[points[0]]
            for point in points[1:-1]:
                if point['time_s']-clean[-1]['time_s']>1e-10 and points[-1]['time_s']-point['time_s']>1e-10:
                    clean.append(point)
            points=clean+([points[-1]] if len(points)>1 else [])
            points[0]['positions_rad']=list(req['start']);points[-1]['positions_rad']=list(req['goal'])
            report['selected']=winner['index'];report.pop('reason',None);winner['workspace_fit']['accepted']=True
            backend=law['details']['actual_backend']
            meta=dict(law['details'],method=law['method'],requested_backend='cpu_sparse',gpu_used=False,
                label=law['details']['backend_label'],
                cached=False,duration_s=law['duration_s'],verification=proof,verification_elapsed_s=verify_s)
            result.update(status='planned',backend=LABEL,outward=points,cycle=cycle(points),timing=law,
                source_timing=winner['timing'],path_waypoints=winner['path'],duration_outward_s=law['duration_s'],
                hold_at_goal_s=2.,reverse_checked_on_same_static_scene=True,final_curve_check=proof['collision_zone'],
                sampled_interpolated_checks=proof['collision_zone']['samples_checked'],
                joint_limit_check=report['joint_limit_check'],velocity_limit_rad_s=limits[0].tolist(),
                acceleration_limit_rad_s2=limits[1].tolist(),jerk_limit_rad_s3=limits[2].tolist(),collision_edge_l1_step_rad=.003)
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
    if context.get('acquired_state'):
        if digest(context['acquired_state'])!=context['acquired_state_sha256']:
            raise ValueError('Acquired camera-mask state changed; relaunch the experiment')
        captured=read_state(context['acquired_state'])[context['moving_side']+'_rad']
        if context.get('scene_robot_start')!=captured:
            raise ValueError('Simulated spawn must not replace the captured camera self-mask pose')
    session=Session()
    session.snapshot_start=context.get('scene_robot_start')
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
                           else 'CPU sparse timing — '+motion_settings.from_context(context)['timing_quality'])+'; 2 s at every checkpoint',
                    constraint_summary=motion_settings.summary(
                        [s['timing'] for s in result['checkpoint_sequence']['segments']]+
                        [result['checkpoint_sequence']['return_leg']['timing']],
                        motion_settings.matrix(motion_settings.from_context(context))))
                atomic_json(Path(folder)/'plan.json',result)
            return result
        return leg(context,folder,method,params,seconds,session)
    finally:session.close()
