"""Workspace curve on acquired inputs; shared experiment generator and timing.

No acquired-state/collision proof is imported from an experiment. Checkpoint
composition is shared, but every leg is generated for this Calculate snapshot.
"""
import time
from pathlib import Path
from ..common import DEFAULT_CALIBRATION,arm_side,arm_joints,goal_recording,atomic_json,digest,read_state
from .. import planning
from ..right_zone import metadata
from ..experiment import workspace_curve,checkpoints,destinations as destination_files
from ..experiment.config import settings,DESCRIPTIONS
from ..timing import pipeline

METHOD='workspace_curve'
BUDGET_SECONDS=120.


def _leg(context,folder,method,params,seconds):
    side=context['moving_side'];folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    began=time.monotonic()
    def log(message):print(f'[Workspace curve {side} +{time.monotonic()-began:6.1f}s] {message}',flush=True)
    from ..experiment.inputs import request
    state=read_state(context['state']);req,manifest=request(context,folder,state);world=req['scene']
    goal_path,req['goal']=destination_files.goal(context)
    req['seed']=params['seed']
    if 'scene_robot_start' in context:req['scene_robot_start']=context['scene_robot_start']
    atomic_json(folder/'backend_request.json',req)
    result=dict(schema='x5-moveit-result-v1',status='not_generated',motion_authorized=False,
        hardware_execution_available=False,synthetic=world.get('synthetic',False),moving_side=side,joint_order=arm_joints(side),
        calibration_sha256=digest(context['calibration']),recording_sha256=digest(goal_path),model=manifest,start_state=state,
        collision_policy=req['collision_policy'],strict_unknown_space=False,scene_is_frozen_snapshot=True,
        actual_current_robot_state_verified=False,experiment_only=False,motor_commands=0,
        provenance=dict(scene=context['scene'],state=context['state'],state_sha256=context['state_sha256']),
        **{side+'_zone_constraint':metadata(req[side+'_zone'])},
        environment_assumption='Only observed obstacles are checked; unseen space is not certified free',
        physical_execution_blockers=['Calculate acquisition and final exact-curve preparation required; explicit Start only'])
    # Shared composition consumes this diagnostics envelope. It is renamed to
    # route_planner on the completed hardware plan; it is not execution authority.
    result['experiment']=dict(method=METHOD,description=DESCRIPTIONS[METHOD],settings=params,budget_s=seconds,
                             goal_source=str(goal_path)+' endpoint only; no interior samples')
    try:
        if context.get('timing_backend')=='sparse':
            log('same Workspace curve generator/settings; separate sparse IPOPT timing fork; no motor commands')
        else:log('retained legacy Workspace curve and arm-aware adaptive timing; no motor commands')
        winner,report=workspace_curve.run(req,context,context['pool_root'],folder/'backend_request.json',params,seconds,log)
        result['experiment']['report']=report
        if winner is None:result.update(status='no_feasible_route',details=report)
        else:
            pipeline.pending(result,winner,report,req,context,folder,time.monotonic()-began,hardware=True)
            if not context.get('_defer_timing'):result=pipeline.finish(result)
    except Exception as exc:
        result.update(status='error',error=str(exc));raise
    finally:
        result['experiment']['elapsed_s']=time.monotonic()-began;atomic_json(folder/'plan.json',result)
    return result


def plan(folder,state_path,scene_path,cal_path=DEFAULT_CALIBRATION,*,moving_side='right',destinations=None,simulation=False,timing_backend='current',motion_profile=None):
    if timing_backend not in ('current','adaptive','sparse'):raise ValueError('Unknown hardware timing backend')
    if timing_backend=='current':
        from .current import plan as current_plan
        return current_plan(folder,state_path,scene_path,cal_path,moving_side=moving_side,
            destinations=destinations,simulation=simulation,motion_profile=motion_profile)
    if motion_profile is not None:raise ValueError('Custom profiles require the current hardware planner, not a legacy fork')
    side=arm_side(moving_side);state=read_state(state_path)
    if not state.get('measured') and not simulation:raise ValueError('Hardware planning requires an acquired arm state')
    folder=Path(folder).resolve();folder.mkdir(parents=True,exist_ok=False);began=time.monotonic()
    context=dict(state=str(Path(state_path).resolve()),scene=str(Path(scene_path).resolve()),
        state_sha256=digest(state_path),scene_sha256=digest(scene_path),calibration=str(Path(cal_path).resolve()),
        moving_side=side,pool_root=str(folder/'candidate_pool'))
    binding=destinations or checkpoints.freeze([goal_recording(side)],side,folder)
    context.update(binding)
    if timing_backend=='sparse':context['timing_backend']='sparse'
    # Frozen selection is validated before search and never reopened from the
    # user's editable destination files while a Calculate job is running.
    bindings=binding.get('checkpoints',[binding])
    for selected in bindings:destination_files.goal(dict(selected,moving_side=side))
    if len(bindings)>1:
        result=checkpoints.plan_sequence(context,folder,METHOD,settings(),BUDGET_SECONDS,_leg,parallel_timing=True)
    else:result=_leg(context,folder,METHOD,settings(),BUDGET_SECONDS)
    result['route_planner']=result.pop('experiment')
    result.update(destination_bindings=bindings,planning_wall_s=time.monotonic()-began,
                  experiment_only=bool(simulation),offline_simulation=bool(simulation))
    if timing_backend=='sparse':
        result.update(hardware_variant='sparse_ipopt',backend='Workspace curve + sparse IPOPT timing (fork)')
    atomic_json(folder/'plan.json',result)
    return result
