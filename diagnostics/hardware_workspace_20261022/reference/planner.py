"""Single arc backend for the measured-start selected-arm workspace.

Generate from supplied acquired joints/scene; never promote an experiment cache
or its saved start to a hardware plan. No acquisition or actuator interfaces.
"""
import time
from pathlib import Path
from ..common import DEFAULT_CALIBRATION,arm_side,arm_joints,goal_recording,atomic_json,digest,read_state
from .. import planning
from ..right_zone import metadata
from ..experiment import algorithms
from ..experiment.config import settings,DESCRIPTIONS
from ..timing.dispatch import retime

METHOD='single_arc'
BUDGET_SECONDS=120.


def plan(folder,state_path,scene_path,cal_path=DEFAULT_CALIBRATION,*,moving_side='right'):
    side=arm_side(moving_side);zone_key=side+'_zone'
    folder=Path(folder).resolve();folder.mkdir(parents=True,exist_ok=False)
    began=time.monotonic();params=settings()
    def log(message):print(f'[Single arc +{time.monotonic()-began:6.1f}s] {message}',flush=True)
    state=read_state(state_path)
    if not state.get('measured'):raise ValueError('Single arc hardware planning requires an acquired arm state')
    world=planning.scene(scene_path,cal_path)
    req,manifest=planning.request(folder,state,world,cal_path,moving_side=side)
    req['seed']=params['seed'];atomic_json(folder/'backend_request.json',req)
    context=dict(state=str(Path(state_path).resolve()),scene=str(Path(scene_path).resolve()),
        state_sha256=digest(state_path),scene_sha256=digest(scene_path),calibration=str(Path(cal_path).resolve()),
        moving_side=side,pool_root=str(folder/'candidate_pool'))
    result=dict(schema='x5-moveit-result-v1',status='not_generated',motion_authorized=False,
        hardware_execution_available=False,synthetic=world.get('synthetic',False),moving_side=side,joint_order=arm_joints(side),
        calibration_sha256=digest(cal_path),recording_sha256=digest(goal_recording(side)),model=manifest,start_state=state,
        collision_policy=req['collision_policy'],strict_unknown_space=False,scene_is_frozen_snapshot=True,
        actual_current_robot_state_verified=False,experiment_only=False,motor_commands=0,
        provenance=dict(scene=context['scene'],state=context['state'],state_sha256=context['state_sha256']),
        **{zone_key+'_constraint':metadata(req[zone_key])},
        environment_assumption='Only observed obstacles are checked; unseen space is not certified free',
        physical_execution_blockers=['Calculate acquisition and final exact-curve preparation required; explicit Start only'])
    try:
        log(f'{side.upper()} arm; current acquisition; {goal_recording(side).name} FINAL pose only; same Single arc method/settings as experiment --{side}')
        winner,report=algorithms.run(METHOD,req,context,context['pool_root'],folder/'backend_request.json',params,BUDGET_SECONDS,log)
        result['route_planner']=dict(method=METHOD,description=DESCRIPTIONS[METHOD],settings=params,report=report,
            goal_source=str(goal_recording(side).relative_to(goal_recording(side).parents[1]))+' final joint pose only; no interior samples')
        if winner is None:result.update(status='no_feasible_route',details=report)
        else:
            outward,law,proof,info=retime(winner,report,req,context,folder,log)
            result.update(status='planned',outward=outward,cycle=planning.cycle(outward),timing=law,retiming=info,
                source_timing=winner['timing'],path_waypoints=winner['path'],duration_outward_s=law['duration_s'],
                hold_at_goal_s=2.,reverse_checked_on_same_static_scene=True,final_curve_check=proof,
                sampled_interpolated_checks=proof['samples_checked'],joint_limit_check=report['joint_limit_check'],
                velocity_limit_rad_s=.6,acceleration_limit_rad_s2=1.2,jerk_limit_rad_s3=6.,
                backend='Single arc + current arm-aware adaptive timing',collision_edge_l1_step_rad=.003)
            log(f'READY: {law["duration_s"]:.3f}s outward → 2s hold → reverse to acquired start. NO motor commands.')
    except Exception as exc:
        result.update(status='error',error=str(exc));raise
    finally:
        result['planning_wall_s']=time.monotonic()-began;atomic_json(folder/'plan.json',result)
    return result
