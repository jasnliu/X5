"""Contained software worker. No telemetry, hardware executor or CAN imports."""
import argparse,json,time,traceback
from pathlib import Path
from ..common import ROOT,DEFAULT_CALIBRATION,DEFAULT_RECORDING,atomic_json,digest,read_state
from .. import planning
from ..right_zone import metadata as zone_metadata
from .config import METHODS,DESCRIPTIONS,settings,budget,no_can


def base_result(context,req,manifest,state):
    return dict(schema='x5-moveit-result-v1',status='not_generated',motion_authorized=False,
        hardware_execution_available=False,synthetic=req['scene'].get('synthetic',False),
        calibration_sha256=digest(context['calibration']),recording_sha256=digest(DEFAULT_RECORDING),
        model=manifest,start_state=state,collision_policy=req['collision_policy'],strict_unknown_space=False,
        scene_is_frozen_snapshot=True,actual_current_robot_state_verified=False,
        provenance=dict(scene=context['scene'],state=context['state'],state_sha256=digest(context['state'])),
        physical_execution_blockers=['SOFTWARE EXPERIMENT ONLY; saved state is not current joint feedback',
            'Experimental method outputs are never eligible for hardware execution'],
        environment_assumption='Only observed obstacles are checked; unseen space is not certified free',
        experiment_only=True,can_access=False,motor_commands=0,
        right_zone_constraint=zone_metadata(req['right_zone']))


def plan(context,folder,method,params,seconds):
    from .algorithms import run
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True);start=time.monotonic()
    def log(message):print(f'[{method} +{time.monotonic()-start:6.1f}s] {message}',flush=True)
    state=read_state(context['state']);world=planning.scene(context['scene'],context['calibration'])
    req,manifest=planning.request(folder,state,world,context['calibration']);req['seed']=params['seed']
    if context.get('right_zone')!=zone_metadata(req['right_zone']):
        raise ValueError('Right zone3/reference model changed or legacy session: restart the experiment to rebuild the cache')
    atomic_json(folder/'backend_request.json',req)
    result=base_result(context,req,manifest,state)
    result['experiment']=dict(method=method,description=DESCRIPTIONS[method],gesture_settings=params,budget_s=seconds,
                              goal_source='ONLY final joint pose of record3; no intermediate positions',
                              state_source='saved measurement; no live CAN',scene_sha256=digest(context['scene']))
    try:
        log(DESCRIPTIONS[method]);log('preparing model + frozen inputs; FCL world is loaded only for a new pool; no motor access')
        log('ZONE3: original right gripper TCP only; legacy 5 mm membership tolerance; left/stick excluded from zone check')
        winner,report=run(method,req,context,context['pool_root'],folder/'backend_request.json',params,seconds,log)
        result['experiment']['report']=report
        atomic_json(folder/'experiment_report.json',report)
        if winner is None:result.update(status='no_feasible_route',details=report)
        else:
            outward=winner['outward'];timing=winner['timing']
            result.update(status='planned',outward=outward,cycle=planning.cycle(outward),path_waypoints=winner['path'],
                duration_outward_s=timing['duration_s'],hold_at_goal_s=2.,
                velocity_limit_rad_s=.6,acceleration_limit_rad_s2=1.2,jerk_limit_rad_s3=6.,
                timing=timing,final_curve_check=winner['proof'],
                sampled_interpolated_checks=winner['proof']['samples_checked'],
                reverse_checked_on_same_static_scene=True,joint_limit_check=report['joint_limit_check'],
                backend=DESCRIPTIONS[method],collision_edge_l1_step_rad=.003)
        result['experiment']['elapsed_s']=time.monotonic()-start
        result.update(hardware_execution_available=False,motion_authorized=False,actual_current_robot_state_verified=False)
        if result['status']=='planned':
            from ..workflow import simulate
            result['experiment']['cycle_test']=simulate(result)
            log(f'READY: {result["duration_outward_s"]:.3f}s outward → 2s hold → exact reverse; SIMULATION ONLY')
        else:log('NO FEASIBLE ROUTE: '+str(result.get('details',{}).get('reason',result['status'])))
    except Exception as e:
        result.update(status='experiment_error',error=str(e));traceback.print_exc();raise
    finally:
        atomic_json(folder/'plan.json',result)
    return result


def initialize(folder,state_path,scene_path,cal_path):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    state_path=str(Path(state_path).resolve());scene_path=str(Path(scene_path).resolve());cal_path=str(Path(cal_path).resolve())
    state=read_state(state_path);world=planning.scene(scene_path,cal_path)
    context=dict(state=state_path,state_sha256=digest(state_path),scene=scene_path,scene_sha256=digest(scene_path),calibration=cal_path,
                 saved_not_live=True,can_access=False,motor_commands=0,pool_root=str((folder/'shared_pools').resolve()))
    req,manifest=planning.request(folder/'initial',state,world,cal_path)
    context['right_zone']=zone_metadata(req['right_zone'])
    initial=folder/'initial';initial.mkdir(exist_ok=True)
    atomic_json(initial/'backend_request.json',req);atomic_json(initial/'plan.json',base_result(context,req,manifest,state))
    atomic_json(folder/'context.json',context);return context


def main():
    no_can();p=argparse.ArgumentParser();p.add_argument('action',choices=['build','capture','initialize','plan'])
    p.add_argument('--folder',required=True);p.add_argument('--state');p.add_argument('--scene');p.add_argument('--context')
    p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION));p.add_argument('--method',choices=METHODS)
    p.add_argument('--settings');p.add_argument('--budget',type=budget,default=60.)
    a=p.parse_args()
    if a.action=='build':
        from .runtime import build
        print(build(),flush=True)
    elif a.action=='capture':
        from .capture import capture
        capture(Path(a.folder)/'camera',a.calibration)
    elif a.action=='initialize':initialize(a.folder,a.state,a.scene,a.calibration)
    else:
        context=json.loads(Path(a.context).read_text())
        if digest(context['state'])!=context['state_sha256'] or digest(context['scene'])!=context['scene_sha256']:
            raise ValueError('Experiment inputs changed; initialize a new frozen scene')
        params=settings(json.loads(Path(a.settings).read_text()) if a.settings else None)
        plan(context,a.folder,a.method,params,a.budget)


if __name__=='__main__':main()
