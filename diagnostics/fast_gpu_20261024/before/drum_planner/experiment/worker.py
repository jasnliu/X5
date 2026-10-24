"""Contained software worker. No telemetry, hardware executor or CAN imports."""
import argparse,json,time,traceback
from pathlib import Path
from ..common import ROOT,DEFAULT_CALIBRATION,DEFAULT_RECORDING,atomic_json,digest,read_state,arm_side,goal_recording,arm_joints
from .. import planning
from ..right_zone import metadata as zone_metadata
from .config import METHODS,DESCRIPTIONS,settings,budget,no_can
from .destinations import goal


def base_result(context,req,manifest,state):
    side=arm_side(context.get('moving_side','right'))
    return dict(schema='x5-moveit-result-v1',status='not_generated',motion_authorized=False,
        hardware_execution_available=False,synthetic=req['scene'].get('synthetic',False),
        calibration_sha256=digest(context['calibration']),recording_sha256=digest(goal(context)[0]),
        moving_side=side,joint_order=arm_joints(side),
        model=manifest,start_state=state,collision_policy=req['collision_policy'],strict_unknown_space=False,
        scene_is_frozen_snapshot=True,actual_current_robot_state_verified=False,
        provenance=dict(scene=context['scene'],state=context['state'],state_sha256=digest(context['state'])),
        physical_execution_blockers=['SOFTWARE EXPERIMENT ONLY; saved state is not current joint feedback',
            'Experimental method outputs are never eligible for hardware execution'],
        environment_assumption='Only observed obstacles are checked; unseen space is not certified free',
        experiment_only=True,can_access=False,motor_commands=0,
        **{side+'_zone_constraint':zone_metadata(req.get(side+'_zone'))})


def plan(context,folder,method,params,seconds):
    # Experiment-only routing. Hardware never calls this worker's plan().
    context=dict(context,timing_backend='vulkan',experiment_gpu_timing=True)
    if len(context.get('checkpoints',[]))>1:
        from .checkpoints import plan_sequence
        result=plan_sequence(context,folder,method,params,seconds,plan,parallel_timing=True)
        if result['status']=='planned':
            exp=result['experiment']
            reports=exp['checkpoint_reports']+[exp['return_report']]
            backends=sorted({r['retiming']['actual_backend'] for r in reports})
            labels={'vulkan':'GPU Vulkan FP64', 'cpu':'CPU FALLBACK — GPU unavailable',
                    'analytic_cpu':'Analytic CPU (no nonlinear solve)'}
            exp['retiming']=dict(label=' / '.join(labels[b] for b in backends)+'; 2 s at every checkpoint',
                                requested_backend='vulkan',actual_backends=backends)
            atomic_json(Path(folder)/'plan.json',result)
        return result
    from .algorithms import run
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True);start=time.monotonic()
    def log(message):print(f'[{method} +{time.monotonic()-start:6.1f}s] {message}',flush=True)
    from .inputs import request
    state=read_state(context['state'])
    side=arm_side(context.get('moving_side','right'))
    req,manifest=request(context,folder,state);req['seed']=params['seed']
    goal_path,req['goal']=goal(context)
    if 'scene_robot_start' in context:req['scene_robot_start']=context['scene_robot_start']
    if context.get(side+'_zone')!=zone_metadata(req.get(side+'_zone')):
        raise ValueError(f'{side.title()} zone/reference model changed: restart the experiment to rebuild the cache')
    atomic_json(folder/'backend_request.json',req)
    result=base_result(context,req,manifest,state)
    result['experiment']=dict(method=method,description=DESCRIPTIONS[method],gesture_settings=params,budget_s=seconds,
                              goal_source=f'ONLY final joint pose of {context.get("destination_source",str(goal_path))}; no intermediate positions',
                              destination_sha256=digest(goal_path),
                              state_source='saved measurement; no live CAN',scene_sha256=digest(context['scene']))
    try:
        log(DESCRIPTIONS[method]);log('preparing model + frozen inputs; FCL world is loaded only for a new pool; no motor access')
        spec=req.get(side+'_zone')
        log(f'ARM: {side}; '+(f'ZONE: {Path(spec["zone_file"]).name}, original {side} gripper TCP only; 5 mm membership tolerance'
                              if spec else 'No point-envelope constraint; full collision checks remain'))
        if method=='workspace_curve':
            # Separate proposals/budget; never add candidates to, or consume
            # the search budget of, the original four methods' shared pool.
            from .workspace_curve import run as workspace_run
            winner,report=workspace_run(req,context,context['pool_root'],folder/'backend_request.json',params,seconds,log)
        else:
            winner,report=run(method,req,context,context['pool_root'],folder/'backend_request.json',params,seconds,log)
        result['experiment']['report']=report
        atomic_json(folder/'experiment_report.json',report)
        if winner is None:result.update(status='no_feasible_route',details=report)
        else:
            from ..timing import pipeline
            pipeline.pending(result,winner,report,req,context,folder,time.monotonic()-start)
            if not context.get('_defer_timing'):result=pipeline.finish(result)
        result['experiment']['elapsed_s']=time.monotonic()-start
        result.update(hardware_execution_available=False,motion_authorized=False,actual_current_robot_state_verified=False)
        if result['status']=='planned':
            log(f'READY: {result["duration_outward_s"]:.3f}s outward → 2s hold → exact reverse; SIMULATION ONLY')
        elif result['status']!='timing_pending':log('NO FEASIBLE ROUTE: '+str(result.get('details',{}).get('reason',result['status'])))
    except Exception as e:
        result.update(status='experiment_error',error=str(e));traceback.print_exc();raise
    finally:
        atomic_json(folder/'plan.json',result)
    return result


def initialize(folder,state_path,scene_path,cal_path,side='right',zone=None):
    arm_side(side)
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    state_path=str(Path(state_path).resolve());scene_path=str(Path(scene_path).resolve());cal_path=str(Path(cal_path).resolve())
    from ..depth import refine_snapshot
    scene_path=str(refine_snapshot(scene_path,folder/'scene_5mm',cal_path))
    state=read_state(state_path);world=planning.scene(scene_path,cal_path)
    context=dict(state=state_path,state_sha256=digest(state_path),scene=scene_path,scene_sha256=digest(scene_path),calibration=cal_path,
                 saved_not_live=True,can_access=False,motor_commands=0,pool_root=str((folder/'shared_pools').resolve()),
                 moving_side=side,goal_recording=str(goal_recording(side)))
    req,manifest=planning.request(folder/'initial',state,world,cal_path,moving_side=side,zone_path=zone)
    spec=req.get(side+'_zone');context[side+'_zone']=zone_metadata(spec)
    context['zone_path']=spec['zone_file'] if spec else 'none'
    initial=folder/'initial';initial.mkdir(exist_ok=True)
    atomic_json(initial/'backend_request.json',req);atomic_json(initial/'plan.json',base_result(context,req,manifest,state))
    atomic_json(folder/'context.json',context);return context


def main():
    no_can();p=argparse.ArgumentParser();p.add_argument('action',choices=['build','capture','initialize','plan'])
    p.add_argument('--folder',required=True);p.add_argument('--state');p.add_argument('--scene');p.add_argument('--context')
    p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION));p.add_argument('--method',choices=METHODS)
    p.add_argument('--settings');p.add_argument('--budget',type=budget,default=60.)
    p.add_argument('--side',choices=['left','right'],default='right');p.add_argument('--zone')
    a=p.parse_args()
    if a.action=='build':
        from .runtime import build
        print(build(),flush=True)
    elif a.action=='capture':
        from .capture import capture
        capture(Path(a.folder)/'camera',a.calibration)
    elif a.action=='initialize':initialize(a.folder,a.state,a.scene,a.calibration,a.side,a.zone)
    else:
        context=json.loads(Path(a.context).read_text())
        if digest(context['state'])!=context['state_sha256'] or digest(context['scene'])!=context['scene_sha256']:
            raise ValueError('Experiment inputs changed; initialize a new frozen scene')
        params=settings(json.loads(Path(a.settings).read_text()) if a.settings else None)
        plan(context,a.folder,a.method,params,a.budget)


if __name__=='__main__':main()
