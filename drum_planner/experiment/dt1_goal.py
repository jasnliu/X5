"""Right-only DT1 endpoint adapter; no devices, motor API, or saved-goal fallback.

Runs before the ordinary path/timing worker in the same bounded serial queue.
DT1 inference replays this launch's final RGBD pair. IK and its approach check
use the planner's five-frame collision world and original acquisition self-mask.
"""
import argparse,json,os,subprocess,time,uuid
from pathlib import Path
from ..common import ROOT,atomic_json,digest,endpoint,read_state
from . import checkpoints,scenario

TOKEN='dt1'


def resolve(value,side):
    if str(value).lower()==TOKEN:
        if side!='right':raise ValueError('dt1 is available only for the right arm')
        return TOKEN
    return scenario.resolve(value,side)


def identity(context,selected):
    from cymbal_destination.config import VERSION,MAX_INWARD_YAW_DEG
    from cymbal_destination.goal_limits import policy
    from ..joint_position_limits import policy as position_policy
    return dict(scene_sha256=context['scene_sha256'],state_sha256=context['state_sha256'],
        calibration_sha256=digest(context['calibration']),moving_side=context.get('moving_side','right'),
        zone=context.get('right_zone'),scene_robot_start=context.get('scene_robot_start'),
        selection=[dict(source=str(p),sha256=None if p==TOKEN else digest(p)) for p in selected],
        version=VERSION,maximum_inward_yaw_deg=MAX_INWARD_YAW_DEG,goal_joint_reserve_policy=policy(),
        joint_position_policy=position_policy())


def start(session,context,selected):
    from .runtime import Job,PYTHON
    target=Path(session)/'dt1_goals'/uuid.uuid4().hex[:12];target.mkdir(parents=True)
    obj=dict(context=context,selected=selected,identity=identity(context,selected))
    atomic_json(target/'input.json',obj)
    job=Job([str(PYTHON),'-m','drum_planner.experiment.dt1_goal','--folder',str(target)],target,
            timeout=90+90*selected.count(TOKEN))
    return target,job


def result(target,context,selected):
    target=Path(target);obj=json.loads((target/'result.json').read_text())
    if obj.get('status')!='ready':raise ValueError(obj.get('error','DT1 destination generation failed'))
    if obj['identity']!=identity(context,selected):raise ValueError('Stale dt1 scene/start/selection; regenerate the destination')
    paths=obj['resolved']
    if len(paths)!=len(selected) or len(obj['resolved_sha256'])!=len(paths):
        raise ValueError('Incomplete dt1 checkpoint result')
    for selection,path,sha in zip(selected,paths,obj['resolved_sha256']):
        if digest(path)!=sha:raise ValueError('Generated destination changed')
        q=endpoint(path,side='right')
        if selection==TOKEN:
            from cymbal_destination import output,goal_limits
            goal=json.loads(Path(path).read_text());output.current_convention(goal)
            goal_limits.verify(q,goal.get('goal_joint_reserve'))
    return paths,obj


def bind_headless(session,context,selected):
    from .app import wait
    target,job=start(session,context,selected);wait(job)
    paths,proof=result(target,context,selected)
    context.update(checkpoints.freeze(paths,'right',session),dt1_generation=proof)
    atomic_json(Path(session)/'context.json',context)
    return context


def generate(folder,context,selected):
    from .config import no_can
    from .rgbd import scene,CAMERA_PYTHON,SCHEMA
    from .inputs import request
    from .runtime import Oracle
    from cymbal_destination import inputs,solver,output,aim
    no_can();folder=Path(folder);began=time.monotonic()
    report=dict(status='failed',error=None,motor_commands=0,can_access=False,motion_authorized=False,
                endpoint_only=True,identity=identity(context,selected))
    try:
        if context.get('moving_side','right')!='right':raise ValueError('dt1 is right-arm only')
        if not 1<=len(selected)<=3 or TOKEN not in selected:raise ValueError('Expected a dt1 checkpoint selection')
        if digest(context['scene'])!=context['scene_sha256'] or digest(context['state'])!=context['state_sha256']:
            raise ValueError('Frozen experiment inputs changed')
        world=scene(context['scene'],context['calibration'])
        if world.get('schema')!=SCHEMA:
            raise ValueError('This scene has no paired RGB image for dt1. Relaunch the right experiment for fresh inputs, '
                             'or use Recapture scene if the arm has not moved. No old destination will be used.')
        pair=Path(context['scene']).parent/world['dt1_pair_file']
        vision=folder/'vision';vision.mkdir()
        env=dict(os.environ,YOLO_CONFIG_DIR=str(vision/'yolo_config'),MPLCONFIGDIR=str(vision/'matplotlib'))
        print('[dt1] Detecting the cymbal in this experiment snapshot; camera stays OFF.',flush=True)
        # Child stays in this job's process/cgroup and has no CAN capability.
        command=[str(CAMERA_PYTHON),'-B',str(ROOT/'cymbal_destination/camera_worker.py'),
                 '--folder',str(vision),'--calibration',context['calibration'],'--replay',str(pair)]
        run=subprocess.run(command,env=env,timeout=70)
        if run.returncode:
            error=vision/'error.json'
            raise ValueError(json.loads(error.read_text())['error'] if error.exists() else 'DT1 perception worker failed')
        target,point,normal=inputs.target(vision/'target.json',context['calibration'])
        inward=aim.reference(target,normal)
        state=read_state(context['state']);resolved=[];goals=[]
        for index,value in enumerate(selected):
            if value!=TOKEN:
                frozen=checkpoints.freeze([value],'right',folder)
                path=frozen['goal_recording'];state['right_rad']=endpoint(path,side='right').tolist()
                resolved.append(path);continue
            step=folder/f'checkpoint_{index+1}';step.mkdir()
            req,manifest=request(context,step,state)
            req.update(goal=state['right_rad'],scene_robot_start=context.get('scene_robot_start',read_state(
                context.get('acquired_state',context['state']))['right_rad']))
            atomic_json(step/'backend_request.json',req)
            oracle=Oracle(step/'backend_request.json',step)
            try:
                log=lambda text:print(f'[dt1 +{time.monotonic()-began:.2f}s] {text}',flush=True)
                winner,search=solver.solve(oracle,req,point,normal,inward,log)
                atomic_json(step/'search.json',search)
                if winner is None:raise ValueError('No feasible dt1 prime destination: '+str(search.get('reason')))
                obj=output.destination(winner['q'],winner['geometry'],point,normal,context['calibration'],context['scene_sha256'],
                                       joint_bounds=(oracle.lower,oracle.upper))
                obj['experiment_dt1_source']=dict(scene=context['scene'],pair_sha256=digest(pair),
                    target_sha256=digest(vision/'target.json'),request_sha256=digest(step/'backend_request.json'))
                path=step/'dt1.json';atomic_json(path,obj);resolved.append(str(path.resolve()))
                state['right_rad']=winner['q']
                goals.append(dict(checkpoint=index+1,path=str(path.resolve()),geometry=winner['geometry'],
                                  approach=winner['approach'],model=manifest))
            finally:oracle.close()
        if report['identity']!=identity(context,selected):raise ValueError('DT1 inputs changed during generation')
        report.update(status='ready',resolved=resolved,resolved_sha256=[digest(p) for p in resolved],goals=goals,
                      scene=context['scene'],pair_sha256=digest(pair),target=target)
        print('[dt1] Destination ready. The normal workspace path and timing planner will run next.',flush=True)
    except BaseException as exc:
        report['error']=str(exc) or type(exc).__name__;raise
    finally:
        report['elapsed_s']=time.monotonic()-began;atomic_json(folder/'result.json',report)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--folder',required=True)
    args=parser.parse_args();folder=Path(args.folder)
    obj=json.loads((folder/'input.json').read_text())
    if obj['identity']!=identity(obj['context'],obj['selected']):raise ValueError('DT1 job inputs changed')
    generate(folder,obj['context'],obj['selected'])
