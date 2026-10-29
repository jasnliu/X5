"""Frozen dual-arm beat preparation. NO motor/device access in this process.

The exact current experiment engine is reused, not sampled/refitted recordings.
Only the measured snapshot robot is masked, including on the second (right) leg.
"""
import copy
import json
from pathlib import Path
import time
import numpy as np
from drum_planner.common import ROOT, DEFAULT_CALIBRATION, atomic_json, digest, read_state
from drum_planner.experiment import fast, config, checkpoints, motion_settings, dt1_goal, rgbd
from drum_planner.right_zone import constraint, metadata
from drum_planner.baseline.curve import Curve
from drum_planner.baseline.paths import validate_timing, validate_drive_speed, recovery, recovery_samples
from drum_planner.baseline.tasks import hard_check

SCHEMA = 'x5-generated-beat-mission-v2'


def context(folder, state, acquired, scene, calibration, side):
    original = read_state(acquired)
    spec = constraint(ROOT / (side+'_zones') / ('zone3.json' if side=='right' else 'zone1.json'), side=side)
    return dict(state=str(state), state_sha256=digest(state), scene=str(scene), scene_sha256=digest(scene),
                calibration=str(calibration), moving_side=side, acquired_state=str(acquired),
                acquired_state_sha256=digest(acquired), both_tools=True,
                scene_robot_start=original[side+'_rad'],
                scene_robot_poses={s:original[s+'_rad'] for s in ('left','right')},
                experiment_start=dict(kind='default', label='Captured rest'),
                motion_settings=motion_settings.settings(side), zone_path=spec['zone_file'],
                pool_root=str(folder/'pool'), **{side+'_zone':metadata(spec)})


def prepare(folder, capture, calibration=DEFAULT_CALIBRATION):
    config.no_can()
    folder, capture, calibration = map(lambda p:Path(p).resolve(), (folder,capture,calibration))
    folder.mkdir(parents=True, exist_ok=True)
    started=time.monotonic()
    acquired=capture/'measured_start.json'; scene=capture/'camera/scene.json'
    original=read_state(acquired); rgbd.scene(scene,calibration)
    choices=sorted((ROOT/'destinations/left').glob('*.json'))
    if len(choices)!=1:raise ValueError('Expected exactly ONE saved left destination; found '+str(len(choices)))
    plans={}; proofs={};curves={}
    for side in ('left','right'):
        output=folder/side; output.mkdir()
        state=copy.deepcopy(original)
        if side=='right':
            state.update(left_rad=plans['left']['outward'][-1]['positions_rad'], measured=False,
                         source='Predicted left goal after checked left path; not a new measurement')
        state_path=output/'leg_start.json'; atomic_json(state_path,state)
        ctx=context(output,state_path,acquired,scene,calibration,side)
        if side=='right':
            target=output/'dt1'; target.mkdir()
            generation=dt1_goal.generate(target,ctx,['dt1'])
            paths=generation['resolved']
        else:paths=choices
        ctx.update(checkpoints.freeze(paths,side,output))
        atomic_json(output/'context.json',ctx)
        print('GENERATED BEAT: '+side.upper()+' from captured rest; opposite arm '+('at rest' if side=='left' else 'at LEFT GOAL'),flush=True)
        plan=fast.plan(ctx,output,'workspace_curve',config.settings(),120.)
        if plan['status']!='planned':raise ValueError(side+' has no feasible route: '+str(plan.get('details')))
        curve=Curve(plan);validate_timing(curve);validate_drive_speed(curve,1.)
        curves[side]=curve
        req=json.loads((output/'backend_request.json').read_text())
        samples=curve.at_many(np.r_[np.arange(0,curve.duration,.002),curve.duration])
        proofs[side]=hard_check(req,samples,output/'dense_check')
        plans[side]=plan
    files=[acquired,scene,calibration]
    world=json.loads(scene.read_text())
    files.extend(scene.parent/name for name in world['files_sha256'])
    for side in plans:
        output=folder/side; files.extend([output/'plan.json',output/'backend_request.json',output/'model/robot.urdf',output/'model/robot.srdf'])
        ctx=json.loads((output/'context.json').read_text())
        req=json.loads((output/'backend_request.json').read_text())
        files.extend([Path(ctx['goal_recording']),Path(req[side+'_zone']['zone_file']),Path(req[side+'_zone']['reference_urdf'])])
    result=dict(schema=SCHEMA,status='ready',capture=str(capture),calibration=str(calibration),
        original=original,plans={s:str(folder/s/'plan.json') for s in plans},proofs=proofs,
        duration_s={s:float(plans[s]['duration_outward_s']) for s in plans},
        files={str(p):digest(p) for p in files},
        motor_commands=0,source_recordings_used=False,frozen_scene=True,order=['left','right'])
    print('GENERATED BEAT: preparing and checking RIGHT then LEFT return now; no end-of-run planning.',flush=True)
    return_file=folder/'return/return.json'
    build_return_cache(result,curves,return_file.parent)
    result['return_file']=str(return_file);result['files'][str(return_file)]=digest(return_file)
    result['calculation_s']=time.monotonic()-started
    atomic_json(folder/'mission.json',result)
    return result


def verify(path,*,recovery_only=False):
    obj=json.loads(Path(path).read_text())
    if obj.get('schema') not in (SCHEMA,'x5-generated-beat-mission-v1') or obj.get('status')!='ready':raise ValueError('Incomplete generated mission')
    if not recovery_only and obj['schema']==SCHEMA and obj.get('return_file') not in obj['files']:raise ValueError('Prepared return is missing from mission identities')
    for name,sha in obj['files'].items():
        if recovery_only and name==obj.get('return_file'):continue  # Recompute, never trust a missing/damaged cache.
        if digest(name)!=sha:raise ValueError('Generated input/route changed: '+name)
    return obj


def load(path,*,recovery_only=False):
    obj=verify(path,recovery_only=recovery_only)
    curves={s:Curve(json.loads(Path(p).read_text())) for s,p in obj['plans'].items()}
    for curve in curves.values():validate_timing(curve);validate_drive_speed(curve,1.)
    if obj.get('return_file') and not recovery_only:
        obj['return_plan']=json.loads(Path(obj['return_file']).read_text())
        cache=obj['return_plan']
        if (cache.get('status')!='ready' or [s['side'] for s in cache['steps']]!=['right','left']
                or any(not p.get('all_valid') for p in cache['proofs'].values())):
            raise ValueError('Incomplete prechecked return')
    return obj,curves


def build_return_cache(obj,curves,folder):
    """Plan/check the entire normal return while the initial planner is running."""
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    current={s:c.at(c.duration) for s,c in curves.items()};steps=[];proofs={}
    for side in ('right','left'):
        curve=curves[side];spec=recovery(curve,current[side],curve.duration,.5)
        proofs[side]=_check_return_step(obj,side,current,recovery_samples(curve,spec),folder/side)
        steps.append(dict(kind='prefix',side=side,spec=spec,duration=spec['bridge_s']+spec['return_s']))
        current[side]=curve.at(0)
    result=dict(status='ready',steps=steps,proofs=proofs,order=['right','left'],
                duration_s=sum(s['duration'] for s in steps),motor_commands=0)
    atomic_json(folder/'return.json',result);return result


def cached_return(obj,curves,held,progress):
    """Use the checked return, or its already-traveled prefix. No native worker.

    At a partial stop only the time law is cropped using the existing bounded
    recovery mathematics; all geometry is a subset of the upfront proof. No
    off-path connector or arbitrary strike-joint retraction is invented here.
    """
    from drum_planner.execution import TRACK_L1
    cache=obj.get('return_plan')
    if cache is None:raise ValueError('No prepared return; a checked return must be calculated')
    if progress['right']>0 and progress['left']!=curves['left'].duration:
        raise ValueError('Return sequence does not match the checked opposite-arm poses')
    steps=[]
    for cached in cache['steps']:
        side=cached['side'];curve=curves[side];tau=float(progress[side]);q=np.asarray(held[side],float)
        if not np.isfinite(tau) or not 0<=tau<=curve.duration:raise ValueError('Invalid traveled progress')
        if q.shape!=(7,) or not np.isfinite(q).all() or np.sum(abs(q-curve.at(tau)))>TRACK_L1:
            raise ValueError(side+' is away from the prepared return; a measured-start return must be checked')
        if tau==0:continue
        if tau==curve.duration:steps.append(copy.deepcopy(cached))
        else:
            # Exact prefix of the cached geometry; deliberately use its nominal
            # anchor, not an unvalidated connector through a measured deviation.
            spec=recovery(curve,curve.at(tau),tau,.5)
            steps.append(dict(kind='prefix',side=side,spec=spec,duration=spec['bridge_s']+spec['return_s']))
    return dict(status='ready',held=held,progress=progress,steps=steps,motor_commands=0,source='upfront_checked_return')


def check_return(folder,mission_path,held,progress):
    """Explicit Return fallback: checked measured connectors, then fresh routes.

    Runs in the bounded planning child while the GUI retains powered Hold.
    Uses the original frozen scene/mask and captured rest, never a generic center.
    Missing return-cache files do not invalidate intact scene/outbound inputs.
    """
    config.no_can();folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    obj,curves=load(mission_path,recovery_only=True)
    current={s:np.asarray(held[s],float) for s in curves}
    if any(q.shape!=(7,) or not np.isfinite(q).all() for q in current.values()):
        raise ValueError('Return requires seven finite measured joints on both arms')
    steps=[]; proofs={}
    try:
        for side in ('right','left'):
            spec=recovery(curves[side],current[side],float(progress[side]),.5)
            samples=recovery_samples(curves[side],spec)
            proofs[side]=_check_return_step(obj,side,current,samples,folder/('prefix-'+side))
            steps.append(dict(kind='prefix',side=side,spec=spec,duration=spec['bridge_s']+spec['return_s']))
            current[side]=curves[side].at(0)
        source='new_measured_prefix_check'
    except ValueError as exc:
        atomic_json(folder/'prefix_unavailable.json',dict(reason=str(exc)))
        print('RETURN: checked prefix unavailable; calculating RIGHT then LEFT from measured Hold to captured rest.',flush=True)
        steps,proofs=_plan_return(obj,curves,held,folder/'replanned')
        source='fresh_workspace_return'
    result=dict(status='ready',held=held,progress=progress,steps=steps,proofs=proofs,
                motor_commands=0,source=source,order=['right','left'])
    atomic_json(folder/'return.json',result);return result


def _plan_return(obj,curves,held,folder):
    """Use the existing planner/timer for off-path holds; no shortcut commands."""
    from drum_planner.common import arm_joints
    current={s:np.asarray(held[s],float) for s in curves};steps=[];proofs={}
    acquired=Path(obj['capture'])/'measured_start.json'
    scene=Path(obj['capture'])/'camera/scene.json'
    for side in ('right','left'):
        output=Path(folder)/side; output.mkdir(parents=True)
        state=copy.deepcopy(obj['original'])
        state.update({s+'_rad':q.tolist() for s,q in current.items()})
        state.update(source='Measured stationary Hold for explicit return; frozen camera unchanged',measured=True)
        state_path=output/'held_start.json';atomic_json(state_path,state)
        ctx=context(output,state_path,acquired,scene,Path(obj['calibration']),side)
        # Same V/A/jerk limits and conservative half-speed recovery scaling.
        limits=np.minimum(curves[side].limits,motion_settings.DEFAULT_LIMITS)*np.array([.5,.25,.125])[:,None]
        ctx['motion_settings']=motion_settings.settings(side,dict(name='Calculated return to captured rest',
            **dict(zip(motion_settings.FIELDS,limits.tolist()))))
        goal=output/'captured_rest.json'
        atomic_json(goal,dict(schema=f'openarmx-{side}-motion-recording-v1',arm=side,
            joint_order=arm_joints(side),joint_position_unit='rad',model_sha256=digest(ROOT/'model/openarmx.urdf'),
            source='Captured original rest endpoint only, not a guessed center',
            samples=[dict(time_s=0.,positions_rad=obj['original'][side+'_rad'])]))
        ctx.update(goal_recording=str(goal.resolve()),destination_sha256=digest(goal))
        atomic_json(output/'context.json',ctx)
        plan=fast.plan(ctx,output,'workspace_curve',config.settings(),120.)
        if plan['status']!='planned':raise ValueError(side+' return planner found no collision-free route: '+str(plan.get('details')))
        curve=Curve(plan);validate_timing(curve);validate_drive_speed(curve,1.)
        if (not np.allclose(curve.at(0),current[side],atol=1e-10,rtol=0) or
                not np.allclose(curve.at(curve.duration),obj['original'][side+'_rad'],atol=1e-10,rtol=0)):
            raise ValueError('Calculated return endpoints differ from measured Hold/captured rest')
        req=json.loads((output/'backend_request.json').read_text())
        proofs[side]=hard_check(req,curve.at_many(np.r_[np.arange(0,curve.duration,.002),curve.duration]),output/'dense_check')
        steps.append(dict(kind='planned',side=side,plan=plan,duration=curve.duration,
                          start=curve.at(0).tolist(),goal=curve.at(curve.duration).tolist()))
        current[side]=curve.at(curve.duration)
    return steps,proofs


def _check_return_step(obj,side,current,samples,folder):
    req=json.loads((Path(obj['plans'][side]).parent/'backend_request.json').read_text())
    other='right' if side=='left' else 'left'
    req.update(start=current[side].tolist(),goal=samples[-1].tolist())
    req[other]=current[other].tolist()
    if side=='left':req['left']=current[side].tolist()
    connector=None
    # Same measured-Hold allowance as baseline.tasks.check_return: an encoder
    # offset just below the captured J2 rest must be checked, not clipped or
    # rejected against the *command* floor. This is private to this connector;
    # the original model, planned rest, stop boundary and TRACK_L1 are unchanged.
    from drum_planner.joint_position_limits import policy,command_lower,model_lower,RIGHT_J2
    plan=json.loads(Path(obj['plans'][side]).read_text())
    if side=='right' and plan.get('model',{}).get('joint_position_policy')==policy():
        import xml.etree.ElementTree as ET
        command_lower(current[side])
        tree=ET.parse(req['urdf']);limit=tree.getroot().find(f"joint[@name='{RIGHT_J2}']/limit")
        lower=float(limit.get('lower'))
        if current[side][1]<lower:
            if np.min(samples[:,1])<current[side][1]-1e-10:
                raise ValueError('Recovery goes below the measured HOLD pose')
            folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
            limit.set('lower',f'{model_lower(current[side]):.17g}')
            private=folder/'recovery_robot.urdf';tree.write(private,encoding='unicode',xml_declaration=True)
            req['urdf']=str(private.resolve())
            connector=dict(original_lower_rad=lower,held_lower_rad=float(current[side][1]),
                           model_sha256=digest(private),original_plan_unchanged=True)
    proof=hard_check(req,samples,folder)
    if connector is not None:proof['measured_j2_connector']=connector
    return proof


def main():
    import argparse
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['prepare','return']);p.add_argument('--folder',type=Path,required=True)
    p.add_argument('--capture',type=Path);p.add_argument('--calibration',type=Path,default=DEFAULT_CALIBRATION)
    p.add_argument('--input',type=Path)
    a=p.parse_args()
    try:
        if a.action=='prepare':prepare(a.folder,a.capture,a.calibration)
        else:
            data=json.loads(a.input.read_text());check_return(a.folder,data['mission'],data['held'],data['progress'])
    except Exception as exc:
        atomic_json(a.folder/'error.json',dict(error=str(exc),motor_commands=0));raise

if __name__=='__main__':main()
