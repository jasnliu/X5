"""Bounded, disposable acquisition/planning/check workers. Never motor commands."""
import argparse,copy,json,time
from pathlib import Path
import numpy as np
from ..common import DEFAULT_CALIBRATION,ROOT,JOINTS,arm_side,arm_joints,goal_recording,atomic_json,digest
from .. import planning
from ..execution import TRACK_L1
from .curve import Curve
from . import planner as route_planner
from ..right_zone import constraint,metadata
from .paths import validate_timing,recovery,recovery_samples
from .arms import side_of,other_side,passive_snapshot


def acquire(folder,cal_path,*,moving_side='right'):
    side=arm_side(moving_side)
    from ..current_plan import capture_current,verify_unchanged
    from ..read_only import Telemetry
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    worker=Telemetry(folder)
    try:
        print(f'READ ONLY: measuring stationary {side} arm + passive {other_side(side)} pose + five depth frames; NO actuator commands',flush=True)
        state,scene=capture_current(folder,worker,cal_path,stationary_joints=arm_joints(side))
        proof=verify_unchanged(worker,state,stationary_joints=arm_joints(side))
    finally:
        worker.close();atomic_json(folder/'query_report.json',worker.report())
    atomic_json(folder/'acquisition.json',dict(state=str(folder/'measured_start.json'),scene=str(folder/'camera/scene.json'),
        state_sha256=digest(folder/'measured_start.json'),scene_sha256=digest(folder/'camera/scene.json'),
        proof=proof,boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),motor_commands=0))


def make_plan(folder,capture,cal_path,*,moving_side='right',destinations=None):
    side=arm_side(moving_side)
    folder=Path(folder);capture=Path(capture)
    acquisition=json.loads((capture/'acquisition.json').read_text())
    print(f'WORKSPACE CURVE: endpoint-only paths → latest {side.upper()}-arm timing → dense zone/collision check',flush=True)
    # Job/main already own folder for logs and error reports. The planner must
    # own a NEW child directory; retain its refusal to overwrite existing plans.
    output=folder/'result'
    options={} if destinations is None else dict(destinations=destinations)
    result=route_planner.plan(output,capture/'measured_start.json',capture/'camera/scene.json',cal_path,moving_side=side,**options)
    result.update(baseline_hardware_session=True,actual_current_robot_state_verified=True,
                  current_state_check=acquisition['proof'],acquisition_boot_id=acquisition['boot_id'])
    atomic_json(output/'plan.json',result)
    if result['status']!='planned':raise ValueError('Workspace curve found no valid trajectory: '+str(result.get('details',result.get('error'))))
    print('WORKSPACE CURVE READY — preview/prepare only; no motors enabled',flush=True)
    return output/'plan.json'


def load_checked(path,cal_path):
    from .sequence import PlanDisplay
    p=PlanDisplay(path);o=p.obj;r=p.req;side=p.side;other=p.other_side
    if (o.get('status')!='planned' or o.get('synthetic') is not False or o.get('experiment_only') or
        o.get('baseline_hardware_session') is not True or not o.get('actual_current_robot_state_verified') or
        o.get('collision_policy')!=planning.COLLISION_POLICY or r.get('collision_policy')!=planning.COLLISION_POLICY or
        r.get('occupied_only_preview') is not False or o.get('strict_unknown_space') is not False or
        o.get('hold_at_goal_s')!=2. or not (o.get('reverse_checked_on_same_static_scene') or o.get('checkpoint_sequence'))):
        raise ValueError('Requires a newly acquired baseline hardware-session plan, not a demo or experiment cache')
    if o.get('acquisition_boot_id')!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():
        raise ValueError('Plan is from another system boot; acquire again')
    if o.get('calibration_sha256')!=digest(cal_path):
        raise ValueError('Calibration or goal changed; replan')
    if digest(r['srdf'])!=o['model']['srdf_sha256']:raise ValueError('SRDF changed')
    spec=constraint() if side=='right' else constraint(ROOT/'left_zones/zone1.json',side='left')
    if r.get(side+'_zone')!=spec or o.get(side+'_zone_constraint')!=metadata(spec):raise ValueError('Arm zone/reference model changed; replan')
    if digest(o['provenance']['state'])!=o['provenance']['state_sha256']:raise ValueError('Starting state changed')
    state=json.loads(Path(o['provenance']['state']).read_text())
    if not state.get('measured') or state[side+'_rad']!=r['start'] or state[other+'_rad']!=r[other]:
        raise ValueError('Plan does not match the acquired joints')
    world=planning.scene(o['provenance']['scene'],cal_path)
    if world['source_scene_sha256']!=r['scene']['source_scene_sha256']:raise ValueError('Scene changed')
    curve=Curve(o);validate_timing(curve)
    from ..common import endpoint
    from ..experiment import destinations,checkpoints
    bindings=o.get('destination_bindings')
    if bindings:
        if not 1<=len(bindings)<=3:raise ValueError('Invalid selected destination count')
        goals=[destinations.goal(dict(b,moving_side=side))[1] for b in bindings]
        if o['recording_sha256']!=bindings[-1]['destination_sha256'] or not np.array_equal(r['goal'],goals[-1]):
            raise ValueError('Selected destination changed; Calculate again')
        if len(bindings)>1:
            seq=o.get('checkpoint_sequence',{})
            if o.get('checkpoint_sha256')!=checkpoints.signature(bindings) or [s['goal'] for s in seq.get('segments',[])]!=goals:
                raise ValueError('Checkpoint selection/sequence mismatch')
        elif o.get('checkpoint_sequence'):raise ValueError('Unexpected checkpoint sequence')
        goal=goals[-1]
    else:
        goal=endpoint(side=side)
        if o.get('recording_sha256')!=digest(goal_recording(side)):raise ValueError('Goal changed; replan')
    goal_time=o['duration_outward_s'] if curve.parts else curve.duration
    if (not np.allclose(curve.at(0),r['start'],atol=1e-10,rtol=0) or
        not np.allclose(curve.at(goal_time),goal,atol=1e-10,rtol=0) or
        (curve.parts and not np.allclose(curve.at(curve.duration),r['start'],atol=1e-10,rtol=0))):
        raise ValueError('Curve endpoints changed')
    import xml.etree.ElementTree as ET
    root=ET.parse(r['urdf']).getroot()
    r['execution_joint_limits']=[[float(root.find(f"joint[@name='{j}']/limit").get(k)) for k in ('lower','upper')] for j in arm_joints(side)]
    return o,r,curve


def margin_contract(req):
    """Use Baseline's requested margins unchanged, including native defaults.

    Zero additional self-clearance does NOT disable self-collision checking.
    Native voxel/edge allowances and the complete arm/stick bodies stay intact.
    The older execution module's separate margins are deliberately unaffected.
    """
    world=float(req['scene']['obstacle_padding_m'])
    self_gap=float(req.get('minimum_self_clearance_m',0.))
    if not np.isfinite(world) or not .02<=world<=.10 or not np.isfinite(self_gap) or not 0<=self_gap<=.10:
        raise ValueError('Invalid Baseline collision margins')
    result=dict(collision_margin_policy='baseline_request',obstacle_padding_m=world,minimum_self_clearance_m=self_gap)
    # Historical packets retain their original proof shape/clearance. New
    # packets bind both validation and playback to the versioned accounting.
    if 'world_padding_policy' in req:result['world_padding']=planning.world_padding_metadata(req)
    return result


class PathBlocked(ValueError):
    def __init__(self,details):
        self.details=details
        super().__init__('Baseline collision/zone check failed; no motion: '+str(details))


def hard_check(req,q,folder):
    margins=margin_contract(req)
    check=copy.deepcopy(req)
    check.update(check_states=np.asarray(q).tolist(),check_path=True,stop_on_invalid=True)
    allowance=('5 mm edge allowance; occupied voxel volumes counted once'
               if req.get('world_padding_policy')=='occupied_volume_v1' else 'native voxel/edge allowances')
    print(f"Checking {len(q):,} exact-curve samples + edges: {side_of(req)} arm zone, full arm/stick, Baseline collision margins "
          f"(world {margins['obstacle_padding_m']*1000:g} mm + {allowance}; "
          f"extra self-clearance {margins['minimum_self_clearance_m']*1000:g} mm)",flush=True)
    result=planning.backend(check,folder,'checked_curve')
    if 'world_padding' in margins and result.get('world_padding')!=margins['world_padding']:
        raise ValueError('Collision backend world-padding proof differs from request; no motion')
    rows=result.get('checks',[])
    if (result.get('status')!='checked' or result.get('collision_policy')!=planning.COLLISION_POLICY or
        result.get('strict_unknown_space') is not False or len(rows)!=len(q) or not all(x.get('valid') is True for x in rows)):
        bad=[dict(x,sample_index=i) for i,x in enumerate(rows) if x.get('valid') is False]
        if (result.get('status')=='checked' and result.get('collision_policy')==planning.COLLISION_POLICY and
            result.get('strict_unknown_space') is False and bad and
            all(x.get('reason') in ('collision','outside_right_zone3','outside_left_zone') for x in bad)):
            raise PathBlocked(bad[:2])
        raise ValueError('Incomplete/invalid Baseline collision check; no motion: '+str(bad[:2] or result.get('error',result.get('status'))))
    return dict(samples=len(q),all_valid=True,**margins)


def prepare(folder,path,capture,cal_path,*,allow_replan=True):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    o,r,c=load_checked(path,cal_path)
    current=json.loads((Path(capture)/'measured_start.json').read_text())
    if np.sum(abs(np.asarray(current[side_of(r)+'_rad'])-r['start']))>TRACK_L1:
        raise ValueError('Arm changed since preview; capture/replan instead of moving to the old start')
    # Passive motion is not a start-position veto. Collision checking uses the
    # acquired passive pose in the plan, just as it uses the frozen depth scene.
    world=planning.scene(Path(capture)/'camera/scene.json',cal_path)
    r['scene']=world
    times=np.r_[np.arange(0,c.duration,.002),c.duration]
    replanned=None
    try:proof=hard_check(r,[c.at(t) for t in times],folder)
    except PathBlocked as exc:
        if not allow_replan:raise ValueError(str(exc)+' — click Calculate trajectory to replace this route') from exc
        # A new snapshot may contain newly occupied voxels next to an old
        # route. Keep every collision/margin; replace the ROUTE, not the scene.
        # Only one bounded replan, no automatic retry loop or motor access.
        replanned=dict(previous_plan=str(Path(path).resolve()),failures=exc.details,
                       first_blocked_time_s=float(times[exc.details[0]['sample_index']]),
                       explicit_start_required=True)
        atomic_json(folder/'replan_reason.json',replanned)
        print(f"Fresh scene blocks the old path at {replanned['first_blocked_time_s']:.3f}s. "
              'Generating a replacement Baseline route; collision margins unchanged. NO motion.',flush=True)
        binding=None
        if o.get('destination_bindings'):
            from ..experiment.checkpoints import signature
            bindings=o['destination_bindings'];binding=dict(bindings[-1])
            if len(bindings)>1:binding.update(checkpoints=bindings,checkpoint_sha256=signature(bindings))
        options={} if binding is None else dict(destinations=binding)
        path=make_plan(folder/'replanned',capture,cal_path,moving_side=side_of(r),**options)
        o,r,c=load_checked(path,cal_path)
        times=np.r_[np.arange(0,c.duration,.002),c.duration]
        proof=hard_check(r,[c.at(t) for t in times],folder/'replacement_check')
    packet=dict(schema='x5-baseline-prepared-v2',plan=str(Path(path).resolve()),plan_sha256=digest(path),
        request=r,object=o,
        calibration=str(Path(cal_path).resolve()),calibration_sha256=digest(cal_path),
        fresh_scene=str((Path(capture)/'camera/scene.json').resolve()),fresh_scene_sha256=world['source_scene_sha256'],
        scene_stamp=world['frames'][-1]['received_monotonic'],proof=proof,
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),motion_authorized=False)
    if replanned:packet['replanned']=replanned
    atomic_json(folder/'prepared.json',packet)
    print('Automatic scene/path checks complete. This worker cannot move motors; only an explicit UI Start request can authorize execution.',flush=True)


def check_packet(path):
    """Final data-integrity gate; frozen scene has no wall-clock expiration."""
    packet=json.loads(Path(path).read_text());r=packet['request'];o=packet['object']
    if packet.get('schema')!='x5-baseline-prepared-v2' or packet.get('motion_authorized') is not False:raise ValueError('Invalid calculation packet; click Calculate again')
    if packet['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():raise ValueError('Calculation is from another system boot; click Calculate again')
    if digest(packet['plan'])!=packet['plan_sha256'] or digest(packet['calibration'])!=packet['calibration_sha256']:
        raise ValueError('Plan/calibration changed after preparation')
    original,req,c=load_checked(packet['plan'],packet['calibration'])
    if original!=o:raise ValueError('Prepared curve differs from preview')
    world=planning.scene(packet['fresh_scene'],packet['calibration'])
    req['scene']=world
    if req!=r or world['source_scene_sha256']!=packet['fresh_scene_sha256'] or packet['scene_stamp']!=world['frames'][-1]['received_monotonic']:
        raise ValueError('Prepared request/scene changed')
    # Frozen-scene workflow: waiting to inspect a preview does not expire its
    # geometry/proof. Calculate explicitly refreshes it. Boot/hash/proof and
    # measured selected-arm start checks still bind execution to this exact plan.
    proof=packet.get('proof',{})
    if (proof.get('all_valid') is not True or proof.get('samples')!=len(np.arange(0,c.duration,.002))+1 or
        any(proof.get(k)!=v for k,v in margin_contract(req).items())):
        raise ValueError('Missing or mismatched Baseline collision proof; click Calculate again')
    return packet,c


def check_return(folder,prepared,held,tau,rate,left_snapshot=None,*,right_snapshot=None):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    packet=json.loads(Path(prepared).read_text());o,r,c=load_checked(packet['plan'],packet['calibration'])
    # Return is operator-authorized against the same static scene; never reject
    # solely because time passed while holding. No claim of live perception.
    world=planning.scene(packet['fresh_scene'],packet['calibration'])
    if world['source_scene_sha256']!=packet['fresh_scene_sha256']:raise ValueError('Recovery scene changed')
    r['scene']=world
    # The other arm stays query-only. Its CURRENT measured pose participates
    # in the native whole-robot collision check; no stale-passive-pose bypass.
    other=other_side(side_of(r))
    snapshot=left_snapshot if other=='left' else right_snapshot
    if (other=='left' and right_snapshot is not None) or (other=='right' and left_snapshot is not None):
        raise ValueError('Recovery snapshot is for the wrong passive arm')
    snapshot=passive_snapshot(r) if snapshot is None else snapshot
    passive=np.asarray(snapshot.get(other),dtype=float)
    if passive.shape!=(7,) or not np.isfinite(passive).all():raise ValueError('Invalid measured passive-arm recovery snapshot')
    r[other]=passive.tolist()
    if other=='left':
        opening=float(snapshot.get('left_opening_m',float('nan')))
        if not np.isfinite(opening) or not 0<=opening<=.044:raise ValueError('Invalid measured left-arm recovery snapshot')
        r['left_opening_m']=opening
    spec=recovery(c,held,tau,rate);q=recovery_samples(c,spec)
    spec[other+'_snapshot']=passive_snapshot(r)
    spec['proof']=hard_check(r,q,folder);spec['plan_sha256']=digest(packet['plan'])
    atomic_json(folder/'return.json',spec)
    print('Traversed-prefix recovery checked; controller must recheck fresh stationary feedback before moving',flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['capture','plan','prepare','return'])
    p.add_argument('--folder',required=True);p.add_argument('--capture');p.add_argument('--plan')
    p.add_argument('--side',choices=['right','left'],default='right');p.add_argument('--input');p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION));a=p.parse_args()
    if a.action!='capture':
        from ..experiment.config import no_can
        no_can()
    folder=Path(a.folder);folder.mkdir(parents=True,exist_ok=True)
    try:
        if a.action=='capture':acquire(folder,a.calibration,moving_side=a.side)
        elif a.action=='plan':make_plan(folder,a.capture,a.calibration,moving_side=a.side)
        elif a.action=='prepare':prepare(folder,a.plan,a.capture,a.calibration)
        else:
            x=json.loads(Path(a.input).read_text());check_return(folder,x['prepared'],x['held'],x['tau'],x['rate'],x.get('left_snapshot'),right_snapshot=x.get('right_snapshot'))
    except Exception as e:
        atomic_json(folder/'error.json',dict(error=str(e),motor_commands=0));raise

if __name__=='__main__':main()
