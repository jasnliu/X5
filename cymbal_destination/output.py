"""One endpoint in X5's destination-compatible container, not a trajectory."""
import json
from pathlib import Path
import numpy as np
from drum_planner.common import JOINTS,ROOT,digest
from drum_planner import joint_position_limits
from .config import VERSION,ANGLE_CONVENTION,AIM_CONVENTION,MAX_INWARD_YAW_DEG
from . import goal_limits


def current_convention(obj,*,allow_legacy_joint_reserve=False):
    # An already prepared v4/v5/v6/v7 hardware mission must remain readable for reverse
    # recovery. New generation, standalone opening/export and DT1 job ingestion
    # never opt into this. No v4 goal is reused by a fresh Calculate.
    legacy=(allow_legacy_joint_reserve and obj.get('version')=='cymbal-prime-destination-v4'
            and 'goal_joint_reserve_policy' not in obj)
    legacy_v5=(allow_legacy_joint_reserve and obj.get('version')=='cymbal-prime-destination-v5')
    legacy_v6=(allow_legacy_joint_reserve and obj.get('version')=='cymbal-prime-destination-v6')
    legacy_v7=(allow_legacy_joint_reserve and obj.get('version')=='cymbal-prime-destination-v7')
    old=legacy or legacy_v5 or legacy_v6 or legacy_v7
    expected_yaw=20. if old else MAX_INWARD_YAW_DEG
    if ((obj.get('version')!=VERSION and not old) or obj.get('angle_convention')!=ANGLE_CONVENTION or
        obj.get('aim_convention')!=AIM_CONVENTION or obj.get('maximum_inward_yaw_deg')!=expected_yaw or
        (not legacy and obj.get('goal_joint_reserve_policy')!=(goal_limits.legacy_policy() if legacy_v5 else goal_limits.v2_policy() if legacy_v6 else goal_limits.policy())) or
        (not legacy and not legacy_v5 and not legacy_v6 and obj.get('joint_position_policy')!=joint_position_limits.policy())):
        raise ValueError('Saved destination constraints are obsolete (angle/inward aiming/joint reserve/right J2 limit). Recalculate in a new run using the saved pair and pose; original data are preserved.')
    if not legacy and not legacy_v5 and not legacy_v6 and 'goal_joint_reserve' in obj:
        joint_position_limits.require_goal_bounds(obj['goal_joint_reserve'].get('model_lower_rad',[]))


def destination(q,proof,point,normal,cal_path,scene_sha256,*,joint_bounds):
    from .solver import acceptable
    q=np.asarray(q,float)
    if q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('Expected seven finite destination angles')
    if not acceptable(proof):raise ValueError('Destination geometry does not satisfy the current angle convention')
    joint_position_limits.require_goal_bounds(joint_bounds[0])
    reserve=goal_limits.require(q,*joint_bounds)
    return dict(schema='openarmx-right-motion-recording-v1',arm='right',version=VERSION,angle_convention=ANGLE_CONVENTION,
        aim_convention=AIM_CONVENTION,maximum_inward_yaw_deg=MAX_INWARD_YAW_DEG,
        goal_joint_reserve_policy=goal_limits.policy(),goal_joint_reserve=reserve,
        joint_position_policy=joint_position_limits.policy(),
        capture_kind='destination',source='Generated software-only cymbal prime endpoint; NOT an encoder capture',
        coordinate_convention='encoder_to_joint',model_sha256=digest(ROOT/'model/openarmx.urdf'),
        joint_order=JOINTS,joint_position_unit='rad',time_unit='s',duration_s=0.,sample_count=1,
        samples=[dict(time_s=0.,positions_rad=q.tolist())],
        motion_authorized=False,hardware_execution_available=False,endpoint_only=True,
        target_geometry=proof,target_world_m=np.asarray(point).tolist(),disc_normal_world=np.asarray(normal).tolist(),
        calibration_sha256=digest(cal_path),source_scene_sha256=scene_sha256,
        note='ONE destination sample, not a playable recording or a validated strike')


def export(source,path,expected_hash):
    source=Path(source)
    if digest(source)!=expected_hash:raise ValueError('Destination changed after validation')
    data=source.read_bytes();obj=json.loads(data)
    current_convention(obj)
    if (obj.get('capture_kind')!='destination' or obj.get('sample_count')!=1 or len(obj.get('samples',[]))!=1 or
        obj.get('duration_s')!=0 or obj.get('motion_authorized') is not False):
        raise ValueError('Expected a single non-executable destination')
    goal_limits.verify(obj['samples'][0]['positions_rad'],obj.get('goal_joint_reserve'))
    path=Path(path)
    with path.open('xb') as f:f.write(data)
    return path


def verified(folder):
    """Bind a saved preview to its frozen payloads before UI or headless opening."""
    from . import inputs,aim
    from .solver import Kinematics,geometry,acceptable
    folder=Path(folder).resolve();r=json.loads((folder/'result.json').read_text())
    current_convention(r)  # Includes old failures: never repeat an obsolete sign diagnosis.
    if (r.get('schema')!='x5-cymbal-destination-result-v1' or r.get('endpoint_only') is not True or
        r.get('motion_authorized') is not False or r.get('hardware_execution_available') is not False):
        raise ValueError('Expected a non-executable destination preview')
    state,world,target,green,normal=inputs.load(folder/'acquisition',r['calibration'])
    req=json.loads((folder/'backend_request.json').read_text())
    if digest(folder/'backend_request.json')!=r['request_sha256'] or digest(req['urdf'])!=r['model']['calibrated_urdf_sha256']:
        raise ValueError('Preview model/request changed')
    if req['scene']['source_scene_sha256']!=world['source_scene_sha256'] or req['start']!=state['right_rad'] or req['left']!=state['left_rad']:
        raise ValueError('Preview does not match its frozen scene/poses')
    if r['status']=='ready':
        if digest(folder/'destination.json')!=r['destination_sha256']:raise ValueError('Destination file changed')
        saved=json.loads((folder/'destination.json').read_text())
        current_convention(saved)
        if saved['samples'][0]['positions_rad']!=r['goal_rad']:raise ValueError('Displayed/saved destination mismatch')
        kin=Kinematics(req['urdf'],'right');q=np.asarray(r['goal_rad'])
        inward=aim.reference(target,normal);proof=geometry(kin,q,green,normal,inward)
        if np.any(q<kin.lower) or np.any(q>kin.upper) or not acceptable(proof):
            raise ValueError('Saved destination no longer satisfies the target constraints')
        goal_limits.verify(q,saved.get('goal_joint_reserve'),(kin.lower,kin.upper))
        for recorded in (r['geometry'],saved['target_geometry']):
            if (recorded.get('angle_convention')!=ANGLE_CONVENTION or not acceptable(recorded) or
                not np.allclose(recorded['inward_reference_world'],inward,rtol=0,atol=1e-10) or any(
                not np.isclose(recorded[key],proof[key],rtol=1e-8,atol=1e-8) for key in
                ('tip_error_m','stick_tilt_deg','j7_plane_error_deg','negative_j7_toward_disc_cosine',
                 'inward_yaw_deg','inward_cosine','stick_plane_projection_norm'))):
                raise ValueError('Saved geometry labels differ from the current model calculation')
    return r,req,state,world,target,green,normal
