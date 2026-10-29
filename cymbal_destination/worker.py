"""Bounded offline geometry worker. Outputs ONE destination, never a motion law."""
import argparse,time
from pathlib import Path
from drum_planner.common import DEFAULT_CALIBRATION,JOINTS,ROOT,atomic_json,digest
from drum_planner import planning
from drum_planner.model import generate
from drum_planner import joint_position_limits
from drum_planner.right_zone import constraint
from drum_planner.experiment.config import no_can
from drum_planner.experiment.runtime import Oracle
from . import inputs,solver,output,aim,goal_limits
from .config import VERSION,ANGLE_CONVENTION,AIM_CONVENTION,MAX_INWARD_YAW_DEG


def run(folder,cal_path=DEFAULT_CALIBRATION):
    no_can();folder=Path(folder).resolve();began=time.monotonic()
    if (folder/'destination.json').exists():raise ValueError('Refusing to overwrite a previous destination; use a new run folder')
    result=dict(schema='x5-cymbal-destination-result-v1',version=VERSION,angle_convention=ANGLE_CONVENTION,status='failed',
        aim_convention=AIM_CONVENTION,maximum_inward_yaw_deg=MAX_INWARD_YAW_DEG,
        goal_joint_reserve_policy=goal_limits.policy(),
        joint_position_policy=joint_position_limits.policy(),
        motion_authorized=False,motor_commands=0,hardware_execution_available=False,
        endpoint_only=True,trajectory_generated=False,calibration=str(Path(cal_path).resolve()),
        calibration_sha256=digest(cal_path),
        program_sources_sha256={str(p.relative_to(ROOT)):digest(p) for p in
            [*sorted((ROOT/'cymbal_destination').glob('*.py')),ROOT/'plan_cymbal_destination.sh']})
    oracle=None
    try:
        state,world,target,point,normal=inputs.load(folder/'acquisition',cal_path)
        inward=aim.reference(target,normal)
        manifest=generate(folder/'model',cal_path,reference_right=state['right_rad'])
        req=dict(schema='x5-moveit-request-v1',execute=False,urdf=str(folder/'model/robot.urdf'),
            srdf=str(folder/'model/robot.srdf'),moving_side='right',start=state['right_rad'],
            left=state['left_rad'],left_opening_m=state['left_gripper_opening_m'],goal=state['right_rad'],
            scene=world,scene_robot_start=state['right_rad'],right_zone=constraint(),
            collision_policy=planning.COLLISION_POLICY,world_padding_policy=planning.WORLD_PADDING_POLICY,
            drumstick_self_filter_padding_m=planning.DRUMSTICK_SELF_FILTER_PADDING_M,
            filter_drumstick_depth_shadow=True,occupied_only_preview=False,planning_seconds=8.,seed=19,candidate_count=6)
        joint_position_limits.bind_request(req,manifest)
        atomic_json(folder/'backend_request.json',req)
        result.update(model=manifest,acquired_state=state,target=target,
            request_sha256=digest(folder/'backend_request.json'),
            acquisition_report=str(folder/'acquisition/acquisition_report.json'))
        try:oracle=Oracle(folder/'backend_request.json',folder)
        except RuntimeError as exc:
            log_path=folder/'oracle.log'
            detail=log_path.read_text(errors='replace')[-2500:] if log_path.exists() else ''
            raise RuntimeError(f'Native collision world did not initialize: {exc}\n{detail}') from exc
        log=lambda s:print(f'[Destination +{time.monotonic()-began:5.2f}s] {s}',flush=True)
        log(f'Inward aiming REQUIRED: disc-plane grip→tip yaw within ±{MAX_INWARD_YAW_DEG:g}°; outward poses rejected.')
        winner,report=solver.solve(oracle,req,point,normal,inward,log)
        result['search']=report
        if winner is None:result.update(status='no_feasible_destination',error=report['reason'])
        else:
            destination=output.destination(winner['q'],winner['geometry'],point,normal,cal_path,world['source_scene_sha256'],
                                           joint_bounds=(oracle.lower,oracle.upper))
            atomic_json(folder/'destination.json',destination)
            result['destination_sha256']=digest(folder/'destination.json')
            result.update(status='ready',goal_rad=winner['q'],geometry=winner['geometry'],approach=winner['approach'])
            log(f'READY: tip error {winner["geometry"]["tip_error_m"]*1000:.6f} mm; '
                f'shaft-above elevation {winner["geometry"]["stick_tilt_deg"]:.4f}°; J7 plane error '
                f'{winner["geometry"]["j7_plane_error_deg"]:.6f}°; inward yaw {winner["geometry"]["inward_yaw_deg"]:+.4f}°. Destination ONLY.')
    except BaseException as exc:
        result['error']=str(exc) or type(exc).__name__;raise
    finally:
        if oracle:oracle.close()
        result['elapsed_s']=time.monotonic()-began
        atomic_json(folder/'result.json',result)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--folder',required=True)
    p.add_argument('--calibration',default=str(DEFAULT_CALIBRATION));a=p.parse_args();run(a.folder,a.calibration)
