"""Current hardware calculation adapter: the SAME numerical engine as experiment.

Never imports an experiment result/eligibility. Calculates anew from this
Calculate's measured inputs; tasks.prepare independently certifies the exact
curve before the existing motor owner can accept an explicit Start.
"""
import time
from pathlib import Path
from ..common import atomic_json,digest,read_state,goal_recording
from ..right_zone import constraint,metadata
from ..experiment import fast,config,checkpoints,rgbd
from . import settings,targets


def plan(folder,state_path,scene_path,cal_path,*,moving_side='right',destinations=None,
         simulation=False,motion_profile=None):
    folder=Path(folder).resolve();folder.mkdir(parents=True,exist_ok=False);began=time.monotonic()
    state_path=Path(state_path).resolve();scene_path=Path(scene_path).resolve();cal_path=Path(cal_path).resolve()
    state=read_state(state_path)
    if not simulation and not state.get('measured'):raise ValueError('Hardware calculation requires measured joints')
    rgbd.scene(scene_path,cal_path)  # Verified five-frame scene; no input fallback.
    profile=settings.settings(moving_side,motion_profile)
    spec=constraint(side=moving_side) if moving_side=='right' else constraint(
        Path(__file__).resolve().parents[2]/'left_zones/zone1.json',side='left')
    context=dict(state=str(state_path),state_sha256=digest(state_path),scene=str(scene_path),
        scene_sha256=digest(scene_path),calibration=str(cal_path),moving_side=moving_side,
        acquired_state=str(state_path),acquired_state_sha256=digest(state_path),
        scene_robot_start=state[moving_side+'_rad'],experiment_start=dict(kind='default',label='Measured start'),
        motion_settings=settings.for_optimizer(profile),zone_path=spec['zone_file'],
        pool_root=str(folder/'candidate_pool'),**{moving_side+'_zone':metadata(spec)})
    binding=destinations or checkpoints.freeze([goal_recording(moving_side)],moving_side,folder)
    binding,dt1=targets.resolve(context,binding,folder/'dt1')
    context.update(binding);atomic_json(folder/'calculation_context.json',context)
    print('CURRENT HARDWARE PLANNER: experiment-equivalent workspace recovery + '+profile['timing_quality']+
          ' per-joint CPU timing. Calculation only; no motor commands.',flush=True)
    # Reuse exact geometry, ranking, timing and final checks, not a fork of them.
    result=fast.plan(context,folder,'workspace_curve',config.settings(),120.)
    result['route_planner']=result.pop('experiment')
    result['route_planner']['state_source']=('Explicit offline replay; not physical readiness' if simulation else
        'This Calculate measured snapshot; computation child has no CAN access')
    result.update(hardware_planning_pipeline=settings.PIPELINE,hardware_motion_profile=profile,
        hardware_motion_profile_sha256=settings.identity(profile),
        destination_bindings=binding.get('checkpoints',[binding]),planning_wall_s=time.monotonic()-began,
        experiment_only=bool(simulation),offline_simulation=bool(simulation),
        actual_current_robot_state_verified=False,hardware_execution_available=False,motion_authorized=False)
    result['physical_execution_blockers']=['Calculate acquisition and independent exact-curve preparation required; explicit Start only']
    if dt1:result['dt1_generation']=dt1
    if result['status']=='planned':settings.plan_limits(result)
    atomic_json(folder/'plan.json',result)
    return result
