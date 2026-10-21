"""File-based MoveIt orchestration; all outputs are non-executable previews."""
import copy
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import time

import numpy as np
from scipy.interpolate import BPoly, PPoly, make_interp_spline

from .common import (ROOT, atomic_json, calibration, endpoint, read_state, digest,
                     DEFAULT_CALIBRATION, DEFAULT_RECORDING)
from .depth import check_metadata
from .model import generate

COLLISION_POLICY = 'observed_obstacles'
DEFAULT_CANDIDATES = 6
MAX_CANDIDATES = 32


def candidate_count(value):
    """Validate before model generation or any live device ownership."""
    try:
        valid=not isinstance(value,bool) and str(value)==str(int(value)) and 1<=int(value)<=MAX_CANDIDATES
    except (TypeError,ValueError,OverflowError):
        valid=False
    if not valid:
        raise ValueError(f'Candidate count must be an integer from 1 to {MAX_CANDIDATES}')
    return int(value)


def build_moveit():
    folder=ROOT/'drum_planner/build/moveit'
    signature='\n'.join(digest(ROOT/'drum_planner/cpp'/name) for name in ('moveit_plan.cpp','CMakeLists.txt'))
    stamp=folder/'source_signature.txt'
    if (folder/'moveit_plan').is_file() and stamp.is_file() and stamp.read_text()==signature:
        return folder/'moveit_plan'
    subprocess.run(['cmake','-S',str(ROOT/'drum_planner/cpp'),'-B',str(folder),
                    '-DCMAKE_BUILD_TYPE=Release','-DPython3_EXECUTABLE=/usr/bin/python3'],check=True,timeout=120,
                   stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    subprocess.run(['cmake','--build',str(folder),'-j2'],check=True,timeout=180,
                   stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    stamp.write_text(signature)
    return folder/'moveit_plan'


def scene(path, cal_path=DEFAULT_CALIBRATION):
    path=Path(path).resolve(); data=json.loads(path.read_text())
    if data.get('schema')!='x5-native-depth-scene-v1' or data.get('synthetic') is not False:
        raise ValueError('Real planning scenes must come from the Gemini depth capture; no manually defined obstacles')
    if data['calibration_sha256']!=digest(cal_path): raise ValueError('Scene uses a different calibration')
    check_metadata(data['camera'],calibration(cal_path))
    if not np.allclose(data['T_base_depth'],calibration(cal_path)['T_base_depth'],atol=1e-10):
        raise ValueError('Scene transform differs from calibration')
    if data['voxel_size_m'] != .01 or data['obstacle_padding_m'] < .02:
        raise ValueError('Unsupported/unsafe scene voxelization or buffer')
    for name, sha in data['files_sha256'].items():
        file=(path.parent/name).resolve()
        if not file.is_relative_to(path.parent) or digest(file)!=sha: raise ValueError('Scene file path/hash mismatch')
    for field in ('points_file','free_depth_file'):
        name=data[field]
        if name not in data['files_sha256']: raise ValueError('Unhashed scene input')
        data[field]=str((path.parent/name).resolve())
    if Path(data['points_file']).stat().st_size != data['point_count']*12:
        raise ValueError('Scene point count mismatch')
    if Path(data['free_depth_file']).stat().st_size!=1280*800*4:
        raise ValueError('Scene depth dimensions mismatch')
    data['source_scene_sha256']=digest(path)
    return data


def request(folder, start, world, cal_path=DEFAULT_CALIBRATION, occupied_only=False,
            candidates=DEFAULT_CANDIDATES):
    candidates=candidate_count(candidates)
    folder=Path(folder).resolve()
    model=generate(folder/'model',cal_path)
    return dict(schema='x5-moveit-request-v1',execute=False,urdf=str(folder/'model/robot.urdf'),
                srdf=str(folder/'model/robot.srdf'),start=start['right_rad'],left=start['left_rad'],
                left_opening_m=start['left_gripper_opening_m'],goal=endpoint().tolist(),scene=world,
                collision_policy=COLLISION_POLICY,
                occupied_only_preview=bool(occupied_only),planning_seconds=8.,seed=19,
                candidate_count=candidates),model


def backend(req, folder, name='backend'):
    folder=Path(folder).resolve(); folder.mkdir(parents=True,exist_ok=True)
    atomic_json(folder/(name+'_request.json'),req)
    binary=build_moveit()
    with (folder/(name+'.log')).open('wb') as log:
        result=subprocess.run([str(binary),str(folder/(name+'_request.json')),str(folder/(name+'_result.json'))],
                              stdout=log,stderr=log,timeout=120)
    path=folder/(name+'_result.json')
    if not path.is_file(): raise RuntimeError('MoveIt backend produced no report; see '+str(folder/(name+'.log')))
    obj=json.loads(path.read_text())
    if result.returncode not in (0,1): raise RuntimeError('MoveIt backend crashed; see '+str(folder/(name+'.log')))
    return obj


def extrema(pp, derivative):
    # Same polynomial-extrema method used by smooth_playback.trajectory.extrema,
    # kept device-independent rather than importing playback/control modules.
    p=pp.derivative(derivative); roots=p.derivative().roots(extrapolate=False)
    roots=roots[np.isfinite(roots)&(roots>=pp.x[0])&(roots<=pp.x[-1])]
    return float(np.max(abs(p(np.r_[pp.x,roots]))))


def smooth_samples(waypoints, stop_at_corners=False):
    """Explicit quintic interpolation, exact extrema limits, 100 Hz preview.

    Time scaling preserves the geometric path. The resulting path is collision
    checked again; this does NOT imply a future motor controller tracks it.
    """
    q=np.asarray(waypoints,dtype=float)
    if q.ndim!=2 or q.shape[1]!=7 or len(q)<2 or not np.isfinite(q).all():
        raise ValueError('Invalid MoveIt geometric path')
    q=q[np.r_[True,np.max(abs(np.diff(q,axis=0)),axis=1)>1e-10]]
    if len(q)<2:raise ValueError('Start already equals goal; no movement plan required')
    t=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(q,axis=0),axis=1))]
    t/=t[-1]
    if stop_at_corners:
        curves=[PPoly.from_bernstein_basis(BPoly.from_derivatives(t,[[v,0.,0.] for v in q[:,j]])) for j in range(7)]
    else:
        # Reuse X5's endpoint-exact quintic spline approach, but apply it to the
        # collision-planned route, not to the recorded route. Only the final
        # record3 pose is a target. No safe-zone dependency is introduced.
        curves=[PPoly.from_spline(make_interp_spline(t,q[:,j],k=5,
                  bc_type=([(1,0.),(2,0.)],[(1,0.),(2,0.)]))) for j in range(7)]
    peaks=np.array([[extrema(p,k) for p in curves] for k in (1,2,3)])
    scale=max(1.,peaks[0].max()/.6,np.sqrt(peaks[1].max()/1.2),np.cbrt(peaks[2].max()/6.))*1.001
    duration=float(t[-1]*scale)
    if duration>120: raise ValueError('Trajectory would be impractically slow; rejected instead of creeping')
    grid=np.r_[np.arange(0,duration,.01),duration]
    values=np.column_stack([p(grid/scale) for p in curves]);values[0]=q[0];values[-1]=q[-1]
    velocities=np.column_stack([p.derivative()(grid/scale)/scale for p in curves])
    accelerations=np.column_stack([p.derivative(2)(grid/scale)/scale**2 for p in curves])
    velocities[[0,-1]]=0.;accelerations[[0,-1]]=0.
    points=[dict(time_s=float(time),positions_rad=pos.tolist(),velocities_rad_s=vel.tolist(),accelerations_rad_s2=acc.tolist())
            for time,pos,vel,acc in zip(grid,values,velocities,accelerations)]
    # Validate finer than the visualization/control grid, as quintic interpolation
    # need not follow the straight chord between raw MoveIt waypoints.
    fine=np.r_[np.arange(0,duration,.002),duration]
    check=np.column_stack([p(fine/scale) for p in curves]);check[0]=q[0];check[-1]=q[-1]
    return points,check,dict(interpolation='piecewise quintic; collision sampled at <=2ms plus edge subdivision',
                             stop_at_corners=stop_at_corners,
                             time_scale=scale,duration_s=duration,preview_rate_hz=100,
                             peak_velocity_rad_s=(peaks[0]/scale).tolist(),
                             peak_acceleration_rad_s2=(peaks[1]/scale**2).tolist(),
                             peak_jerk_rad_s3=(peaks[2]/scale**3).tolist(),
                             coefficients=[p.c.tolist() for p in curves],knots_unscaled_s=t.tolist(),
                             polynomial_breaks_unscaled_s=[p.x.tolist() for p in curves])


def cycle(outward):
    result=[dict(p,phase='outward') for p in outward]
    finish=outward[-1]['time_s'];zero=[0.]*7
    result.append(dict(time_s=finish+2.,positions_rad=outward[-1]['positions_rad'],
                       velocities_rad_s=zero,accelerations_rad_s2=zero,phase='hold_complete'))
    for point in reversed(outward[:-1]):
        result.append(dict(point,time_s=finish+2.+finish-point['time_s'],phase='return',
                           velocities_rad_s=[-v for v in point['velocities_rad_s']]))
    return result


def plan(folder, state_path, scene_path, cal_path=DEFAULT_CALIBRATION, occupied_only=False,
         candidates=DEFAULT_CANDIDATES):
    candidates=candidate_count(candidates)
    folder=Path(folder).resolve();folder.mkdir(parents=True,exist_ok=False)
    start=read_state(state_path); world=scene(scene_path,cal_path)
    req,manifest=request(folder,start,world,cal_path,occupied_only,candidates)
    return finish_plan(folder,req,manifest,start,cal_path,dict(scene=str(Path(scene_path).resolve()),
                      state=str(Path(state_path).resolve()),state_sha256=digest(state_path)))


def fastest_plan(folder, req):
    """Independent OMPL searches, ranked by FINAL jerk-limited motion duration.

    Each search has its own process/RNG. Two CPU workers bound resource use; no
    ROS controller or device is involved. Smooth and path-preserving variants
    are timed first, then collision checked in ascending duration order. Once a
    variant passes, no slower variant can win, so unnecessary checks are skipped.
    This is best-of-N, NOT a proof of the global time-optimal robot trajectory.
    """
    folder=Path(folder);count=candidate_count(req.get('candidate_count',DEFAULT_CANDIDATES))
    seed=req.get('seed',19)
    if isinstance(seed,bool) or not isinstance(seed,int) or not 1<=seed<=2**32-1:
        raise ValueError('Seed must be a positive uint32 integer')
    began=time.monotonic();rows=[];raw_results=[];variants=[];selected=None
    atomic_json(folder/'backend_request.json',req)

    def search(index):
        trial=copy.deepcopy(req)
        trial['seed']=1+(seed-1+index*7919)%(2**32-1)
        target=folder/'candidates'/f'{index+1:02d}'
        t=time.monotonic();raw=backend(trial,target)
        row=dict(index=index+1,seed=trial['seed'],status=raw['status'],
                 planning_wall_s=time.monotonic()-t,backend_folder=str(target.relative_to(folder)),variants=[])
        return trial,raw,row,target

    def collect(item):
        trial,raw,row,target=item;rows.append(row);raw_results.append(item)
        print(f"Candidate {row['index']}/{count}, seed {row['seed']}: {raw['status']}",flush=True)
        if raw['status']!='planned':
            row['details']=raw.get('details',raw.get('error',raw['status']))
            return
        q=np.asarray(raw.get('path_waypoints'),dtype=float)
        # Never let a malformed backend success change the measured start or
        # the exact record3 joint goal. No clipping or approximate-goal acceptance.
        if (q.ndim!=2 or q.shape[1]!=7 or len(q)<2 or not np.isfinite(q).all()
                or not np.allclose(q[0],req['start'],atol=1e-10,rtol=0)
                or not np.allclose(q[-1],req['goal'],atol=1e-10,rtol=0)):
            row.update(status='invalid_path',details='Invalid geometric path or changed endpoints')
            return
        row.update(waypoint_count=len(q),moveit_seed_duration_s=raw['duration_outward_s'])
        for stop in (False,True):
            variant=dict(stop_at_corners=stop,status='pending_collision_check')
            row['variants'].append(variant)
            try:
                _,_,timing=smooth_samples(q,stop)
            except ValueError as exc:
                variant.update(status='rejected_timing',details=str(exc));continue
            duration=timing['duration_s']
            if not np.isfinite(duration) or duration<=0:
                variant.update(status='rejected_timing',details='Nonfinite or nonpositive duration');continue
            variant.update(duration_outward_s=duration,duration_cycle_s=2*duration+2.)
            variants.append((duration,row['index'],stop,item,variant))

    first=search(0);collect(first)
    # These failures depend on the common input, not on the random search.
    fatal={'blocked_start','blocked_goal','error'}
    if first[1]['status'] not in fatal and count>1:
        # The first call has also finished any needed build, avoiding CMake races.
        with ThreadPoolExecutor(max_workers=2) as pool:
            for item in pool.map(search,range(1,count)):
                collect(item)
    fatal_result=next((item for item in raw_results if item[1]['status'] in fatal),None)
    if fatal_result is None:
        for _,_,stop,item,variant in sorted(variants,key=lambda x:x[:3]):
            trial,raw,row,target=item
            points,check,timing=smooth_samples(raw['path_waypoints'],stop)
            verify=copy.deepcopy(trial);verify['check_states']=check.tolist();verify['check_path']=True
            verify['stop_on_invalid']=True
            print(f"Checking candidate {row['index']}, {'corner-stop' if stop else 'smooth'}: "
                  f"{timing['duration_s']:.3f}s outward",flush=True)
            validation=backend(verify,target,'interpolated_check_'+str(int(stop)))
            checks=validation.get('checks',[])
            failures=[r for r in checks if r.get('valid') is not True]
            passed=(validation.get('status')=='checked' and len(checks)==len(check)
                    and len(check)>1 and not failures
                    and validation.get('collision_policy')==raw.get('collision_policy')
                    and validation.get('strict_unknown_space')==raw.get('strict_unknown_space'))
            variant.update(status='selected' if passed else 'rejected_collision_check',
                           passed=passed,checked_samples=len(checks),failures=failures[:5])
            if not passed and not failures:
                variant['details']='Incomplete or unsuccessful collision-check report: '+str(validation.get('status'))
            if passed:
                selected=(item,points,check,timing);break
    for *_,variant in variants:
        if variant['status']=='pending_collision_check':
            variant['status']='not_checked_slower_or_equal' if selected else 'not_checked_input_error'

    chosen=selected[0] if selected else (fatal_result or first)
    trial,raw,row,target=chosen;result=copy.deepcopy(raw)
    atomic_json(folder/'backend_request.json',trial)
    atomic_json(folder/'backend_result.json',raw)
    if selected:
        _,points,check,timing=selected
        result.update(outward=points,timing=timing,cycle=cycle(points),
                      moveit_seed_duration_s=raw['duration_outward_s'],duration_outward_s=timing['duration_s'],
                      reverse_checked_on_same_static_scene=True,hold_at_goal_s=2.,sampled_interpolated_checks=len(check),
                      final_action='SUPPORTED_REST_THEN_RELAX_INTENT_ONLY; separately armed live execution required')
        result['smoothing_attempts']=[dict(stop_at_corners=v['stop_at_corners'],passed=v['passed'],
            failures=v['failures']) for v in row['variants'] if 'passed' in v]
    elif fatal_result is None and any(item[1]['status']=='planned' for item in raw_results):
        result.update(status='blocked_interpolated_path',
                      details='No candidate has a valid final timed, collision-checked trajectory; see optimization.candidates')
        result.pop('outward',None)
    optimization=dict(method='independent OMPL RRTConnect seeds; best final quintic duration',
        objective='minimum outward duration_s (equivalently 2 * outward + 2 second hold)',
        requested_candidates=count,searched_candidates=len(rows),search_workers=min(2,count),
        candidates=rows,selected_candidate=None if selected is None else row['index'],
        selected_seed=None if selected is None else row['seed'],
        selected_stop_at_corners=None if selected is None else selected[3]['stop_at_corners'],
        selected_duration_s=None if selected is None else selected[3]['duration_s'],
        wall_time_s=time.monotonic()-began,globally_time_optimal=False,
        collision_check_order='ascending final duration; slower/equal unchecked variants cannot beat winner')
    result['optimization']=optimization
    atomic_json(folder/'optimization.json',optimization)
    if selected:
        print(f"Selected candidate {row['index']}/{len(rows)} (seed {row['seed']}): "
              f"{selected[3]['duration_s']:.3f}s outward, {2*selected[3]['duration_s']+2:.3f}s full cycle",flush=True)
    return result


def finish_plan(folder,req,manifest,start,cal_path,provenance):
    result=fastest_plan(folder,req)
    result.update(motion_authorized=False,hardware_execution_available=False,synthetic=req['scene'].get('synthetic',False),
                  calibration_sha256=digest(cal_path),recording_sha256=digest(DEFAULT_RECORDING),
                  model=manifest,start_state=start,provenance=provenance,
                  physical_execution_blockers=['Saved plans never authorize motion; explicit live --execute and commissioning are required',
                    'Actual resting and static left-arm states must be measured with power available',
                    'Verify depth alignment and CAD mounting plane/yaw; left stick geometry remains unverified',
                    'Verify supported resting pose; detected table/other collisions are not exempted',
                    'Fresh static-scene recheck, bounded tracking, supervised area and supported-rest verification required before torque control'])
    if req.get('occupied_only_preview'):
        result['physical_execution_blockers'].append('Legacy occupied-only-preview flag: visualization only, never executable')
    if result.get('collision_policy')==COLLISION_POLICY:
        result['environment_assumption']='Unobserved space does not block planning. Obstacles absent from the captured depth map are not accounted for.'
    result['scene_is_frozen_snapshot']=True
    result['actual_current_robot_state_verified']=False
    atomic_json(Path(folder)/'plan.json',result)
    return result
