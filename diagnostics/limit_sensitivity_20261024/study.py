"""OFFLINE ONLY: fresh fixed-path limit sensitivity; no application edits.

Each case is a fresh Python process inside a no-AF_CAN bounded systemd job.
Only that process's in-memory timing limits change. No paths are regenerated,
no recorded interior motion is used to construct a route, and nothing executable
is exported to the hardware planner. Results are diagnostics, not commissioned limits.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
DENIED = []


def guard(event, args):
    if event == 'socket.__new__' and args[1] == 29:
        DENIED.append('AF_CAN'); raise PermissionError('No CAN in timing study')
    if event == 'open' and isinstance(args[0], (str, bytes)):
        name = os.fsdecode(args[0])
        if name.startswith(('/dev/video', '/dev/bus/usb', '/dev/ttyUSB', '/dev/ttyACM', '/dev/dri')):
            DENIED.append(name); raise PermissionError('No camera/serial/GPU devices in timing study')


sys.addaudithook(guard)


def read(path): return json.loads(Path(path).read_text())
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def artifact(path, suffix): return Path(str(path)+suffix)
def save(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')


def configurations():
    rows = [dict(name='baseline', factors=[1.,1.,1.])]
    for axis, label in enumerate(('velocity','acceleration','jerk')):
        for factor in (1.1,1.5,2.,4.,8.,64.):
            f = [1.,1.,1.]; f[axis] = factor
            rows.append(dict(name=f'{label}_x{factor:g}', factors=f))
    for factor in (2.,4.,8.,16.):
        rows.append(dict(name=f'all_x{factor:g}', factors=[factor]*3))
    for axes,label in [((0,1),'velocity_acceleration'),((0,2),'velocity_jerk'),((1,2),'acceleration_jerk')]:
        f=[1.,1.,1.]
        for axis in axes: f[axis]=4.
        rows.append(dict(name=label+'_x4', factors=f))
    # Equal speedups require powers 1,2,3, NOT the same multiplier for all limits.
    for speed in (2.,4.,8.):
        rows.append(dict(name=f'time_compression_x{speed:g}', factors=[speed,speed**2,speed**3]))
    return rows


def worker(args):
    import numpy as np
    from scipy.integrate import quad
    from drum_planner.timing import core, motion_quality
    from drum_planner.experiment import fast_timing
    original_limits = core.LIMITS.copy()
    started = time.monotonic()
    source_file = ROOT/f'diagnostics/fast_gpu_20261024/evidence/{args.path}.json'
    data = read(source_file); source=data['source']; path=core.original(source)
    core.LIMITS = original_limits*np.asarray(args.factors)
    # Optional independently labelled research rerun, never a production edit.
    if args.refined:
        fast_timing.CONTROLS=192;fast_timing.MAX_ITERATIONS=300;fast_timing.SOLVE_WALL_SECONDS=3.
    law = fast_timing.optimize(source, lambda s: print(s, flush=True))
    elapsed = time.monotonic()-started
    exact = core.decode(law['path'])
    assert np.array_equal(exact.c, path.c) and np.array_equal(exact.x, path.x)
    assert law['source_path_sha256'] == core.identity(source)
    density=core.decode(law['density'])
    # Independent original root checker, not the optimizer's batched checker.
    validation_start=time.monotonic()
    peaks, where = core.density_peaks(path, density)
    assert np.all(peaks <= core.LIMITS[:,None]*(1+1e-9))
    np.testing.assert_allclose(peaks, law['peaks'], rtol=1e-7, atol=1e-7)
    evaluator=core.Evaluator(law)
    q,v,a,j=evaluator(np.linspace(0, evaluator.duration, 10001))
    assert np.max(abs(v[[0,-1]])) < 1e-7 and np.max(abs(a[[0,-1]])) < 1e-6
    assert np.allclose(q[[0,-1]],path([0,1]),atol=1e-10,rtol=0)
    sampled=np.array([np.max(abs(x),axis=0) for x in (v,a,j)])
    assert np.all(sampled <= peaks*(1+1e-7)+1e-7)
    quality=motion_quality.law_metrics(law)
    assert quality['internal_stops'] == 0
    displacement=abs(path(1)-path(0))
    scalar_bounds=[0. if d<1e-10 else float(core.scurve(float(d))[1][-1]) for d in displacement]
    vel_integral=quad(lambda s: max(abs(path(s,nu=1)))/core.LIMITS[0],0,1,
                      points=np.unique(path.x),epsabs=1e-8,limit=500)[0]
    velocity_only_raw=vel_integral*core.LIMITS[0]
    law['diagnostic_only']=True;law['motion_authorized']=False
    save(artifact(args.output, '.law.json'),law)
    result=dict(name=args.name,path=args.path,factors=args.factors,limits=core.LIMITS.tolist(),
        movement_s=law['duration_s'],optimizer_s=law['details']['calculation_seconds'],
        imports_excluded_total_s=elapsed,independent_check_s=time.monotonic()-validation_start,
        original_checker_peaks=peaks.tolist(),extrema_s=where.tolist(),
        utilization=(peaks/core.LIMITS[:,None]).tolist(),quality=quality,
        near_limit_time_fraction={key:float(np.mean(np.max(abs(vals),axis=1)>=limit*.98))
            for key,vals,limit in zip(('velocity','acceleration','jerk'),(v,a,j),core.LIMITS)},
        scalar_lower_bounds_s=scalar_bounds,velocity_only_lower_bound_s=vel_integral,
        combined_lower_bound_s=max(max(scalar_bounds),vel_integral),
        path_velocity_integral_rad=velocity_only_raw,joint_displacement_rad=displacement.tolist(),
        same_exact_geometry=True,endpoint_velocity_acceleration_zero=True,
        reserve=core.RESERVE,details=law['details'],source_file=str(source_file),
        source_file_sha256=sha(source_file),source_path_sha256=core.identity(source),
        denied_device_attempts=DENIED,motor_commands=0,camera_access=False,
        recording_used_for_route_generation=False,refined=args.refined)
    save(artifact(args.output, '.json'),result)
    print(f'CASE {args.name} {args.path}: movement={law["duration_s"]:.6f}s; '
          f'lower_bound={result["combined_lower_bound_s"]:.6f}s; '
          f'compute={result["optimizer_s"]:.3f}s; geometry and continuous limits verified',flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker',action='store_true');parser.add_argument('--name')
    parser.add_argument('--path',default='checkpoint_2')
    parser.add_argument('--factors',type=float,nargs=3);parser.add_argument('--output',type=Path)
    parser.add_argument('--refined',action='store_true')
    args=parser.parse_args()
    if args.worker: return worker(args)
    output=args.output or HERE/'primary';output.mkdir(exist_ok=False,parents=True)
    configs=configurations();save(output/'configurations.json',configs)
    results=[];start=time.monotonic()
    for index,cfg in enumerate(configs):
        print(f'[{index+1}/{len(configs)}] {cfg["name"]} limits factors {cfg["factors"]}',flush=True)
        target=output/cfg['name'];case_start=time.monotonic()
        command=[sys.executable,str(Path(__file__)), '--worker','--name',cfg['name'],
                 '--path',args.path,'--factors',*map(str,cfg['factors']),'--output',str(target)]
        if args.refined: command.append('--refined')
        with artifact(target, '.log').open('w') as log:
            subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=45)
        row=read(artifact(target, '.json'));row['cold_process_including_validation_s']=time.monotonic()-case_start
        results.append(row);save(output/'results.json',results)
        print(f'  {row["movement_s"]:.4f}s movement; {row["optimizer_s"]:.3f}s optimization; '
              f'{row["details"]["selected_stage"]}; flags={row["details"]["warnings"]}',flush=True)
    save(output/'run.json',dict(cases=len(results),elapsed_s=time.monotonic()-start,hardware_access=False,
        study_sha256=sha(__file__),source_policy='same archived exact route; not current camera readiness'))


if __name__=='__main__':main()
