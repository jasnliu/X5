"""Per-joint speed-bias study on freshly collision-checked R1/R2 paths.

Run after acquisition/path creation, inside no-AF_CAN bounded jobs. Never
captures hardware, changes installed limits or uses recorded interior motion.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))


def guard(event,args):
    if event=='socket.__new__' and args[1]==29:raise PermissionError('Study forbids CAN')
    if event=='open' and isinstance(args[0],(str,bytes)) and os.fsdecode(args[0]).startswith(
            ('/dev/video','/dev/bus/usb','/dev/ttyUSB','/dev/ttyACM','/dev/dri')):
        raise PermissionError('Study forbids hardware devices')


sys.addaudithook(guard)


def read(p):return json.loads(Path(p).read_text())
def save(p,obj):Path(p).write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def configurations():
    import numpy as np
    baseline=np.tile(np.array([.6,1.2,6.])[:,None],(1,7));rows=[]
    def add(name,factors):
        rows.append(dict(name=name,limits=(baseline*factors).tolist(),factors=np.broadcast_to(factors,(3,7)).tolist()))
    def speeds(proximal=1.,forearm=1.,wrist=1.):
        k=np.array([proximal]*4+[forearm,wrist,wrist])
        return k[None,:]**np.arange(1,4)[:,None]
    add('baseline',np.ones((3,7)))
    for axis,name in enumerate(['velocity','acceleration','jerk']):
        f=np.ones((3,7));f[axis]=2.;add(name+'_x2',f)
    for factor in [2.,4.]:add(f'all_limits_x{factor:g}',np.full((3,7),factor))
    for k in [2.,4.]:add(f'uniform_speed_x{k:g}',speeds(k,k,k))
    for k in [2.,4.,8.]:add(f'wrist_only_speed_x{k:g}',speeds(1.,1.,k))
    for k in [2.,4.,8.]:add(f'distal_speed_x{k:g}',speeds(1.,k,k))
    for label,p,f,w in [('gentle_body_fast_wrist',.75,2.,4.),('body1_forearm2_wrist4',1.,2.,4.),
                         ('body1p5_forearm2_wrist4',1.5,2.,4.),('body2_forearm3_wrist4',2.,3.,4.),
                         ('body2p5_forearm3_wrist4',2.5,3.,4.),('body3_forearm4_wrist6',3.,4.,6.)]:
        add(label,speeds(p,f,w))
    return rows


def validate(law,source,limits,folder,*,collision=False):
    import numpy as np
    from drum_planner.timing import core,motion_quality
    from drum_planner.timing.benchmark import check_geometry
    path=core.original(source);h=core.decode(law['density'])
    peaks,locations=core.density_peaks(path,h)
    assert np.all(peaks<=limits*(1+1e-9)),peaks/limits
    np.testing.assert_allclose(peaks,law['peaks'],rtol=1e-7,atol=1e-7)
    geometry=check_geometry(law,source)
    grid=np.unique(np.r_[np.linspace(0,law['duration_s'],10001),np.arange(0,law['duration_s'],.002),
                         core.event_times(law),law['duration_s']])
    q,v,a,j=core.evaluate(law,grid)
    assert np.max(abs(v[[0,-1]]))<1e-8 and np.max(abs(a[[0,-1]]))<1e-7
    assert np.allclose(q[[0,-1]],path([0,1]),atol=1e-10,rtol=0)
    sampled=np.array([np.max(abs(z),axis=0) for z in [v,a,j]])
    assert np.all(sampled<=peaks*(1+1e-7)+1e-7)
    quality=motion_quality.law_metrics(law)
    assert quality['internal_stops']==0
    result=dict(peaks=peaks.tolist(),normalized_peaks=(peaks/limits).tolist(),
        extrema_s=locations.tolist(),geometry=geometry,endpoint_rest=True,dense_samples=len(grid),quality=quality)
    if collision:
        from drum_planner.experiment.runtime import Oracle
        from drum_planner.experiment.algorithms import dense_check
        from drum_planner.common import atomic_json
        folder=Path(folder);folder.mkdir(exist_ok=True,parents=True)
        atomic_json(folder/'request.json',source['request']);oracle=Oracle(folder/'request.json',folder)
        try:
            assert np.all(q>=np.asarray(oracle.lower)-1e-10) and np.all(q<=np.asarray(oracle.upper)+1e-10)
            ok,proof=dense_check(oracle,q);assert ok,proof
            result['collision_zone']=proof
        finally:oracle.close()
    return result


def bounds(source,limits):
    import numpy as np
    from scipy.integrate import quad
    from scipy.interpolate import PPoly
    from drum_planner.timing import core
    path=core.original(source);original=core.LIMITS;out=[];travel=[]
    try:
        for i,delta in enumerate(abs(path(1)-path(0))):
            core.LIMITS=limits[:,i]
            out.append(float(core.scurve(float(delta))[1][-1]) if delta>1e-10 else 0.)
            p=PPoly(path.c[:,:,i],path.x);roots=p.derivative().roots(extrapolate=False)
            s=np.unique(np.r_[path.x,roots[np.isfinite(roots)&(roots>=0)&(roots<=1)]])
            travel.append(float(sum(abs(np.diff(p(s))))))
    finally:core.LIMITS=original
    velocity=quad(lambda s:np.max(abs(path(s,nu=1))/limits[0]),0,1,
                  points=np.unique(path.x),epsabs=1e-8,limit=500)[0]
    return dict(scalar_minimum_s=out,velocity_only_minimum_s=velocity,
        combined_minimum_s=max(velocity,max(out)),limiting_scalar_joint=int(np.argmax(out))+1,
        net_joint_change_rad=abs(path(1)-path(0)).tolist(),total_joint_travel_rad=travel)


def worker(args):
    import numpy as np
    from drum_planner.timing import core
    from drum_planner.experiment import fast_timing
    import joint_timing
    cfg=next(x for x in configurations() if x['name']==args.case)
    limits=np.array(cfg['limits']);source_file=HERE/args.direction/'source.json';source=read(source_file)
    t=time.monotonic()
    if args.original:
        assert args.case=='baseline'
        law=fast_timing.optimize(source,print)
        law['limits']=limits.tolist();law.update(diagnostic_only=True,motion_authorized=False)
    else:law=joint_timing.optimize(source,limits,refined=args.refined,log=print)
    elapsed=time.monotonic()-t
    proof=validate(law,source,limits,Path(str(args.output)+'-check'),collision=args.collision)
    assert np.array_equal(core.LIMITS,[.6,1.2,6.])
    output=dict(case=args.case,direction=args.direction,configuration=cfg,refined=args.refined,
        original_implementation=args.original,duration_s=law['duration_s'],calculation_s=elapsed,
        proof=proof,bounds=bounds(source,limits),details=law['details'],
        source_file=str(source_file),source_sha256=sha(source_file),source_path_sha256=core.identity(source),
        study_sha256=sha(__file__),extension_sha256=sha(HERE/'joint_timing.py'),
        device_access=False,motor_commands=0,diagnostic_only=True)
    save(str(args.output)+'.law.json',law);save(str(args.output)+'.json',output)
    print(args.direction,args.case,f'{law["duration_s"]:.6f}s motion; {elapsed:.3f}s calc',flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker',action='store_true');parser.add_argument('--case')
    parser.add_argument('--direction',choices=['R1_to_R2','R2_to_R1'],default='R1_to_R2')
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--refined',action='store_true')
    parser.add_argument('--original',action='store_true');parser.add_argument('--collision',action='store_true')
    parser.add_argument('--cases',nargs='*')
    args=parser.parse_args()
    if args.worker:return worker(args)
    args.output.mkdir(exist_ok=False,parents=True);configs=configurations()
    if args.cases:configs=[c for c in configs if c['name'] in args.cases]
    rows=[]
    for i,(direction,cfg) in enumerate((d,c) for d in ['R1_to_R2','R2_to_R1'] for c in configs):
        name=direction+'_'+cfg['name'];target=args.output/name
        cmd=[sys.executable,__file__,'--worker','--direction',direction,'--case',cfg['name'],'--output',str(target)]
        if args.refined:cmd.append('--refined')
        if args.collision:cmd.append('--collision')
        t=time.monotonic()
        with Path(str(target)+'.log').open('w') as log:
            subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=60)
        row=read(str(target)+'.json');row['fresh_process_total_s']=time.monotonic()-t
        rows.append(row);save(args.output/'results.json',rows)
        print(f'[{i+1}/{2*len(configs)}] {name}: {row["duration_s"]:.4f}s motion, '
              f'{row["calculation_s"]:.3f}s calc; {row["details"]["warnings"]}',flush=True)


if __name__=='__main__':main()
