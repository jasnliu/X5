"""Two-phase planning: serial candidate searches, then bounded leg timing.

All geometric choices are frozen BEFORE timing starts. Thus CPU contention
cannot shorten a search budget or change which candidates are considered.
The two child processes stay inside the parent's 3 GiB job/control group.
"""
import argparse
import json
import signal
import subprocess
import sys
from pathlib import Path
import time

from ..common import atomic_json


def pending(result,winner,report,req,context,folder,elapsed,*,hardware=False):
    context={k:v for k,v in context.items() if k not in ('_sequence_inputs','_defer_timing')}
    result.update(status='timing_pending',_timing_job=dict(winner=winner,report=report,
        request=req,context=context,folder=str(folder),route_elapsed_s=elapsed,hardware=hardware))
    return result


def finish(result):
    from .dispatch import retime
    from ..planning import cycle
    from ..workflow import simulate
    job=result.pop('_timing_job');folder=Path(job['folder']);started=time.monotonic()
    winner=job['winner'];report=job['report'];req=job['request'];hardware=job['hardware']
    def log(message):
        print(f'[{req.get("moving_side","right")} {folder.name} timing +{time.monotonic()-started:6.1f}s] {message}',flush=True)
    try:
        outward,law,proof,info=retime(winner,report,req,job['context'],folder,log)
        result.update(status='planned',outward=outward,cycle=cycle(outward),timing=law,
            source_timing=winner['timing'],path_waypoints=info.get('smoothed_path_waypoints',winner['path']),
            duration_outward_s=law['duration_s'],hold_at_goal_s=2.,reverse_checked_on_same_static_scene=True,
            final_curve_check=proof,sampled_interpolated_checks=proof['samples_checked'],
            joint_limit_check=report['joint_limit_check'],velocity_limit_rad_s=.6,
            acceleration_limit_rad_s2=1.2,jerk_limit_rad_s3=6.,collision_edge_l1_step_rad=.003)
        if hardware:
            result.update(retiming=info,backend='Workspace curve + current arm-aware adaptive timing')
        else:
            from ..experiment.config import DESCRIPTIONS
            result['backend']=DESCRIPTIONS[result['experiment']['method']]
            result['experiment']['retiming']=info
            if info.get('smoothing_applied'):result['original_path_waypoints']=winner['path']
            result['experiment']['cycle_test']=simulate(result)
        log(f'LEG READY: {law["duration_s"]:.3f}s; exact timing and collision/zone checks complete; NO motor commands.')
    except Exception as exc:
        result.update(status='error' if hardware else 'experiment_error',error=str(exc))
        raise
    finally:
        result['experiment']['elapsed_s']=job['route_elapsed_s']+time.monotonic()-started
        atomic_json(folder/'plan.json',result)
    return result


def finish_all(legs,*,workers=2):
    if not 1<=workers<=2:raise ValueError('Timing pool supports one or two workers only')
    if workers==1:return [finish(leg) for leg in legs]
    print(f'[checkpoints] All {len(legs)} geometric routes locked. Timing with two workers; '
          'same refinement/limits, shared job memory cap; no concurrent path searches.',flush=True)
    # Fresh interpreters, not forked GUI/native state. File IPC avoids named
    # multiprocessing semaphores/resource trackers being leaked by group Cancel.
    # Do NOT create new process sessions: feedback failure's existing killpg and
    # systemd Cancel must include every child/native checker in this calculation.
    active={};ordered=[None]*len(legs);next_index=0
    try:
        while next_index<len(legs) or active:
            while next_index<len(legs) and len(active)<workers:
                leg=legs[next_index];folder=Path(leg['_timing_job']['folder'])
                folder.mkdir(parents=True,exist_ok=True)
                source=folder/'timing_input.json';atomic_json(source,leg)
                process=subprocess.Popen([sys.executable,'-m',__name__,'--job',str(source.resolve())],
                                         close_fds=True)
                active[next_index]=(process,folder);next_index+=1
            for index,(process,folder) in list(active.items()):
                code=process.poll()
                if code is None:continue
                path=folder/'plan.json'
                result=json.loads(path.read_text()) if path.exists() else {}
                if code or result.get('status')!='planned':
                    raise RuntimeError(f'Timing failed for {folder.name}: '+
                        str(result.get('error',f'worker exited {code}; see calculation log')))
                ordered[index]=result;del active[index]
            if active:time.sleep(.02)
        return ordered
    finally:
        for process,_ in active.values():
            if process.poll() is None:process.terminate()
        for process,_ in active.values():
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)


def _interrupted(signum,frame):
    # Unwind world/temp-file finalizers on a group Cancel. No motor ownership
    # lives in this module. SIGKILL remains the job's bounded last resort.
    raise SystemExit(128+signum)


if __name__=='__main__':
    from ..experiment.config import no_can
    no_can();signal.signal(signal.SIGTERM,_interrupted)
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--job',type=Path,required=True)
    args=parser.parse_args();finish(json.loads(args.job.read_text()))
