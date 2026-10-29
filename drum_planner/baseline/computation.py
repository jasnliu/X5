"""Device-free planning/check subprocess, isolated from the encoder reader.

SLSQP can retain the Python GIL longer than an entire CAN query deadline. A
thread is therefore not sufficient isolation: acquisition/monitoring stays in
work.py's process, and all expensive trajectory work runs here. Both processes
remain in the existing memory-limited, cancellable calculation cgroup.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ..common import atomic_json


def _run(command, worker):
    """Inherit terminal logging, never CAN descriptors; reap on every exit."""
    if worker.error:
        raise RuntimeError(worker.error)
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    process = subprocess.Popen(command, close_fds=True, start_new_session=True, env=env)
    try:
        while True:
            if worker.error:
                raise RuntimeError(worker.error)
            code = process.poll()
            if code is not None:
                return code
            time.sleep(.02)
    finally:
        # Include native FCL descendants on error/cancellation, not just Python.
        # GUI cancellation also stops the enclosing systemd control group.
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=3)
        finally:
            # A failed Python leader may exit before a native descendant.
            # Do not leave that descendant consuming memory after a retry.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def run(folder, cal_path, worker, *, plan=None, moving_side='right', destinations=None,timing_backend='current',motion_profile=None):
    folder = Path(folder).resolve()
    from ..experiment.runtime import PYTHON
    command = [str(PYTHON) if timing_backend=='current' else sys.executable, '-m', 'drum_planner.baseline.computation',
               '--folder', str(folder), '--calibration', str(Path(cal_path).resolve()), '--side', moving_side]
    if plan is not None:
        command += ['--plan', str(Path(plan).resolve())]
    if timing_backend!='current':command+=['--timing-backend',timing_backend]
    if destinations is not None:
        atomic_json(folder/'destinations.json',destinations)
        command+=['--destinations',str(folder/'destinations.json')]
    if motion_profile is not None:
        atomic_json(folder/'motion_profile.json',motion_profile)
        command+=['--motion-profile',str(folder/'motion_profile.json')]
    output = folder/'computation'
    output.mkdir()  # one computation per disposable Calculate job
    print('Planning/timing isolated from encoder monitoring; feedback stays active. NO motor commands.', flush=True)
    code = _run(command, worker)
    if code:
        error = output/'error.json'
        detail = json.loads(error.read_text())['error'] if error.exists() else f'Calculation process exited ({code}); see worker.log'
        raise RuntimeError(detail)
    return json.loads((output/'result.json').read_text())


def compute(folder, cal_path, *, plan=None, moving_side='right', destinations=None,timing_backend='current',motion_profile=None):
    # Install before importing planning dependencies. This child has no sensor
    # ownership and must never open a physical CAN socket, even accidentally.
    from ..experiment.config import no_can
    no_can()
    from . import tasks
    folder = Path(folder)
    capture = folder/'capture'
    if plan is None:
        options={} if destinations is None else dict(destinations=destinations)
        if timing_backend!='current':options['timing_backend']=timing_backend
        if motion_profile is not None:options['motion_profile']=motion_profile
        plan = tasks.make_plan(folder/'plan', capture, cal_path, moving_side=moving_side,**options)
    from .arms import require_side
    require_side(json.loads((Path(plan).parent/'backend_request.json').read_text()), moving_side)
    tasks.prepare(folder/'checked', plan, capture, cal_path, allow_replan=False)
    return dict(plan=str(plan), prepared=str(folder/'checked/prepared.json'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', required=True)
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--plan')
    parser.add_argument('--side', choices=['right','left'], default='right')
    parser.add_argument('--destinations')
    parser.add_argument('--motion-profile')
    parser.add_argument('--timing-backend',choices=['current','adaptive','sparse'],default='current')
    args = parser.parse_args()
    folder = Path(args.folder)
    try:
        result = compute(folder, args.calibration, plan=args.plan, moving_side=args.side,
                         destinations=json.loads(Path(args.destinations).read_text()) if args.destinations else None,
                         motion_profile=json.loads(Path(args.motion_profile).read_text()) if args.motion_profile else None,
                         **({'timing_backend':args.timing_backend} if args.timing_backend!='current' else {}))
        atomic_json(folder/'computation/result.json', result)
    except Exception as exc:
        atomic_json(folder/'computation/error.json', dict(error=str(exc), motor_commands=0))
        raise


if __name__ == '__main__':
    main()
