"""Scene-checked sparse timing, isolated from the original hardware retimer."""
import json
from pathlib import Path
import subprocess
import time

from ..common import atomic_json,digest
from ..experiment.runtime import PYTHON
from . import core
from .benchmark import validate


def retime(winner,report,req,context,folder,log):
    began=time.monotonic();folder=Path(folder)/'sparse_timing';folder.mkdir(parents=True,exist_ok=True)
    source=dict(path_waypoints=winner['path'],timing=winner['timing'],request=req)
    atomic_json(folder/'source.json',source)
    log('SPARSE TIMING FORK: IPOPT; unchanged Workspace curve, limits and refinement levels; no timing-cache lookup')
    # The optional dependency lives only in the planner venv. Inherit the
    # existing process group/cgroup so cancellation includes this child.
    process=subprocess.Popen([str(PYTHON),'-m','drum_planner.timing.sparse',
        '--input',str((folder/'source.json').resolve()),'--output',str((folder/'law.json').resolve())],close_fds=True)
    try:
        code=process.wait()
        if code:raise RuntimeError(f'Sparse timing worker exited {code}; see calculation log. '
            'Install its dependency with ./setup_planner_sparse.sh if missing.')
    finally:
        if process.poll() is None:
            process.terminate()
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
    law=json.loads((folder/'law.json').read_text())
    if law['source_path_sha256']!=core.identity(source):raise ValueError('Sparse timing source identity mismatch')
    log('SPARSE VERIFY: unchanged exact extrema, rest endpoints, dense collision and zone checks')
    proof=validate(law,source,folder/'check');atomic_json(folder/'verification.json',proof)
    points=core.samples(law,source)
    points[0]['positions_rad']=list(req['start']);points[-1]['positions_rad']=list(req['goal'])
    meta=dict(method='sparse_ipopt',label='Sparse IPOPT fixed-path timing (hardware fork)',
        implementation_sha256=digest(Path(__file__).with_name('sparse.py')),
        source_path_sha256=core.identity(source),original_duration_s=winner['duration_s'],
        duration_s=law['duration_s'],geometry_preserved=True,limits=core.LIMITS.tolist(),
        exact_extrema_peaks=law['peaks'],verification=proof,cached=False,elapsed_s=time.monotonic()-began)
    log(f'SPARSE READY: {law["duration_s"]:.3f}s movement; {meta["elapsed_s"]:.1f}s computation/checking')
    return points,law,proof['collision_zone'],meta
