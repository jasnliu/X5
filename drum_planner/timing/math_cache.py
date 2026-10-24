"""Exact, scene-independent timing mathematics; NEVER a collision/readiness cache.

Only the complete curve, original clock, policy, implementation and numerical
environment identify a result. Current-scene validation remains the caller's job.
Uses the existing bounded/checksummed atomic store; unavailable storage is a miss.
"""
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
import tempfile
from functools import lru_cache

import numpy as np
import scipy
from ..common import atomic_json, digest
from ..experiment import cache
from . import core


@lru_cache(maxsize=32)
def _library_digest(path, size, mtime_ns):
    return digest(path)


def identity(source, cfg):
    here=Path(__file__).parent
    files=('core.py','adaptive.py','math_cache.py','integration.py','smooth_v2.py')
    # This Linux application may use distro BLAS rather than a wheel's BLAS.
    # Hash the loaded numerical binaries too; package versions alone are not
    # enough to distinguish a changed LAPACK/SLSQP installation. No dependency.
    libraries=sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
        if '/' in line and '.so' in line and any(k in line.lower() for k in
            ('blas','lapack','libgfortran','_slsqp'))})
    numerical={}
    for name in libraries:
        path=Path(name);stat=path.stat()
        numerical[name]=_library_digest(name,stat.st_size,stat.st_mtime_ns)
    cpu={}
    for line in Path('/proc/cpuinfo').read_text().split('\n\n',1)[0].splitlines():
        if ':' in line:
            key,value=line.split(':',1)
            if key.strip() in ('vendor_id','cpu family','model','stepping','model name','flags'):
                cpu[key.strip()]=value.strip()
    data=dict(schema='x5-pure-timing-v1',
        timing=source['timing'],waypoints=source.get('path_waypoints',source.get('path')),
        policy=cfg,limits=core.LIMITS.tolist(),reserve=core.RESERVE,
        code={f:digest(here/f) for f in files},python=sys.version,
        numpy=np.__version__,scipy=scipy.__version__,machine=platform.machine(),
        libraries=numerical,cpu=cpu,threads={k:os.environ.get(k) for k in
            ('OPENBLAS_NUM_THREADS','OPENBLAS_CORETYPE','OMP_NUM_THREADS','MKL_NUM_THREADS','MKL_CBWR')})
    return hashlib.sha256(json.dumps(data,sort_keys=True,allow_nan=False).encode()).hexdigest()


def solve(source, cfg, calculate, log=lambda _:None):
    """Return payload + reuse metadata; payload contains a law, never a proof.

calculate() returns {'law': ...} and optional numerical solver diagnostics.
No endpoint rounding, approximate keys, robot state, or world proof is stored.
"""
    try:key=identity(source,cfg)
    except (OSError,ValueError,TypeError):key=None
    try:temporary=tempfile.TemporaryDirectory(prefix='x5-timing-math-')
    except OSError:return calculate(),dict(cached=False,key=key)
    with temporary as tmp:
        folder=Path(tmp);path=folder/'math.json'
        if cache.restore(key,folder):
            try:
                payload=json.loads(path.read_text());law=payload['law']
                if (law['source_path_sha256']!=core.identity(source) or
                    law['limits']!=core.LIMITS.tolist() or
                    not np.isfinite(law['duration_s']) or law['duration_s']<=0):
                    raise ValueError('Timing math identity mismatch')
                log(f'TIMING MATH CACHE: identical curve/clock/limits; {law["duration_s"]:.3f}s. Current scene is checked separately.')
                return payload,dict(cached=True,key=key)
            except (OSError,ValueError,KeyError,TypeError):pass
        payload=calculate()
        # A failed optional cache write must never fail a successful solve.
        try:
            atomic_json(path,payload);cache.publish(key,folder,['math.json'])
        except OSError:pass
        return payload,dict(cached=False,key=key)
