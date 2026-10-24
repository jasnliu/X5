"""Bounded, content-checked reuse between saved-input software sessions.

Run-local files remain authoritative. This optional disk mirror only restores
completed artifacts for an identical request/code/environment; unavailable,
busy, corrupt or full storage is a cache miss, never a planning failure.
It does not cache live acquisition, readiness or motor-controller state.
"""
import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import sys
import tempfile
import time

import numpy as np
import scipy
from ..common import ROOT, digest

MAX_BYTES = 512 * 1024 * 1024
MAX_ENTRIES = 128
MAX_ENTRY_BYTES = 64 * 1024 * 1024
SCHEMA = 'x5-saved-planning-cache-v1'


def root():
    override = os.environ.get('X5_PLANNER_CACHE_DIR')
    if override == 'off':
        return None
    if override:
        return Path(override).expanduser()
    base = Path(os.environ.get('XDG_CACHE_HOME', str(Path.home() / '.cache')))
    project = hashlib.sha256(str(ROOT.resolve()).encode()).hexdigest()[:16]
    return base / 'openarmx' / 'drum-planner' / project


def identity(context, request, namespace, signature):
    try:
        return _identity(context, request, namespace, signature)
    except (OSError, ValueError, KeyError, TypeError):
        return None


def _identity(context, request, namespace, signature):
    # Never transfer an experiment's cached validation into a live workflow.
    if not context.get('saved_not_live') or context.get('can_access') is not False:
        return None
    req = dict(request)
    for name in ('urdf', 'srdf'):
        req[name] = digest(req[name])
    for side in ('left', 'right'):
        if side + '_zone' in req:
            zone = dict(req[side + '_zone'])
            for name in ('reference_urdf', 'reference_srdf', 'zone_file'):
                if zone.get(name):
                    zone[name] = digest(zone[name])
            req[side + '_zone'] = zone
    from .runtime import BINARY
    data = dict(schema=SCHEMA, namespace=namespace, signature=signature, request=req,
                python=sys.version, numpy=np.__version__, scipy=scipy.__version__,
                machine=platform.machine(), oracle=digest(BINARY),
                threads={k: os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS')})
    return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).hexdigest()


@contextlib.contextmanager
def locked(wait_s=0.):
    folder = root()
    if folder is None:
        yield None
        return
    folder.mkdir(parents=True, exist_ok=True)
    with (folder / '.lock').open('a') as lock:
        deadline=time.monotonic()+wait_s
        while True:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB);break
            except BlockingIOError:
                if time.monotonic()>=deadline:raise
                time.sleep(.01)
        # A worker may have been cancelled mid-publication. The lock proves no
        # other publication is active, so only our own abandoned stages expire.
        for stage in folder.glob('.stage-*'):
            if stage.is_dir() and not stage.is_symlink():
                shutil.rmtree(stage)
        yield folder


def valid_name(name):
    return isinstance(name, str) and Path(name).name == name and name.endswith('.json') and name != 'manifest.json'


def valid_key(key):
    return isinstance(key, str) and len(key) == 64 and all(c in '0123456789abcdef' for c in key)


def restore(key, destination, log=lambda _: None):
    if not valid_key(key):
        return False
    try:
        # Two timing children often arrive in the same millisecond. A brief
        # metadata-copy lock is not a reason to redo minutes of exact math.
        # Storage remains optional: bounded wait, then a normal cache miss.
        with locked(wait_s=.5) as base:
            if base is None:
                return False
            entry = base / key
            manifest_path=entry / 'manifest.json'
            if entry.is_symlink() or manifest_path.stat().st_size > 1024 * 1024:
                return False
            manifest = json.loads(manifest_path.read_text())
            if manifest['schema'] != SCHEMA or manifest['key'] != key:
                return False
            files = manifest['files']
            if not isinstance(files,dict) or not files or len(files)>512 or any(not valid_name(n) for n in files):
                return False
            size = 0
            for name, sha in files.items():
                path = entry / name
                if path.is_symlink():
                    return False
                size += path.stat().st_size
                if size > MAX_ENTRY_BYTES or digest(path) != sha:
                    return False
            destination = Path(destination)
            destination.mkdir(parents=True, exist_ok=True)
            # All files are checked before the first becomes visible locally.
            for name in files:
                with tempfile.NamedTemporaryFile(dir=destination, prefix='.cache-', delete=False) as stream:
                    temp = Path(stream.name)
                    try:
                        with (entry / name).open('rb') as src:
                            shutil.copyfileobj(src, stream)
                        stream.close()
                        os.replace(temp, destination / name)
                    finally:
                        temp.unlink(missing_ok=True)
            entry.touch()
        log('PERSISTENT CACHE: restored identical saved-input artifacts; no approximation')
        return True
    except (OSError, ValueError, KeyError, TypeError):
        return False


def publish(key, source, names, log=lambda _: None):
    if not valid_key(key):
        return False
    try:
        names = list(names)
        if not names or len(names)>512 or len(set(names)) != len(names) or any(not valid_name(n) for n in names):
            return False
        source = Path(source)
        size = sum((source / name).stat().st_size for name in names)
        if size > MAX_ENTRY_BYTES or size > MAX_BYTES:
            return False
        with locked(wait_s=.5) as base:
            if base is None:
                return False
            stage = Path(tempfile.mkdtemp(dir=base, prefix='.stage-'))
            try:
                files = {}
                for name in names:
                    shutil.copyfile(source / name, stage / name)
                    files[name] = digest(stage / name)
                (stage / 'manifest.json').write_text(json.dumps(dict(schema=SCHEMA, key=key, files=files)))
                entry = base / key
                if entry.exists():
                    shutil.rmtree(entry)
                os.replace(stage, entry)
                entries = sorted((p for p in base.iterdir() if len(p.name) == 64 and p.is_dir() and not p.is_symlink()),
                                 key=lambda p: p.stat().st_mtime, reverse=True)
                total = 0
                for index, path in enumerate(entries):
                    total += sum(f.stat().st_size for f in path.iterdir() if f.is_file())
                    if index >= MAX_ENTRIES or total > MAX_BYTES:
                        shutil.rmtree(path)
            finally:
                if stage.exists():
                    shutil.rmtree(stage)
        return True
    except (OSError, ValueError, TypeError):
        return False
