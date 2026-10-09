"""Persistent memoization of the existing, unmodified recording preflights.

Only successful calculation results are cached, never live readiness or motor
state. A miss or any cache error calls the original loader with the same inputs.
JSON stores exact float64 coefficients, not executable pickle data. Recording
files are never written. Runtime geometry is rebound to the current model/zone.
"""
from dataclasses import fields
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import scipy
from scipy.interpolate import PPoly, PchipInterpolator

from safe_zone.geometry import Model, Zone
from smooth_playback.trajectory import Geometry, Trajectory
from . import smooth_recording, trajectory


ROOT = Path(__file__).resolve().parents[1]
CACHE_FORMAT = 1
MAX_CACHE_BYTES = 128_000_000
# Conservative dependency closure: changes anywhere in these local packages
# invalidate the result, including imported safety constants/helper functions.
CODE_PACKAGES = ('camera_playback', 'smooth_playback', 'safe_zone',
                 'cartesian_goal', 'motion_recording', 'camera_search',
                 'goal_motion', 'centering')


def _code_digest():
    digest = hashlib.sha256()
    for package in CODE_PACKAGES:
        for path in sorted((ROOT / package).glob('*.py')):
            digest.update(str(path.relative_to(ROOT)).encode())
            digest.update(b'\0')
            digest.update(path.read_bytes())
            digest.update(b'\0')
    return digest.hexdigest()


# Do not stamp results with newly edited on-disk code while an older process is
# still running. Such a process simply bypasses caching until restarted.
try:
    _LOADED_CODE_DIGEST = _code_digest()
except OSError:
    _LOADED_CODE_DIGEST = None


def cache_directory():
    return Path(os.environ.get('XDG_CACHE_HOME') or Path.home() / '.cache') / \
        'openarmx' / 'recording-preflight'


def _pack(value):
    if isinstance(value, np.ndarray):
        if value.dtype != np.dtype('float64') or not np.isfinite(value).all():
            raise ValueError('Unsupported cache array')
        return {'__type__': 'array', 'shape': list(value.shape),
                'values': value.ravel().tolist()}
    if isinstance(value, np.generic):
        return _pack(value.item())
    if type(value) in (PPoly, PchipInterpolator):
        return {'__type__': type(value).__name__, 'c': _pack(value.c),
                'x': _pack(value.x), 'extrapolate': value.extrapolate,
                'axis': value.axis}
    if isinstance(value, Path):
        return {'__type__': 'path', 'value': str(value)}
    if isinstance(value, dict):
        if '__type__' in value:
            raise ValueError('Reserved cache key')
        return {k: _pack(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_pack(v) for v in value]
    if value is None or type(value) in (str, bool, int, float):
        return value
    raise ValueError('Unsupported cache value')


def _unpack(value):
    if isinstance(value, list):
        return [_unpack(v) for v in value]
    if not isinstance(value, dict):
        return value
    kind = value.get('__type__')
    if kind == 'array':
        array = np.asarray(value['values'], dtype=np.float64).reshape(value['shape'])
        if not np.isfinite(array).all():
            raise ValueError('Nonfinite cache array')
        return array
    if kind in ('PPoly', 'PchipInterpolator'):
        cls = PPoly if kind == 'PPoly' else PchipInterpolator
        # Both use the same piecewise-power basis; preserve exact coefficients
        # and class rather than refitting/resampling the validated trajectory.
        poly = PPoly(_unpack(value['c']), _unpack(value['x']),
                     extrapolate=value['extrapolate'], axis=value['axis'])
        return cls.construct_fast(poly.c, poly.x, poly.extrapolate, poly.axis)
    if kind == 'path':
        return Path(value['value'])
    if kind is not None:
        raise ValueError('Unknown cache type')
    return {k: _unpack(v) for k, v in value.items()}


def _json_bytes(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      allow_nan=False).encode('utf-8')


def _runtime_constants():
    # Also invalidate in-process changes to numeric/string safety settings.
    modules = (trajectory, smooth_recording,
               sys.modules['smooth_playback.trajectory'],
               sys.modules['cartesian_goal.ik'],
               sys.modules['safe_zone.geometry'],
               sys.modules['motion_recording.recording'])
    supported = (str, bool, int, float, tuple, list, dict, np.ndarray, Path)
    return {module.__name__: {name: _pack(value)
            for name, value in vars(module).items()
            if name.isupper() and isinstance(value, supported)} for module in modules}


def _fingerprint(kind, path, model, zone, lower, upper, center, speed, playback_speed):
    # Custom implementations cannot inherit a pass from the built-in geometry.
    if type(model) is not Model or type(zone) is not Zone:
        raise ValueError('Uncacheable custom geometry')
    code = _code_digest()
    if code != _LOADED_CODE_DIGEST:
        raise ValueError('Code changed since import')
    source = Path(path).expanduser().resolve()
    if source.suffix.lower() != '.json':
        raise ValueError('Uncacheable recording suffix')
    with source.open('rb') as stream:
        data = stream.read(trajectory.MAX_RECORDING_BYTES + 1)
    if len(data) > trajectory.MAX_RECORDING_BYTES:
        raise ValueError('Uncacheable recording size')
    inputs = {
        'format': CACHE_FORMAT, 'kind': kind, 'workspace': str(ROOT),
        'source': str(source), 'source_sha256': hashlib.sha256(data).hexdigest(),
        'code': code, 'constants': _runtime_constants(),
        'python': sys.version, 'numpy': np.__version__, 'scipy': scipy.__version__,
        'byteorder': sys.byteorder,
        'model_digest': model.digest,
        'model_joints': [ET.tostring(j).decode('utf-8') for j in model.joints],
        'zone': {'model_hash': zone.model_hash, 'tcp': zone.tcp,
                 'points': zone.points, 'dirty': zone._dirty,
                 'planes': None if zone.hull is None else zone.hull.equations.tolist()},
        'lower': np.asarray(lower).tolist(), 'upper': np.asarray(upper).tolist(),
        'center': np.asarray(center).tolist(), 'speed': float(speed),
        'playback_speed': None if playback_speed is None else float(playback_speed),
    }
    return hashlib.sha256(_json_bytes(inputs)).hexdigest()


def _encode(recording):
    data = {'base': {f.name: _pack(getattr(recording, f.name))
                     for f in fields(trajectory.PlaybackTrajectory)}}
    if type(recording) is smooth_recording.SmoothRecording:
        data['motion'] = {f.name: _pack(getattr(recording.smooth_motion, f.name))
                          for f in fields(Trajectory)}
    elif type(recording) is not trajectory.PlaybackTrajectory:
        raise ValueError('Unsupported recording result')
    return data


def _decode(data, kind, path, model, zone, lower, upper, center):
    base = _unpack(data['base'])
    if base['source'] != Path(path).expanduser().resolve():
        raise ValueError('Cache source mismatch')
    if kind == 'legacy':
        if 'motion' in data:
            raise ValueError('Cache mode mismatch')
        return trajectory.PlaybackTrajectory(**base)
    motion = Trajectory(**_unpack(data['motion']))
    if len(motion.curves) != 7 or not all(type(p) is PPoly for p in motion.curves):
        raise ValueError('Invalid cached curves')
    g = Geometry.__new__(Geometry)
    g.model, g.zone = model, zone
    g.lower, g.upper, g.center = map(np.array, (lower, upper, center))
    return smooth_recording.SmoothRecording(**base, smooth_motion=motion, geometry=g)


def _read(path, key):
    with path.open('rb') as stream:
        raw = stream.read(MAX_CACHE_BYTES + 1)
    if len(raw) > MAX_CACHE_BYTES:
        raise ValueError('Cache too large')
    envelope = json.loads(raw)
    if envelope['format'] != CACHE_FORMAT or envelope['key'] != key:
        raise ValueError('Stale cache')
    payload = envelope['payload']
    if hashlib.sha256(_json_bytes(payload)).hexdigest() != envelope['sha256']:
        raise ValueError('Damaged cache')
    return payload


def _write(path, key, payload):
    raw = _json_bytes({'format': CACHE_FORMAT, 'key': key, 'payload': payload,
                      'sha256': hashlib.sha256(_json_bytes(payload)).hexdigest()})
    if len(raw) > MAX_CACHE_BYTES:
        return
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.preflight-', suffix='.tmp', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def _load(kind, loader, path, model, zone, lower, upper, center, speed,
          playback_speed, progress, cache_dir):
    args = (path, model, zone, lower, upper, center, speed)
    key_args = (kind, *args, playback_speed)
    key = entry = None
    try:
        key = _fingerprint(*key_args)
        entry = (cache_directory() if cache_dir is None else Path(cache_dir)) / (key + '.json')
        cached = _decode(_read(entry, key), kind, *args[:-1])
        if _fingerprint(*key_args) != key:
            raise ValueError('Inputs changed while reading cache')
    except Exception:
        # Cache failures never authorize anything, nor reject an otherwise valid
        # recording: the original preflight remains the sole authority.
        pass
    else:
        if progress:
            progress('Loaded cached recording preflight (all inputs unchanged)')
        return cached

    # Deliberately outside the cache exception handlers: preserve the original
    # loader's exact checks, progress, errors, trajectory and mode behavior.
    result = loader(*args, playback_speed=playback_speed, progress=progress)
    if key is not None and entry is not None:
        try:
            if _fingerprint(*key_args) == key:
                _write(entry, key, _encode(result))
        except Exception:
            pass  # Read-only/full disks cannot turn a successful preflight into failure.
    return result


def load_cached_smooth_recording(path, model, zone, lower, upper, center, speed,
                                 playback_speed=.8, progress=None, *, cache_dir=None):
    return _load('smooth', smooth_recording.load_smooth_recording, path, model,
                 zone, lower, upper, center, speed, playback_speed, progress, cache_dir)


def load_cached_playback_trajectory(path, model, zone, lower, upper, center, speed,
                                   progress=None, playback_speed=None, *, cache_dir=None):
    return _load('legacy', trajectory.load_playback_trajectory, path, model,
                 zone, lower, upper, center, speed, playback_speed, progress, cache_dir)
