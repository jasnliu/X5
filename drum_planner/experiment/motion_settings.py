"""Explicit per-joint experiment constraints. Never motor/firmware settings."""
import hashlib
import json
from pathlib import Path

import numpy as np

from ..common import ROOT, arm_side, atomic_json

SCHEMA = 'x5-experiment-motion-profile-v1'
FIELDS = ('velocity_rad_s', 'acceleration_rad_s2', 'jerk_rad_s3')
QUALITIES = {'fast': 'Fast', 'balanced': 'Balanced (recommended)', 'thorough': 'Thorough'}
DEFAULT_LIMITS = np.repeat(np.array([.6, 1.2, 6.])[:, None], 7, axis=1)
PROFILE_ROOT = ROOT/'config/experiment_motion_profiles'


def checked_limits(values):
    raw=np.asarray(values,dtype=object)
    if any(isinstance(x,(bool,np.bool_)) for x in raw.ravel()):
        raise ValueError('Joint limits must be numbers, not booleans')
    limits = np.asarray(values, dtype=float)
    if limits.shape != (3, 7) or not np.isfinite(limits).all() or np.any(limits <= 0):
        raise ValueError('Enter positive finite velocity, acceleration and jerk limits for all seven joints')
    return limits.copy()


def settings(side='right', values=None):
    side = arm_side(side)
    data = {} if values is None else dict(values)
    if data.get('schema', SCHEMA) != SCHEMA:
        raise ValueError('Unknown experiment motion-profile schema')
    if data.get('side', side) != side:
        raise ValueError(f'Motion profile belongs to the other arm; expected {side}')
    if data.get('simulation_only', True) is not True:
        raise ValueError('Motion profiles are simulation-only, not hardware authorization')
    unknown = set(data)-{'schema', 'side', 'name', 'timing_quality', 'simulation_only', *FIELDS}
    if unknown:
        raise ValueError('Unknown motion settings: '+', '.join(sorted(unknown)))
    supplied = [key in data for key in FIELDS]
    if any(supplied) and not all(supplied):
        raise ValueError('A motion profile must specify all three limit rows')
    limits = checked_limits([data[key] for key in FIELDS] if all(supplied) else DEFAULT_LIMITS)
    quality = data.get('timing_quality', 'balanced')
    if quality not in QUALITIES:
        raise ValueError('Unknown timing quality')
    name = data.get('name', 'Original limits')
    if not isinstance(name, str) or not name.strip() or len(name) > 120:
        raise ValueError('Profile name must contain 1–120 characters')
    return dict(schema=SCHEMA, side=side, name=name, simulation_only=True,
                timing_quality=quality, **dict(zip(FIELDS, limits.tolist())))


def matrix(profile):
    return checked_limits([profile[key] for key in FIELDS])


def identity(profile):
    # Display name does not affect the optimization; side and policy do.
    data={k:v for k,v in profile.items() if k != 'name'}
    return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).hexdigest()


def load(path, side):
    return settings(side, json.loads(Path(path).read_text()))


def save(path, profile):
    validated = settings(profile['side'], profile)
    path = Path(path)
    if path.suffix.lower() != '.json':
        raise ValueError('Save motion profiles as .json files')
    atomic_json(path, validated)


def from_context(context):
    return settings(context.get('moving_side', 'right'), context.get('motion_settings'))


def summary(laws, limits):
    peaks=np.max([np.asarray(law['peaks']) for law in laws], axis=0)
    use=peaks/limits
    order=np.argsort(use.ravel())[::-1]
    active=[dict(joint=int(i%7+1), derivative=FIELDS[int(i//7)],
                 peak=float(peaks.ravel()[i]), limit=float(limits.ravel()[i]),
                 utilization=float(use.ravel()[i])) for i in order[:3]]
    return dict(peaks=peaks.tolist(), limits=limits.tolist(), utilization=use.tolist(),
                highest_utilization=active, interpretation='Near-active constraints, not proof of a global optimum')
