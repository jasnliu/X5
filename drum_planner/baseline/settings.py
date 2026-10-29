"""Hash-bound hardware planning limits, NOT firmware settings or motor ratings.

Uses the same numerical profile as experiments. The hardware envelope is
explicit, separately serialized, and checked again by the execution packet.
"""
import hashlib,json
from pathlib import Path
import numpy as np
from ..common import atomic_json
from ..experiment import motion_settings as numerical

SCHEMA='x5-hardware-planning-profile-v1'
PIPELINE='workspace-current-v1'
FIELDS=numerical.FIELDS
QUALITIES=numerical.QUALITIES
DEFAULT_LIMITS=numerical.DEFAULT_LIMITS
PROFILE_ROOT=numerical.PROFILE_ROOT.parent/'hardware_motion_profiles'


def settings(side='right',values=None):
    data={} if values is None else dict(values)
    schema=data.pop('schema',SCHEMA)
    if schema not in (SCHEMA,numerical.SCHEMA):raise ValueError('Unknown planning profile schema')
    if schema==numerical.SCHEMA:
        data=numerical.settings(side,dict(data,schema=schema))
        data.pop('schema');data.pop('simulation_only')
    for key in ('firmware_settings_changed','hardware_limits_verified'):
        if data.pop(key,False) is not False:raise ValueError('A planning profile cannot certify hardware or change firmware')
    parsed=numerical.settings(side,data)
    parsed.pop('simulation_only');parsed['schema']=SCHEMA
    return dict(parsed,firmware_settings_changed=False,hardware_limits_verified=False)


def for_optimizer(profile):
    checked=settings(profile['side'],profile)
    return numerical.settings(checked['side'],{k:checked[k] for k in ('name','timing_quality',*FIELDS)})


def matrix(profile):return numerical.matrix(profile)


def identity(profile):
    checked=settings(profile['side'],profile)
    return hashlib.sha256(json.dumps({k:v for k,v in checked.items() if k!='name'},sort_keys=True,allow_nan=False).encode()).hexdigest()


def load(path,side):return settings(side,json.loads(Path(path).read_text()))


def save(path,profile):
    path=Path(path)
    if path.suffix.lower()!='.json':raise ValueError('Save planning profiles as .json files')
    atomic_json(path,settings(profile['side'],profile))


def plan_limits(plan):
    """Legacy packets keep their original limits; current packets bind all legs."""
    if plan.get('hardware_planning_pipeline')!=PIPELINE:
        if 'hardware_motion_profile' in plan:raise ValueError('Unrecognized hardware profile/pipeline')
        return DEFAULT_LIMITS.copy()
    profile=settings(plan.get('moving_side','right'),plan['hardware_motion_profile'])
    if identity(profile)!=plan.get('hardware_motion_profile_sha256'):raise ValueError('Hardware motion profile changed')
    limits=matrix(profile)
    sequence=plan.get('checkpoint_sequence')
    laws=[s['timing'] for s in sequence['segments']+[sequence['return_leg']]] if sequence else [plan['timing']]
    if any(not np.array_equal(law.get('limits'),limits) for law in laws):raise ValueError('Timing law/profile limits differ')
    for key,row in zip(('velocity_limit_rad_s','acceleration_limit_rad_s2','jerk_limit_rad_s3'),limits):
        if not np.array_equal(plan.get(key),row):raise ValueError('Reported hardware limits differ from validated profile')
    return limits
