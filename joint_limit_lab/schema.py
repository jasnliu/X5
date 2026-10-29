"""Explicit study settings and provenance, not actuator ratings or protections."""
import hashlib
import json
from pathlib import Path

import numpy as np

from drum_planner.experiment.motion_settings import settings, matrix

VERSION = 'x5-joint-limit-study-v1'


def read(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError('Nonfinite JSON: '+x)))


def write(path, data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    content=json.dumps(data,indent=2,allow_nan=False)+'\n'
    temporary=path.with_suffix(path.suffix+'.tmp');temporary.write_text(content);temporary.replace(path)


def identity(data):
    return hashlib.sha256(json.dumps(data,sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def array(value, shape, name, *, positive=False):
    raw=np.asarray(value,dtype=object)
    if any(isinstance(x,(bool,np.bool_)) for x in raw.ravel()):raise ValueError(name+' cannot contain booleans')
    a=np.asarray(value,float)
    if a.shape!=shape or not np.isfinite(a).all() or (positive and np.any(a<=0)):
        raise ValueError(f'{name}: expected finite {shape}'+(' positive values' if positive else ''))
    return a


def number(value, name, lo, hi):
    if isinstance(value,bool) or not isinstance(value,(float,int)) or not np.isfinite(value) or not lo<=value<=hi:
        raise ValueError(f'{name} must be in [{lo}, {hi}]')
    return float(value)


def defaults(side):
    return dict(schema=VERSION,side=side,profile=settings(side),
        test_levels=[.4,.6,.8,1.],displacements_rad=[.04,.08],repeats=3,
        sample_period_s=.01,settle_s=.4,rest_between_trials_s=1.,
        operating_speed_fraction=.8,
        acceptance=dict(max_tracking_rad=.025,rms_tracking_rad=.01,
            max_held_joint_drift_rad=.01,max_velocity_error_rad_s=.12,
            end_position_error_rad=.005,end_velocity_rad_s=.03,
            max_feedback_gap_s=.03,max_command_gap_s=.03,max_current_age_s=.15,
            max_temperature_c=65.,max_temperature_rise_c=10.,
            current_limits_a=[6.,6.,4.,4.,2.,2.,2.],max_current_fraction=.9,
            minimum_peak_tracking_fraction=.9,minimum_motion_samples=20),
        conditions=dict(payload_id='UNSPECIFIED',controller_id='UNSPECIFIED',
            supply_voltage_v=None,notes='Fill in actual payload/controller/supply before physical tests.'),
        interpretation='Test ceilings and analysis criteria, NOT measured hardware capabilities. '
            'Original software ceilings only; do not increase without an agreed physical test envelope.')


def validate(data):
    d=dict(data)
    if d.get('schema')!=VERSION or d.get('side') not in ('right','left'):raise ValueError('Unknown study schema/arm')
    expected=set(defaults(d['side']))
    if set(d)!=expected:raise ValueError('Missing/unknown study settings: '+str(set(d)^expected))
    d['profile']=settings(d['side'],d['profile']);matrix(d['profile'])
    levels=array(d['test_levels'],(len(d['test_levels']),),'test_levels',positive=True)
    if not 1<=len(levels)<=12 or np.any(np.diff(levels)<=0) or levels[-1]>1:
        raise ValueError('Use 1–12 strictly increasing levels in (0,1], relative to explicit ceilings')
    distances=array(d['displacements_rad'],(len(d['displacements_rad']),),'displacements_rad',positive=True)
    if not 1<=len(distances)<=6 or np.any(np.diff(distances)<=0) or distances[-1]>1.:
        raise ValueError('Use 1–6 increasing probe displacements, at most 1 rad; no automatic expansion')
    if type(d['repeats']) is not int or not 2<=d['repeats']<=10:raise ValueError('Use 2–10 independent repeats')
    number(d['sample_period_s'],'sample_period_s',.002,.02)
    number(d['settle_s'],'settle_s',.2,5.)
    number(d['rest_between_trials_s'],'rest_between_trials_s',.2,60.)
    number(d['operating_speed_fraction'],'operating_speed_fraction',.1,.95)
    a=d['acceptance'];expected=set(defaults(d['side'])['acceptance'])
    if set(a)!=expected:raise ValueError('Missing/unknown acceptance criteria')
    for key in expected-{'current_limits_a','minimum_motion_samples'}:number(a[key],key,1e-6,1000.)
    array(a['current_limits_a'],(7,),'current_limits_a',positive=True)
    if a['max_temperature_c']>65.:raise ValueError('Existing 65 C protection must not be weakened')
    for key in ('max_current_fraction','minimum_peak_tracking_fraction'):
        number(a[key],key,.1,1.)
    if type(a['minimum_motion_samples']) is not int or not 10<=a['minimum_motion_samples']<=1000:
        raise ValueError('Invalid minimum_motion_samples')
    if a['max_feedback_gap_s']<d['sample_period_s']:raise ValueError('Feedback gap is shorter than sample period')
    c=d['conditions']
    if set(c)!=set(defaults(d['side'])['conditions']):raise ValueError('Missing/unknown condition fields')
    for key in ('payload_id','controller_id','notes'):
        if not isinstance(c[key],str) or not c[key].strip():raise ValueError('Invalid '+key)
    if c['supply_voltage_v'] is not None:number(c['supply_voltage_v'],'supply_voltage_v',1.,100.)
    return d
