"""Explicit experiment contracts and assumed (not measured) error bounds."""
import json
from pathlib import Path
import numpy as np
from ..common import ROOT, read_state, digest

METHODS = {
 'baseline': 'Baseline — weighted route score',
 'clearance': 'Clearance first — widest transit',
 'robust': 'Robust — bounded-error envelope',
 'chance': 'Chance constraint — Monte Carlo scenarios',
 'cvar': 'CVaR — worst-tail scenario loss',
 'families': 'Diverse route families',
 'iris_gcs': 'Sampled C-space IRIS + Drake GCS',
 'chomp': 'CHOMP — native MoveIt optimizer',
 'stomp': 'STOMP — native MoveIt optimizer',
}
DESCRIPTIONS = {
 'baseline': 'Existing 40/30/20/10 workspace, bending, proximity, joint-economy score with a 2x fastest-valid duration cap.',
 'clearance': 'Search increasing distal clearance constraints, then rank bottleneck transit clearance before path economy. End approach tapers only the additional comfort objective; original collision margins never taper.',
 'robust': 'Full-body world padding from bounded geometric errors, plus conservative self-distance and joint-limit envelopes. No physical MPC controller. May legitimately reject a near-limit resting pose.',
 'chance': 'From-scratch candidate search + whole-path Monte Carlo scenario filter. Uses a one-sided Wilson upper bound on the assumed scenario failure rate, NOT a calibrated hardware probability.',
 'cvar': 'From-scratch candidate search + CVaR of worst scenario clearance/collision losses. Optimizes the bad tail rather than mean proximity; nominal full-body validity remains mandatory.',
 'families': 'Joint-space waypoint and independent-route search, geometric route-family clustering, then clearance-first ranking. This is a diversity heuristic, not a proof of distinct mathematical homotopy classes.',
 'iris_gcs': 'IRIS with sampled FCL configuration-space collision witnesses and inward-shrunk cells, iteratively refined regions, then real Drake Graphs of Convex Sets optimization. Regions are sampled approximations, not continuous certificates; the final curve is independently FCL-validated.',
 'chomp': 'Installed MoveIt CHOMP, initialized by a new camera/model-planned path, not a recording. Native distance-field optimization followed by exact same full-body FCL validation as other modes.',
 'stomp': 'Installed MoveIt STOMP, initialized by a new camera/model-planned path. Native stochastic rollouts and smoothing followed by independent dense FCL validation.',
}
BOUND_FIELDS = dict(depth_mm=(0,30),camera_translation_mm=(0,30),camera_rotation_deg=(0,2),
                    joint_deg=(0,1),tool_mm=(0,30),tool_angle_deg=(0,3),scene_motion_mm=(0,30))
PRESETS = {
 'Small (experimental)': dict(depth_mm=1.,camera_translation_mm=1.,camera_rotation_deg=.02,
      joint_deg=.02,tool_mm=1.,tool_angle_deg=.1,scene_motion_mm=1.),
 'Moderate (experimental)': dict(depth_mm=3.,camera_translation_mm=2.,camera_rotation_deg=.05,
      joint_deg=.05,tool_mm=2.,tool_angle_deg=.25,scene_motion_mm=2.),
 'Large (experimental)': dict(depth_mm=8.,camera_translation_mm=5.,camera_rotation_deg=.15,
      joint_deg=.15,tool_mm=4.,tool_angle_deg=.5,scene_motion_mm=5.),
}
DEFAULT = dict(PRESETS['Moderate (experimental)'],scenarios=64,chance_limit=.15,cvar_alpha=.9,seed=19)


def settings(value=None):
    data=dict(DEFAULT);data.update(value or {})
    if set(data)!=set(DEFAULT):raise ValueError('Unknown uncertainty setting')
    for key,(low,high) in BOUND_FIELDS.items():
        if isinstance(data[key],bool) or not np.isfinite(data[key]) or not low<=data[key]<=high:
            raise ValueError(f'{key} must be within {low}–{high}')
        data[key]=float(data[key])
    for key,low,high in [('scenarios',16,512),('seed',1,2**31-1)]:
        if isinstance(data[key],bool) or int(data[key])!=data[key] or not low<=data[key]<=high:
            raise ValueError(f'{key} must be an integer within {low}–{high}')
        data[key]=int(data[key])
    for key,low,high in [('chance_limit',.001,.5),('cvar_alpha',.5,.99)]:
        if isinstance(data[key],bool) or not np.isfinite(data[key]) or not low<=data[key]<=high:
            raise ValueError(f'{key} must be within {low}–{high}')
        data[key]=float(data[key])
    return data


def budget(value):
    value=float(value)
    if not np.isfinite(value) or not 5<=value<=600:raise ValueError('Planning budget must be 5–600 seconds')
    return value


def latest_state(root=ROOT/'planner_runs'):
    # A preview interrupted AFTER successful acquisition is still a valid saved
    # measurement. Reject mere example states and failed/incomplete acquisition.
    for folder in sorted(Path(root).glob('*-current-plan-*'),reverse=True):
        try:
            p=folder/'measured_start.json';s=read_state(p)
            result=json.loads((folder/'plan/plan.json').read_text())
            if (result.get('actual_current_robot_state_verified') is True
                and result.get('status')=='planned'
                and result['provenance']['state_sha256']==digest(p)
                and all(m['mode']==0 and m['fault_bits']==0 for side in ('left','right') for m in s['motor_telemetry'][side])):
                return p.resolve()
        except (OSError,ValueError,KeyError,TypeError):continue
    raise ValueError('No successful saved disabled-arm measurement. Supply --state PATH; no live queries will be attempted.')


def no_can():
    """Fail closed even if a future accidental import tries to open SocketCAN."""
    import socket,sys
    def audit(event,args):
        if event=='socket.__new__' and len(args)>1 and args[1]==getattr(socket,'AF_CAN',29):
            raise PermissionError('Experiment mode forbids CAN sockets')
    sys.addaudithook(audit)
