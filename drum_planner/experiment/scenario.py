"""Immutable simulated starts, separate from measured perception self-masking."""
import copy
import json
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

from ..common import atomic_json, digest, endpoint, read_state, arm_joints
from . import destinations
from .checkpoints import derived_state


def resolve(value, side):
    if value is None or str(value).lower() == 'default':
        return None
    path=Path(value)
    if path.is_file():
        endpoint(path, side=side)
        return path.resolve()
    matches=[p for p in destinations.files(side)
             if str(value) in (p.name, p.stem, p.stem.removeprefix('destination'))]
    if len(matches) != 1:
        raise ValueError(f'Unknown {side} destination: {value}')
    endpoint(matches[0], side=side)
    return matches[0]


def ensure(session):
    session=Path(session)
    context=json.loads((session/'context.json').read_text())
    if 'acquired_state' not in context:
        context.update(acquired_state=context['state'], acquired_state_sha256=context['state_sha256'],
                       scene_robot_start=read_state(context['state'])[context['moving_side']+'_rad'],
                       experiment_start=dict(kind='default', label='Default', destination_source=None))
        saved=session/'acquired_initial';saved.mkdir(exist_ok=True)
        for name in ('plan.json', 'backend_request.json'):
            (saved/name).write_bytes((session/'initial'/name).read_bytes())
        atomic_json(session/'context.json', context)
    if digest(context['acquired_state']) != context['acquired_state_sha256']:
        raise ValueError('Acquired state changed; relaunch with an intact snapshot')
    return context


def set_start(session, destination=None):
    """Only update saved preview inputs; never acquire devices or send commands."""
    session=Path(session);context=ensure(session);side=context['moving_side']
    acquired=read_state(context['acquired_state'])
    target=resolve(destination, side)
    if target is None:
        state=acquired;path=context['acquired_state']
        binding=dict(kind='default', label='Default', destination_source=None)
    else:
        frozen=destinations.freeze(target, side, session)
        pose=endpoint(frozen['goal_recording'], side=side).tolist()
        req=json.loads((session/'acquired_initial/backend_request.json').read_text())
        model=ET.parse(req['urdf']).getroot()
        for name,value in zip(arm_joints(side),pose):
            limit=model.find(f"./joint[@name='{name}']/limit")
            if limit is None or not float(limit.get('lower'))<=value<=float(limit.get('upper')):
                raise ValueError(f'Simulated start {name} is outside its calibrated position limits')
        state=derived_state(acquired, side, pose, context['acquired_state_sha256'])
        state['source']='Simulated destination spawn; NOT measured, NOT motor movement'
        path=str((session/'simulated_starts'/uuid.uuid4().hex[:12]/'state.json').resolve())
        Path(path).parent.mkdir(parents=True,exist_ok=True)
        atomic_json(path, state)
        binding=dict(kind='destination', label=target.stem, **frozen)
    context.update(state=str(path), state_sha256=digest(path), experiment_start=binding,
                   scene_robot_start=acquired[side+'_rad'])
    initial=session/'acquired_initial'
    req=json.loads((initial/'backend_request.json').read_text())
    req.update(start=state[side+'_rad'], scene_robot_start=acquired[side+'_rad'])
    if side == 'left':req['left']=state['left_rad']
    plan=json.loads((initial/'plan.json').read_text())
    plan.update(start_state=copy.deepcopy(state), experiment_start=binding)
    plan['provenance'].update(state=str(path), state_sha256=digest(path),
                              acquired_state=context['acquired_state'],
                              acquired_state_sha256=context['acquired_state_sha256'])
    # Existing preview readers expect these filenames. Running jobs have already
    # snapshotted their context and refer to immutable private state files.
    atomic_json(session/'initial/backend_request.json', req)
    atomic_json(session/'initial/plan.json', plan)
    atomic_json(session/'context.json', context)
    return context


def rebase_acquisition(session):
    """After explicit camera-only recapture, keep the original measured pose.

    The caller installs a new initial plan built from acquired_state, never the
    teleported state. Save it as the new preview template before reapplying spawn.
    """
    session=Path(session)
    for name in ('plan.json', 'backend_request.json'):
        (session/'acquired_initial'/name).write_bytes((session/'initial'/name).read_bytes())
