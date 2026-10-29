"""Reuse validated snapshot metadata and model bytes within ONE checkpoint job.

Not a persistent scene cache. Per-leg starts, goals and snapshot self-masking
remain explicit; native worlds/checks are still built from each exact request.
"""
import copy
from pathlib import Path
from .. import planning


def request(context,folder,state):
    side=context.get('moving_side','right');folder=Path(folder)
    shared=context.get('_sequence_inputs')
    if not shared:
        from .rgbd import scene
        world=scene(context['scene'],context['calibration'])
        req,manifest=planning.request(folder,state,world,context['calibration'],
            moving_side=side,zone_path=context.get('zone_path'),
            **(dict(both_tools=True) if context.get('both_tools') else {}),
            **(dict(reference_right=context.get('scene_robot_start',state['right_rad'])) if side=='right' else {}))
        if shared is not None:
            shared.update(request=copy.deepcopy(req),manifest=copy.deepcopy(manifest),
                files={n:(folder/'model'/n).read_bytes() for n in
                       ('robot.urdf','robot.srdf','model_manifest.json')})
    else:
        req=copy.deepcopy(shared['request']);manifest=copy.deepcopy(shared['manifest'])
        target=folder/'model';target.mkdir(parents=True,exist_ok=True)
        for name,data in shared['files'].items():(target/name).write_bytes(data)
        req.update(urdf=str((target/'robot.urdf').resolve()),srdf=str((target/'robot.srdf').resolve()),
            start=state[side+'_rad'],left=state['left_rad'],left_opening_m=state['left_gripper_opening_m'])
        if side=='left':req['right']=state['right_rad']
    if 'scene_robot_poses' in context:
        from ..common import read_state,digest
        original=read_state(context['acquired_state'])
        if (digest(context['acquired_state'])!=context['acquired_state_sha256'] or
                context['scene_robot_poses']!={s:original[s+'_rad'] for s in ('left','right')}):
            raise ValueError('Both-arm self-mask must match the original measured acquisition')
        req['scene_robot_poses']=copy.deepcopy(context['scene_robot_poses'])
    return req,manifest
