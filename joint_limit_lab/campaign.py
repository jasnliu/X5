"""Saved-world test preparation; never modifies or operates the original planner."""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from drum_planner.common import endpoint
from drum_planner.experiment.algorithms import dense_check
from drum_planner.experiment.motion_settings import matrix
from drum_planner.experiment.runtime import Oracle
from . import trajectories
from .schema import read,write,validate,identity,file_hash,VERSION


def prepare(config,plan_path,poses,folder,joints=range(1,8)):
    config=validate(config);folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    plan_path=Path(plan_path).resolve();plan=read(plan_path)
    req_path=plan_path.parent/'backend_request.json';req=read(req_path)
    side=config['side']
    if plan.get('moving_side')!=side or req.get('moving_side','right')!=side:raise ValueError('Plan/config arm mismatch')
    if plan.get('status')!='planned' or plan.get('checkpoint_sequence'):raise ValueError('Use a successful single-leg experiment plan')
    if req.get('execute') is not False:raise ValueError('Expected a non-executable saved experiment request')
    if file_hash(req['urdf'])!=plan['model']['calibrated_urdf_sha256'] or file_hash(req['srdf'])!=plan['model']['srdf_sha256']:
        raise ValueError('Saved model bytes changed')
    anchor_items=[]
    if poses:
        for path in poses:
            path=Path(path).resolve();anchor_items.append(dict(name=path.stem,q=endpoint(path,side=side).tolist(),
                source=str(path),sha256=file_hash(path)))
    else:
        anchor_items=[dict(name='route_start',q=req['start']),dict(name='route_goal',q=req['goal'])]
    if not 1<=len(anchor_items)<=6 or len({identity(p['q']) for p in anchor_items})!=len(anchor_items):
        raise ValueError('Use one to six distinct anchor poses')
    joints=list(joints)
    if not joints or len(set(joints))!=len(joints) or any(type(j) is not int or not 1<=j<=7 for j in joints):
        raise ValueError('Invalid joint selection')
    if len(joints)*len(anchor_items)*2*len(config['displacements_rad'])*len(config['test_levels'])*config['repeats']>2500:
        raise ValueError('Campaign exceeds 2500 bounded trials; split it into smaller studies')
    write(folder/'request.json',req)
    trials=[];blocked=[];oracle=Oracle(folder/'request.json',folder)
    bounds=np.column_stack([oracle.lower,oracle.upper]).tolist()
    try:
        for pi,pose in enumerate(anchor_items):
            q=np.array(pose['q']);valid=oracle.check([q.tolist()])
            if not all(valid['valid']):
                blocked.append(dict(pose=pose['name'],reason='anchor_invalid',details=valid.get('details')));continue
            for joint in joints:
                for direction in (-1,1):
                    for di,displacement in enumerate(config['displacements_rad']):
                        target=q.copy();target[joint-1]+=direction*displacement
                        key=f'p{pi}-j{joint}-d{direction:+d}-a{di}'
                        if np.any(target<oracle.lower) or np.any(target>oracle.upper):
                            blocked.append(dict(group=key,pose=pose['name'],joint=joint,direction=direction,reason='position_bounds'));continue
                        # This is a straight one-joint excursion. Check its entire
                        # geometric interval once, with the native edge checker.
                        points=np.linspace(q,target,max(2,int(displacement/.002)+2))
                        ok,proof=dense_check(oracle,points)
                        proof.pop('max_time_step_s',None)
                        proof['spatial_interval_check_only']=True
                        if not ok:
                            blocked.append(dict(group=key,pose=pose['name'],joint=joint,direction=direction,reason='collision_or_zone',proof=proof));continue
                        for li,level in enumerate(config['test_levels']):
                            limits=matrix(config['profile'])*level**np.arange(1,4)[:,None]
                            law=trajectories.probe(q,joint,direction,displacement,limits,config['settle_s'])
                            trajectories.verify(law)
                            for repeat in range(config['repeats']):
                                trial=dict(id=f'{key}-l{li}-r{repeat}',group=key,pose_index=pi,pose_name=pose['name'],
                                    joint=joint,direction=direction,level_index=li,level=level,repeat=repeat,
                                    law=law,geometry_check=proof,hardware_execution_available=False)
                                trial['sha256']=identity(trial);trials.append(trial)
                        print(f'Prepared {key}: {len(config["test_levels"])} levels × {config["repeats"]} repeats',flush=True)
    finally:oracle.close()
    meshes={m.get('filename').removeprefix('file://') for m in ET.parse(req['urdf']).findall('.//collision/geometry/mesh')}
    campaign=dict(schema=VERSION,kind='prepared_campaign',config=config,config_sha256=identity(config),
        anchors=anchor_items,joints=joints,trials=trials,blocked=blocked,
        source_plan=str(plan_path),source_plan_sha256=file_hash(plan_path),request=req,
        request_sha256=identity(req),model=plan['model'],position_bounds_rad=bounds,
        input_files_sha256={str(Path(p).resolve()):file_hash(p) for p in
            [plan_path,req_path,req['urdf'],req['srdf'],req['scene']['points_file'],req['scene']['free_depth_file'],*sorted(meshes)]},
        recorded_world_only=True,hardware_execution_available=False,
        interpretation='Geometric preparation only, not live clearance, hardware ratings or permission to move')
    campaign['sha256']=identity(campaign);write(folder/'campaign.json',campaign)
    print(f'{len(trials)} prepared trials; {len(blocked)} blocked excursions. NO HARDWARE ACCESS.',flush=True)
    return campaign


def load(path):
    data=read(path)
    if data.get('schema')!=VERSION or data.get('kind')!='prepared_campaign':raise ValueError('Invalid campaign')
    unsigned=copy.deepcopy(data);claimed=unsigned.pop('sha256',None)
    if identity(unsigned)!=claimed:raise ValueError('Campaign checksum mismatch')
    validate(data['config'])
    if data['config_sha256']!=identity(data['config']) or data['request_sha256']!=identity(data['request']):raise ValueError('Campaign identity mismatch')
    seen=set()
    for trial in data['trials']:
        item=dict(trial);h=item.pop('sha256',None)
        if identity(item)!=h or trial['id'] in seen:raise ValueError('Trial identity mismatch/duplicate')
        seen.add(trial['id']);trajectories.verify(trial['law'])
    return data
