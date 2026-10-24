"""Selected-arm constraint with EXACT legacy gripper-TCP membership semantics.

Right zone3 remains the default and retains its existing serialized contract.
Left experiments explicitly provide their separate left envelope and TCP.

The envelope was taught using the original URDF, not camera-corrected FK or
the custom stick. Keep those coordinates for membership; collisions continue
to use the separately calibrated complete robot/tool model.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from safe_zone.geometry import Model, Zone, FRAME, RIGHT_TCP, LEFT_TCP, MEMBERSHIP_BUFFER_M
from .common import ROOT, JOINTS, digest, arm_joints, arm_side

ZONE = ROOT/'right_zones/zone3.json'
REFERENCE_MODEL = ROOT/'model/openarmx.urdf'


def constraint(zone_path=ZONE, model_path=REFERENCE_MODEL, *, side='right'):
    """Rebuild the recorded hull and compile the legacy world->TCP FK chain.

    Never trust the JSON's imported planes, and never use the calibrated
    planner URDF to interpret a zone recorded with the original model.
    """
    arm_side(side);tcp=RIGHT_TCP if side=='right' else LEFT_TCP;joints=arm_joints(side)
    zone_path=Path(zone_path).resolve();model_path=Path(model_path).resolve()
    model=Model(model_path);zone=Zone.load(zone_path,model.digest,tcp)
    if zone.hull is None:raise ValueError(f'{side.title()} zone must contain a non-degenerate 3-D envelope')
    root=ET.parse(model_path).getroot()
    parents={j.find('child').get('link'):j for j in root.findall('joint')}
    chain=[];link=tcp;seen=set()
    while link!=FRAME:
        if link in seen or link not in parents:raise ValueError('Disconnected/cyclic selected TCP chain')
        seen.add(link);j=parents[link];name=j.get('name');kind=j.get('type')
        index=-1 if kind=='fixed' else joints.index(name) if name in joints else None
        if index is None or kind not in ('fixed','revolute','continuous') or j.find('mimic') is not None:
            raise ValueError('Zone membership must depend only on the seven original selected-arm joints')
        origin=j.find('origin');T=np.eye(4)
        if origin is not None:
            T[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
        axis=np.array([0.,0.,1.]) if index<0 else np.fromstring(j.find('axis').get('xyz'),sep=' ')
        if axis.shape!=(3,) or not np.isfinite(axis).all() or np.linalg.norm(axis)<1e-12:
            raise ValueError('Invalid original selected joint axis')
        axis/=np.linalg.norm(axis)
        chain.append(dict(joint=name,q_index=index,origin=T.tolist(),axis=axis.tolist()))
        link=j.find('parent').get('link')
    chain.reverse()
    if [item['q_index'] for item in chain if item['q_index']>=0]!=list(range(7)):
        raise ValueError('Unexpected original selected TCP joint order')
    spec=dict(schema=f'x5-{side}-zone-constraint-v1',enabled=True,zone_file=str(zone_path),
        zone_sha256=digest(zone_path),reference_urdf=str(model_path),reference_urdf_sha256=model.digest,
        frame=FRAME,tcp=tcp,buffer_m=MEMBERSHIP_BUFFER_M,numerical_tolerance_m=1e-9,
        point_only=True,left_arm_constrained=False,stick_constrained=False,
        kinematics='original URDF, saved joint coordinates; identical to safe_zone.geometry.Model',
        membership='recomputed convex hull; signed plane distance <= buffer_m + 1e-9',
        point_count=len(zone.points),plane_count=len(zone.hull.equations),
        planes=zone.hull.equations.tolist(),kinematic_chain=chain)
    if side=='left':
        del spec['left_arm_constrained']
        spec['right_arm_constrained']=False
    spec['constraint_sha256']=hashlib.sha256(json.dumps(spec,sort_keys=True,allow_nan=False).encode()).hexdigest()
    return spec


def metadata(spec):
    if spec is None:return dict(enabled=False)
    return {key:value for key,value in spec.items() if key not in ('planes','kinematic_chain')}
