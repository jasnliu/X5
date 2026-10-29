"""Freeze saved choices at Calculate; resolve right dt1 only on its new capture."""
import hashlib,json
from pathlib import Path
from ..common import atomic_json
from ..experiment import checkpoints,destinations,dt1_goal

SCHEMA='x5-hardware-dt1-selection-v1'


def signature(entries):
    return hashlib.sha256(json.dumps(entries,sort_keys=True,allow_nan=False).encode()).hexdigest()


def freeze(paths,side,session):
    if dt1_goal.TOKEN not in paths:return checkpoints.freeze(paths,side,session)
    if side!='right':raise ValueError('dt1 is right-arm only')
    if not 1<=len(paths)<=3 or any(not p for p in paths):raise ValueError('Select one to three destinations')
    if any(a==b for a,b in zip(paths,paths[1:])):raise ValueError('Consecutive destinations must differ')
    entries=[dict(kind='dt1') if p==dt1_goal.TOKEN else dict(kind='saved',**destinations.freeze(p,side,session)) for p in paths]
    return dict(schema=SCHEMA,entries=entries,selection_sha256=signature(entries))


def has_dt1(binding):return bool(binding and binding.get('schema')==SCHEMA)


def count(binding):
    if not binding:return 1
    return len(binding['entries']) if has_dt1(binding) else max(1,len(binding.get('checkpoints',[])))


def resolve(context,binding,folder):
    if not has_dt1(binding):return binding,None
    if context['moving_side']!='right':raise ValueError('dt1 is right-arm only')
    entries=binding['entries']
    if (not 1<=len(entries)<=3 or signature(entries)!=binding['selection_sha256']
            or not any(e.get('kind')=='dt1' for e in entries)):
        raise ValueError('DT1 selection changed after Calculate')
    selected=[]
    for e in entries:
        if e.get('kind')=='dt1':
            if e!={'kind':'dt1'}:raise ValueError('dt1 must come from this capture, not a stored goal')
            selected.append('dt1')
        elif e.get('kind')=='saved':selected.append(str(destinations.goal(dict(e,moving_side='right'))[0]))
        else:raise ValueError('Unknown destination type')
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    # This is the existing no-device calculation subprocess, not another owner
    # or concurrent heavy job. Inference replays the capture; no camera opens.
    dt1_goal.generate(folder,context,selected)
    resolved,proof=dt1_goal.result(folder,context,selected)
    final=checkpoints.freeze(resolved,'right',folder)
    bindings=final.get('checkpoints',[final])
    for item,entry in zip(bindings,entries):
        if entry['kind']=='dt1':item['destination_kind']='dt1'
        else:item['destination_source']=entry['destination_source']
    if len(bindings)>1:final.update(bindings[-1])
    atomic_json(folder/'binding.json',final)
    return final,proof


def validate(plan,world):
    """DT1 is derived from this acquisition, never from a previous camera run."""
    from ..common import digest
    bindings=plan.get('destination_bindings',[])
    indices=[i for i,b in enumerate(bindings) if b.get('destination_kind')=='dt1']
    if not indices:return
    from ..experiment.rgbd import SCHEMA as SCENE_SCHEMA
    from cymbal_destination.output import current_convention
    from cymbal_destination.solver import acceptable
    from cymbal_destination import goal_limits
    from ..common import JOINTS
    proof=plan.get('dt1_generation',{})
    expected=proof.get('identity',{})
    scene=Path(plan['provenance']['scene'])
    if (plan.get('moving_side')!='right' or world['schema']!=SCENE_SCHEMA or proof.get('status')!='ready'
        or expected.get('scene_sha256')!=world['source_scene_sha256']
        or expected.get('state_sha256')!=plan['provenance']['state_sha256']
        or expected.get('calibration_sha256')!=plan['calibration_sha256']
        or proof.get('pair_sha256')!=digest(scene.parent/'pair.npz')):
        raise ValueError('DT1 does not belong to this Calculate acquisition')
    for i in indices:
        path=Path(bindings[i]['goal_recording']);obj=json.loads(path.read_text())
        current_convention(obj,allow_legacy_joint_reserve=True)  # Keep existing v4 mission/recovery readable.
        if (obj.get('source_scene_sha256')!=world['source_scene_sha256']
            or obj.get('calibration_sha256')!=plan['calibration_sha256']
            or proof['resolved_sha256'][i]!=digest(path) or not acceptable(obj['target_geometry'])):
            raise ValueError('DT1 endpoint geometry/provenance changed')
        if obj.get('version')!='cymbal-prime-destination-v4':
            mapping=plan['model']['joint_limit_mapping']
            bounds=([mapping[j]['saved_coordinate_lower_rad'] for j in JOINTS],
                    [mapping[j]['saved_coordinate_upper_rad'] for j in JOINTS])
            goal_limits.verify(obj['samples'][0]['positions_rad'],obj.get('goal_joint_reserve'),bounds,
                               allow_legacy=obj.get('version') in ('cymbal-prime-destination-v5','cymbal-prime-destination-v6'))
