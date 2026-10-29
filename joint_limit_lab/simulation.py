"""Synthetic trace generator for software validation, NOT a calibrated robot model."""
import json
from pathlib import Path

import numpy as np

from drum_planner.experiment.motion_settings import matrix
from . import trajectories
from .analysis import score
from .schema import write,identity,VERSION,defaults,validate


def trace(campaign,trial,*,failure=None,delay_s=.003):
    config=campaign['config'];law=trial['law'];dt=config['sample_period_s']
    t=np.unique(np.r_[np.arange(0,law['duration_s'],dt),law['duration_s']])
    desired=trajectories.evaluate(law,t)[0]
    q,v,_,_=trajectories.evaluate(law,np.maximum(0,t-delay_s))
    n=len(t);rng=np.random.default_rng(int(trial['sha256'][:8],16))
    q=q+rng.normal(0,2e-5,q.shape);v=v+rng.normal(0,.0003,v.shape)
    data=dict(time_s=t,command_time_s=t.copy(),command_q_rad=desired,q_rad=q,velocity_rad_s=v,
        current_a=np.full((n,7),.12),temperature_c=np.full((n,7),28.)+t[:,None]*.01,
        fault_bits=np.zeros((n,7)),mode=np.full((n,7),2),feedback_age_s=np.zeros((n,7)),
        feedback_sequence=np.repeat(np.arange(n)[:,None],7,axis=1),
        current_age_s=np.zeros((n,7)),torque_nm=np.full((n,7),.1))
    j=trial['joint']-1;mid=n//2
    if failure=='tracking':data['q_rad'][n//4:mid,j]+=.08
    elif failure=='stale':data['feedback_sequence'][n//4:mid,j]=data['feedback_sequence'][n//4,j]
    elif failure=='temperature':data['temperature_c'][mid:,j]=70.
    elif failure=='current':data['current_a'][mid:,j]=config['acceptance']['current_limits_a'][j]*1.1
    elif failure=='motor_fault':data['fault_bits'][mid,j]=1
    elif failure=='slow':
        data['q_rad'],data['velocity_rad_s'],_,_=trajectories.evaluate(law,t*.7)
        data['velocity_rad_s']*=.7
    elif failure=='command_gap':data['command_time_s'][mid:]=t[mid]
    elif failure is not None:raise ValueError('Unknown synthetic failure')
    metadata=dict(schema=VERSION,origin='synthetic',campaign_sha256=campaign['sha256'],
        trial_sha256=trial['sha256'],trial_id=trial['id'],side=config['side'],conditions=config['conditions'],
        description='Generated test signals; not a robot physics prediction or physical measurement',
        motor_commands=0,hardware_access=False)
    return metadata,data


def save_trace(path,metadata,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('xb') as f:np.savez_compressed(f,metadata=json.dumps(metadata,allow_nan=False),**data)


def simulate(campaign,output,*,failure=None,fail_level=.8):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    blocked=set();rows=[];written=0
    for trial in campaign['trials']:
        if trial['group'] in blocked:continue
        injected=failure if trial['level']>=fail_level else None
        meta,data=trace(campaign,trial,failure=injected)
        result=score(campaign,trial,meta,data)
        path=output/(trial['id']+'.npz')
        if written+sum(v.nbytes for v in data.values())>256*1024**2:
            raise ValueError('Synthetic trace storage budget (256 MiB) reached; split the study')
        save_trace(path,meta,data);written+=path.stat().st_size
        rows.append(dict(id=trial['id'],passed=result['passed'],reasons=result['reasons']))
        if not result['passed']:blocked.add(trial['group'])
    report=dict(synthetic=True,hardware_access=False,motor_commands=0,trials=rows,
        stopped_groups=sorted(blocked),not_run_after_failure=len(campaign['trials'])-len(rows))
    write(output/'simulation.json',report)
    return report


def demo_campaign(side):
    """Complete fake coverage fixture; deliberately has NO real-world clearance proof."""
    config=defaults(side);config.update(test_levels=[.5,.75,1.],displacements_rad=[.04],repeats=2)
    config['conditions']=dict(payload_id='SYNTHETIC',controller_id='SYNTHETIC',supply_voltage_v=48.,notes='Fake conditions; not measured')
    validate(config);trials=[];anchors=[dict(name='fake_pose_a',q=[0.]*7),dict(name='fake_pose_b',q=[.1]*7)]
    for pi,pose in enumerate(anchors):
        for joint in range(1,8):
            for direction in (-1,1):
                group=f'p{pi}-j{joint}-d{direction:+d}-a0'
                for li,level in enumerate(config['test_levels']):
                    law=trajectories.probe(pose['q'],joint,direction,.04,
                        matrix(config['profile'])*level**np.arange(1,4)[:,None],config['settle_s'])
                    for repeat in range(config['repeats']):
                        trial=dict(id=f'{group}-l{li}-r{repeat}',group=group,pose_index=pi,pose_name=pose['name'],
                            joint=joint,direction=direction,level_index=li,level=level,repeat=repeat,law=law,
                            geometry_check=dict(synthetic=True,not_a_collision_proof=True),hardware_execution_available=False)
                        trial['sha256']=identity(trial);trials.append(trial)
    campaign=dict(schema=VERSION,kind='prepared_campaign',config=config,config_sha256=identity(config),
        anchors=anchors,joints=list(range(1,8)),trials=trials,blocked=[],request={},request_sha256=identity({}),
        synthetic=True,recorded_world_only=False,hardware_execution_available=False,
        interpretation='Synthetic numerical fixture, not robot geometry or live readiness')
    campaign['sha256']=identity(campaign)
    return campaign
