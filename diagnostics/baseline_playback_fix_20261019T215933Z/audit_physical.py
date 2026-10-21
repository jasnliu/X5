import collections,json,struct,sys
from pathlib import Path
import numpy as np
from safe_zone.encoder import encoder_to_joint,request_frame
from drum_planner.common import atomic_json
folder=Path(sys.argv[1]);result=json.loads((folder/'result.json').read_text())
owner=Path(result['controller']['folder']);events=[json.loads(s) for s in (owner/'events.jsonl').read_text().splitlines()]
rest=next(e['monotonic'] for e in events if e['phase']=='VERIFIED_START_POSITION')
prepared=json.loads(Path(result['controller']['prepared']).read_text());q0=np.array(prepared['request']['start'])
counts=collections.Counter();disables=[];left_actuators=[];faults=[];states={};temperatures=[];pre_release=None
queries={request_frame(i) for i in range(1,9)}
paths=[owner/f'execution_can_audit.jsonl.{i}' for i in (3,2,1)]+[owner/'execution_can_audit.jsonl']
for path in paths:
 if not path.exists():continue
 for line in path.open():
  o=json.loads(line);frame=bytes.fromhex(o['packet_hex']);cid,dlc,p=struct.unpack('=IB3x8s',frame);kind=(cid>>24)&31
  if o['event']=='tx_sent':
   if frame in queries:counts['state_query']+=1;continue
   counts[f'{o["side"]}_kind_{kind}']+=1
   if o['side']=='left':left_actuators.append(o)
   if kind==4:
    disables.append(dict(t=o['monotonic'],motor=cid&255));assert o['monotonic']>=rest,'Midair disable!'
    if pre_release is None:pre_release=[states['right',i]['q'] for i in range(1,8)]
   if kind==18 and cid&255==8 and p[:2]==b'\x16\x70':
    from centering.motors import RIGHT_GRIPPER_CLOSED
    assert abs(struct.unpack('<f',p[4:])[0]+RIGHT_GRIPPER_CLOSED)<1e-6,'Gripper opening!'
  elif o['event']=='rx' and kind==2 and cid&0x80000000:
   i=(cid>>8)&255
   if i not in range(1,9):continue
   if (cid>>16)&63:faults.append(o)
   if o['side']=='left':assert (cid>>22)&3==0, 'Left arm enabled during trial'
   temperature=int.from_bytes(p[6:8],'big')*.1;temperatures.append(temperature)
   raw=int.from_bytes(p[:2],'big')/65535*25.14-12.57
   states[o['side'],i]=dict(q=encoder_to_joint(o['side'],i,raw) if i<=7 else -raw,mode=(cid>>22)&3,t=o['monotonic'])
assert result['status']['phase']=='RELAXED' and result['status']['disabled']
assert len(disables)==8 and sorted(x['motor'] for x in disables)==list(range(1,9))
assert not left_actuators and not faults
assert all(states['right',i]['mode']==0 for i in range(1,9))
assert all(states['left',i]['mode']==0 for i in range(1,9))
error=np.degrees(np.array(pre_release)-q0);assert max(abs(error))<=.2+1e-8
hold=None
if result['trial']=='normal':
 a=next(e['monotonic'] for e in events if e['phase']=='GOAL_HOLD');b=next(e['monotonic'] for e in events if e['phase']=='RETURN')
 hold=b-a;assert hold>=2
 assert not any(e['phase'] in ('FAULT_HOLD','HELD') for e in events)
else:
 assert any(e['phase']=='HELD' for e in events) and any(e['phase']=='RECOVERY' for e in events)
assert not result['last_error']
r=dict(passed=True,trial=result['trial'],phases=[e['phase'] for e in events],goal_hold_s=hold,
       peak_tracking_error_deg=result['status']['peak_tracking_error_deg'],pre_release_error_deg=error.tolist(),
       right_disabled_confirmed=True,left_disabled_throughout=True,left_actuator_commands=0,
       disable_commands=len(disables),all_disables_after_verified_rest=True,motor_fault_frames=0,
       maximum_temperature_c=max(temperatures),packets=dict(counts),owner=str(owner))
atomic_json(folder/'wire_audit_report.json',r);print(json.dumps(r,indent=2))
