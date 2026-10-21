"""Replay captured data through native recovery check; NEVER opens devices."""
import json,struct,sys
from pathlib import Path
import numpy as np
from safe_zone.encoder import encoder_to_joint,FRAME
from drum_planner.baseline.tasks import check_return
from drum_planner.experiment.config import no_can
from drum_planner.common import atomic_json
no_can();d=Path(sys.argv[1]);owner=Path('planner_runs/20261019T222544Z-baseline-c0f907/controller-f1d46a7f')
info=json.loads((owner/'control.json').read_text());packet=json.loads(Path(info['prepared']).read_text())
events=[json.loads(s) for s in (owner/'events.jsonl').read_text().splitlines()]
fault=next(e for e in events if e['phase']=='FAULT_HOLD');left={};targets={};latest_right={}
for path in [owner/f'execution_can_audit.jsonl.{i}' for i in (3,2,1)]+[owner/'execution_can_audit.jsonl']:
 if not path.exists():continue
 for line in path.open():
  o=json.loads(line)
  if o['monotonic']>fault['monotonic']:break
  cid,n,p=FRAME.unpack(bytes.fromhex(o['packet_hex']));kind=(cid>>24)&31
  if o['event']=='rx' and o['side']=='left' and kind==2:
   i=(cid>>8)&255;raw=int.from_bytes(p[:2],'big')/65535*25.14-12.57
   left[i]=encoder_to_joint('left',i,raw) if i<=7 else -raw
  elif o['event']=='tx_sent' and o['side']=='right' and kind==18 and p[:2]==b'\x16\x70':
   i=cid&255
   if i<=7:targets[i]=encoder_to_joint('right',i,struct.unpack('<f',p[4:])[0])
assert len(left)==8 and len(targets)==7
snapshot=dict(left=[left[i] for i in range(1,8)],left_opening_m=float(np.clip(.044*left[8]/1.0472,0,.044)))
held=[targets[i] for i in range(1,8)]
request=dict(prepared=info['prepared'],held=held,tau=fault['tau'],rate=info['rate'],left_snapshot=snapshot)
atomic_json(d/'recorded_fault_request.json',request)
print('Original fault:',fault['error']);print('Measured left change (degrees):',np.degrees(np.array(snapshot['left'])-packet['request']['left']))
check_return(d/'recorded_recovery',**request)
result=json.loads((d/'recorded_recovery/return.json').read_text());assert result['proof']['all_valid']
print('SAVED ACTUAL FAULT RECOVERY: PASSED',result['proof'],result['connector_l1_rad'])
