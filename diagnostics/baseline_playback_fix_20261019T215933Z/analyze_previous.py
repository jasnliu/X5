import json,struct,sys
from pathlib import Path
import numpy as np
from safe_zone.encoder import encoder_to_joint
from drum_planner.common import atomic_json
folder=Path('planner_runs/20261019T215312Z-baseline-ba479b/controller-c80996aa');stop=258858.003796914
q={};target={};latest={};rows=[]
for path in [folder/'execution_can_audit.jsonl.2',folder/'execution_can_audit.jsonl.1',folder/'execution_can_audit.jsonl']:
 for line in path.open():
  o=json.loads(line);t=o['monotonic']
  if t>stop:break
  cid,n,p=struct.unpack('=IB3x8s',bytes.fromhex(o['packet_hex']));kind=(cid>>24)&31
  if o['side']!='right':continue
  if o['event']=='rx' and kind==2:
   i=(cid>>8)&255
   if i<=7:q[i]=encoder_to_joint('right',i,int.from_bytes(p[:2],'big')/65535*25.14-12.57);latest[i]=t
  if o['event']=='tx_sent' and kind==18 and p[:2]==b'\x16\x70':
   i=cid&255
   if i<=7:target[i]=encoder_to_joint('right',i,struct.unpack('<f',p[4:])[0])
  if len(q)==len(target)==7:
   error=np.array([target[i]-q[i] for i in range(1,8)])
   rows.append(dict(t=t,errors_deg=np.degrees(error).tolist(),l1_rad=float(sum(abs(error)))))
r=max((r for r in rows if stop-.03<r['t']<=stop),key=lambda r:r['l1_rad']);print(r);atomic_json(Path(sys.argv[1])/'previous_following_fault.json',r)
