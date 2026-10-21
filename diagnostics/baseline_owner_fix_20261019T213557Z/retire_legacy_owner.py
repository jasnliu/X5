"""One-time descriptor-only exit of the identified, already-disabled legacy owner.
No CAN sockets, hold, enable, disable, target, reset or motion requests.
"""
import json,os,socket,struct,sys,time
from pathlib import Path
from drum_planner.baseline.service import Client
from drum_planner.common import atomic_json
real=socket.socket
def unix_only(family=socket.AF_INET,*a,**k):
 if family!=socket.AF_UNIX:raise AssertionError('Only local controller IPC is permitted')
 return real(family,*a,**k)
socket.socket=unix_only
d=Path(sys.argv[1]);folder=Path('planner_runs/20261019T211907Z-baseline-c6eab9/controller-ac12418e').resolve()
info=json.loads((folder/'control.json').read_text());assert info['simulated'] is False
log=folder/'execution_can_audit.jsonl';began=time.monotonic();observations=[]
def latest():
 with log.open('rb') as f:
  f.seek(max(0,os.fstat(f.fileno()).st_size-128*1024));f.readline();lines=f.readlines()
 records=[]
 for line in lines:
  try:records.append(json.loads(line))
  except ValueError:pass
 now=time.monotonic();states={};actuators=[]
 for e in records:
  if now-e['monotonic']>.5:continue
  cid,dlc,payload=struct.unpack('=IB3x8s',bytes.fromhex(e['packet_hex']));kind=cid>>24&31
  if e['event']=='rx' and cid&0x80000000 and not cid&0x60000000 and kind==2 and dlc==8 and 1<=cid>>8&255<=8:
   states[e['side'],cid>>8&255]=(cid>>22&3,cid>>16&63,e['monotonic'])
  if e['event']=='tx_sent' and not (kind==2 and payload==bytes.fromhex('0100000000000000')):actuators.append(e)
 assert not actuators and len(states)==16
 assert all(mode==0 and fault==0 and now-stamp<.15 for mode,fault,stamp in states.values()),states
 return dict(monotonic=now,disabled_motors=len(states),maximum_age_s=max(now-s[2] for s in states.values()))
for _ in range(12):observations.append(latest());time.sleep(.05)
status=json.loads((folder/'status.json').read_text());assert status['phase']=='FAULT_HOLD' and not status['hold_confirmed']
observations.append(latest())
client=Client(info)
try:
 # hello/heartbeat only; neither command can enable or release motors.
 deadline=time.monotonic()+2
 while time.monotonic()<deadline:
  s=client.poll()
  if s.get('phase')=='FAULT_HOLD':break
  time.sleep(.01)
 else:raise AssertionError('Expected old stopped controller')
 observations.append(latest())
 # Legacy IPC spelling: this exits WITHOUT sending disable/relax.
 client.send('supported_exit',confirmation='ARM PHYSICALLY SUPPORTED')
 deadline=time.monotonic()+3
 while not (folder/'final_status.json').exists() and time.monotonic()<deadline:time.sleep(.02)
 final=json.loads((folder/'final_status.json').read_text());assert final['phase']=='SUPPORTED_EXIT'
finally:client.close()
records=[json.loads(x) for x in log.read_text().splitlines() if x]
transmitted=[e for e in records if e['event']=='tx_sent' and e['monotonic']>=began]
for e in transmitted:
 cid,dlc,data=struct.unpack('=IB3x8s',bytes.fromhex(e['packet_hex']))
 assert cid>>24&31==2 and data==bytes.fromhex('0100000000000000'),e
atomic_json(d/'legacy_owner_exit_report.json',dict(passed=True,disabled_feedback_observations=observations,
 commands_sent_to_owner=['hello','heartbeat','supported_exit (descriptor close only)'],
 motor_commands=0,remaining_tx_was_state_queries_only=True,state_queries=len(transmitted),final=final))
print('Old already-disabled owner exited. Audit: zero actuator commands; only remaining state queries.',flush=True)
