"""Read-only, deterministic old/new 100 BPM controller comparison. No devices."""
import ast,importlib.util,json,math,sys,threading
from dataclasses import asdict,replace
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from camera_playback.hybrid_strike import HybridController,load_tuning
from camera_playback.tempo import swing_events
from strike_lab.backends import SimBackend
from strike_lab.config import Plant
from strike_lab.engine import limit_command
before=ROOT/'diagnostics/beat_tempo_20261005/before/camera_playback'
name='camera_playback._pre_bpm_hybrid'
spec=importlib.util.spec_from_file_location(name,before/'hybrid_strike.py')
old=importlib.util.module_from_spec(spec);sys.modules[name]=old;spec.loader.exec_module(old)
select={'STRIKE_BPM','STRIKE_PERIOD_SECONDS','SWING_TRIPLET_SECONDS','SWING_EVENTS'}
nodes=[n for n in ast.parse((before/'app.py').read_text()).body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in select for t in n.targets)]
scope={};exec(compile(ast.Module(body=nodes,type_ignores=[]),'<original 100 BPM>','exec'),scope)
assert scope['SWING_EVENTS']==swing_events(100)
rows=[]
for depth in (5.5,10.5,12.5):
 p,r,l=load_tuning();l=replace(l,hard_depth_deg=13.)
 b=SimBackend(Plant(inertia=p['inertia'],gravity=p['load_torque'],friction=p['friction']))
 anchor,bias=b.prepare(threading.Event(),lambda _:None)
 args=(anchor,anchor-math.radians(13),anchor+math.radians(.6),p,r,l,bias)
 a,c=old.HybridController(*args),HybridController(*args)
 for controller in (a,c):controller.request_search(math.radians(depth))
 stage='search';last_send=None;last_torque=bias;checks=0
 for i in range(18000):
  sample,now=b.receive(),b.now()
  ca,cc=a.update(sample,now),c.update(sample,now)
  assert ca==cc,(depth,i,'command')
  assert asdict(a.status)==asdict(c.status),(depth,i,'status')
  checks+=1
  bounded,last_torque,_=limit_command(cc,sample,now,c.anchor,l,last_send,last_torque)
  b.send(bounded);last_send=now;b.advance(.002)
  if stage=='search' and c.status.ready:
   a.request_swing(math.radians(depth),scope['SWING_EVENTS']);c.request_swing(math.radians(depth),swing_events(100));stage='swing'
  elif stage=='swing' and c.status.count>=45:
   a.stop_swing();c.stop_swing();stage='return'
  elif stage=='return' and c.status.ready:break
 assert stage=='return' and c.status.ready
 rows.append(dict(depth=depth,control_ticks_compared=checks,swing_hits=c.status.count,
                  all_commands_and_status_exactly_equal=True))
result=dict(bpm=100,events_exactly_equal=True,physical_devices_used=False,comparisons=rows)
Path(__file__).with_name('compare_100.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
