"""Audio-only supervision check: closed graph + unexpected shared-child exit."""
import json,os,signal,subprocess,sys,tempfile,time
from pathlib import Path
root=Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(root))
from camera_playback.audio import AudioReceiver
out=Path(sys.argv[1]);p=None;receivers={}
try:
 with tempfile.TemporaryDirectory(prefix='x5-audio-fault-') as tmp:
  receivers={n:AudioReceiver(Path(tmp)/(n+'.sock'),n) for n in ['ride','hihat','snare']}
  cmd=['/usr/bin/python3','-m','camera_playback.shared_audio_bridge','--root',str(root.parent/'st7')]
  for name,r in receivers.items():cmd+=['--channel',name,str(r.path),str(Path(tmp)/(name+'-closed-graph.sock'))]
  env=os.environ.copy();env.pop('PYTHONPATH',None);env.pop('LD_LIBRARY_PATH',None)
  with (out/'fault.stdout').open('w') as so,(out/'fault.stderr').open('w') as se:p=subprocess.Popen(cmd,cwd=root,env=env,stdout=so,stderr=se)
  deadline=time.monotonic()+25
  while True:
   for r in receivers.values():r.poll();assert r.state not in ('error','stopped'),r.detail
   if all(r.ready() for r in receivers.values()):break
   assert time.monotonic()<deadline and p.poll() is None
   time.sleep(.02)
  children=[int(c) for c in Path(f'/proc/{p.pid}/task/{p.pid}/children').read_text().split()]
  assert len(children)==1
  os.kill(children[0],signal.SIGTERM)
  deadline=time.monotonic()+8
  while True:
   for r in receivers.values():r.poll()
   if all(r.state=='error' for r in receivers.values()) and p.poll() is not None:break
   assert time.monotonic()<deadline
   time.sleep(.02)
  assert p.returncode==1
  assert all(not r.ready() for r in receivers.values())
  report=dict(no_graph_required=True,all_three_ready_before_failure=True,one_shared_child=True,
              unexpected_child_exit_notified_all_three=True,bridge_exit_code=p.returncode,physical_arm=False)
  (out/'fault_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
finally:
 if p and p.poll() is None:
  p.terminate()
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:p.kill();p.wait(timeout=5)
 for r in receivers.values():r.close()
