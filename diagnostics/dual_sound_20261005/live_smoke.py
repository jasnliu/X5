"""Real microphone / both detector bridges only; no ROS, camera or motors."""
import json,os,subprocess,sys,tempfile,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from camera_playback.audio import AudioReceiver

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
processes={};receivers={};logs={};messages={'ride':[],'hihat':[]};first_ready={};routes=[]
with tempfile.TemporaryDirectory(prefix='x5-dual-sound-') as tmp:
 try:
  for instrument in ('ride','hihat'):
   path=Path(tmp)/f'{instrument}.sock'
   receivers[instrument]=AudioReceiver(path,instrument=instrument)
   logs[instrument]=(OUT/f'live_{instrument}.log').open('w')
   command=['/usr/bin/python3','-m','camera_playback.audio_bridge','--root',str(ROOT.parent/'st7'),
            '--socket',str(path),'--instrument',instrument]
   processes[instrument]=subprocess.Popen(command,cwd=ROOT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'),stdout=logs[instrument],stderr=subprocess.STDOUT)
  deadline=time.monotonic()+40.;ready_since=None
  while time.monotonic()<deadline:
   now=time.monotonic()
   for name,r in receivers.items():
    batch=r.poll();messages[name].extend(dict(received_at=now,**m) for m in batch)
    assert processes[name].poll() is None,(name,'exited',processes[name].returncode)
    assert r.state!='error',(name,r.detail)
    if r.ready() and name not in first_ready:first_ready[name]=time.monotonic()
   now=time.monotonic()
   if all(r.ready(now) for r in receivers.values()):
    if ready_since is None:
     ready_since=now
     routes=json.loads(subprocess.check_output(['pactl','-f','json','list','source-outputs']))
    if now-ready_since>=10.:break
   else:ready_since=None
   time.sleep(.02)
  else:raise RuntimeError('Both models did not remain live-ready for ten seconds')
  sources=json.loads(subprocess.check_output(['pactl','-f','json','list','sources']))
  tonor={s['index'] for s in sources if 'tonor' in s['name'].lower()}
  streams=[s for s in routes if s.get('properties',{}).get('application.name') in {'X5-ride-detector','X5-hihat-detector'}]
  assert len(streams)==2,[(s.get('properties',{}),s.get('source')) for s in routes]
  assert all(s['source'] in tonor for s in streams)
  results={}
  for name,all_messages in messages.items():
   progress=[m for m in all_messages if m['kind']=='progress']
   assert len(progress)>=20,(name,len(progress))
   gaps=[b['received_at']-a['received_at'] for a,b in zip(progress,progress[1:])]
   backlog=[m['received_at']-m['captured_at'] for m in progress]
   assert max(gaps)<2.,(name,max(gaps))
   assert max(backlog)<1.,(name,max(backlog))
   results[name]=dict(progress_updates=len(progress),max_progress_gap_s=max(gaps),max_capture_to_bridge_s=max(backlog),hits=sum(m['kind']=='hit' for m in all_messages))
  result=dict(success=True,simultaneously_ready_seconds=time.monotonic()-ready_since,
              models=results,tonor_streams=streams,physical_hits_attempted=False,motor_access=False)
  (OUT/'live_verification.json').write_text(json.dumps(result,indent=2)+'\n')
  print(json.dumps({k:v for k,v in result.items() if k!='tonor_streams'},indent=2))
 finally:
  for proc in processes.values():
   if proc.poll() is None:proc.terminate()
  for proc in processes.values():
   try:proc.wait(timeout=10.)
   except subprocess.TimeoutExpired:proc.kill();proc.wait()
  for stream in logs.values():stream.close()
  for r in receivers.values():r.close()
  (OUT/'live_messages.json').write_text(json.dumps(messages,indent=2)+'\n')
