"""Bounded, audio-only comparison. No robot/ROS/camera/serial programs launched."""
import json,os,statistics,subprocess,sys,tempfile,time
from pathlib import Path
root=Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(root))
from camera_playback.audio import AudioReceiver
out=Path(sys.argv[1]);ticks=os.sysconf('SC_CLK_TCK');reports={}
def usage(parents):
 ids=set(parents)
 for pid in list(parents):
  try:ids.update(int(s) for s in Path(f'/proc/{pid}/task/{pid}/children').read_text().split())
  except FileNotFoundError:pass
 cpu=0.;pss=0
 for pid in ids:
  try:
   fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split();cpu+=(int(fields[11])+int(fields[12]))/ticks
   for line in Path(f'/proc/{pid}/smaps_rollup').read_text().splitlines():
    if line.startswith('Pss:'):pss+=int(line.split()[1])
  except (FileNotFoundError,ProcessLookupError):pass
 return cpu,pss,len(ids)
for mode in ['separate','shared']:
 workers=[];receivers={};opened=[]
 try:
  with tempfile.TemporaryDirectory(prefix='x5-resource-audio-') as tmp:
   receivers={n:AudioReceiver(Path(tmp)/(n+'.sock'),n) for n in ['ride','hihat','snare']}
   if mode=='separate':
    commands=[['/usr/bin/python3','-m','camera_playback.audio_bridge','--root',str(root.parent/'st7'),'--socket',str(r.path),'--instrument',name] for name,r in receivers.items()]
   else:
    command=['/usr/bin/python3','-m','camera_playback.shared_audio_bridge','--root',str(root.parent/'st7')]
    for name,r in receivers.items():command+=['--channel',name,str(r.path),'']
    commands=[command]
   for i,cmd in enumerate(commands):
    so=(out/f'{mode}-live-{i}.stdout').open('w');se=(out/f'{mode}-live-{i}.stderr').open('w');opened.extend([so,se])
    env=os.environ.copy();env.pop('PYTHONPATH',None);env.pop('LD_LIBRARY_PATH',None)
    workers.append(subprocess.Popen(cmd,cwd=root,env=env,stdout=so,stderr=se))
   deadline=time.monotonic()+90;ready=None;start_cpu=None;measure_start=None;pss=[];processes=[];lag={n:[] for n in receivers}
   while True:
    now=time.monotonic()
    for name,r in receivers.items():
     for event in r.poll():
      if measure_start and event.get('kind')=='progress':lag[name].append(now-event['captured_at'])
     assert r.state not in ['error','stopped'],(mode,name,r.detail)
    assert all(w.poll() is None for w in workers),(mode,'worker exited')
    if all(r.ready(now) for r in receivers.values()) and ready is None:ready=now
    if ready and now-ready>=6:
     cpu,kib,n=usage([w.pid for w in workers]);pss.append(kib);processes.append(n)
     if measure_start is None:measure_start=now;start_cpu=cpu
     if now-measure_start>=30:
      def quantile(values,q):return sorted(values)[int((len(values)-1)*q)]
      report=dict(cpu_seconds=cpu-start_cpu,wall_seconds=now-measure_start,
          aggregate_cpu_percent=100*(cpu-start_cpu)/(now-measure_start),pss_median_mib=statistics.median(pss)/1024,
          pss_peak_mib=max(pss)/1024,process_count=max(processes),capture_to_receiver_ms={n:dict(median=statistics.median(v)*1000,p95=quantile(v,.95)*1000,progress_updates=len(v)) for n,v in lag.items()})
      reports[mode]=report;print(json.dumps({mode:report}),flush=True);break
    if now>deadline:raise TimeoutError('bounded audio-only resource smoke')
    time.sleep(.02)
   for w in workers:w.terminate()
   for w in workers:assert w.wait(timeout=8)==0
   for i in range(len(workers)):
    assert not (out/f'{mode}-live-{i}.stderr').read_text()
    assert 'Traceback' not in (out/f'{mode}-live-{i}.stdout').read_text()
 finally:
  for w in workers:
   if w.poll() is None:
    w.terminate()
    try:w.wait(timeout=8)
    except subprocess.TimeoutExpired:w.kill();w.wait(timeout=5)
  for r in receivers.values():r.close()
  for f in opened:f.close()
(out/'audio_resources.json').write_text(json.dumps(reports,indent=2)+'\n')
print('PASS: bounded microphone-only comparison, clean shutdown, no robot processes.',flush=True)
