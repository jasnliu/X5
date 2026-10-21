"""Live three-model TONOR-only smoke; no robot/serial/camera processes.

No audio recording is written. Own child processes are always stopped.
"""
from pathlib import Path
import json,os,signal,socket,subprocess,sys,tempfile,time,tkinter as tk
from PIL import ImageGrab
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from camera_playback.audio import AudioReceiver
from camera_playback.audio_waveform import WaveformApp,configure_capture_source
from camera_playback.audio_bridge import DEFAULT_TONOR_SOURCE
OUT=Path(__file__).resolve().parent

def audit(event,args):
    if event=='socket.__new__' and args[1]==socket.PF_CAN:raise AssertionError('No CAN allowed')
    if event=='open' and isinstance(args[0],str) and args[0].startswith(('/dev/tty','/dev/serial','/dev/video')):
        raise AssertionError('No robot/camera devices allowed')
sys.addaudithook(audit)
root=None;app=None;children={};streams=[];controls=[]
old_source=subprocess.check_output(['pactl','get-default-source'],text=True).strip()
try:
 with tempfile.TemporaryDirectory(prefix='x5-snare-live-') as tmp:
    p=Path(tmp)
    source=configure_capture_source(DEFAULT_TONOR_SOURCE)
    root=tk.Tk();app=WaveformApp(root,p/'ride-visual',p/'hihat-visual',snare_socket=p/'snare-visual')
    env=os.environ.copy();env.pop('PYTHONPATH',None);env.pop('LD_LIBRARY_PATH',None)
    env['PYTHONDONTWRITEBYTECODE']='1'
    for inst in ('ride','hihat','snare'):
        cmd=['/usr/bin/python3','-m','camera_playback.audio_bridge',
             '--root',str(ROOT.parent/'st7'),'--instrument',inst,'--source',source]
        if inst=='snare':cmd+=['--socket',str(p/'snare-visual')]
        else:
            controls.append(AudioReceiver(p/(inst+'-control'),inst))
            cmd+=['--socket',str(p/(inst+'-control')),'--visual-socket',str(p/(inst+'-visual'))]
        f=(OUT/(inst+'_live.log')).open('w');streams.append(f)
        children[inst]=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT)
    app.start();started=time.monotonic();max_blocks=0;max_age=0.;ready_since=None
    while time.monotonic()-started<40:
        root.update()
        for c in controls:c.poll()
        max_blocks=max(max_blocks,len(app.timeline.blocks))
        if app.timeline.latest_sample is not None:max_age=max(max_age,time.monotonic()-app.timeline.latest_sample)
        if any(c.poll() is not None for c in children.values()):raise RuntimeError('Audio detector exited; inspect per-model logs')
        ready=all(c.ready() for c in controls) and all(c.ready() for c in app.receivers.values())
        if ready and ready_since is None:ready_since=time.monotonic()
        if not ready:ready_since=None
        if ready_since is not None and time.monotonic()-ready_since>=12:break
        time.sleep(.01)
    assert ready_since is not None and time.monotonic()-ready_since>=12,'Models did not maintain ready state'
    assert app.capture.captured_frames>12*16000
    assert app.capture.dropped_blocks==0,app.capture.error
    assert max_blocks<=550
    routes=json.loads(subprocess.check_output(['pactl','-f','json','list','source-outputs']))
    sources=json.loads(subprocess.check_output(['pactl','-f','json','list','sources']))
    source_index=next(s['index'] for s in sources if s['name']==source)
    (OUT/'live_routes.json').write_text(json.dumps(routes,indent=2)+'\n')
    names={'X5-audio-waveform','X5-ride-detector','X5-hihat-detector','X5-snare-detector'}
    relevant=[r for r in routes if r.get('properties',{}).get('application.name') in names]
    assert len(relevant)==4 and all(r['source']==source_index for r in relevant)
    root.update();box=(root.winfo_rootx(),root.winfo_rooty(),root.winfo_rootx()+root.winfo_width(),root.winfo_rooty()+root.winfo_height())
    ImageGrab.grab(bbox=box).save(OUT/'live_tonor.png')
    result=dict(success=True,live_tonor=True,robot_or_serial_access=False,source=source,
        captured_seconds=app.capture.captured_frames/16000,capture_gaps=app.capture.dropped_blocks,
        max_history_blocks=max_blocks,max_capture_age_s=max_age,
        three_models_ready=True,live_visual_markers=len(app.timeline.markers),
        deliberate_drum_hits=False,audio_saved=False,visual_status=app.window.status.get())
    # Deliberately stop only the visualization-only snare process. The existing
    # ride/hi-hat receiver states and heartbeats must continue unaffected.
    before=[c.received_at for c in controls]
    children['snare'].send_signal(signal.SIGINT)
    end=time.monotonic()+4
    while time.monotonic()<end:
        root.update()
        for c in controls:c.poll()
        time.sleep(.01)
    assert children['snare'].poll()==0
    assert all(children[i].poll() is None for i in ('ride','hihat'))
    assert all(c.ready() and c.received_at>t for c,t in zip(controls,before))
    assert app.receivers['snare'].state=='stopped'
    result['snare_exit_kept_both_existing_feeds_live']=True
    before=[c.received_at for c in controls]
    app.request_close();root.update();app.close();root.destroy();root=None
    end=time.monotonic()+2
    while time.monotonic()<end:
        for c in controls:c.poll()
        time.sleep(.02)
    assert all(children[i].poll() is None for i in ('ride','hihat'))
    assert all(c.ready() and c.received_at>t for c,t in zip(controls,before))
    result['viewer_close_kept_both_existing_feeds_live']=True
    (OUT/'live_verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
finally:
 if app:app.close()
 if root:root.destroy()
 for child in children.values():
    if child.poll() is None:child.send_signal(signal.SIGINT)
 for child in children.values():
    try:child.wait(timeout=8)
    except subprocess.TimeoutExpired:child.kill();child.wait()  # Own audio-only child.
 for c in controls:c.close()
 for f in streams:f.close()
 current=subprocess.check_output(['pactl','get-default-source'],text=True).strip()
 if current!=old_source:subprocess.run(['pactl','set-default-source',old_source],check=True)
