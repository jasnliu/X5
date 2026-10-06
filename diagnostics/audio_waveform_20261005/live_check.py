"""Microphone + unmodified models only. No arm, hi-hat serial or movement."""
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
    if event=='open' and isinstance(args[0],str) and args[0].startswith(('/dev/ttyUSB','/dev/serial')):raise AssertionError('No motor serial allowed')
sys.addaudithook(audit)
root=None;app=None;children=[];streams=[];controls=[]
try:
 with tempfile.TemporaryDirectory(prefix='x5-visual-live-') as tmp:
    p=Path(tmp)
    source=configure_capture_source(DEFAULT_TONOR_SOURCE)
    root=tk.Tk();app=WaveformApp(root,p/'ride-visual',p/'hihat-visual')
    env=os.environ.copy();env.pop('PYTHONPATH',None);env.pop('LD_LIBRARY_PATH',None)
    env['PYTHONDONTWRITEBYTECODE']='1'
    for inst in ('ride','hihat'):
        controls.append(AudioReceiver(p/(inst+'-control'),inst))
        f=(OUT/(inst+'_live.log')).open('w');streams.append(f)
        children.append(subprocess.Popen(['/usr/bin/python3','-m','camera_playback.audio_bridge',
            '--root',str(ROOT.parent/'st7'),'--instrument',inst,
            '--socket',str(p/(inst+'-control')),'--visual-socket',str(p/(inst+'-visual'))],
            cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT))
    app.start();started=time.monotonic();max_blocks=0;max_age=0.;events=[]
    while time.monotonic()-started<15:
        root.update()
        for c in controls:events.extend(c.poll())
        max_blocks=max(max_blocks,len(app.timeline.blocks))
        if app.timeline.latest_sample is not None:max_age=max(max_age,time.monotonic()-app.timeline.latest_sample)
        if any(c.poll() is not None for c in children):raise RuntimeError('Detector exited')
        time.sleep(.01)
    assert app.capture.captured_frames>12*16000,app.capture.captured_frames
    assert app.capture.dropped_blocks==0,app.capture.error
    assert all(c.ready() for c in controls)
    assert all(c.ready() for c in app.receivers.values())
    assert max_blocks<=550
    routes=json.loads(subprocess.check_output(['pactl','-f','json','list','source-outputs']))
    sources=json.loads(subprocess.check_output(['pactl','-f','json','list','sources']))
    source_index=next(s['index'] for s in sources if s['name']==source)
    (OUT/'live_routes.json').write_text(json.dumps(routes,indent=2)+'\n')
    relevant=[r for r in routes if r.get('properties',{}).get('application.name')=='X5-audio-waveform']
    assert relevant and all(r['source']==source_index for r in relevant),(source_index,routes)
    detector_routes=[r for r in routes if r.get('properties',{}).get('application.name') in ('X5-ride-detector','X5-hihat-detector')]
    assert len(detector_routes)==2 and all(r['source']==source_index for r in detector_routes)
    root.update();time.sleep(.15);root.update()
    box=(root.winfo_rootx(),root.winfo_rooty(),root.winfo_rootx()+root.winfo_width(),root.winfo_rooty()+root.winfo_height())
    ImageGrab.grab(bbox=box).save(OUT/'live_tonor.png')
    before=[c.received_at for c in controls]
    result=dict(success=True,live_tonor=True,arm_or_hihat_motor_access=False,source=source,
        captured_seconds=app.capture.captured_frames/16000,capture_gaps=app.capture.dropped_blocks,
        max_history_blocks=max_blocks,max_capture_age_s=max_age,
        both_models_ready=True,live_visual_markers=len(app.timeline.markers),
        audio_saved=False,visual_status=app.window.status.get())
    # Closing the graph must not stop either detector or primary event route.
    app.request_close();root.update();app.close();root.destroy();root=None
    end=time.monotonic()+2.0
    while time.monotonic()<end:
        for c in controls:c.poll()
        time.sleep(.03)
    assert all(child.poll() is None for child in children)
    assert all(c.ready() and c.received_at>t for c,t in zip(controls,before))
    result['viewer_close_kept_both_control_feeds_live']=True
    (OUT/'live_verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
finally:
 if app:app.close()
 if root:root.destroy()
 for child in children:
    if child.poll() is None:child.send_signal(signal.SIGINT)
 for child in children:
    try:child.wait(timeout=8)
    except subprocess.TimeoutExpired:child.kill();child.wait() # audio-only process
 for c in controls:c.close()
 for f in streams:f.close()
