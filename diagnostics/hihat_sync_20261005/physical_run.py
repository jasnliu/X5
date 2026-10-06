"""Authorized one-run motion test; no forced termination of arm controller."""
import importlib.util,json,os,signal,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
run=OUT/('run_'+time.strftime('%Y%m%d_%H%M%S'));run.mkdir()
source='alsa_input.usb-TONOR_TONOR_TD510_Dynamic_Mic_0000KT5a300000135-00.analog-stereo'
spec=importlib.util.spec_from_file_location('ntfy',Path.home()/'.local/bin/codex-ntfy.py')
ntfy=importlib.util.module_from_spec(spec);spec.loader.exec_module(ntfy)
message=('Arm and hi-hat move in 10 seconds: center + close gripper, record3, normal ride search, '
         'up to 40 seconds of swing with automatic HI-HAT-only timing correction, then verified CENTER and relax. Keep clear.')
ok=ntfy.post_ntfy(message,'X5 synchronization test: robot will move','warning,robot')
(run/'notification.json').write_text(json.dumps(dict(delivered=bool(ok),message=message,at=time.monotonic()),indent=2))
if not ok:raise SystemExit('Notification failed; motion blocked')
ntfy.play_sound('{"type":"agent-turn-complete"}')
time.sleep(10.)
audio_log=(run/'audio.log').open('w');motion_log=(run/'launch.log').open('w')
env=dict(os.environ,PULSE_SOURCE=source,PIPEWIRE_PROPS=json.dumps({'application.name':'X5-sync-evidence','stream.capture.sink':False,'target.object':source}))
env.pop('PYTHONPATH',None);env.pop('LD_LIBRARY_PATH',None)
audio=subprocess.Popen([str(ROOT.parent/'st7/.venv/bin/python'),'-m','x5_collection.audio_capture',str(run)],cwd=ROOT,env=env,stdout=audio_log,stderr=subprocess.STDOUT)
motion=None
try:
    deadline=time.monotonic()+10
    while not (run/'audio_status.json').exists():
        if audio.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Evidence audio not ready')
        time.sleep(.1)
    routes=json.loads(subprocess.check_output(['pactl','-f','json','list','source-outputs']))
    (run/'routes_start.json').write_text(json.dumps(routes,indent=2))
    motion=subprocess.Popen(['/usr/bin/python3',str(OUT/'run_launcher.py'),'--hardware','--verify-swing',
        '--recording',str(ROOT/'recordings/record3.json'),'--camera','0'],cwd=ROOT,stdout=motion_log,stderr=subprocess.STDOUT)
    (run/'pids.json').write_text(json.dumps(dict(launcher=motion.pid,audio=audio.pid)))
    print('PHYSICAL RUN '+str(run),flush=True)
    # App's existing bounded evidence monitor handles STOP/CENTER/RELAX.
    rc=motion.wait()
    (run/'exit.json').write_text(json.dumps(dict(launcher_exit=rc,at=time.monotonic())))
finally:
    # NEVER kill/relax an airborne arm from this wrapper.
    if motion is not None and motion.poll() is None:
        print('Motion controller still running: requesting its safe close with SIGINT',flush=True)
        motion.send_signal(signal.SIGINT)
        motion.wait()
    audio.send_signal(signal.SIGINT);audio.wait(timeout=10)
    audio_log.close();motion_log.close()
print('PHYSICAL RUN FINISHED '+str(run),flush=True)
