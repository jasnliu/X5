"""Hi-hat-only physical acceptance of the production startup calibrator.

No arm/CAN access. Both unmodified ST7 models run on TONOR. Only actual
hi-hat-model messages feed the production calibrator; no synthetic hits.
The production runtime sends the ntfy warning and 10-second countdown.
"""
from pathlib import Path
import json,os,signal,socket,subprocess,sys,tempfile,time
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from camera_playback.hihat import HiHatController,DEFAULT_ESP_PORT
from camera_playback.audio import AudioReceiver
from camera_playback.hihat_calibration_runtime import HiHatCalibrationRuntime
OUT=Path(__file__).resolve().parent
assert json.loads((OUT/'firmware_verification.json').read_text())['success']
result=subprocess.run(['fuser',str(Path(DEFAULT_ESP_PORT).resolve())],capture_output=True,text=True)
if result.returncode!=1:raise RuntimeError('Serial port is not idle')
def audit(event,args):
    if event=='socket.__new__' and args[1]==socket.PF_CAN:raise AssertionError('All arm/CAN access forbidden')
sys.addaudithook(audit)
commands=[];firmware=[];audio=[];errors=[];children=[];streams=[];receivers=[];runtime=None
class Controller(HiHatController):
    def _write(self,command):
        if command not in (b'H',b'S',b'O',b'Q',b'B') and command not in [f'A{x}\n'.encode() for x in (90,95,100,105,110,115)]:
            raise RuntimeError('Command outside reviewed hi-hat-only calibration protocol')
        super()._write(command)
        commands.append(dict(at=time.monotonic(),command=command.decode()))
    def _handle_line(self,line):
        firmware.append(dict(at=time.monotonic(),line=line))
        super()._handle_line(line)
class Label:
    def set(self,text): print(text,flush=True)
    def config(self,**kwargs):pass
c=Controller()
try:
 with tempfile.TemporaryDirectory(prefix='x5-hhc-live-') as tmp:
    p=Path(tmp)
    hi=AudioReceiver(p/'hihat',instrument='hihat');ride=AudioReceiver(p/'ride');receivers=[hi,ride]
    env=os.environ.copy();env.pop('PYTHONPATH',None);env.pop('LD_LIBRARY_PATH',None)
    env.pop('STRIKE_LAB_OFFLINE_ONLY',None);env.pop('PLAYBACK_OFFLINE_ONLY',None)
    for instrument in ('ride','hihat'):
        stream=(OUT/f'physical_{instrument}_detector.log').open('w');streams.append(stream)
        children.append(subprocess.Popen(['/usr/bin/python3','-m','camera_playback.audio_bridge',
            '--socket',str(p/instrument),'--root',str(ROOT.parent/'st7'),'--instrument',instrument],
            cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
    if not c.connect():raise RuntimeError(c.detail)
    app=SimpleNamespace(hihat=c,bus=SimpleNamespace(active=False),
        hihat_sound_monitor=SimpleNamespace(receiver=hi),hihat_fault_detail=None,
        status=Label(),result_status=Label(),result_label=Label(),fail=lambda msg:errors.append(str(msg)))
    runtime=HiHatCalibrationRuntime(app,directory=OUT/'physical_events')
    start=time.monotonic();deadline=start+150
    while time.monotonic()<deadline:
        c.tick()
        messages=hi.poll();audio.extend(messages);runtime.feed(messages)
        ride.poll()  # Exercise simultaneous detector load; ride never controls this test.
        runtime.tick()
        if runtime.ready or runtime.engine.phase=='FAULT':break
        if any(child.poll() is not None for child in children):
            runtime.engine.abort('Detector process exited',time.monotonic())
        time.sleep(.01)
    else:runtime.engine.abort('Physical test deadline reached',time.monotonic())
    # Complete any controlled open return before stopping the serial owner.
    until=time.monotonic()+4
    while runtime.busy and time.monotonic()<until:
        c.tick();runtime.feed(hi.poll());runtime.tick();time.sleep(.01)
    e=runtime.engine
    if not e.ready:errors.append(e.failure or 'Calibration did not finish')
    open_verified=bool(c.telemetry and c.telemetry['released'] and c.telemetry['target']==0
        and c.telemetry['opened']==1 and time.monotonic()-c.telemetry_at<.6)
    c.release_all();stopped_at=time.monotonic()
    until=time.monotonic()+1
    while time.monotonic()<until:
        c.tick();time.sleep(.01)
        if any(r['at']>=stopped_at and r['line'].startswith('STOP -- all motors released') for r in firmware):break
    ack=any(r['at']>=stopped_at and r['line'].startswith('STOP -- all motors released') for r in firmware)
    summary=dict(success=e.ready and open_verified and ack and not errors,physical=True,
        selected_angle=e.selected_angle,hit=e.hit,phase=e.phase,failure=e.failure,
        open_verified=open_verified,open_rest_position_counts=c.telemetry['pos'],stop_ack=ack,arm_access=False,synthetic_events=False,
        detector_models=['v2','hihat_v1'],commands=commands,firmware=firmware,audio=audio,errors=errors,
        elapsed=time.monotonic()-start)
    (OUT/'physical_result.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('PHYSICAL RESULT: '+json.dumps({k:v for k,v in summary.items() if k not in ('commands','firmware','audio')}),flush=True)
finally:
 if runtime and runtime.busy:
    runtime.cancel('Physical test cleanup')
    deadline=time.monotonic()+4
    while runtime.busy and time.monotonic()<deadline:
        c.tick();runtime.tick();time.sleep(.01)
 if runtime:runtime.close()
 c.close()
 for child in children:
    if child.poll() is None:child.send_signal(signal.SIGTERM)
 for child in children:
    try:child.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=3)
 for receiver in receivers:receiver.close()
 for stream in streams:stream.close()
