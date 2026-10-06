"""Actual normal-hardware app wiring with in-memory devices; no physical IO."""
import importlib.util,json,socket,sys,tempfile,time
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
import rclpy
from PIL import ImageGrab
from camera_playback.app import App
from camera_playback.audio import AudioSender
from camera_playback.simulation import SimulatedMotors
from camera_playback.hihat_calibration_runtime import HiHatCalibrationRuntime
spec=importlib.util.spec_from_file_location('caltest', ROOT/'tests/test_hihat_calibration.py')
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
OUT=Path(__file__).resolve().parent

def audit(event,args):
    if event=='socket.__new__' and args[1]==socket.PF_CAN:raise AssertionError('No CAN allowed')
    if event=='open' and isinstance(args[0],str) and args[0].startswith(('/dev/ttyUSB','/dev/serial')):raise AssertionError('No serial allowed')
sys.addaudithook(audit)
c=mod.Controller();c.now=c.telemetry_at=time.monotonic()
c.connect=lambda:True
c.pending_reply=None
c.query_times=[]
def query():
    c.query_times.append(time.monotonic())
    c.pending_reply=time.monotonic()+.005
c.query_status=query
def tick():
    c.now=time.monotonic()
    if c.pending_reply is not None and c.now>=c.pending_reply:
        c.telemetry_at=c.now
        c.pending_reply=None
c.tick=tick
_set_angle=c.set_calibration_angle
def set_angle(degrees):
    _set_angle(degrees)
    c.angle_ack=(degrees,c.counts_for(degrees),time.monotonic())
c.set_calibration_angle=set_angle
c.stop_sequence=lambda:None
c.close=lambda:None
bus=SimulatedMotors([0,0,0,0,0,0,1.4]);bus.center=lambda *a,**kw: (_ for _ in ()).throw(AssertionError('No physical arm or simulated enabling in this UI test'))
app=None;senders=[];rclpy.init(args=[])
try:
 with tempfile.TemporaryDirectory(prefix='x5-hhc-ui-') as tmp, \
      patch('goal_motion.app.Motors',return_value=bus), \
      patch('camera_playback.app.HiHatController',return_value=c), \
      patch('camera_playback.app.HiHatCalibrationRuntime',side_effect=lambda a:HiHatCalibrationRuntime(a,directory=OUT/'ui_events')), \
      patch('urllib.request.urlopen', side_effect=AssertionError('No network notification allowed')), \
      patch('camera_playback.hihat_calibration.CLOSE_SECONDS',.2), \
      patch('camera_playback.hihat_calibration.OPEN_SECONDS',.2):
    p=Path(tmp)
    app=App(True,str(p/'camera'),str(p/'ride'),None,hihat_audio_socket=str(p/'hihat'))
    app.root.title('OFFLINE STARTUP CALIBRATION CHECK — no physical devices')
    h=AudioSender(p/'hihat',instrument='hihat');senders=[h]
    started=time.monotonic();hit=False;first_ready_sent=None;load_started=None;max_gap=0.;last_tick=started;loaded=False
    while time.monotonic()-started<60:
        now=time.monotonic()
        if first_ready_sent is None:first_ready_sent=now
        h.status('ready','Offline synthetic test only');h.progress(now-.05,now)
        e=app.hihat_calibration.engine
        if e.angle==95 and e.phase=='HOLDING CLOSED' and not hit:
            h.hit(now,now,.9,80);hit=True
        app.root.update();time.sleep(.01)
        if load_started is not None:
            max_gap=max(max_gap,time.monotonic()-last_tick)
        last_tick=time.monotonic()
        if e.ready and load_started is None:
            load_started=time.monotonic()
            assert app.load_recording(ROOT/'recordings/record3.json')
            assert app.playback_trajectory is None
            assert app.continue_button['state']=='disabled'
        if load_started is not None and app.recording_preflight is None:
            loaded=app.playback_trajectory is not None
            break
        if e.failure:raise RuntimeError(e.failure)
    assert e.ready and e.selected_angle==95,(e.phase,e.failure)
    assert loaded and max_gap<.4,(loaded,max_gap)
    assert time.monotonic()-c.telemetry_at<1.5
    assert len([t for t in c.query_times if t>=load_started])>5
    load_seconds=time.monotonic()-load_started
    assert not bus.active and app.phase=='READY'
    rows=[json.loads(line) for line in (OUT/'ui_events/events.jsonl').read_text().splitlines()]
    startup_latency=next(r['at'] for r in rows if r['kind']=='open')-first_ready_sent
    assert 0 <= startup_latency < 1., startup_latency
    assert not any(r['kind']=='notification_delivered' for r in rows)
    assert 'CALIBRATED 95' in app.hihat_status.get()
    assert 'calibration + timing' in app.hihat_sound_monitor.text.get()
    assert app.hihat_sound_monitor.count==1
    app.root.update()
    box=(app.root.winfo_rootx(),app.root.winfo_rooty(),app.root.winfo_rootx()+app.root.winfo_width(),app.root.winfo_rooty()+app.root.winfo_height())
    ImageGrab.grab(bbox=box).save(OUT/'ui_calibrated.png')
    (OUT/'ui_verification.json').write_text(json.dumps(dict(success=True,actual_normal_hardware_app=True,physical_io=False,synthetic_hit=True,angle=95,automatic_without_button=True,startup_latency_seconds=startup_latency,notification=False,countdown=False,arm_active=False,phase=app.phase,real_record3_validation=True,load_seconds=load_seconds,max_gui_gap_seconds=max_gap,status_queries_during_load=len([t for t in c.query_times if t>=load_started])),indent=2))
    print('Actual app calibration then real recording validation passed with ongoing queried feedback; arm remained disabled.')
finally:
 for s in senders:s.close()
 if app:
    app.hihat_calibration.close();app.hihat_sound_monitor.close();app.audio_receiver.close();app.receiver.close()
    app._cancel_planning();app.planner_executor.shutdown(wait=False,cancel_futures=True)
    app.root.destroy();app.node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
