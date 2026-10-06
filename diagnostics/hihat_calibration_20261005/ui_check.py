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
c.tick=lambda: (setattr(c,'now',time.monotonic()),setattr(c,'telemetry_at',time.monotonic()))
_set_angle=c.set_calibration_angle
def set_angle(degrees):
    _set_angle(degrees)
    c.angle_ack=(degrees,c.counts_for(degrees),time.monotonic())
c.set_calibration_angle=set_angle
c.stop_sequence=lambda:None
c.close=lambda:None
bus=SimulatedMotors([0,0,0,0,0,0,1.4]);bus.center=lambda *a,**kw: (_ for _ in ()).throw(AssertionError('RUN was not requested'))
app=None;senders=[];rclpy.init(args=[])
try:
 with tempfile.TemporaryDirectory(prefix='x5-hhc-ui-') as tmp, \
      patch('goal_motion.app.Motors',return_value=bus), \
      patch('camera_playback.app.HiHatController',return_value=c), \
      patch('camera_playback.app.HiHatCalibrationRuntime',side_effect=lambda a:HiHatCalibrationRuntime(a,notifier=lambda:None,directory=OUT/'ui_events')), \
      patch('camera_playback.hihat_calibration_runtime.WARNING_SECONDS',.1), \
      patch('camera_playback.hihat_calibration.CLOSE_SECONDS',.2), \
      patch('camera_playback.hihat_calibration.OPEN_SECONDS',.2):
    p=Path(tmp)
    app=App(True,str(p/'camera'),str(p/'ride'),None,hihat_audio_socket=str(p/'hihat'))
    app.root.title('OFFLINE STARTUP CALIBRATION CHECK — no physical devices')
    h=AudioSender(p/'hihat',instrument='hihat');senders=[h]
    started=time.monotonic();hit=False
    while time.monotonic()-started<12:
        now=time.monotonic();h.status('ready','Offline synthetic test only');h.progress(now-.05,now)
        e=app.hihat_calibration.engine
        if e.angle==95 and e.phase=='HOLDING CLOSED' and not hit:
            h.hit(now,now,.9,80);hit=True
        app.root.update();time.sleep(.01)
        if e.ready:break
        if e.failure:raise RuntimeError(e.failure)
    assert e.ready and e.selected_angle==95,(e.phase,e.failure)
    assert not bus.active and app.phase=='READY'
    assert 'CALIBRATED 95' in app.hihat_status.get()
    assert 'calibration + timing' in app.hihat_sound_monitor.text.get()
    assert app.hihat_sound_monitor.count==1
    app.root.update()
    box=(app.root.winfo_rootx(),app.root.winfo_rooty(),app.root.winfo_rootx()+app.root.winfo_width(),app.root.winfo_rooty()+app.root.winfo_height())
    ImageGrab.grab(bbox=box).save(OUT/'ui_calibrated.png')
    (OUT/'ui_verification.json').write_text(json.dumps(dict(success=True,actual_normal_hardware_app=True,physical_io=False,synthetic_hit=True,angle=95,automatic_without_button=True,arm_active=False,phase=app.phase),indent=2))
    print('Actual app startup calibration completed automatically with mocked devices; arm remained disabled.')
finally:
 for s in senders:s.close()
 if app:
    app.hihat_calibration.close();app.hihat_sound_monitor.close();app.audio_receiver.close();app.receiver.close()
    app._cancel_planning();app.planner_executor.shutdown(wait=False,cancel_futures=True)
    app.root.destroy();app.node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
