"""Actual Tk panel integration with offline model events; forbids hardware."""
import json,socket,sys,tempfile,time
from pathlib import Path
from unittest.mock import Mock,patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import rclpy
from PIL import ImageGrab
from camera_playback.app import App
from camera_playback.audio import AudioSender
from camera_playback.audio_bridge import forward_audio_message

OUT=ROOT/'diagnostics/dual_sound_20261005'
def audit(event,args):
    if event=='socket.__new__' and args[1]==socket.PF_CAN:
        raise AssertionError('UI verification forbids SocketCAN')
sys.addaudithook(audit)

with tempfile.TemporaryDirectory(prefix='x5-hihat-ui-') as tmp:
    p=Path(tmp);app=None;senders=[]
    rclpy.init(args=[])
    try:
        with patch('goal_motion.app.Motors',side_effect=AssertionError('No Motors allowed')), \
             patch('camera_playback.app.HiHatController.connect',side_effect=AssertionError('No serial allowed')):
            app=App(False,str(p/'camera.sock'),str(p/'ride.sock'),None,
                    hihat_audio_socket=str(p/'hihat.sock'))
        app.root.title('OFFLINE DISPLAY CHECK — no motor access')
        app.root.geometry('+30+30')
        app.root.attributes('-topmost', True)
        app.root.update()
        def paint(seconds=.25):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                app.root.update();time.sleep(.01)
        paint(.7)  # wait for the desktop compositor's window-open animation
        assert app.bus is None
        monitor=app.hihat_sound_monitor
        app._program_failure=Mock()
        ride=AudioSender(p/'ride.sock');hihat=AudioSender(p/'hihat.sock',instrument='hihat');senders=[ride,hihat]
        for s in senders:s.status('ready','Offline display test')
        app.sound_tick();monitor.tick();app.root.update()
        assert app._sound_ready_for_start()
        assert 'READY' in monitor.text.get()
        before=(app.phase,app.strike_index,app.continuous_strike_active)
        event=json.loads((OUT/'replay_verification.json').read_text())['r21.wav']['hihat']['events'][0]
        now=time.monotonic()
        forward_audio_message(dict(event,event='HIT',event_monotonic=now-.2),hihat,now,'hihat')
        monitor.tick();app.sound_tick();app.root.update()
        paint()
        assert monitor.count==1
        assert 'CLOSURE DETECTED' in monitor.text.get()
        assert monitor.label.cget('background')=='#c8f5d2'
        assert (app.phase,app.strike_index,app.continuous_strike_active)==before
        assert app.strike_hit_pending is None
        app._program_failure.assert_not_called()
        box=(app.root.winfo_rootx(),app.root.winfo_rooty(),
             app.root.winfo_rootx()+app.root.winfo_width(),app.root.winfo_rooty()+app.root.winfo_height())
        ImageGrab.grab(bbox=box).save(OUT/'ui_closure.png')
        hihat.status('error','Test: optional hi-hat detector unavailable')
        monitor.tick();app.root.update()
        paint()
        assert 'beat unaffected' in monitor.text.get()
        assert app._sound_ready_for_start()
        app._program_failure.assert_not_called()
        assert (app.phase,app.strike_index,app.continuous_strike_active)==before
        ImageGrab.grab(bbox=box).save(OUT/'ui_optional_error.png')
        (OUT/'ui_verification.json').write_text(json.dumps(dict(
            offline=True,motor_access=False,actual_tk_widgets=True,
            event_source='hihat_v1 offline WAV detection, replayed as a test notification',
            closure_flash=True,counter=monitor.count,optional_error_did_not_change_workflow=True,
            window_size=[app.root.winfo_width(),app.root.winfo_height()]),indent=2)+'\n')
    finally:
        for s in senders:s.close()
        if app is not None:
            app.hihat_sound_monitor.close();app.audio_receiver.close();app.receiver.close()
            app._cancel_planning();app.planner_executor.shutdown(wait=False,cancel_futures=True)
            app.root.destroy();app.node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
print('PASS: actual offline Tk panel flashes a real model replay event; optional failure never changes arm workflow.')
