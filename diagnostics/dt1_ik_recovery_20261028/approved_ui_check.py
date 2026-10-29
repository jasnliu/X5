"""Actual Tk buttons + spawned 200 Hz writers; fake motors, NO devices."""
import json
from pathlib import Path
import socket
import sys
import time
from unittest.mock import Mock,patch
import numpy as np
import rclpy

def no_can(event,args):
    if event=='socket.__new__' and len(args)>1 and args[1]==socket.AF_CAN:
        raise AssertionError('Physical CAN forbidden')
sys.addaudithook(no_can)

from camera_playback.generated_app import GeneratedApp
from drum_planner.common import atomic_json


def main():
    root=Path('diagnostics/dt1_ik_recovery_20261028/approved-ui')
    mission=Path('planner_runs/dt1-ik-recovery-20261028/approved-saved/mission.json')
    report=dict(passed=False,hardware_access=False,motor_commands=0,checks=[])
    app=None;rclpy.init(args=[])
    def pump(seconds=.05):
        until=time.monotonic()+seconds
        while time.monotonic()<until:app.root.update();time.sleep(.002)
    def until(predicate,seconds=45):
        end=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>end:raise AssertionError((app.motion.stage,app.status.get()))
            pump(.01)
    try:
        with patch('centering.motors.Motors.__init__',side_effect=AssertionError('No CAN owner')),\
             patch('camera_playback.hihat.HiHatController.connect',side_effect=AssertionError('No ESP32')):
            app=GeneratedApp(False,'/tmp/generated-stream-camera.sock','/tmp/generated-stream-audio.sock',
                             simulate_mission=mission,auto_preview=False)
            pump(.25);assert not app.bus.active and app.motion.stage=='READY'
            app.continue_button.invoke()
            until(lambda:app.motion.stream is not None and app.motion.progress['left']>.1)
            session=app.motion.stream;before=session.poll()['tau']
            time.sleep(.18)  # Deliberately stop servicing Tk, not the motor writer.
            after=session.poll()['tau'];assert after-before>.14
            report['checks'].append('spawned exact-curve writer advances independently during 180 ms Tk pause')
            pump(.02);app.emergency_button.invoke()
            stopped=app.motion.progress.copy();held={s:q.copy() for s,q in app.bus.q.items()}
            pump(.8)
            assert app.bus.active and app.motion.stage=='HOLD' and app.motion.progress==stopped
            assert not session.process.is_alive() and app.bus.path_session is None
            for s in held:np.testing.assert_array_equal(app.bus.q[s],held[s])
            report['checks'].append('Hold joins the position writer, freezes both arms, retains latest traveled prefix')
            with patch.object(app,'_spawn',side_effect=AssertionError('No end replanning')):
                app.center_relax_button.invoke();until(lambda:app.motion.stage=='DONE')
            assert not app.bus.active
            report['checks'].append('partial cached return streams at 200 Hz and releases at the original rest')
            app.recalculate_button.invoke();pump(.1);app.continue_button.invoke()
            until(lambda:app.motion.stage=='ARRIVED')
            for s,c in app.curves.items():np.testing.assert_array_equal(app.bus.q[s],c.at(c.duration))
            assert not app.beat_started and app.bus.active
            report['checks'].append('full left then right reaches exact saved goals; no acoustic/hardware test invoked')
            with patch.object(app,'_spawn',side_effect=AssertionError('No end replanning')):
                app.center_relax_button.invoke();until(lambda:app.motion.stage=='DONE')
            assert not app.bus.active
            for s,q in app.motion.rest.items():np.testing.assert_allclose(app.bus.q[s],q,atol=1e-14,rtol=0)
            report['checks'].append('full prepared right then left return remains unchanged')
            app.recalculate_button.invoke();pump(.1);app.continue_button.invoke()
            until(lambda:app.motion.stream is not None and app.motion.progress['left']>.1)
            session=app.motion.stream
            app.bus.fresh=Mock(return_value=False)
            app.emergency_relax_button.invoke()
            assert not app.bus.active and app.emergency_latched and not session.process.is_alive()
            report['checks'].append('Emergency Relax stops active writer and releases despite stale feedback')
            app.recalculate_button.invoke();pump(.1);app.continue_button.invoke()
            until(lambda:app.motion.stream is not None and app.motion.progress['left']>.1)
            session=app.motion.stream;app.stop_button.invoke()
            assert app.closed and not app.bus.active and not session.process.is_alive()
            report['checks'].append('Stop + Close joins active writer and exits without a position prerequisite')
            report.update(passed=True,session=str(app.session))
    finally:
        if app:
            app.request_safe_close();app.audio_receiver.close();app.receiver.close();app.hihat_sound_monitor.close()
            if app.job:app.job.close()
            app.planner_executor.shutdown(wait=False,cancel_futures=True)
            app.root.destroy();app.node.destroy_node()
        rclpy.shutdown();atomic_json(root/'ui_report.json',report)
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
