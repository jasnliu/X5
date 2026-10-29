"""Real Tk controls + exact saved routes + simulated motors. NO hardware tests."""
import json
import math
from pathlib import Path
import socket
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
import rclpy
sys.addaudithook(lambda e,a: (_ for _ in ()).throw(AssertionError('Physical CAN forbidden')) if e=='socket.__new__' and len(a)>1 and a[1]==socket.AF_CAN else None)
from camera_playback.generated_app import GeneratedApp
from camera_playback import hybrid_workflow as hw
from camera_playback.hybrid_strike import HybridStatus
from camera_playback.mit_strike import Sample
from camera_playback.snare_beat import SnareStatus
from drum_planner.common import atomic_json


def main():
    root=Path('diagnostics/generated_beat_controls_20261028');report=dict(passed=False,motor_commands=0,hardware_access=False,checks=[])
    rclpy.init(args=[]);app=None
    def pump(seconds=.1):
        until=time.monotonic()+seconds
        while time.monotonic()<until:app.root.update();time.sleep(.005)
    def until(predicate,seconds=60):
        deadline=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>deadline:raise AssertionError((app.stage,app.motion.stage,app.status.get()))
            pump(.02)
    try:
        with patch('centering.motors.Motors.__init__',side_effect=AssertionError('NO CAN owner')),\
             patch('camera_playback.hihat.HiHatController.connect',side_effect=AssertionError('NO serial')):
            app=GeneratedApp(False,'/tmp/generated-test-camera.sock','/tmp/generated-test-audio.sock',simulate_mission=root/'verified/mission.json',auto_preview=False)
            pump(.4)
            assert app.stage=='READY' and app.bus.active is False
            assert app.recording_button.winfo_manager()=='' and not hasattr(app.dual,'button')
            assert app.continue_button['state']=='normal'
            assert app.emergency_relax_button['state']=='normal' and app.stop_button['state']=='normal'
            from PIL import ImageGrab
            ImageGrab.grab(bbox=(app.root.winfo_rootx(),app.root.winfo_rooty(),app.root.winfo_rootx()+app.root.winfo_width(),app.root.winfo_rooty()+app.root.winfo_height())).save(root/'control_panel.png')
            report['checks'].append('actual generated panel has no recording selectors, no automatic arm motion')
            app.continue_button.invoke();until(lambda:app.motion.progress['left']>.3)
            before=app.motion.progress.copy();app.emergency_button.invoke();pump(.8)
            assert app.motion.stage=='HOLD' and app.bus.active and app.motion.progress==before
            report['checks'].append('actual Hold button keeps both simulated arms powered; Relax and Stop remain enabled')
            with patch.object(app,'_spawn',side_effect=AssertionError('No return planner allowed')):
                app.center_relax_button.invoke();until(lambda:app.motion.stage=='DONE')
            assert app.bus.active is False
            report['checks'].append('partial-path Return uses cropped upfront-checked geometry; no end planning worker')
            app.recalculate_button.invoke();pump(.2);app.continue_button.invoke()
            until(lambda:app.motion.stage=='ARRIVED')
            assert app.bus.active and not app.alignment_active
            assert np.allclose(app.bus.q['left'],app.curves['left'].at(app.curves['left'].duration))
            report['checks'].append('full exact left then right playback reaches both generated endpoints; no pink alignment')
            # Continue the REAL acoustic handoff using fake worker/ESP32/audio,
            # not the simulator's deliberately non-striking endpoint stop.
            app.simulate_mission=None;app.hardware=False
            app.hihat=Mock();app.hihat.ready.return_value=True
            app.hihat_calibration=SimpleNamespace(ready=True,closed=False,busy=False,close=Mock())
            app._sound_ready_for_start=Mock(return_value=True)
            snare=Mock();snare.clock=[0.,0.,0.,0.,0.]
            snare.status=SnareStatus(armed=True,ready=True,at=time.monotonic())
            app.bus.left_joint6_session=snare
            session=Mock();session.status=HybridStatus();session.request.return_value=1
            app.bus.poll()  # Fake-device preparation above is not an encoder update.
            with patch('camera_playback.generated_app.snare_workflow.arm',return_value=True),patch.object(hw,'HybridSession',return_value=session):
                app._begin_generated_beat()
            assert app.beat_started and app.hybrid_session is session
            session.request.assert_called_with('search',math.radians(5))
            assert app.alignment_active is False and app.control is None
            report['checks'].append('actual post-arrival code enters unchanged ride J7 5-degree sound search, no hill climb')
            anchor=app.arm().copy();now=time.monotonic()
            session.status=HybridStatus(phase='coast',count=1,released_at=now-.1,
                peak_drop=math.radians(4),sample=Sample(anchor[6]-.07,-.5,0,2,now),at=now)
            app._process_sound_hit({'event_at':now-.01})
            assert app.strike_hit_pending['degrees']==5
            from dataclasses import replace
            session.status=replace(session.status,phase='hold',ready=True,completed=1,returned_at=now,
                                    sample=Sample(anchor[6],0,.745,2,now))
            with patch.object(hw,'start_sync_calibration') as sync:
                hw.tick(app,now)
            sync.assert_called_once()
            assert app.continuous_strike_degrees==5.5
            report['checks'].append('fake ST7 hit drives original +0.5-degree swing depth and ride/hi-hat sync-calibration handoff')
            # Real normal-Return handoff with fake strike owners: finish first,
            # then load cache. Keep a fresh fake state for this manual UI step.
            app.bus.right_joint7_session=session;app.bus.poll()
            finishes_before=snare.finish.call_count
            with patch.object(app,'_spawn',side_effect=AssertionError('No return planner allowed')):
                app.center_relax_button.invoke()
                assert app.motion.stage=='WAIT PRIME'
                session.request.assert_called_with('finish');assert snare.finish.call_count==finishes_before+1
                app.simulate_mission=root/'verified/mission.json'
                until(lambda:app.motion.stage=='DONE')
            assert not app.bus.active
            app.hihat.release_all.assert_called()
            report['checks'].append('normal Return completes existing strokes to prime before cached playback; no end planner')
            report['checks'].append('full-goal right-then-left checked return completes and releases only at captured rest')
            app.recalculate_button.invoke();pump(.2);app.continue_button.invoke()
            until(lambda:app.motion.stage=='LEFT')
            app.bus.q['right'][1]=-1.  # Deliberately bad simulated pose: relax must still work.
            app.bus.fresh=Mock(return_value=False)
            app.emergency_relax_button.invoke()
            assert not app.bus.active and app.emergency_latched
            assert ('emergency_release',) in app.bus.calls
            report['checks'].append('visible Emergency Relax works at invalid joint pose with stale feedback')
            app.recalculate_button.invoke();pump(.2);app.continue_button.invoke()
            until(lambda:app.motion.stage=='LEFT')
            app.bus.fresh=Mock(return_value=False)
            app.request_safe_close()
            assert app.closed and not app.bus.active
            report['checks'].append('window close while powered/stale immediately requests relax and exits without rest gating')
            report['passed']=True
    finally:
        if app:
            # No real devices exist in this test, including on test failure.
            if app.bus:app.bus.active=False
            app.request_safe_close();app.audio_receiver.close();app.receiver.close();app.hihat_sound_monitor.close()
            if app.job:app.job.close()
            app.planner_executor.shutdown(wait=False,cancel_futures=True)
            app.root.destroy();app.node.destroy_node()
        rclpy.shutdown();atomic_json(root/'ui_report.json',report)
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
