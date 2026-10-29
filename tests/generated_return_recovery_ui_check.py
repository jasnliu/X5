"""Actual Return/Hold buttons, injected faults, real saved-scene planner; fake motors only."""
import json,socket,sys,time
from pathlib import Path
from unittest.mock import Mock,patch
import numpy as np
import rclpy
from types import SimpleNamespace,MethodType
from camera_playback.mit_strike import Status
from camera_playback.generated_transport import GeneratedMotors
from drum_planner.common import atomic_json

def no_can(event,args):
    if event=='socket.__new__' and len(args)>1 and args[1]==socket.AF_CAN:raise AssertionError('No physical CAN')
sys.addaudithook(no_can)
from camera_playback.generated_app import GeneratedApp


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir',type=Path,default=Path('diagnostics/generated_return_recovery_20261028/recovery-ui'))
    root=parser.parse_args().report_dir;root.mkdir(parents=True,exist_ok=True)
    mission=Path('planner_runs/generated-beat-20261028T165028-904eb7ef/plan-443f6066/mission.json')
    report=dict(passed=False,hardware_access=False,motor_commands=0,checks=[])
    app=None;watchdogs=[];rclpy.init(args=[])
    def pump(seconds=.05):
        end=time.monotonic()+seconds
        while time.monotonic()<end:app.root.update();time.sleep(.002)
    def until(predicate,seconds=80):
        end=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>end:raise AssertionError((app.motion.stage,app.status.get()))
            pump(.01)
    def goal_fixture():
        # Install a saved pose in ideal simulator memory, NOT a hardware command.
        app.recalculate_button.invoke();pump(.05)
        app.bus.active=True
        for side,c in app.curves.items():
            q=c.at(c.duration);app.bus.q[side][:]=q;app.bus.target[side]=q.copy()
            app.motion.targets[side]=q.copy();app.motion.progress[side]=c.duration
        app.bus.poll();app.motion.emit('ARRIVED');app.emergency_button.invoke();pump(.75)
    def install_held_watchdogs():
        # No motor threads/sockets: model the existing MIT 500 ms parent
        # heartbeat deadline. GUI stalls must fail this test, not be hidden
        # by the position writer's separate cold-start acknowledgement.
        class HeldOwner:
            def __init__(self,side,motor):
                self.side,self.motor=side,motor;self.at=time.monotonic();self.max_gap=0.;self.error=None
                self.process=SimpleNamespace(is_alive=lambda:self.error is None)
                self.emergency_hold()
            @property
            def status(self):
                now=time.monotonic();gap=now-self.at;self.max_gap=max(self.max_gap,gap)
                if gap>.5:self.error='SIMULATED powered strike-owner heartbeat lost'
                self.at=now
                return Status(phase='held',ready=self.error is None,error=self.error)
            def emergency_hold(self):self.hold_position=float(app.bus.q[self.side][self.motor-1])
        for side,motor in (('right',7),('left',6)):
            worker=HeldOwner(side,motor);watchdogs.append(worker)
            setattr(app.bus,f'{side}_joint{motor}_session',worker)
        app.bus.sync_worker_holds=MethodType(GeneratedMotors.sync_worker_holds,app.bus)
        app.bus.recover_return_workers=MethodType(GeneratedMotors.recover_return_workers,app.bus)
    try:
        with patch('centering.motors.Motors.__init__',side_effect=AssertionError('No CAN owner')),\
             patch('camera_playback.hihat.HiHatController.connect',side_effect=AssertionError('No ESP32')):
            app=GeneratedApp(False,'/tmp/return-recovery-camera.sock','/tmp/return-recovery-audio.sock',simulate_mission=mission)
            pump(.15)
            # Start actual simulated left path, lose only GUI encoder polling,
            # click Return ONCE while still lost, then restore real fake polling.
            app.continue_button.invoke()
            until(lambda:app.motion.progress['left']>.3)
            poll=app.bus.poll;fresh=app.bus.fresh
            app.bus.poll=Mock(side_effect=RuntimeError('Encoder feedback lost'))
            app.bus.fresh=Mock(return_value=False);pump(.1)
            assert app.motion.stage=='HOLD' and app.bus.active
            app.center_relax_button.invoke();pump(.4)
            assert app.return_requested and app.motion.stage=='HOLD'
            from PIL import ImageGrab
            ImageGrab.grab().save(root/'return-pending.png')
            report['checks'].append('one Return click stays pending through repeated feedback errors; no automatic relax')
            app.bus.poll=poll;app.bus.fresh=fresh
            until(lambda:app.motion.stage=='RETURN' and app.motion.stream is not None)
            # The SAME fault text must stop a new return too, not merely reuse
            # the old error label while the position writer keeps advancing.
            pump(.25)
            app.bus.poll=Mock(side_effect=RuntimeError('Encoder feedback lost'))
            app.bus.fresh=Mock(return_value=False);pump(.12)
            assert app.bus.active and app.motion.stage=='HOLD' and not app.return_requested
            assert app.motion.stream is None
            app.bus.poll=poll;app.bus.fresh=fresh;pump(.7)
            app.center_relax_button.invoke();until(lambda:app.motion.stage=='DONE')
            assert not app.bus.active
            report['checks'].append('fresh feedback resumes return without a second click; repeating the same fault stops the new return and permits retry')
            # Hold must cancel intent even if feedback never comes back.
            goal_fixture();poll=app.bus.poll;fresh=app.bus.fresh
            app.bus.poll=Mock(side_effect=RuntimeError('Encoder feedback lost'))
            app.bus.fresh=Mock(return_value=False);pump(.1);app.center_relax_button.invoke();pump(.1)
            app.emergency_button.invoke();assert not app.return_requested
            app.bus.poll=poll;app.bus.fresh=fresh;pump(.8)
            assert app.motion.stage=='HOLD' and app.bus.active and not app.return_requested
            report['checks'].append('Hold cancels pending Return; restored feedback cannot queue surprise motion')
            # Missing in-memory cache -> actual bounded collision-check child.
            install_held_watchdogs()
            app.mission.pop('return_plan');app.center_relax_button.invoke()
            until(lambda:app.job_kind=='RETURN')
            first=app.job.folder
            until(lambda:app.motion.stage=='DONE')
            result=json.loads((first/'return.json').read_text())
            assert result['source']=='new_measured_prefix_check' and all(p['all_valid'] for p in result['proofs'].values())
            report['checks'].append('missing return cache automatically rebuilt and checked on saved camera scene; both arms returned')
            # Lost progress metadata: no valid prefix -> brand-new workspace paths.
            goal_fixture();install_held_watchdogs();app.motion.progress=dict(left=0.,right=0.);app.mission.pop('return_plan')
            app.center_relax_button.invoke();until(lambda:app.job_kind=='RETURN')
            cancelled=app.job.folder;app.emergency_button.invoke()
            until(lambda:app.job is None)
            assert app.motion.stage=='HOLD' and app.bus.active
            pump(.3);assert app.motion.stage=='HOLD'
            report['checks'].append('Hold cancels an in-flight return calculation without releasing or starting delayed motion')
            app.center_relax_button.invoke();until(lambda:app.job_kind=='RETURN');fresh_folder=app.job.folder
            until(lambda:app.motion.stage=='RETURN' and app.motion.stream is not None)
            result=json.loads((fresh_folder/'return.json').read_text())
            assert result['source']=='fresh_workspace_return' and all(p['all_valid'] for p in result['proofs'].values())
            assert [s['kind'] for s in result['steps']]==['planned','planned']
            until(lambda:app.motion.stage=='DONE')
            for side,q in app.motion.rest.items():np.testing.assert_allclose(app.bus.q[side],q,atol=1e-12,rtol=0)
            assert not app.bus.active
            ImageGrab.grab().save(root/'returned.png')
            report['checks'].append('retry calculates fresh RIGHT then LEFT paths, executes shared controller, and releases only at captured rest')
            assert all(w.error is None and w.max_gap<.5 for w in watchdogs)
            report['checks'].append('both held MIT-owner heartbeat deadlines stay below unchanged 500 ms during calculation and both return startups')
            workers=[json.loads(p.read_text()) for p in app.session.glob('stream-*/result.json')]
            assert all(r['success'] or r.get('cancelled') for r in workers),workers
            assert all(r['executor']=='plan_drums.baseline.motion.Controller' for r in workers)
            report.update(passed=True,session=str(app.session),mission=str(mission),workers=workers,
                          prefix_folder=str(first),fresh_folder=str(fresh_folder),cancelled_folder=str(cancelled),
                          held_owner_max_heartbeat_gap_s=max(w.max_gap for w in watchdogs))
    finally:
        if app:
            app.request_safe_close();app.audio_receiver.close();app.receiver.close();app.hihat_sound_monitor.close()
            if app.job:app.job.close()
            app.planner_executor.shutdown(wait=False,cancel_futures=True)
            app.root.destroy();app.node.destroy_node()
        from camera_playback.generated_preview import close_previews
        close_previews()
        rclpy.shutdown();atomic_json(root/'ui_report.json',report)
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
