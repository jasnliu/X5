"""Real Tk Start/arming/polling with packet-only simulated devices; no hardware.

Unlike the ideal simulator, each encoder changes age only on a generated reply.
The program's REAL arming generator, CAN decoder, gates and spawned path worker
are retained. Only device connections and peripheral actions are substituted.
"""
import argparse
import json
import time
from pathlib import Path
from unittest.mock import patch
import numpy as np
import rclpy
from tests.generated_packet_device import PacketDevice,PacketStreamMotors,forbid_devices
forbid_devices()
from camera_playback.generated_app import GeneratedApp
from camera_playback.generated_transport import GeneratedMotors
from camera_playback.generated_motion import Motion
from camera_playback.generated_stream import StreamSession
from drum_planner.common import atomic_json
from safe_zone.encoder import request_frame


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir',required=True,type=Path)
    parser.add_argument('--expect-startup-failure',action='store_true')
    parser.add_argument('--mission',type=Path,default=Path('planner_runs/generated-beat-20261028T221006-81fb4d62/plan-11753555/mission.json'))
    args=parser.parse_args();args.report_dir.mkdir(parents=True,exist_ok=True)
    report=dict(passed=False,hardware_access=False,physical_motor_commands=0,checks=[])
    app=None;device=None;polls=[];ages=[];faults=[];rclpy.init(args=[])
    def pump(seconds=.02):
        until=time.monotonic()+seconds
        while time.monotonic()<until:app.root.update();time.sleep(.001)
    def wait(predicate,seconds=80):
        until=time.monotonic()+seconds
        while not predicate():
            if faults:raise AssertionError(faults[-1])
            if time.monotonic()>until:raise AssertionError((app.motion.stage,app.status.get()))
            pump()
    try:
        with patch('centering.motors.Motors.__init__',side_effect=AssertionError('Physical owner forbidden')),\
             patch('camera_playback.hihat.HiHatController.connect',side_effect=AssertionError('ESP32 forbidden')):
            app=GeneratedApp(False,'/tmp/packet-startup-camera.sock','/tmp/packet-startup-audio.sock',
                             simulate_mission=args.mission,auto_preview=False)
            rest={s:c.at(0) for s,c in app.curves.items()}
            device=PacketDevice(rest)
            app.bus.close()
            app.bus=GeneratedMotors.from_owner(device.owner(),rest)
            app.bus.stream_geometry=app.geometry;app.bus.stream_directory=app.session
            app.motion=Motion(app.bus,app.curves)
            app.root.title('OFFLINE packet-level startup verification — NO HARDWARE')
            app.hihat_status.set('FAKE CAN + FAKE peripheral readiness; all physical devices prohibited')
            real_poll=app.bus.poll
            def poll():
                real_poll();now=time.monotonic();polls.append(now)
                if len(app.bus.states)==16:ages.append((app.motion.stage,max(now-v[2] for v in app.bus.states.values())))
            app.bus.poll=poll
            real_fail=app.fail
            def fail(message):faults.append(str(message));real_fail(message)
            app.fail=fail
            publish=app.publish
            def loaded_publish():
                # Match the observed ~33–40 ms total GUI polling interval.
                # after(20) is a delay AFTER callback work, not a 50 Hz clock.
                result=publish();time.sleep(.018);return result
            app.publish=loaded_publish
            def fake_stream(bus,*a,**kw):
                return StreamSession(bus,*a,**kw,factory=PacketStreamMotors,shared=device.shared)
            with patch('camera_playback.generated_stream.StreamSession',side_effect=fake_stream):
                wait(lambda:app._ready());pump(.35)
                assert not app.bus.active and all(state==0 for state in device.shared.state[:])
                assert all(c[2]==2 for c in device.calls)
                report['checks'].append('startup obtains disabled encoder replies with state-only queries, no automatic arming')
                if args.expect_startup_failure:
                    # Make the first Start fresh so this replay tests the
                    # reported ARMING failure, not just rejection at Start.
                    for sock in device.sockets.values():
                        for i in range(1,9):sock.send(request_frame(i))
                    time.sleep(.002);app.bus.poll()
                print('PRE-START:',app.continue_button.cget('state'),app._ready(),app.status.get(),flush=True)
                app.continue_button.invoke()
                print('POST-START:',app.motion.stage,app.status.get(),flush=True)
                if args.expect_startup_failure:
                    deadline=time.monotonic()+8
                    while not faults and time.monotonic()<deadline:pump()
                    assert faults and 'Fresh feedback missing' in faults[0],(faults,app.motion.stage,app.status.get())
                    report['checks'].append('unmodified two-query scheduler reproduces the exact ARMING fault without hardware')
                else:
                    wait(lambda:app.motion.stage=='LEFT')
                    assert len([c for c in device.calls if c[2]==3])==16
                    assert all(s==2 for s in device.shared.state[:])
                    for s in rest:np.testing.assert_allclose(app.bus.positions_for(s),rest[s],atol=.00021,rtol=0)
                    report['checks'].append('real confirmed-CSP arming configures all 16 simulated drives at acquired rest, no centering')
                    wait(lambda:app.motion.stage=='ARRIVED')
                    for s,c in app.curves.items():np.testing.assert_allclose(app.bus.positions_for(s),c.at(c.duration),atol=.00021,rtol=0)
                    assert not app.beat_started
                    report['checks'].append('actual 200 Hz path workers with packet-level sockets reach LEFT then RIGHT goals; physical/audio calibration skipped')
                    app.emergency_button.invoke();pump(.8)
                    assert app.motion.stage=='HOLD' and app.bus.active
                    report['checks'].append('Emergency Hold retains both simulated arms with query-only freshness tracking')
                    app.center_relax_button.invoke();wait(lambda:app.motion.stage=='DONE')
                    assert not app.bus.active and all(s==0 for s in device.shared.state[:])
                    for s in rest:np.testing.assert_allclose(app.bus.positions_for(s),rest[s],atol=.00021,rtol=0)
                    report['checks'].append('prepared RIGHT then LEFT return reaches captured rest and confirms simulated disable replies')
                    assert max(age for stage,age in ages if stage not in ('READY','DONE'))<.120
                    assert not faults
                    report['checks'].append('unchanged 120 ms feedback guard passes at realistic GUI cadence throughout setup/playback/hold/return')
                from PIL import ImageGrab
                ImageGrab.grab().save(args.report_dir/'panel.png')
                report.update(passed=True,expected_failure=args.expect_startup_failure,session=str(app.session),
                    mission=str(args.mission),faults=faults,simulated_enable_count=sum(c[2]==3 for c in device.calls),
                    polling_interval_median_s=float(np.median(np.diff(polls))),
                    max_feedback_age_after_start_s=max(age for stage,age in ages if stage!='READY'),
                    ages_by_phase={stage:max(age for st,age in ages if st==stage) for stage,_ in ages})
                # Use the real Stop button, still only sending into fake sockets.
                app.stop_button.invoke();assert app.closed
                report['checks'].append('Stop/Close completes with fake devices, no real motor owner ever constructed')
    finally:
        if app:
            if not app.closed:app.request_safe_close()
            app.audio_receiver.close();app.receiver.close();app.hihat_sound_monitor.close()
            if app.job:app.job.close()
            app.planner_executor.shutdown(wait=False,cancel_futures=True)
            app.root.destroy();app.node.destroy_node()
        rclpy.shutdown();atomic_json(args.report_dir/'ui_report.json',report)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
