"""Offline dynamics/transport/lifecycle tests: never open a CAN device."""
from dataclasses import replace
import math
import queue
import threading
import sys
import socket
import struct
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from camera_playback.mit_strike import (
    Command, Joint7Channel, Joint7Session, Joint7Worker, Sample, Status, StrikeController, StrikeSettings,
)
from centering.motors import Motors, packet, parameter
from safe_zone.encoder import EFF, FRAME, request_frame


def sample(q=0.5, v=0.0, at=1.0, state=2):
    return Sample(q, v, 0.16, state, at)


def ready_controller(settings=StrikeSettings()):
    c = StrikeController(0.5, settings)
    c.bias = .16
    for i in range(30):
        now = 1.0 + i*.002
        c.update(sample(at=now), now)
    assert c.ready
    return c


def simulate(degrees, inertia=.02, gravity=.16, delay=.004, feedback_period=.002,
             repeats=1):
    """Independent inertial plant; not the application's position-only simulator.

    Includes gravity, viscous friction, command delay, quantization, sampled
    feedback and repeated strikes. These are synthetic assumptions, not measured
    OpenArmX parameters or a substitute for loaded-arm calibration.
    """
    c = StrikeController(.5)
    c.bias = gravity
    q, v = .5, 0.0
    command = c.hold_command()
    delayed = []
    results = []
    started = 0
    dt = .0005
    latest = sample(at=0)
    for i in range(20000):
        now = i*dt
        while delayed and delayed[0][0] <= now:
            _, command = delayed.pop(0)
        torque = command.kp*(command.position-q) + command.kd*(command.velocity-v) + command.torque
        acceleration = (torque-gravity-.02*v)/inertia
        v += acceleration*dt
        q += v*dt
        if i % round(feedback_period/dt) == 0:
            lsb = 25.14/65535
            latest = Sample(round(q/lsb)*lsb, round(v/(66/65535))*(66/65535), torque, 2, now)
        if i % 4 == 0:
            if c.ready and started < repeats:
                c.strike(math.radians(degrees), latest, now)
                started += 1
            output = c.update(latest, now)
            delayed.append((now+delay, output))
            if c.completed > len(results):
                results.append((math.degrees(c.peak_drop), math.degrees(c.overshoot),
                                now-c.started_at))
                if len(results) == repeats:
                    return results
    raise AssertionError('Synthetic controller failed to complete')


class ControllerTests(unittest.TestCase):
    def test_requires_stationary_encoder_position_and_trend_before_ready(self):
        c = StrikeController(.5)
        for i in range(100):
            now = 1+i*.002
            c.update(sample(q=.5+.2*(now-1), v=.2, at=now), now)
        self.assertFalse(c.ready)
        for i in range(100):
            now = 2+i*.002
            c.update(sample(q=.51, at=now), now)
        self.assertFalse(c.ready)
        c = ready_controller()
        self.assertTrue(c.ready)

    def test_reused_sample_cannot_satisfy_settle_dwell(self):
        c = StrikeController(.5)
        c.update(sample(at=1), 1)
        for t in (1.002, 1.01, 1.019):
            c.update(sample(at=1), t)
        self.assertFalse(c.ready)
        c.update(sample(at=1), 1.04)
        self.assertFalse(c.ready)
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            c.update(sample(at=1), 1.30)

    def test_old_invalid_disabled_and_out_of_order_feedback(self):
        for bad in (sample(at=0.5), sample(q=math.nan), sample(state=0)):
            with self.subTest(bad=bad), self.assertRaises(RuntimeError):
                StrikeController(.5).update(bad, 1.0)

    def test_press_rechecks_encoder_movement_instead_of_cached_ready(self):
        c = ready_controller()
        with self.assertRaisesRegex(ValueError, 'stationary'):
            c.strike(math.radians(1), sample(q=.5+math.radians(.12), v=0, at=1.06), 1.06)
        self.assertEqual(c.phase, 'hold')

    def test_noisy_raw_velocity_does_not_block_stationary_encoder_or_press(self):
        c=StrikeController(.5)
        for i in range(100):
            now=1+i*.01
            q=.5+(i%2)*math.radians(.022)
            c.update(sample(q=q,v=.13 if i%2 else -.13,at=now),now)
            if i>=6:self.assertTrue(c.ready)
        c.strike(math.radians(1),sample(v=.13,at=2.),2.)
        self.assertEqual(c.phase,'fall')

    def test_hysteresis_retains_ready_without_widening_anchor_tolerance(self):
        c=ready_controller()
        # Above entry range, below exit range: no single marginal reset.
        c.update(sample(q=.5+math.radians(.06),at=1.06),1.06)
        self.assertTrue(c.ready)
        c.update(sample(q=.5+math.radians(.16),at=1.062),1.062)
        self.assertFalse(c.ready)

    def test_one_missed_idle_sample_does_not_erase_stable_history(self):
        c=ready_controller()
        c.update(sample(v=.13,at=1.08),1.08)  # 22 ms since last sample.
        self.assertTrue(c.ready)

    def test_short_idle_feedback_age_keeps_ready_but_cannot_authorize_a_strike(self):
        c=ready_controller();previous=sample(at=1.058)
        c.update(previous,1.088)  # 30 ms scheduler slip; retained static hold.
        self.assertTrue(c.ready)
        with self.assertRaisesRegex(RuntimeError,'stale'):
            c.strike(.02,previous,1.088)
        self.assertEqual(c.phase,'hold')
        c.update(previous,1.100)  # Beyond the 40 ms settling window.
        self.assertFalse(c.ready)

    def test_idle_refresh_retains_intermediate_encoder_point(self):
        c=ready_controller()  # Last controller observation at 1.058.
        # Reply to the last hold arrived at 1.068, before the worker was paused.
        # Keep this point when fetching a fresh reply after a 30 ms delay.
        c.observe_settling(sample(at=1.068))
        c.update(sample(at=1.099),1.099)
        self.assertTrue(c.ready)

    def test_actual_oscillation_is_not_hidden_by_zero_average_velocity(self):
        c=StrikeController(.5)
        for i in range(80):
            now=1+i*.01
            c.update(sample(q=.5+math.radians(.12)*(-1)**i,v=0,at=now),now)
            self.assertFalse(c.ready)

    def test_feedback_gap_requires_a_new_full_settling_window(self):
        c=ready_controller()
        c.update(sample(at=1.11),1.11)
        self.assertFalse(c.ready)
        for i in range(1,5):
            now=1.11+i*.01;c.update(sample(at=now),now)
        self.assertTrue(c.ready)

    def test_no_old_hold_history_can_complete_a_return(self):
        c=ready_controller();c.strike(.02,sample(at=1.06),1.06)
        c.phase='return';c.phase_at=1.06;c.return_duration=.04;c.return_position=.48
        c.join_accel=10.0
        c.update(sample(at=1.11),1.11)
        self.assertFalse(c.ready)
        self.assertEqual(c.completed,0)
        for i in range(1,6):
            now=1.11+i*.01;c.update(sample(at=now),now)
        self.assertEqual(c.completed,1)

    def test_fall_is_gravity_driven_with_real_damping(self):
        c = ready_controller(); c.strike(math.radians(5), sample(at=1.06), 1.06)
        command = c.update(sample(at=1.062), 1.062)
        self.assertEqual(c.phase, 'fall')
        self.assertEqual((command.kp, command.velocity, command.torque), (0, 0, 0))
        self.assertEqual(command.kd, .04)
        self.assertGreater(command.kd, 100*5/65535)

    def test_predictive_catch_precedes_target_and_returns_only_catch_command(self):
        c = ready_controller(); c.strike(math.radians(2), sample(at=1.06), 1.06)
        q = c.target + math.radians(.4)
        command = c.update(sample(q=q, v=-.6, at=1.08), 1.08)
        self.assertGreater(q, c.target)
        self.assertEqual(c.phase, 'catch')
        self.assertGreater(command.kp, 0)
        self.assertAlmostEqual(command.velocity, -.6)

    def test_faster_fall_or_older_feedback_triggers_earlier_catch(self):
        for v, age, expected in ((-.1, 0, 'fall'), (-.6, 0, 'catch'), (-.3, .019, 'catch')):
            c = ready_controller(); c.strike(math.radians(2), sample(at=1.06), 1.06)
            c.update(sample(q=c.target+math.radians(.4), v=v, at=1.08), 1.08+age)
            self.assertEqual(c.phase, expected)

    def test_return_is_zero_velocity_at_endpoint_not_constant_upward_bias(self):
        c = ready_controller()
        c.phase = 'return'; c.phase_at = 1.0
        c.return_position = .45; c.return_duration = .2
        c.join_accel = 5.0
        mid = c.update(sample(q=.47, v=.4, at=1.1), 1.1)
        near = c.update(sample(q=.5, v=.02, at=1.19), 1.19)
        end = c.update(sample(q=.5, at=1.2), 1.2)
        self.assertLess(near.velocity, mid.velocity)
        self.assertAlmostEqual(end.position, .5)
        self.assertAlmostEqual(end.velocity, 0)
        self.assertAlmostEqual(end.torque, c.bias)

    def begin_catch(self, settings=StrikeSettings(), degrees=2, speed=.6):
        c=ready_controller(settings)
        c.strike(math.radians(degrees),sample(at=1.06),1.06)
        c.update(sample(q=c.target, v=-speed, at=1.08),1.08)
        self.assertEqual(c.phase,'catch')
        return c

    def test_catch_and_return_join_with_continuous_upward_acceleration(self):
        c=self.begin_catch()
        join=c.phase_at+c.catch_duration
        before=c.update(sample(at=join-1e-9),join-1e-9)
        at=c.update(sample(at=join),join)
        after=c.update(sample(at=join+.002),join+.002)
        self.assertEqual(c.phase,'return')
        self.assertAlmostEqual(before.position,at.position,places=9)
        self.assertAlmostEqual(before.velocity,at.velocity,places=5)
        self.assertAlmostEqual(before.torque,at.torque,places=5)
        self.assertAlmostEqual(at.velocity,0,places=8)
        self.assertGreater(at.torque-c.bias,0)
        self.assertAlmostEqual((at.torque-c.bias)/c.settings.inertia,c.join_accel)
        self.assertGreater(after.position,at.position)
        self.assertGreater(after.velocity,0)
        # Much faster initial withdrawal than the old zero-acceleration start,
        # comparing identical return depth and duration, not just larger gains.
        u=.002/c.return_duration
        old_velocity=(c.anchor-c.return_position)/c.return_duration*(30*u*u-60*u**3+30*u**4)
        self.assertGreater(after.velocity,3*old_velocity)

    def test_prediction_matches_modified_catch_endpoint(self):
        c=ready_controller();c.strike(math.radians(2),sample(at=1.06),1.06)
        speed=.6;lead=c.settings.command_latency+1/c.settings.hz
        # Solve position whose predicted low point is exactly the target.
        lo,hi=c.target,c.anchor
        for _ in range(50):
            q=(lo+hi)/2
            if c.plan_reversal(q-speed*lead,-speed)[2]<c.target:lo=q
            else:hi=q
        q=(lo+hi)/2
        c.update(sample(q=q+1e-6,v=-speed,at=1.08),1.08)
        self.assertEqual(c.phase,'fall')
        c.update(sample(q=q-1e-6,v=-speed,at=1.082),1.082)
        self.assertEqual(c.phase,'catch')
        self.assertAlmostEqual(c.return_position,c.target,places=5)
        join=c.phase_at+c.catch_duration
        end=c.update(sample(at=join),join)
        self.assertAlmostEqual(end.position,c.return_position)

    def test_trajectory_bounds_and_monotonic_return_across_tuning(self):
        for kwargs in ({}, {'brake_accel':10}, {'return_accel':5},
                       {'return_speed':.1}, {'hz':50}, {'hz':1000}):
            # Deliberately slow speed cases exercise trajectory bounds, not
            # the separately tested production three-second strike timeout.
            settings=replace(StrikeSettings(),motion_timeout=30,**kwargs)
            for degrees in (.05,1,2,10):
                for speed in (0,.001,.1,.6,2.):
                    with self.subTest(kwargs=kwargs,degrees=degrees,speed=speed):
                        c=self.begin_catch(settings,degrees,speed)
                        start=c.phase_at;join=start+c.catch_duration
                        for i in range(100):
                            t=start+c.catch_duration*i/100
                            out=c.update(sample(at=t),t)
                            accel=(out.torque-c.bias)/settings.inertia
                            self.assertLessEqual(out.velocity,1e-10)
                            self.assertGreaterEqual(accel,-1e-10)
                            self.assertLessEqual(accel,settings.brake_accel+1e-9)
                            out.frame()
                        for i in range(101):
                            t=join+c.return_duration*i/100
                            out=c.update(sample(at=t),t)
                            accel=(out.torque-c.bias)/settings.inertia
                            self.assertGreaterEqual(out.position,c.return_position-1e-9)
                            self.assertLessEqual(out.position,c.anchor+1e-9)
                            self.assertGreaterEqual(out.velocity,-1e-9)
                            self.assertLessEqual(out.velocity,settings.return_speed+1e-9)
                            self.assertLessEqual(abs(accel),settings.return_accel+1e-9)
                            out.frame()
                        self.assertAlmostEqual(out.position,c.anchor)
                        self.assertAlmostEqual(out.velocity,0)
                        self.assertAlmostEqual(out.torque,c.bias)

    def test_late_tick_carries_time_forward_into_return_without_bottom_hold(self):
        c=self.begin_catch();join=c.phase_at+c.catch_duration
        now=join+.003
        out=c.update(sample(at=now),now)
        self.assertEqual(c.phase,'return')
        self.assertAlmostEqual(c.phase_at,join)
        self.assertGreater(out.position,c.return_position)
        self.assertGreater(out.velocity,0)

    def test_crossed_anchor_while_moving_does_not_complete(self):
        c = ready_controller(); c.pending_completion = True; c.started_at = 1
        c.progress_at = 1; c.phase = 'hold'; c.reset_readiness()
        for i in range(50):
            now=1.1+i*.002
            c.update(sample(q=.5+math.radians(2), v=.3, at=now), now)
        self.assertFalse(c.ready)
        self.assertEqual(c.completed, 0)

    def test_stale_sample_cannot_release_or_finish(self):
        c = ready_controller()
        with self.assertRaises(RuntimeError):
            c.strike(.02, sample(at=1), 1.1)
        self.assertEqual(c.completed, 0)
        self.assertEqual(c.phase, 'hold')

    def test_motion_timeout_and_existing_stall_detection(self):
        c = ready_controller(); c.strike(.2, sample(at=1.06), 1.06)
        with self.assertRaisesRegex(RuntimeError, 'timeout'):
            c.update(sample(at=5), 5)
        c = ready_controller(); c.strike(.2, sample(at=1.06), 1.06)
        c.phase='hold'; c.progress_position=.3; c.progress_at=1.06
        with self.assertRaisesRegex(RuntimeError, 'STALL: motor 7'):
            c.update(sample(q=.3, at=1.7), 1.7)

    def test_dynamics_small_and_large_drops(self):
        for degrees in (1, 2, 5, 10):
            for delay in (.002, .004, .006):
                with self.subTest(degrees=degrees, delay=delay):
                    depth, overshoot, duration = simulate(degrees, delay=delay)[0]
                    self.assertLess(abs(depth-degrees), .55)
                    self.assertLess(overshoot, .25)
                    self.assertLess(duration, 1.)

    def test_repeated_strikes_share_anchor_and_remain_consistent(self):
        for degrees in (1,2,5):
            results = simulate(degrees, repeats=5)
            depths = [v[0] for v in results]
            self.assertLess(max(depths)-min(depths), .2)

    def test_moderate_inertia_mismatch_and_slower_feedback(self):
        for inertia in (.016, .024):
            for degrees in (1,2,5):
                result=simulate(degrees, inertia=inertia, feedback_period=.004)[0]
                self.assertLess(abs(result[0]-degrees), .75)
                self.assertLess(result[1], .3)

    def test_invalid_tuning_rejected(self):
        for kwargs in ({'fall_kd': 0}, {'inertia': math.nan}, {'hz': 2000}, {'kd': 6}):
            with self.assertRaises(ValueError):
                StrikeSettings(**kwargs)


class TransportTests(unittest.TestCase):
    def channel(self):
        channel = Joint7Channel.__new__(Joint7Channel)
        channel.sample=None; channel.mode=None; channel.mode_at=0
        channel.motor=7; channel.sign=1.0
        return channel

    def feedback(self, q=.5, v=-.4, torque=.16, state=2):
        def enc(value, limit):
            return int((value+limit)*65535/(2*limit))
        data=struct.pack('>HHHH', enc(-q,12.57), enc(-v,33), enc(-torque,14), 250)
        return FRAME.pack(EFF | 2<<24 | state<<22 | 7<<8 | 0xfd,8,data)

    def test_decodes_position_velocity_torque_and_ignores_old_packets(self):
        ch=self.channel(); ch.decode(self.feedback(), 1.)
        self.assertAlmostEqual(ch.sample.position,.5,delta=.0004)
        self.assertAlmostEqual(ch.sample.velocity,-.4,delta=.0011)
        self.assertAlmostEqual(ch.sample.torque,.16,delta=.0005)
        previous=ch.sample
        ch.decode(self.feedback(q=1), .99)
        self.assertIs(ch.sample, previous)

    def test_mode_readback_timestamp(self):
        ch=self.channel()
        ch.decode(FRAME.pack(EFF|17<<24|7<<8|0xfd,8,bytes.fromhex('0570000000000000')), 1.2)
        self.assertEqual((ch.mode,ch.mode_at), (0,1.2))

    def test_fault_frame_is_not_a_position(self):
        ch=self.channel()
        with self.assertRaisesRegex(RuntimeError,'Motor 7'):
            ch.decode(FRAME.pack(EFF|21<<24|7<<8,8,bytes(8)),1)
        with self.assertRaisesRegex(RuntimeError,'transport'):
            ch.decode(FRAME.pack(0x20000000,8,bytes(8)),1)

    def test_socket_filters_and_timestamping_are_configured_before_bind(self):
        with patch('camera_playback.mit_strike.socket.socket') as factory:
            ch=Joint7Channel('fake-can')
            calls=factory.return_value.method_calls
            self.assertEqual(calls[0][0], 'setsockopt')
            self.assertEqual(calls[2][0], 'setsockopt')
            self.assertEqual(calls[3][0], 'bind')
            ch.close()

    def test_kernel_timestamp_preserves_queued_packet_age(self):
        ch=self.channel(); ch.socket=Mock()
        ancillary=[(socket.SOL_SOCKET,ch.timestamp_option,struct.pack('@ll',100,0))]
        ch.socket.recvmsg.side_effect=[(self.feedback(),ancillary,0,None),BlockingIOError()]
        with patch('camera_playback.mit_strike.time.time',return_value=100.05), patch(
                'camera_playback.mit_strike.time.monotonic',return_value=10.05):
            ch.receive()
        self.assertAlmostEqual(ch.sample.at, 10.0)
        with self.assertRaisesRegex(RuntimeError,'stale'):
            StrikeController(.5).check_sample(ch.sample,10.05)

    def test_missing_timestamp_is_not_relabelled_as_fresh(self):
        ch=self.channel(); ch.socket=Mock()
        ch.socket.recvmsg.return_value=(self.feedback(),[],0,None)
        with self.assertRaisesRegex(RuntimeError,'timestamp'):
            ch.receive()

    def test_command_signs_and_nonfinite_validation(self):
        command=Command(.5,-.4,40,1.8,.16)
        cid,_,data=FRAME.unpack(command.frame())
        self.assertEqual(cid&255,7)
        motor_q=int.from_bytes(data[:2],'big')/65535*25.14-12.57
        motor_v=int.from_bytes(data[2:4],'big')/65535*66-33
        self.assertAlmostEqual(motor_q,-.5,delta=.0004)
        self.assertAlmostEqual(motor_v,.4,delta=.0011)
        with self.assertRaises(ValueError):
            replace(command,torque=math.nan).frame()


class FakeChannel:
    """Ideal drive for worker ownership/timing tests, not dynamics validation."""
    def __init__(self, interface):
        self.sample = None
        self.mode = None; self.mode_at = 0.
        self.state = 2
        self.sent=[]; self.closed=False
        self.last_command=Command(.5,0,40,1.8,.16)

    def send(self, frame):
        assert not self.closed
        self.sent.append(frame)
        cid,_,data=FRAME.unpack(frame); kind=(cid>>24)&31
        if kind==4:self.state=0
        if kind==3:self.state=2
        if kind==18 and data[:2]==b'\x05\x70':self.mode=data[4]
        if kind==17:self.mode_at=time.monotonic()
        if kind==1:
            q=-(int.from_bytes(data[:2],'big')/65535*25.14-12.57)
            v=-(int.from_bytes(data[2:4],'big')/65535*66-33)
            kp=int.from_bytes(data[4:6],'big')/65535*500
            if kp>0:self.last_command=Command(q,v,kp,1.8,.16)

    def receive(self):
        q,v=self.last_command.position,self.last_command.velocity
        # Fall advances independently of Tk; enough for a fake transport test.
        if self.sent:
            cid,_,data=FRAME.unpack(self.sent[-1])
            if (cid>>24)&31==1 and data[4:6]==b'\0\0':
                q-=.002;v=-.5
                self.last_command=Command(q,v,0,.08,0)
        self.sample=Sample(q,v,.16,self.state,time.monotonic())

    def wait(self, timeout):
        time.sleep(min(.002,max(0,timeout)))

    def close(self):self.closed=True


class ThreadSession:
    """Unit harness for the same engine; production always uses a process."""
    def __init__(self, bus, anchor, lower, upper, settings=StrikeSettings(), channel_factory=FakeChannel):
        self.bus=bus;bus.right_joint7_session=self
        self.status=Status()
        self.requests=queue.Queue(maxsize=1);self.stop_event=threading.Event()
        self.worker=Joint7Worker('fake-can',anchor,lower,upper,settings,self.requests,
                                 self.stop_event,lambda status:setattr(self,'status',status),channel_factory)
        self.controller=self.worker.controller
        self.thread=threading.Thread(target=self.worker.run)
        self.thread.start()
    def strike(self, amount):self.requests.put_nowait(amount)
    def stop(self):
        self.stop_event.set();self.thread.join(2)
        assert not self.thread.is_alive()
        self.bus.right_joint7_session=None


class SessionTests(unittest.TestCase):
    def bus(self):
        sock=Mock();sock.getsockname.return_value=('fake-can',)
        return SimpleNamespace(control_side='right',active=True,right_joint7_session=None,
                               sockets={'right':sock})

    def wait_for(self, predicate, timeout=2):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if predicate():return
            time.sleep(.005)
        self.fail('Worker condition did not arrive')

    def test_worker_runs_without_gui_and_switches_mode_only_once(self):
        bus=self.bus(); channel=FakeChannel('fake-can')
        settings=StrikeSettings()  # Actual production timing limits.
        session=ThreadSession(bus,.5,-2,2,settings,lambda _:channel)
        try:
            self.wait_for(lambda:session.status.ready)
            first_count=len(channel.sent)
            time.sleep(.08)  # Deliberately no GUI/control callbacks.
            self.assertGreater(len(channel.sent),first_count+4)
            session.strike(math.radians(1))
            self.wait_for(lambda:session.status.completed==1)
            self.assertTrue(session.status.ready)
            mode_changes=[f for f in channel.sent if FRAME.unpack(f)[2][:2]==b'\x05\x70'
                          and (FRAME.unpack(f)[0]>>24)&31==18]
            self.assertEqual(mode_changes,[parameter(7,0x7005,0,True)])
            self.assertEqual(channel.sent.count(packet(4,7)),1)
            self.assertEqual(channel.sent.count(packet(3,7)),1)
        finally:
            session.stop()
        self.assertIsNone(bus.right_joint7_session)
        count=len(channel.sent);time.sleep(.01)
        self.assertEqual(len(channel.sent),count)
        self.assertTrue(channel.closed)

    def test_arming_retries_ignored_mode_writes_before_enabling(self):
        bus=self.bus(); channel=FakeChannel('fake-can');channel.mode=5
        original=channel.send
        modes=[]
        def send(frame):
            if frame == parameter(7,0x7005,0,True):
                modes.append(frame)
                if len(modes)<=2:
                    channel.sent.append(frame)  # Drive still completing disable.
                    return
            if frame == packet(3,7):
                self.assertEqual(channel.mode,0)
            original(frame)
        channel.send=send
        session=ThreadSession(bus,.5,-2,2,channel_factory=lambda _:channel)
        try:
            self.wait_for(lambda:session.status.ready)
            self.assertEqual(len(modes),3)
            self.assertIsNone(session.status.error)
        finally:
            session.stop()

    def test_arming_retries_an_ignored_enable_after_confirming_mode(self):
        bus=self.bus(); channel=FakeChannel('fake-can')
        original=channel.send
        enables=[]
        def send(frame):
            if frame == packet(3,7):
                self.assertEqual(channel.mode,0)
                enables.append(frame)
                if len(enables)==1:
                    channel.sent.append(frame)  # Firmware ignores first enable.
                    return
            original(frame)
        channel.send=send
        session=ThreadSession(bus,.5,-2,2,channel_factory=lambda _:channel)
        try:
            self.wait_for(lambda:session.status.ready)
            self.assertEqual(len(enables),2)
            self.assertIsNone(session.status.error)
        finally:
            session.stop()

    def test_gui_keeps_state_queries_during_worker_process_startup(self):
        bus=Motors.__new__(Motors)
        bus.active=False;bus.control_side='right';bus.control_gripper=True
        bus.right_joint7_session=SimpleNamespace(owns_feedback=False);bus.last_query=0
        bus.states={};bus.modes={}
        bus.sockets={side:Mock() for side in ('left','right')}
        for sock in bus.sockets.values():
            sock.recv.side_effect=BlockingIOError();sock.send.return_value=16
        bus.poll()
        frames=[v.args[0] for v in bus.sockets['right'].send.call_args_list]
        self.assertIn(request_frame(7),frames)

    def test_idle_worker_recovers_one_60ms_gap_without_gui_help(self):
        bus=self.bus(); channel=FakeChannel('fake-can')
        original=channel.receive
        freeze=[False]
        def delayed_receive():
            if freeze[0]:
                freeze[0]=False
                time.sleep(.06)
            original()
        channel.receive=delayed_receive
        session=ThreadSession(bus,.5,-2,2,channel_factory=lambda _:channel)
        try:
            self.wait_for(lambda:session.status.ready)
            freeze[0]=True
            self.wait_for(lambda:session.status.hold_recoveries > 0)
            self.assertIsNone(session.status.error)
            self.assertGreater(session.status.max_gap,.05)
            self.wait_for(lambda:session.status.ready)
        finally:
            session.stop()

    def test_active_motion_still_faults_at_20ms_not_idle_limit(self):
        worker=Joint7Worker('fake-can',.5,-2,2,StrikeSettings(),queue.Queue(),
                           threading.Event(),Mock(),FakeChannel)
        worker.controller.phase='fall'
        worker.last_command_at=1.0
        with self.assertRaisesRegex(RuntimeError,'gap=30.0 ms.*limit=20 ms'):
            worker._check_timing(sample(at=1.03),1.03)

    def test_press_after_short_idle_delay_is_accepted_without_active_deadline_fault(self):
        bus=self.bus();channel=FakeChannel('fake-can');original=channel.receive
        freeze=threading.Event();paused=threading.Event()
        def delayed_receive():
            if freeze.is_set():
                freeze.clear();paused.set();time.sleep(.022)
            original()
        channel.receive=delayed_receive
        session=ThreadSession(bus,.5,-2,2,channel_factory=lambda _:channel)
        try:
            self.wait_for(lambda:session.status.ready)
            freeze.set();self.assertTrue(paused.wait(1))
            session.strike(math.radians(1))
            self.wait_for(lambda:session.status.completed==1)
            self.assertIsNone(session.status.error)
            self.assertIsNone(session.status.rejected)
            self.assertGreater(session.status.hold_recoveries,0)
        finally:
            session.stop()

    def test_aged_idle_reply_is_refreshed_without_ready_flicker(self):
        bus=self.bus();channel=FakeChannel('fake-can');original=channel.receive
        freeze=threading.Event();published=[]
        def delayed_receive():
            if freeze.is_set():
                freeze.clear();time.sleep(.025)
                return  # Actual stale reply, not FakeChannel's fresh timestamp.
            original()
        channel.receive=delayed_receive
        session=ThreadSession(bus,.5,-2,2,channel_factory=lambda _:channel)
        try:
            self.wait_for(lambda:session.status.ready)
            def publish(status):
                published.append(status);session.status=status
            session.worker.publish=publish
            freeze.set()
            self.wait_for(lambda:session.status.hold_recoveries>0)
            time.sleep(.06)
            self.assertTrue(published)
            self.assertTrue(all(status.ready for status in published))
            self.assertIsNone(session.status.error)
        finally:
            session.stop()

    def test_late_idle_send_with_fresh_encoder_preserves_readiness(self):
        worker=Joint7Worker('fake-can',.5,-2,2,StrikeSettings(),queue.Queue(),
                           threading.Event(),Mock(),FakeChannel)
        worker.controller=ready_controller();worker.last_command_at=1.058
        self.assertFalse(worker._check_timing(sample(at=1.08),1.08))
        self.assertTrue(worker.controller.ready)
        # The hold refresh, not a fabricated timestamp, resynchronizes sending.
        worker._send(FakeChannel('fake-can'),worker.controller.hold_command().frame())
        self.assertEqual(worker.commands_sent,1)

    def test_idle_transport_loss_is_still_bounded(self):
        worker=Joint7Worker('fake-can',.5,-2,2,StrikeSettings(),queue.Queue(),
                           threading.Event(),Mock(),FakeChannel)
        worker.last_command_at=1.0
        with self.assertRaisesRegex(RuntimeError,'limit=250 ms'):
            worker._check_timing(sample(at=1.3),1.3)

    def test_spawned_worker_survives_parent_interpreter_contention(self):
        bus=self.bus()
        session=Joint7Session(bus,.5,-2,2,channel_factory=FakeChannel)
        try:
            self.wait_for(lambda:session.status.ready,5)
            previous=session.status.commands_sent
            old_interval=sys.getswitchinterval()
            try:
                sys.setswitchinterval(.15)
                deadline=time.monotonic()+.4
                value=0
                while time.monotonic()<deadline:
                    value+=sum(range(1000))  # Parent/GIL deliberately busy.
            finally:
                sys.setswitchinterval(old_interval)
            status=session.status
            self.assertIsNone(status.error)
            self.assertGreater(status.commands_sent-previous,20)
            self.assertGreater(session.process.pid,0)
            session.strike(math.radians(1))
            self.wait_for(lambda:session.status.completed==1)
            self.assertIsNone(session.status.error)
        finally:
            session.stop()
        self.assertIsNone(bus.right_joint7_session)
        self.assertFalse(session.process.is_alive())

    def test_gui_poll_skips_j7_queries_owned_by_worker(self):
        bus=Motors.__new__(Motors)
        bus.active=False;bus.control_side='right';bus.control_gripper=True
        bus.right_joint7_session=object();bus.last_query=0
        bus.states={};bus.modes={}
        bus.sockets={side:Mock() for side in ('left','right')}
        for sock in bus.sockets.values():
            sock.recv.side_effect=BlockingIOError()
            sock.send.return_value=16
        bus.poll()
        frames=[v.args[0] for v in bus.sockets['right'].send.call_args_list]
        self.assertNotIn(request_frame(7),frames)
        self.assertIn(request_frame(6),frames)
        self.assertIn(request_frame(7),[v.args[0] for v in bus.sockets['left'].send.call_args_list])

    def test_second_worker_cannot_take_ownership(self):
        bus=self.bus(); bus.right_joint7_session=object()
        with self.assertRaisesRegex(RuntimeError,'already'):
            Joint7Session(bus,.5,-2,2,channel_factory=Mock())

    def test_open_failure_is_reported_and_can_release_ownership(self):
        bus=self.bus()
        session=ThreadSession(bus,.5,-2,2,channel_factory=Mock(side_effect=OSError('no CAN')))
        self.wait_for(lambda:session.status.error is not None)
        self.assertIn('no CAN',session.status.error)
        session.stop()
        self.assertIsNone(bus.right_joint7_session)

    def test_stop_during_arming_never_reenables_after_join(self):
        bus=self.bus(); channel=FakeChannel('fake-can')
        session=ThreadSession(bus,.5,-2,2,channel_factory=lambda _:channel)
        session.stop()
        count=len(channel.sent);time.sleep(.02)
        self.assertEqual(len(channel.sent),count)
        self.assertTrue(channel.closed)

    def test_gui_motor_commands_are_blocked_while_worker_owns_j7(self):
        bus=Motors.__new__(Motors)
        bus.control_side='right';bus.right_joint7_session=object()
        with self.assertRaisesRegex(RuntimeError,'worker'):
            bus._send('right',packet(3,7))

    def test_motor_relax_joins_worker_before_disabling(self):
        bus=Motors.__new__(Motors);bus.active=True
        events=[]
        session=Mock()
        session.stop.side_effect=lambda:events.append('join')
        bus.right_joint7_session=session
        bus.send_control=lambda frame:events.append('disable')
        bus.relax()
        self.assertEqual(events[0],'join')
        self.assertFalse(bus.active)


if __name__=='__main__':unittest.main()
