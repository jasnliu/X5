"""Pre-beat acoustic calibration: pure clocks, simulated plant, mocked adapters.

No CAN/serial/camera/microphone is opened. Hardware success is not implied.
"""
from dataclasses import replace
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from camera_playback.hihat_sync import Closure, HiHatSynchronizer, TimingFault
from camera_playback.hihat_sync_runtime import HiHatSyncRuntime
from camera_playback.hybrid_strike import HybridStatus
from camera_playback import hybrid_workflow as hw
from camera_playback.tempo import SYNC_PERIOD, swing_events
from camera_playback.mit_strike import Sample
from test_hybrid_strike import Harness
from test_hybrid_workflow import app_fixture


class CalibrationTests(unittest.TestCase):
    def engine(self):
        self.now = 10.
        self.commands, self.rows = [], []
        def send(closed):
            self.commands.append((self.now, closed))
            return self.now
        e = HiHatSynchronizer(send, self.rows.append, calibration=True)
        e.start(self.now)
        e.set_epoch(11., self.now)
        return e

    def tick(self, e, at):
        self.now = at
        e.tick(at, {'ride': True, 'hihat': True})

    def pair(self, e, index, estimate, used=0., actual=None,
             detections=('ride', 'hihat'), duplicate=None):
        target = 12.+index*SYNC_PERIOD
        actual = used if actual is None else actual
        command = target-actual
        # Negative estimates represent a ride contact later than its deadline.
        ride_shift = .12 if estimate < 0 else 0.
        now = target+.55
        e.pending.append(Closure(index*2, target, command, used))
        for instrument, at in [('ride', target+ride_shift),
                               ('hihat', command+estimate+ride_shift)]:
            messages = [dict(kind='progress', instrument=instrument, finalized_at=now)]
            if instrument in detections:
                messages.append(dict(kind='hit', instrument=instrument, event_at=at))
                if duplicate == instrument:
                    messages.append(dict(kind='hit', instrument=instrument, event_at=at+.02))
            e.feed(instrument, messages, now)
        e.evaluate(now, {'ride': True, 'hihat': True})

    def test_straight_pairs_close_every_750ms_open_between_no_pickup(self):
        e = self.engine()
        for i in range(12):
            self.tick(e, 11.+i*SYNC_PERIOD/2+1e-9)
        self.assertEqual([c for _,c in self.commands], [True,False]*6)
        self.assertEqual([r['beat'] for r in self.rows if r['kind']=='command' and r['closed']],
                         [1,2,3,4,5,6])
        self.assertEqual(e.advance, 0.)

    def test_first_valid_pair_updates_full_absolute_offset(self):
        for estimate in (.10, -.08, .24):
            e=self.engine()
            self.pair(e, 0, estimate)
            self.assertEqual(e.matches,1)
            self.assertAlmostEqual(e.advance,estimate)
            self.assertFalse(e.locked)

    def test_single_pair_has_no_neighbor_ride_requirement(self):
        e=self.engine()
        self.pair(e, 0, 0.)
        self.pair(e, 1, 0.)
        self.assertTrue(e.locked)
        self.assertTrue(e.complete)

    def test_old_feedback_uses_historical_actual_command_not_current_offset(self):
        e=self.engine()
        e.advance=e.pair_advance=.1
        for i,actual in enumerate((0., .04, .08, 0.)):
            self.pair(e,i,.1,used=.02,actual=actual)
            self.assertAlmostEqual(e.advance,.1)
            self.assertFalse(e.locked)

    def test_bad_paired_detection_breaks_two_good_streak(self):
        e=self.engine()
        self.pair(e,0,.004)
        self.pair(e,1,.035)
        e.pair_advance=e.advance
        self.assertEqual(e.good_matches,0)
        self.pair(e,2,e.advance+.004,used=e.advance)
        self.assertFalse(e.locked)
        self.pair(e,3,e.advance+.004,used=e.advance)
        self.assertTrue(e.locked)

    def test_one_sided_or_empty_windows_do_not_count_reset_or_adjust_offset(self):
        for detections in (('hihat',), ('ride',), ()):
            for advance in (0., .1, -.08):
                with self.subTest(detections=detections,advance=advance):
                    e=self.engine();e.advance=e.pair_advance=advance
                    self.pair(e,0,advance+.004,used=advance)
                    self.assertEqual(e.good_matches,1)
                    # More than MAX_PAIR_AGE between good pairs, but each
                    # missing window is finalized promptly by both detectors.
                    for i in range(1,16):
                        self.pair(e,i,advance+.004,used=advance,detections=detections)
                        self.assertEqual(e.good_matches,1)
                        self.assertEqual(e.matches,1)
                        self.assertEqual(e.advance,advance)
                        self.assertEqual(e.pair_advance,advance)
                        self.assertFalse(e.locked)
                        self.assertTrue(self.rows[-1]['lock_streak_preserved'])
                    self.pair(e,16,advance+.004,used=advance)
                    self.assertTrue(e.locked)
                    self.assertEqual(e.good_matches,2)
                    self.assertEqual(e.matches,2)
                    self.assertEqual(e.skips,15)

    def test_missing_hits_before_first_pair_never_count_as_good(self):
        e=self.engine()
        for i in range(4):
            self.pair(e,i,0.,detections=('ride',) if i%2 else ('hihat',))
        self.assertEqual(e.good_matches,0)
        self.pair(e,4,.004)
        self.assertEqual(e.good_matches,1)
        self.assertFalse(e.locked)
        self.pair(e,5,.004)
        self.assertTrue(e.locked)

    def test_ambiguous_paired_detection_still_resets_streak(self):
        for duplicate in ('ride','hihat'):
            with self.subTest(duplicate=duplicate):
                e=self.engine();self.pair(e,0,.004)
                self.pair(e,1,.004,duplicate=duplicate)
                self.assertEqual(e.good_matches,0)
                self.assertFalse(self.rows[-1]['lock_streak_preserved'])
                self.pair(e,2,.004)
                self.assertFalse(e.locked)
                self.pair(e,3,.004)
                self.assertTrue(e.locked)

    def test_unfinalized_detector_timeout_still_resets_streak(self):
        e=self.engine();self.pair(e,0,.004)
        e.pending.append(Closure(2,12.75,12.75,0.))
        e.evaluate(19.,{'ride':False,'hihat':True})
        self.assertEqual(e.good_matches,0)
        self.assertFalse(self.rows[-1]['lock_streak_preserved'])
        self.assertEqual(self.rows[-1]['reason'],'detector timeout')

    def test_sparse_good_pairs_must_still_validate_the_same_applied_offset(self):
        e=self.engine();self.pair(e,0,.004)
        self.pair(e,1,.004,detections=('hihat',))
        e.advance=e.pair_advance=.1
        self.pair(e,2,.104,used=.1)
        self.assertEqual(e.good_matches,1)
        self.assertFalse(e.locked)
        self.pair(e,3,.104,used=.1)
        self.assertTrue(e.locked)

    def test_lock_finishes_open_then_cancels_future_closes(self):
        e=self.engine()
        self.tick(e,11.)
        e.locked=True
        self.assertFalse(e.complete)
        self.tick(e,11.+SYNC_PERIOD/2)
        self.assertTrue(e.complete)
        self.tick(e,11.+SYNC_PERIOD)
        self.assertEqual(self.commands,[(11.,True),(11.+SYNC_PERIOD/2,False)])

    def test_large_reversals_preserve_paired_open_and_minimum_open_gap(self):
        e=self.engine()
        for i in range(24):
            e.advance=.25 if (i//4)%2 else -.25
            now=11.+(i//2)*SYNC_PERIOD+(0. if i%2==0 else SYNC_PERIOD/2)-e.pair_advance
            self.tick(e,now+1e-9)
        for i,(first,second) in enumerate(zip(self.commands,self.commands[1:])):
            gap=second[0]-first[0]
            if i%2==0:self.assertAlmostEqual(gap,SYNC_PERIOD/2)
            else:self.assertGreaterEqual(gap,.175-1e-8)
        self.assertEqual(e.epoch,11.)

    def test_deadlines_timeout_and_invalid_tempo_fail_without_catchup(self):
        e=self.engine()
        with self.assertRaises(TimingFault):self.tick(e,11.1)
        self.assertFalse(self.commands)
        with self.assertRaises(TimingFault):self.tick(e,56.)
        with self.assertRaises(ValueError):HiHatSynchronizer(Mock(),period=.5,calibration=True)

    def test_fixed_swing_starts_at_exact_learned_offset_without_ramp_or_tuning(self):
        for bpm in (20,60,100,120):
            for advance in (-.25,.24):
                now=10.
                rows=[]
                e=HiHatSynchronizer(lambda closed:now,rows.append,60/bpm,fixed_advance=advance)
                e.start(now);e.set_epoch(11.,now)
                for index in range(5):
                    now=11.+index*e.period-(0. if index==0 else advance)
                    e.tick(now,{})
                self.assertTrue(e.locked)
                for r in rows:
                    if r['kind']=='command' and r['index']:
                        self.assertAlmostEqual(r['advance'],advance)
                self.assertEqual(e.advance,advance)

    def simulate(self, latency, ride_shift, delay, *, ride_every=1, hihat_every=1):
        e=self.engine()
        pending=[];next_ride=11.;seen=0;ride_index=hat_index=0
        for step in range(20001):
            now=10.+step*.002
            while next_ride+ride_shift<=now:
                if ride_index % ride_every == 0:
                    pending.append((next_ride+ride_shift+delay,'ride',next_ride+ride_shift))
                ride_index+=1
                next_ride+=SYNC_PERIOD
            while seen<len(self.commands):
                command,closed=self.commands[seen];seen+=1
                if closed:
                    if hat_index % hihat_every == 0:
                        pending.append((command+latency+delay+.2,'hihat',command+latency))
                    hat_index+=1
            ready=[p for p in pending if p[0]<=now]
            pending=[p for p in pending if p[0]>now]
            for _,k,onset in reversed(ready):
                e.feed(k,[dict(kind='hit',instrument=k,event_at=onset)],now)
            for k,lag in [('ride',delay),('hihat',delay+.2)]:
                e.feed(k,[dict(kind='progress',instrument=k,finalized_at=now-lag-.002)],now)
            self.tick(e,now)
            if e.complete:break
        self.assertTrue(e.complete)
        self.assertAlmostEqual(e.advance,latency-ride_shift,delta=.004)
        self.assertEqual(len([r for r in self.rows if r['kind']=='locked']),1)
        self.assertFalse(self.commands[-1][1])
        return e,now

    def test_positive_negative_and_long_delayed_sound_converge_before_swing(self):
        for values in ((.11,-.025,.3),(.03,.11,.7),(.1,0.,2.4),(.24,0.,.3)):
            with self.subTest(values=values):self.simulate(*values)

    def test_real_scheduler_locks_across_many_single_instrument_detections(self):
        for sparse in ('ride','hihat'):
            with self.subTest(sparse=sparse):
                e,now=self.simulate(.003,0.,.3,**{sparse+'_every':16})
                matches=[r for r in self.rows if r['kind']=='match']
                self.assertEqual(e.matches,2)
                self.assertEqual(e.good_matches,2)
                self.assertGreaterEqual(matches[1]['index']-matches[0]['index'],32)
                self.assertGreaterEqual(e.skips,15)
                self.assertEqual(e.advance,0.)
                self.assertLess(now-e.started,45.)


class CalibrationRideTests(unittest.TestCase):
    def test_real_controller_80bpm_deadlines_full_returns_then_user_swing(self):
        h=Harness(10)
        h.until(lambda s:s.completed==1)
        epoch=h.backend.now()+1.
        h.c.request_calibration(math.radians(10.5),epoch)
        seen=0;bottoms=[]
        for _ in range(3500):
            h.tick();s=h.c.status
            if s.bottoms>seen:
                seen=s.bottoms;bottoms.append(s)
            if seen>=8:break
        self.assertEqual(len(bottoms),8)
        for i,s in enumerate(bottoms):
            self.assertEqual(s.label,'sync pair')
            self.assertTrue(s.main_beat)
            self.assertTrue(s.calibrating)
            self.assertAlmostEqual(s.scheduled_at,epoch+i*SYNC_PERIOD)
            self.assertLess(abs(s.bottom_at-s.scheduled_at),.025)
            self.assertEqual(s.partial_returns,0)
        h.c.stop_swing();h.until(lambda s:s.ready and not s.swing)
        h.c.request_swing(math.radians(10.5),swing_events(120))
        h.until(lambda s:s.bottoms>=4)
        self.assertFalse(h.c.status.calibrating)
        self.assertEqual(h.c.events,swing_events(120))


class CalibrationWorkflowTests(unittest.TestCase):
    def app(self):
        a=app_fixture();a.strike_active=False;a.continuous_strike_active=True
        a.continuous_strike_degrees=10.5;a._strike_bpm=80.
        a.hihat_sync=Mock(calibration_complete=False)
        a.snare_sync=Mock(calibration_complete=False,calibrated_advance=None)
        return a

    def test_start_uses_shared_epoch_and_does_not_start_snare(self):
        a=self.app()
        with patch.object(hw.snare,'start') as snare, patch.object(hw.time,'monotonic',return_value=10.):
            hw.start_sync_calibration(a)
        a.hybrid_session.request.assert_called_once_with('calibrate',math.radians(10.5),11.)
        a.hihat_sync.start_calibration.assert_called_once_with(11.)
        snare.assert_not_called()
        self.assertEqual(a.phase,hw.SYNC_CALIBRATION)
        self.assertEqual(a.strike_bpm,80.)

    def test_checkpoint_waits_for_worker_ack_fresh_return_then_swing_at_user_bpm(self):
        a=self.app();hw.start_sync_calibration(a)
        a.hihat_sync.calibration_complete=True
        a.hihat_sync.finish_calibration.return_value=.1
        a.hybrid_session.status=HybridStatus(swing=True,calibrating=True,at=10.)
        with patch.object(hw,'start_swing') as start:
            hw.tick(a,10.)
            a.hybrid_session.request.assert_called_with('finish')
            start.assert_not_called()
            s=HybridStatus(ready=True,request_id=8,at=10.1,sample=Sample(0,0,0,2,10.1))
            hw.tick_sync_calibration(a,s,10.1)  # allow hi-hat open return too
            hw.tick_sync_calibration(a,replace(s,request_id=7,at=10.5),10.5)
            hw.tick_sync_calibration(a,replace(s,at=10.5),10.5)  # stale sample
            start.assert_not_called()
            with patch.object(hw.snare,'finished',return_value=False):
                hw.tick_sync_calibration(a,replace(s,at=10.5,sample=Sample(0,0,0,2,10.5)),10.5)
                start.assert_not_called()
            hw.tick_sync_calibration(a,replace(s,at=10.5,sample=Sample(0,0,0,2,10.5)),10.5)
            start.assert_called_once_with(a,swing_events(80))
            a.snare_sync.start_calibration.assert_not_called()
        a.bus.restore_right_joint7_csp.assert_not_called()
        a.bus.disable_all.assert_not_called()

    def test_stop_during_calibration_never_starts_swing(self):
        a=self.app();hw.start_sync_calibration(a)
        with patch.object(hw,'start_swing') as start:
            hw.request_stop(a)
            a.hihat_sync.stop.assert_called_once()
            a.hybrid_session.request.assert_called_with('finish')
            self.assertTrue(a.continuous_stop_requested)
            start.assert_not_called()
        a.bus.disable_all.assert_not_called()

    def test_runtime_carries_checkpoint_to_fresh_user_tempo_clock(self):
        a=SimpleNamespace(root=Mock(),hihat=Mock())
        r=HiHatSyncRuntime(a)
        with self.assertRaises(TimingFault):r.start(.5)
        with patch('camera_playback.hihat_sync_runtime.Path.open',Mock()), \
             patch('camera_playback.hihat_sync_runtime.Path.mkdir'), \
             patch('camera_playback.hihat_sync_runtime.time.monotonic',return_value=10.):
            r.start_calibration(11.)
            self.assertEqual(r.engine.period,SYNC_PERIOD)
            r.engine.advance=r.engine.pair_advance=.13
            r.engine.locked=True
            self.assertEqual(r.finish_calibration(),.13)
            r.start(.75)
            self.assertTrue(r.engine.locked)
            self.assertEqual(r.engine.advance,.13)
            self.assertEqual(r.engine.pair_advance,.13)
            self.assertEqual(r.engine.period,.75)
            self.assertFalse(r.engine.calibration)
            r.start_calibration(11.)
            self.assertFalse(r.engine.locked)
            self.assertIsNone(r.calibrated_advance)
            r.stop()

    def test_detector_loss_uses_controlled_stop_not_relax(self):
        a=SimpleNamespace(root=Mock(),hihat=Mock(),continuous_strike_active=True,
            continuous_stop_requested=False,audio_receiver=Mock(),hihat_sound_monitor=None,
            _request_continuous_stop=Mock())
        r=HiHatSyncRuntime(a)
        r.engine=HiHatSynchronizer(Mock(),calibration=True);r.engine.start(10.)
        r.last_observed=10.
        with patch('camera_playback.hihat_sync_runtime.time.monotonic',return_value=10.):r.tick()
        a._request_continuous_stop.assert_called_once()
        a.hihat.release_all.assert_not_called()


class EndToEndCalibrationTests(unittest.TestCase):
    def test_real_hihat_workflow_hands_off_directly_to_swing(self):
        """Real calibration/runtime/workflow + hybrid plant, virtual I/O only."""
        h=Harness(10)
        h.until(lambda s:s.completed==1)
        a=app_fixture();a._strike_bpm=80.
        a.strike_active=False;a.continuous_strike_active=True
        a.continuous_strike_degrees=10.5
        a.audio_receiver=Mock();a.audio_receiver.ready.return_value=True
        a.hihat_sound_monitor=SimpleNamespace(receiver=Mock())
        a.hihat_sound_monitor.receiver.ready.return_value=True
        a.root=Mock()
        a.hihat_sync=r=HiHatSyncRuntime(a)
        a.snare_sync=Mock(calibration_complete=False,calibrated_advance=None)
        rows=[];r.log=rows.append
        pending=[]
        now=h.backend.now()
        def hat_edge(closed):
            if closed:pending.append((now+.1+.8,'hihat',now+.1))
        a.hihat.send_state.side_effect=hat_edge
        class Session:
            request_id=0
            @property
            def status(self):return h.c.status
            def request(self, action, *args):
                self.request_id+=1
                if action=='calibrate':h.c.request_calibration(*args)
                elif action=='swing':h.c.request_swing(*args)
                elif action=='finish':h.c.stop_swing()
                else:raise AssertionError(action)
                h.c.status=replace(h.c.status,request_id=self.request_id)
                return self.request_id
        a.hybrid_session=Session()
        was_calibrating=True;checkpoint=None;seen_bottom=0;last_gui=last_hat=-1.
        with patch('camera_playback.hihat_sync_runtime.time.monotonic',side_effect=lambda:now), \
             patch('camera_playback.hihat_sync_runtime.Path.open',Mock()), \
             patch('camera_playback.hihat_sync_runtime.Path.mkdir'), \
             patch.object(hw.snare,'start') as snare:
            hw.start_sync_calibration(a)
            for _ in range(16000):
                h.tick();now=h.backend.now();s=h.c.status
                if s.bottoms>seen_bottom:
                    seen_bottom=s.bottoms
                    pending.append((s.bottom_at+.6,'ride',s.bottom_at))
                ready=[p for p in pending if p[0]<=now]
                pending=[p for p in pending if p[0]>now]
                for _,instrument,onset in reversed(ready):
                    r.feed(instrument,[dict(kind='hit',instrument=instrument,event_at=onset)])
                for instrument,lag in [('ride',.6),('hihat',.8)]:
                    r.feed(instrument,[dict(kind='progress',instrument=instrument,finalized_at=now-lag-.004)])
                if now-last_gui>=.02:
                    hw.tick(a,now);last_gui=now
                if now-last_hat>=.006:
                    r.tick();last_hat=now
                self.assertFalse(a.continuous_stop_requested)
                if not a.sync_calibrating:
                    checkpoint=r.calibrated_advance
                    break
                snare.assert_not_called()
            self.assertFalse(a.sync_calibrating,'Hi-hat never handed off to swing')
            self.assertTrue(.07<checkpoint<.13)
            self.assertTrue(r.active)
            self.assertTrue(r.engine.locked)
            self.assertTrue(any(row['kind']=='locked' for row in rows))
            a.snare_sync.start_calibration.assert_not_called()
            snare.assert_called_once_with(a)
            hw.request_stop(a)
            h.until(lambda s:s.ready and not s.swing)
            self.assertFalse(r.active)
            self.assertTrue(a.continuous_stop_requested)
