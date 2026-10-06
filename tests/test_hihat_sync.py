"""No device access: acoustic servo, delayed delivery, signed edges and isolation."""
import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from camera_playback.hihat_sync import HiHatSynchronizer, TimingFault, MAX_STEP
from camera_playback.hihat_sync_runtime import HiHatSyncRuntime
from camera_playback.hybrid_strike import HybridStatus
from camera_playback import hybrid_workflow as hw
from camera_playback.app import App, SWING_EVENTS
from test_hybrid_workflow import app_fixture


class SyncTests(unittest.TestCase):
    def engine(self):
        self.now = 10.
        self.rows = []
        self.sent = []
        def send(closed):
            self.sent.append((self.now, closed))
            return self.now
        e = HiHatSynchronizer(send, self.rows.append)
        e.start(10.)
        e.set_epoch(10.2, 10.)
        return e

    def tick(self, e, at):
        self.now = at
        e.tick(at, {'ride': True, 'hihat': True})

    def hit(self, e, instrument, at, delivered):
        e.feed(instrument, [dict(kind='hit', instrument=instrument,
               event_at=at, detected_at=delivered, score=.99)], delivered)

    def watermarks(self, e, at, now):
        for k in ('ride', 'hihat'):
            e.feed(k, [dict(instrument=k, kind='progress', finalized_at=at)], now)

    def test_zero_open13_close24_and_no_pickup(self):
        e=self.engine()
        self.tick(e, 10.1)
        self.assertFalse(self.sent)
        for at in (10.2,10.8,11.4,12.):self.tick(e, at+1e-9)
        self.assertEqual([closed for at,closed in self.sent], [False,True,False,True])
        self.assertEqual(e.advance, 0.)

    def test_signed_advance_moves_close_AND_following_open_not_ride_grid(self):
        for sign in (-1,1):
            e=self.engine()
            e.advance=sign*.1
            self.tick(e,10.2)
            self.assertAlmostEqual(e.pair_advance, sign*MAX_STEP)
            due=10.8-e.pair_advance
            self.tick(e,due-.001);self.assertEqual(len(self.sent),1)
            self.tick(e,due+1e-9)
            e.advance=-sign*.2  # cannot change paired open
            self.tick(e,due+.6+1e-9)
            self.assertAlmostEqual(self.sent[2][0]-self.sent[1][0],.6)
            self.assertEqual(e.epoch,10.2)

    def test_late_command_never_catches_up_and_stop_cancels(self):
        e=self.engine()
        with self.assertRaises(TimingFault):self.tick(e,10.5)
        self.assertFalse(self.sent)
        e.stop(10.5);self.tick(e,12.)
        self.assertFalse(self.sent)

    def test_stale_epoch_rejected(self):
        e=HiHatSynchronizer(Mock());e.start(10.)
        with self.assertRaises(TimingFault):e.set_epoch(9.,10.)

    def one_pair(self):
        e=self.engine()
        self.tick(e,10.2);self.tick(e,10.8+1e-9)
        self.hit(e,'ride',10.2,11.4)
        self.hit(e,'ride',10.78,11.4)
        self.hit(e,'hihat',10.91,11.4)
        return e

    def test_delivery_order_watermarks_and_duplicates(self):
        e=self.one_pair()
        self.hit(e,'ride',10.78,11.5)
        self.assertEqual(len(e.events['ride']),2)
        e.evaluate(11.5,{'ride':True,'hihat':True})
        self.assertEqual(e.matches,0)
        e.feed('ride',[dict(kind='progress', finalized_at=11.4)],11.5)
        e.evaluate(11.5,{'ride':True,'hihat':True})
        self.assertEqual(e.matches,0)
        self.watermarks(e,11.4,11.5)
        e.evaluate(11.5,{'ride':True,'hihat':True})
        self.assertEqual(e.matches,1)
        row=[r for r in self.rows if r['kind']=='match'][0]
        self.assertAlmostEqual(row['error'],.13)
        self.assertEqual(e.advance,0.)  # require multiple observations
        e.evaluate(12.,{'ride':True,'hihat':True});self.assertEqual(e.matches,1)

    def test_missing_ambiguous_or_cross_responsive_reference_freezes(self):
        for extra in ('ride','hihat'):
            e=self.one_pair()
            self.hit(e,extra,10.92,11.4)
            self.watermarks(e,11.4,11.5)
            e.evaluate(11.5,{'ride':True,'hihat':True})
            self.assertEqual(e.matches,0);self.assertEqual(e.advance,0.)
            self.assertEqual(e.skips,1)

    def test_unavailable_or_stale_detector_freezes_and_expires(self):
        e=self.one_pair();self.watermarks(e,11.4,11.5)
        e.advance=.07
        e.evaluate(11.5,{'ride':True,'hihat':False})
        self.assertEqual(e.matches,0);self.assertEqual(e.advance,.07)
        e.evaluate(18.,{'ride':True,'hihat':False})
        self.assertEqual(len(e.pending),0);self.assertEqual(e.advance,.07)

    def test_bad_and_wrong_instrument_timestamps_are_ignored(self):
        e=self.engine()
        for at in (float('nan'),float('inf'),9.,100.):self.hit(e,'ride',at,11.)
        e.feed('ride',[dict(instrument='hihat',kind='hit',event_at=10.4)],11.)
        self.assertEqual(len(e.events['ride']),0)

    def simulation(self, latency, ride_shift, delivery_delay):
        e=self.engine()
        pending=[];ride_index=0;closed_index=0
        ride_times=[10.2+bar*2.4+offset+ride_shift for bar in range(30)
                    for offset in (0,.6,1.,1.2,1.8,2.2)]
        for step in range(28001):
            now=10.+step*.002
            while ride_index < len(ride_times) and ride_times[ride_index] <= now:
                onset=ride_times[ride_index];ride_index+=1
                pending.append((onset+delivery_delay,'ride',onset))
            while closed_index < len(self.sent):
                command,closed=self.sent[closed_index];closed_index+=1
                if closed:pending.append((command+latency+delivery_delay+.31,'hihat',command+latency))
            ready=[p for p in pending if p[0]<=now]
            pending=[p for p in pending if p[0]>now]
            for _,k,onset in reversed(ready):self.hit(e,k,onset,now)
            # Each listener's watermark follows its own event delivery lag.
            for k,lag in [('ride',delivery_delay),('hihat',delivery_delay+.31)]:
                e.feed(k,[dict(instrument=k,kind='progress',finalized_at=now-lag-.002)],now)
            self.tick(e,now)
        matches=[r for r in self.rows if r['kind']=='match']
        self.assertGreater(len(matches),30)
        self.assertLess(abs(sum(r['error'] for r in matches[-8:])/8),.020)
        self.assertLess(abs(e.advance-(latency-ride_shift)),.020)
        commands=[r for r in self.rows if r['kind']=='command']
        for c in commands:self.assertAlmostEqual(c['target'],10.2+c['index']*.6)
        for c,o in zip(commands[1::2],commands[2::2]):
            self.assertAlmostEqual(c['advance'],o['advance'])
        return e,matches

    def test_delayed_positive_feedback_converges_without_moving_ride(self):
        self.simulation(.110,-.025,1.1)

    def test_negative_advance_converges_and_is_not_clamped_to_zero(self):
        self.simulation(.030,.110,.7)

    def test_long_notification_delay_does_not_repeatedly_integrate_old_error(self):
        self.simulation(.070,-.030,2.4)

    def test_real_workflow_hands_schedule_to_sync_and_suppresses_legacy_double_send(self):
        a=app_fixture();a.hihat_sync=Mock(active=True)
        a.continuous_strike_active=True;a.strike_active=False
        a.continuous_strike_degrees=10
        hw.start_swing(a,SWING_EVENTS)
        a.hihat_sync.start.assert_called_once_with(.6)
        a.hybrid_session.status=HybridStatus(swing=True, main_count=1, main_at=10.,at=10.)
        hw.tick(a,10.)
        a.hihat_sync.observe.assert_called_once()
        a.hihat.send_beat.assert_not_called()
        a._stop_hihat_sequence()
        a.hihat_sync.stop.assert_called_once()
        a.hihat.stop_sequence.assert_called_once()

    def test_runtime_seeds_only_from_unmodified_ride_epoch_and_resets(self):
        a=SimpleNamespace(root=Mock(),hihat=Mock())
        r=HiHatSyncRuntime(a);r.engine=self.engine();r.engine.epoch=None
        r.observe(HybridStatus(swing=True,count=1,bottoms=1,bottom_at=10.5),10.51)
        self.assertAlmostEqual(r.engine.epoch,10.7)
        r.observe(HybridStatus(swing=True,count=3,bottoms=2,bottom_at=11.),11.01)
        self.assertAlmostEqual(r.engine.epoch,10.7)
        r.stop();self.assertFalse(r.active)

    def test_runtime_fault_uses_controlled_stop_not_motor_release(self):
        a=SimpleNamespace(root=Mock(),hihat=Mock(),continuous_strike_active=True,
                          continuous_stop_requested=False,_request_continuous_stop=Mock())
        r=HiHatSyncRuntime(a);r.engine=self.engine();r.last_observed=1.
        with patch('camera_playback.hihat_sync_runtime.time.monotonic',return_value=10.):r.tick()
        a._request_continuous_stop.assert_called_once()
        a.hihat.release_all.assert_not_called()
        self.assertFalse(r.active)


if __name__=='__main__':unittest.main()
