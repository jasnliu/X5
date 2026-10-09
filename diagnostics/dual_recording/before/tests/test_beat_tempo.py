"""Tempo regression tests: no CAN, microphone, camera or serial hardware."""
import math
import unittest
from unittest.mock import Mock, patch

import numpy as np

from camera_playback.app import App, SWING_EVENTS, SWING_PICKUP_INDEX
from camera_playback.tempo import parse_bpm, swing_events, MIN_BPM, MAX_BPM
from camera_playback.hihat_sync import HiHatSynchronizer, MAX_ADVANCE, RIDE_GATE
from camera_playback.hihat_sync_runtime import HiHatSyncRuntime
from camera_playback.hybrid_strike import HybridStatus
from test_hybrid_workflow import app_fixture
from test_hybrid_strike import Harness


class TempoTests(unittest.TestCase):
    def test_default_pattern_unchanged(self):
        self.assertEqual(swing_events(), SWING_EVENTS)
        np.testing.assert_allclose([e[2] for e in SWING_EVENTS], [.6, .4, .2, .6, .4, .2])

    def test_validation_including_fractional_bpm(self):
        for value in (' 87.5 ', '100', MIN_BPM, MAX_BPM):
            self.assertEqual(parse_bpm(value), float(value))
        for value in ('', '-', 'abc', 'nan', 'inf', '-inf', 0, -20, 19.9, 180.1, True, None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_bpm(value)

    def test_every_bar_and_short_triplet_scale(self):
        for bpm in (20, 60, 87.5, 100, 120, 150, 180):
            with self.subTest(bpm=bpm):
                e = swing_events(bpm)
                self.assertEqual([x[:2] for x in e], [x[:2] for x in SWING_EVENTS])
                self.assertAlmostEqual(sum(x[2] for x in e), 4*60/bpm)
                self.assertAlmostEqual(e[-1][2], 20/bpm)
                self.assertAlmostEqual(e[1][2], 2*e[2][2])

    def test_simulation_uses_selected_event_intervals_including_pickup(self):
        a = App.__new__(App)
        a._strike_bpm = 87.5
        a.continuous_swing_index = SWING_PICKUP_INDEX
        a.continuous_first_swing_hit = True
        e = [a._take_next_swing_event() for _ in range(13)]
        self.assertEqual(e[0][0], 'pickup (extra before beat 1)')
        for i, row in enumerate(e):
            self.assertAlmostEqual(row[2], swing_events(87.5)[(i-1)%6][2])
        self.assertEqual(sum(row[1] for row in e[:7]), 4)

    def test_instances_do_not_mutate_default_or_each_other(self):
        a, b = App.__new__(App), App.__new__(App)
        a._strike_bpm, b._strike_bpm = 80, 120
        self.assertEqual(a.swing_events, swing_events(80))
        self.assertEqual(b.swing_events, swing_events(120))
        self.assertEqual(SWING_EVENTS, swing_events(100))

    def test_selected_tempo_reaches_hybrid_and_hihat_not_search(self):
        for bpm in (60, 87.5, 120, 180):
            with self.subTest(bpm=bpm):
                a = app_fixture()
                a._strike_bpm = bpm
                a.hihat_sync = Mock()
                a._begin_strike_attempt()
                a.hybrid_session.request.assert_called_once_with('search', math.radians(5))
                a.hybrid_session.request.reset_mock()
                target = a.strike_plan.anchor_joints.copy()
                target[6] -= math.radians(10.5)
                a._begin_continuous_striking(10.5, target)
                a.hybrid_session.request.assert_called_once_with('swing', math.radians(10.5), swing_events(bpm))
                self.assertAlmostEqual(a.hihat_sync.start.call_args.args[0], 60/bpm)
                self.assertEqual(a.continuous_strike_degrees, 10.5)
                self.assertIn(f'{bpm:g} BPM', a.result_status.set.call_args.args[0])
                a.bus.set_positions.assert_not_called()

    def test_input_editing_is_locked_for_all_active_phases(self):
        a = App.__new__(App)
        a.bus = Mock(active=False)
        for phase in ('READY', 'RELAXED'):
            a.phase = phase
            self.assertTrue(a._tempo_editable())
        for phase in ('PLAYBACK PREFLIGHTED', 'CENTERING', 'WAITING FOR LOAD', 'STRIKE MOVING OUT',
                      'SWING STRIKE RETURNING', 'CENTER RELAX RECENTERING'):
            a.phase = phase
            self.assertFalse(a._tempo_editable())
        a.phase = 'READY'; a.bus.active = True
        self.assertFalse(a._tempo_editable())

    def test_invalid_input_rejected_by_start_before_preparation(self):
        a = App.__new__(App)
        a.hardware = True; a.playback_trajectory = Mock()
        a.phase = 'READY'; a.bus = Mock(active=False)
        a.bus.fresh.return_value = True
        a.bpm_text = Mock(); a.bpm_text.get.return_value = 'nan'
        a.status = Mock(); a.root = Mock()
        a.start()
        self.assertIn('Cannot start:', a.status.set.call_args.args[0])
        a.bus.center.assert_not_called(); a.root.after.assert_not_called()
        self.assertEqual(a.phase, 'READY')

    def test_disabled_widget_changes_cannot_retime_current_run(self):
        a = App.__new__(App)
        a._strike_bpm = 85; a.phase = 'SWING STRIKE RETURNING'; a.bus = Mock(active=True)
        a.bpm_entry = Mock(); a.tempo_hint = Mock()
        a.bpm_text = Mock(); a.bpm_text.get.return_value = '150'
        self.assertTrue(a._refresh_tempo_controls())
        self.assertEqual(a.strike_bpm, 85)
        a.bpm_text.get.assert_not_called()
        a.bpm_entry.config.assert_called_once_with(state='disabled')

    def test_real_hybrid_controller_scaled_grid_without_changing_stroke_tuning(self):
        for bpm in (60, 87.5, 120):
            with self.subTest(bpm=bpm):
                h = Harness(10)
                parameters = h.p.copy()
                h.until(lambda s: s.completed == 1)
                h.c.request_swing(math.radians(10.5), swing_events(bpm))
                main, scheduled = [], []
                bottoms = mains = 0
                for _ in range(9000):
                    h.tick(); s = h.c.status
                    if s.bottoms > bottoms:
                        scheduled.append(s.scheduled_at if s.scheduled_at is not None else s.bottom_at)
                        bottoms = s.bottoms
                    if s.main_count > mains:
                        main.append(s.main_at); mains = s.main_count
                    if s.bottoms >= 13:
                        break
                self.assertGreaterEqual(s.bottoms, 13)
                self.assertIsNone(s.error)
                self.assertEqual(h.p, parameters)
                for i, (a,b) in enumerate(zip(scheduled, scheduled[1:])):
                    self.assertAlmostEqual(b-a, swing_events(bpm)[(i-1)%6][2])
                for a,b in zip(main, main[1:]):
                    self.assertAlmostEqual(b-a, 60/bpm)
                h.c.stop_swing()
                h.until(lambda s: s.ready and not s.swing)


class HiHatTempoTests(unittest.TestCase):
    def test_default_bounds_unchanged_faster_tempo_bounds_scale(self):
        for bpm in (20, 100, 120, 150, 180):
            period = 60/bpm
            e = HiHatSynchronizer(Mock(), period=period)
            self.assertEqual(e.max_advance, min(MAX_ADVANCE, period/2))
            self.assertEqual(e.ride_gate, min(RIDE_GATE, period/3))
        with self.assertRaises(ValueError):
            HiHatSynchronizer(Mock(), period=float('nan'))

    def test_opens_1_3_closes_2_4_and_signed_advance_preserves_closed_duration(self):
        for bpm in (20, 60, 87.5, 120, 150, 180):
            for sign in (-1, 0, 1):
                period = 60/bpm
                sent, rows = [], []
                now = 10.
                def send(closed):
                    sent.append((now, closed))
                    return now
                e = HiHatSynchronizer(send, rows.append, period)
                e.start(10.); e.set_epoch(10.2, 10.)
                e.advance = sign*.08
                for index in range(12):
                    advance = 0 if index == 0 else e.pair_advance
                    now = 10.2+index*period-advance+1e-9
                    e.tick(now, {})
                self.assertEqual([s[1] for s in sent], [False, True]*6)
                commands = [r for r in rows if r['kind']=='command']
                for c in commands:
                    self.assertAlmostEqual(c['target'], 10.2+c['index']*period)
                    self.assertEqual(c['closed'], c['beat'] in (2,4))
                for close,opened in zip(commands[1::2], commands[2::2]):
                    self.assertEqual(close['advance'], opened['advance'])
                    self.assertAlmostEqual(opened['at']-close['at'], period)

    def test_continuous_audio_feedback_converges_at_multiple_tempos(self):
        for bpm in (60, 120, 180):
            for latency, ride_shift in ((.110, -.025), (.030, .070)):
                with self.subTest(bpm=bpm, latency=latency):
                    period = 60/bpm
                    now = 10.
                    sent, rows, pending = [], [], []
                    def send(closed):
                        sent.append((now, closed))
                        return now
                    e = HiHatSynchronizer(send, rows.append, period)
                    e.start(now); e.set_epoch(10.2, now)
                    ride_times = [10.2+bar*4*period+offset*period+ride_shift
                                  for bar in range(50) for offset in (0,1,5/3,2,3,11/3)]
                    ri = ci = 0
                    duration = max(60, 60*period)
                    for step in range(int(duration/.005)):
                        now = 10.+step*.005
                        while ri < len(ride_times) and ride_times[ri] <= now:
                            onset = ride_times[ri]; ri += 1
                            pending.append((onset+.7, 'ride', onset))
                        while ci < len(sent):
                            command, closed = sent[ci]; ci += 1
                            if closed:
                                pending.append((command+latency+1., 'hihat', command+latency))
                        ready = [p for p in pending if p[0] <= now]
                        pending = [p for p in pending if p[0] > now]
                        for _, instrument, onset in reversed(ready):
                            e.feed(instrument, [dict(kind='hit', instrument=instrument,
                                   event_at=onset, score=.99)], now)
                        for instrument, lag in [('ride', .7), ('hihat', 1.)]:
                            e.feed(instrument, [dict(kind='progress', instrument=instrument,
                                   finalized_at=now-lag-.005)], now)
                        e.tick(now, {'ride': True, 'hihat': True})
                    matches = [r for r in rows if r['kind']=='match']
                    self.assertGreater(len(matches), 20)
                    self.assertLess(abs(e.advance-(latency-ride_shift)), .025)
                    self.assertLess(abs(sum(r['error'] for r in matches[-5:])/5), .025)
                    self.assertEqual(e.epoch, 10.2)  # Read-only ride reference.

    def test_runtime_pickup_to_beat1_scales_with_bpm(self):
        for bpm in (60, 120, 180):
            a = Mock()
            r = HiHatSyncRuntime(a)
            r.engine = HiHatSynchronizer(Mock(), period=60/bpm)
            r.engine.start(10.)
            r.observe(HybridStatus(swing=True, count=1, bottoms=1, bottom_at=10.5), 10.51)
            self.assertAlmostEqual(r.engine.epoch, 10.5+20/bpm)


if __name__ == '__main__':
    unittest.main()
