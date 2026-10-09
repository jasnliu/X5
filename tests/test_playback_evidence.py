"""Acceptance recorder cannot turn a partial/faulted run into a pass. NO I/O."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
from camera_playback.playback_evidence import PlaybackEvidence
from camera_playback.snare_beat import SnareStatus


class PlaybackEvidenceTests(unittest.TestCase):
    def fixture(self,root):
        bus=SimpleNamespace(active=False, fresh=lambda:True,
            states={(s,i):(0.,0,1.) for s in ('left','right') for i in range(1,9)},
            left_joint6_session=SimpleNamespace(status=SnareStatus(armed=True)),
            disable_centered_side=Mock())
        a=SimpleNamespace(bus=bus, root=Mock(), dual=SimpleNamespace(left_done=True,left_return=None),
            phase='RELAXED',status=Mock(get=lambda:'Both disabled'),fail=Mock(),_program_failure=Mock(),
            center_relax=Mock(),request_safe_close=Mock())
        a.disable_mock = bus.disable_centered_side
        with patch('camera_playback.playback_evidence.__file__', str(Path(root)/'camera_playback'/'evidence.py')):
            e=PlaybackEvidence(a)
        e.started=e.endpoint=True
        e.disabled_proofs={'left':{},'right':{}}
        self.addCleanup(e.stream.close)
        return a,e

    def result(self,e):return json.loads((e.directory/'result.json').read_text())

    def test_complete_run_passes_only_after_all_16_disabled(self):
        with tempfile.TemporaryDirectory() as p:
            a,e=self.fixture(p);e.tick()
            self.assertTrue(self.result(e)['success'])
            a.request_safe_close.assert_called_once()

    def test_missing_endpoint_or_prior_fault_cannot_pass(self):
        for missing in ('endpoint','fault','left','snare','proof'):
            with self.subTest(missing=missing),tempfile.TemporaryDirectory() as p:
                a,e=self.fixture(p)
                if missing=='endpoint':e.endpoint=False
                if missing=='fault':e.faults.append({'message':'real fault'})
                if missing=='left':a.dual.left_done=False
                if missing=='snare':a.bus.left_joint6_session=None
                if missing=='proof':e.disabled_proofs.pop('left')
                e.tick();self.assertFalse(self.result(e)['success'])

    def test_left_return_fault_is_recorded_even_without_app_fail_callback(self):
        with tempfile.TemporaryDirectory() as p:
            a,e=self.fixture(p);a.phase='CENTER RELAX RECENTERING';a.bus.active=True
            a.dual.left_return=SimpleNamespace(failure='Joint limit violation')
            e.tick();self.assertEqual(e.faults[0]['source'],'LeftReturn')
            self.assertFalse(self.result(e)['success'])
            a.request_safe_close.assert_not_called()
            a.disable_mock.assert_not_called()

    def test_strike_entry_points_are_fail_closed(self):
        with tempfile.TemporaryDirectory() as p:
            a,e=self.fixture(p)
            for method in (a._begin_strike_attempt,a._begin_continuous_striking):
                with self.assertRaisesRegex(RuntimeError,'Strike forbidden'):method()
            a._begin_playback_alignment(1.)
            a.center_relax.assert_called_once()

    def test_failure_is_preserved_and_does_not_force_relax(self):
        with tempfile.TemporaryDirectory() as p:
            a,e=self.fixture(p);a.bus.active=True;a.phase='LEFT RECORDING PLAYBACK'
            a.fail('queue fault');e.tick()
            self.assertEqual(e.faults[0]['message'],'queue fault')
            a.disable_mock.assert_not_called()
            a.request_safe_close.assert_not_called()
