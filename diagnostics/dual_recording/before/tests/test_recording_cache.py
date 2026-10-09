"""Persistent preflight memoization; real geometry, no physical devices."""
from dataclasses import fields, replace
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from camera_playback import recording_cache as cache
from camera_playback.smooth_recording import load_smooth_recording
from camera_playback.trajectory import PlaybackTrajectory, load_playback_trajectory
from smooth_playback.trajectory import Geometry, ROOT


def forbid_can(event, args):
    if event == 'socket.__new__' and args[1] == socket.PF_CAN:
        raise RuntimeError('Cache tests prohibit physical CAN')


sys.addaudithook(forbid_can)


class RecordingCacheTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        g = Geometry()
        cls.original = ROOT / 'recordings/record1.json'
        cls.raw = cls.original.read_bytes()
        args = (cls.original, g.model, g.zone, g.lower, g.upper, g.center, .4)
        cls.smooth = load_smooth_recording(*args)
        cls.legacy = load_playback_trajectory(*args, playback_speed=.8)

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.path = self.directory / 'recording.json'
        self.path.write_bytes(self.raw)
        self.cache_dir = self.directory / 'cache'
        self.g = Geometry()
        self.args = (self.path, self.g.model, self.g.zone, self.g.lower,
                     self.g.upper, self.g.center, .4)
        self.result = replace(self.smooth, source=self.path)

    def key(self, kind='smooth', args=None, playback_speed=.8):
        return cache._fingerprint(kind, *(args or self.args), playback_speed)

    def seed(self, kind='smooth'):
        result = self.result if kind == 'smooth' else replace(self.legacy, source=self.path)
        key = self.key(kind)
        path = self.cache_dir / (key + '.json')
        cache._write(path, key, cache._encode(result))
        return path

    def load(self, **kwargs):
        return cache.load_cached_smooth_recording(*self.args, cache_dir=self.cache_dir, **kwargs)

    def assert_same(self, expected, actual):
        self.assertIs(type(expected), type(actual))
        for field in fields(PlaybackTrajectory):
            a, b = getattr(expected, field.name), getattr(actual, field.name)
            if isinstance(a, np.ndarray):
                self.assertEqual(a.shape, b.shape)
                np.testing.assert_array_equal(a, b)
            else:
                self.assertEqual(a, b)
        for t in np.linspace(-.1, expected.duration_s + .1, 2001):
            a, af = expected.joints_at(t)
            b, bf = actual.joints_at(t)
            np.testing.assert_array_equal(a, b)
            self.assertEqual(af, bf)
        if hasattr(expected, 'smooth_motion'):
            a, b = expected.smooth_motion, actual.smooth_motion
            self.assertEqual(a.metadata, b.metadata)
            self.assertEqual(a.name, b.name)
            self.assertEqual(a.scale, b.scale)
            self.assertEqual(a.source_duration, b.source_duration)
            self.assertEqual(a.command_schedule, b.command_schedule)
            for pa, pb in zip(a.curves + [a.reference_map], b.curves + [b.reference_map]):
                self.assertIs(type(pa), type(pb))
                if pa is not None:
                    np.testing.assert_array_equal(pa.c, pb.c)
                    np.testing.assert_array_equal(pa.x, pb.x)
                    self.assertEqual(pa.axis, pb.axis)
                    self.assertEqual(pa.extrapolate, pb.extrapolate)
            self.assertIs(actual.geometry.model, self.g.model)
            self.assertIs(actual.geometry.zone, self.g.zone)

    def test_cold_runs_exact_loader_then_hit_restores_exact_result(self):
        progress = []
        before = (self.path.read_bytes(), self.path.stat().st_mtime_ns)
        with patch.object(cache.smooth_recording, 'load_smooth_recording', wraps=load_smooth_recording) as loader:
            cold = self.load(progress=progress.append)
        loader.assert_called_once()
        self.assertEqual(loader.call_args.args, self.args)
        self.assertEqual(loader.call_args.kwargs['playback_speed'], .8)
        self.assertIn('Reading JSON', progress[0])
        self.assertTrue(any('31 representative recovery' in s for s in progress))
        self.assertEqual(len(list(self.cache_dir.glob('*.json'))), 1)
        with patch.object(cache.smooth_recording, 'load_smooth_recording', side_effect=AssertionError('must hit cache')) as loader:
            progress.clear()
            warm = self.load(progress=progress.append)
        loader.assert_not_called()
        self.assertEqual(progress, ['Loaded cached recording preflight (all inputs unchanged)'])
        self.assert_same(cold, warm)
        self.assertEqual(before, (self.path.read_bytes(), self.path.stat().st_mtime_ns))
        self.assertEqual(self.original.read_bytes(), self.raw)

    def test_cache_survives_fresh_python_process(self):
        self.seed()
        script = '''
import socket,sys
def audit(event,args):
    if event=='socket.__new__' and args[1]==socket.PF_CAN: raise RuntimeError('No CAN')
sys.addaudithook(audit)
from unittest.mock import patch
from camera_playback import recording_cache as c
from smooth_playback.trajectory import Geometry
g=Geometry()
with patch.object(c.smooth_recording,'load_smooth_recording',side_effect=AssertionError('cache missed')) as loader:
    r=c.load_cached_smooth_recording(sys.argv[1],g.model,g.zone,g.lower,g.upper,g.center,.4,cache_dir=sys.argv[2],progress=print)
    loader.assert_not_called()
    assert r.sample_count==109
print('PERSISTENT HIT')
'''
        result = subprocess.run([sys.executable, '-B', '-c', script, str(self.path), str(self.cache_dir)],
                                cwd=ROOT, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('PERSISTENT HIT', result.stdout)

    def test_simulation_and_hardware_caches_are_separate_and_exact(self):
        self.seed('smooth')
        with patch.object(cache.trajectory, 'load_playback_trajectory', wraps=load_playback_trajectory) as loader:
            cold = cache.load_cached_playback_trajectory(*self.args, playback_speed=.8, cache_dir=self.cache_dir)
        loader.assert_called_once()
        with patch.object(cache.trajectory, 'load_playback_trajectory', side_effect=AssertionError('must hit cache')):
            warm = cache.load_cached_playback_trajectory(*self.args, playback_speed=.8, cache_dir=self.cache_dir)
        self.assert_same(cold, warm)
        self.assertEqual(len(list(self.cache_dir.glob('*.json'))), 2)

    def test_changed_contents_same_size_and_mtime_cannot_inherit_pass(self):
        self.seed()
        timestamp = self.path.stat().st_mtime_ns
        changed = self.raw.replace(b'"rad"', b'"deg"')
        self.assertEqual(len(changed), len(self.raw))
        self.path.write_bytes(changed)
        os.utime(self.path, ns=(timestamp, timestamp))
        with self.assertRaises(ValueError) as original:
            load_smooth_recording(*self.args)
        with self.assertRaises(ValueError) as cached:
            self.load()
        self.assertEqual(str(original.exception), str(cached.exception))
        self.assertEqual(len(list(self.cache_dir.glob('*.json'))), 1)

    def test_all_geometry_settings_and_versions_invalidate(self):
        before = self.key()
        for index in (3, 4, 5):
            args = list(self.args)
            args[index] = args[index].copy()
            args[index][0] += .001
            self.assertNotEqual(before, self.key(args=tuple(args)))
        self.assertNotEqual(before, self.key(args=(*self.args[:-1], .41)))
        self.assertNotEqual(before, self.key(playback_speed=.7))
        copy = self.directory / 'renamed.json'
        copy.write_bytes(self.raw)
        self.assertNotEqual(before, self.key(args=(copy, *self.args[1:])))
        for owner, name, value in (
                (self.g.model, 'digest', 'different'),
                (self.g.zone, 'model_hash', 'different'),
                (self.g.zone, 'tcp', 'different'),
                (self.g.zone, 'points', []),
                (self.g.zone, '_dirty', True),
                (cache.trajectory, 'RETURN_CHECK_ANCHORS', 32),
                (cache.smooth_recording, 'RECORD3_DURATION_RATIO', 1.3),
                (cache.np, '__version__', 'different'),
                (cache.scipy, '__version__', 'different'),
                (cache.sys, 'version', 'different'),
                (cache, 'CACHE_FORMAT', cache.CACHE_FORMAT + 1)):
            with self.subTest(name=name), patch.object(owner, name, value):
                self.assertNotEqual(before, self.key())
        self.g.model.joints[0].set('name', 'changed-in-memory')
        self.assertNotEqual(before, self.key())
        self.g = Geometry()
        self.args = (self.path, self.g.model, self.g.zone, self.g.lower, self.g.upper, self.g.center, .4)
        self.g.zone.hull.equations[0, 3] += .0001
        self.assertNotEqual(before, self.key())

    def test_changed_setting_runs_original_loader_not_cached_result(self):
        self.seed()
        with patch.object(cache.trajectory, 'RETURN_CHECK_ANCHORS', 32), \
             patch.object(cache.smooth_recording, 'load_smooth_recording', side_effect=ValueError('original check')) as loader:
            with self.assertRaisesRegex(ValueError, 'original check'):
                self.load()
        loader.assert_called_once()

    def test_live_code_edit_disables_cache_until_restart(self):
        self.seed()
        with patch.object(cache, '_code_digest', return_value='changed'), \
             patch.object(cache.smooth_recording, 'load_smooth_recording', return_value=self.result) as loader, \
             patch.object(cache, '_write') as write:
            self.assertIs(self.load(), self.result)
        loader.assert_called_once()
        write.assert_not_called()

    def test_changed_code_after_restart_gets_new_key(self):
        before = self.key()
        with patch.object(cache, '_code_digest', return_value='changed'), \
             patch.object(cache, '_LOADED_CODE_DIGEST', 'changed'):
            self.assertNotEqual(before, self.key())

    def test_corrupt_stale_and_incomplete_entries_fall_back(self):
        for damage in ('truncate', 'checksum', 'key', 'format', 'structure'):
            with self.subTest(damage=damage):
                path = self.seed()
                data = json.loads(path.read_text())
                if damage == 'truncate':
                    path.write_text('{')
                else:
                    if damage == 'checksum':
                        data['payload']['base']['time_scale'] = 999
                    elif damage == 'structure':
                        del data['payload']['base']['joints']
                        data['sha256'] = hashlib.sha256(cache._json_bytes(data['payload'])).hexdigest()
                    else:
                        data[damage] = 'wrong'
                    path.write_text(json.dumps(data))
                with patch.object(cache.smooth_recording, 'load_smooth_recording', return_value=self.result) as loader:
                    self.assertIs(self.load(), self.result)
                loader.assert_called_once()
                self.assert_same(self.result, self.load())

    def test_failed_preflight_never_writes_pass(self):
        with patch.object(cache.smooth_recording, 'load_smooth_recording', side_effect=ValueError('unsafe')):
            with self.assertRaisesRegex(ValueError, 'unsafe'):
                self.load()
        self.assertFalse(self.cache_dir.exists())

    def test_cache_io_failure_does_not_change_success_or_original_error(self):
        for result, error in ((self.result, None), (None, ValueError('unsafe'))):
            with patch.object(cache, '_read', side_effect=PermissionError), \
                 patch.object(cache, '_write', side_effect=OSError('disk full')), \
                 patch.object(cache.smooth_recording, 'load_smooth_recording', return_value=result, side_effect=error):
                if error:
                    with self.assertRaisesRegex(ValueError, 'unsafe'):
                        self.load()
                else:
                    self.assertIs(self.load(), result)

    def test_inputs_changed_during_validation_are_not_cached(self):
        with patch.object(cache, '_fingerprint', side_effect=['before', 'after']), \
             patch.object(cache.smooth_recording, 'load_smooth_recording', return_value=self.result), \
             patch.object(cache, '_write') as write:
            self.assertIs(self.load(), self.result)
        write.assert_not_called()

    def test_inputs_changed_during_cache_read_force_original_preflight(self):
        self.seed()
        key = self.key()
        with patch.object(cache, '_fingerprint', side_effect=[key, 'changed', 'changed']), \
             patch.object(cache.smooth_recording, 'load_smooth_recording', return_value=self.result) as loader, \
             patch.object(cache, '_write') as write:
            self.assertIs(self.load(), self.result)
        loader.assert_called_once()
        write.assert_not_called()

    def test_source_missing_or_renamed_non_json_still_runs_original_errors(self):
        self.seed()
        self.path.unlink()
        with self.assertRaisesRegex(ValueError, 'does not exist'):
            self.load()
        self.path = self.directory / 'not-json.txt'
        self.path.write_bytes(self.raw)
        self.args = (self.path, *self.args[1:])
        with self.assertRaisesRegex(ValueError, 'Select a .json'):
            self.load()

    def test_atomic_write_failure_keeps_previous_entry_and_cleans_temp(self):
        path = self.seed()
        before = path.read_bytes()
        with patch.object(cache.os, 'replace', side_effect=OSError('write failed')):
            with self.assertRaises(OSError):
                cache._write(path, self.key(), cache._encode(self.result))
        self.assertEqual(before, path.read_bytes())
        self.assertEqual(list(self.cache_dir.glob('*.tmp')), [])
        self.assertEqual(path.stat().st_mode & 0o777, 0o600)

    def test_default_location_honors_xdg(self):
        with patch.dict(os.environ, {'XDG_CACHE_HOME': str(self.directory)}):
            self.assertEqual(cache.cache_directory(), self.directory / 'openarmx/recording-preflight')

    def test_background_worker_uses_cache_without_changing_queue_contract(self):
        from camera_playback.recording_preflight import RecordingPreflight
        self.seed()
        with patch.object(cache, 'cache_directory', return_value=self.cache_dir), \
             patch.object(cache.smooth_recording, 'load_smooth_recording', side_effect=AssertionError('must hit cache')) as loader:
            job = RecordingPreflight(*self.args, playback_speed=.8)
            job.thread.join(5)
        self.assertFalse(job.thread.is_alive())
        loader.assert_not_called()
        events = job.poll()
        self.assertEqual([kind for kind, value in events], ['progress', 'result'])
        self.assertIn('cached recording preflight', events[0][1])
        self.assert_same(self.result, events[1][1])

    def test_single_frame_and_stationary_roundtrip_preserve_shapes(self):
        for count in (1, 3):
            with self.subTest(count=count):
                data = json.loads(self.raw)
                sample = data['samples'][0]
                data['samples'] = [dict(sample, time_s=i*.1) for i in range(count)]
                data['sample_count'] = count
                self.path.write_text(json.dumps(data))
                cold = self.load()
                with patch.object(cache.smooth_recording, 'load_smooth_recording', side_effect=AssertionError('must hit cache')):
                    warm = self.load()
                self.assert_same(cold, warm)
                self.assertEqual(warm.curve_coefficients.shape, (count - 1, 4, 7))


if __name__ == '__main__':
    unittest.main()
