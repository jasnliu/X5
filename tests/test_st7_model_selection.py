"""Offline ST7 version-selection checks; no microphone, ROS processes or CAN."""
from pathlib import Path
import runpy
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch, call

from camera_playback.audio_bridge import (
    detector_command, ST7_MODEL_VERSION, HIHAT_MODEL_VERSION, SNARE_MODEL_VERSION,
    MODEL_VERSIONS,
)


ROOT = Path(__file__).resolve().parents[1]
ASSETS = (".venv/bin/python", "cymbal.py", "models/ride_v4/config.yaml",
          "models/ride_v4/best.pt", "models/ride_v4/normality/reference.npz")


class ModelSelectionTests(unittest.TestCase):
    def assets(self, root):
        for name in ASSETS:
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()

    def test_all_new_versions_require_complete_matching_bundles(self):
        expected = {'ride': 'ride_v4', 'hihat': 'hihat_v2', 'snare': 'snare_v2'}
        self.assertEqual(MODEL_VERSIONS, expected)
        self.assertEqual(SNARE_MODEL_VERSION, 'snare_v2')
        assets = ('config.yaml', 'best.pt', 'normality/reference.npz')
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.assets(root)
            for version in (*expected.values(), 'ride_v3', 'v2', 'hihat_v1', 'snare_v1',
                            'hihat_v3', 'snare_v3'):
                for name in assets:
                    path = root / 'models' / version / name
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.touch()
            for instrument, version in expected.items():
                command = detector_command(root, 'pipewire', instrument)
                for flag, name in zip(
                    ('--config', '--checkpoint', '--normality-artifact'), assets
                ):
                    path = root / 'models' / version / name
                    self.assertEqual(command[command.index(flag)+1], str(path))
                    with self.subTest(instrument=instrument, missing=name):
                        path.unlink()
                        with self.assertRaisesRegex(RuntimeError, f'{version} is incomplete'):
                            detector_command(root, 'pipewire', instrument)
                        path.touch()

    def test_explicit_matching_ride_v4_assets_not_baseline_defaults(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.assets(root)
            self.assertEqual(ST7_MODEL_VERSION, "ride_v4")
            self.assertEqual(detector_command(root, "pipewire"), [
                str(root / ".venv/bin/python"), str(root / "cymbal.py"),
                "--config", str(root / "models/ride_v4/config.yaml"), "listen",
                "--checkpoint", str(root / "models/ride_v4/best.pt"),
                "--normality-artifact", str(root / "models/ride_v4/normality/reference.npz"),
                "--device", "pipewire",
            ])

    def test_missing_ride_v4_never_falls_back_to_existing_older_assets(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.assets(root)
            for name in ("configs/baseline.yaml", "runs/cymbal-fmn-tcn/best.pt",
                         "runs/cymbal-normality/reference.npz", "models/v1/best.pt",
                         "models/ride_v3/config.yaml", "models/ride_v3/best.pt",
                         "models/ride_v3/normality/reference.npz"):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.touch()
            for name in ASSETS:
                with self.subTest(missing=name):
                    path = root / name
                    path.unlink()
                    with self.assertRaisesRegex(RuntimeError, "ST7 ride_v4 is incomplete") as error:
                        detector_command(root, "pipewire")
                    self.assertIn(str(path), str(error.exception))
                    path.touch()

    def test_hihat_explicit_matching_assets_and_no_ride_fallback(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp);self.assets(root)
            self.assertEqual(HIHAT_MODEL_VERSION, 'hihat_v2')
            with self.assertRaisesRegex(RuntimeError, 'hihat_v2 is incomplete'):
                detector_command(root, 'pipewire', 'hihat')
            for name in ('config.yaml', 'best.pt', 'normality/reference.npz'):
                path = root/'models/hihat_v2'/name
                path.parent.mkdir(parents=True, exist_ok=True);path.touch()
            command = detector_command(root, 'pipewire', 'hihat')
            self.assertEqual(command[3], str(root/'models/hihat_v2/config.yaml'))
            self.assertEqual(command[6], str(root/'models/hihat_v2/best.pt'))
            self.assertEqual(command[8], str(root/'models/hihat_v2/normality/reference.npz'))

    def launch(self, mode, preflight_error=None):
        # Build only the launch description. No actual ROS launch APIs run.
        description = SimpleNamespace(actions=[])
        description.add_action = description.actions.append

        def describe(actions):
            description.actions.extend(actions)
            return description

        process = Mock(side_effect=lambda **kw: SimpleNamespace(**kw))
        service = Mock()
        service.run.return_value = 0
        modules = {}
        for name in ("launch", "launch.actions", "launch.event_handlers",
                     "launch.events", "launch_ros", "launch_ros.actions"):
            modules[name] = ModuleType(name)
        modules['launch'].LaunchDescription = describe
        modules['launch'].LaunchService = Mock(return_value=service)
        modules['launch.actions'].ExecuteProcess = process
        modules['launch.actions'].EmitEvent = Mock()
        modules['launch.actions'].RegisterEventHandler = Mock()
        modules['launch.event_handlers'].OnProcessExit = Mock()
        modules['launch.events'].Shutdown = Mock()
        modules['launch_ros.actions'].Node = Mock()
        with patch.dict(sys.modules, modules), \
                patch.object(sys, 'argv', ['launch_right_camera_playback.py', *mode]), \
                patch('pathlib.Path.is_file', return_value=True), \
                patch('camera_playback.audio_bridge.detector_command',
                      side_effect=preflight_error) as command:
            with self.assertRaises(SystemExit) as exit_status:
                runpy.run_path(str(ROOT / 'launch_right_camera_playback.py'), run_name='__main__')
        return command, process, service, description, exit_status.exception.code

    def test_all_audio_modes_preflight_ride_v4_and_pass_sibling_root(self):
        for mode in ([], ['--hardware'], ['--hardwaretest']):
            with self.subTest(mode=mode):
                command, _, service, description, code = self.launch(mode)
                self.assertEqual(code, 0)
                self.assertEqual(command.call_args_list, [
                    call(ROOT.parent / 'st7', 'pipewire'),
                    call(ROOT.parent / 'st7', 'pipewire', 'hihat'),
                    call(ROOT.parent / 'st7', 'pipewire', 'snare')])
                audio = [a for a in description.actions if isinstance(a, SimpleNamespace)
                         and 'camera_playback.shared_audio_bridge' in getattr(a, 'cmd', [])]
                self.assertEqual(len(audio), 1)
                cmd = audio[0].cmd
                channels = {cmd[i+1]: cmd[i+2:i+4]
                            for i, arg in enumerate(cmd) if arg == '--channel'}
                self.assertEqual(set(channels), {'ride', 'hihat', 'snare'})
                self.assertEqual(len({paths[0] for paths in channels.values()}), 3)
                self.assertEqual(len({paths[1] for paths in channels.values()}), 3)
                for a in audio:
                    self.assertEqual(a.cmd[a.cmd.index('--root')+1], str(ROOT.parent / 'st7'))
                app = next(a for a in description.actions if isinstance(a, SimpleNamespace)
                           and 'camera_playback.app' in getattr(a, 'cmd', []))
                self.assertEqual(app.cmd[app.cmd.index('--hihat-audio-socket')+1],
                                 channels['hihat'][0])
                self.assertEqual(app.cmd[app.cmd.index('--snare-audio-socket')+1],
                                 channels['snare'][0])
                service.run.assert_called_once()

    def test_pure_test_mode_does_not_require_or_launch_st7(self):
        command, _, _, description, code = self.launch(['--test'], RuntimeError('missing ride_v4'))
        self.assertEqual(code, 0)
        command.assert_not_called()
        self.assertFalse(any(isinstance(a, SimpleNamespace)
                             and any(m in getattr(a, 'cmd', []) for m in (
                                 'camera_playback.audio_bridge', 'camera_playback.shared_audio_bridge'))
                             for a in description.actions))

    def test_missing_ride_v4_stops_before_any_process_is_constructed(self):
        command, process, service, _, code = self.launch(['--hardware'], RuntimeError('missing ride_v4'))
        command.assert_called_once()
        process.assert_not_called()
        service.run.assert_not_called()
        self.assertEqual(code, 'missing ride_v4')

    def test_missing_optional_hihat_does_not_gate_ride_workflow(self):
        def preflight(root, device, instrument='ride'):
            if instrument == 'hihat':raise RuntimeError('missing optional hi-hat')
        _, _, service, description, code = self.launch(['--hardware'], preflight)
        self.assertEqual(code, 0)
        service.run.assert_called_once()
        self.assertEqual(sum('camera_playback.audio_bridge' in getattr(a, 'cmd', [])
                             for a in description.actions if isinstance(a, SimpleNamespace)), 3)

    def test_missing_visual_only_snare_does_not_block_hardware_launch(self):
        def preflight(root, device, instrument='ride'):
            if instrument == 'snare':
                raise RuntimeError('missing visual-only snare')
        _, _, service, description, code = self.launch(['--hardware'], preflight)
        self.assertEqual(code, 0)
        service.run.assert_called_once()
        self.assertEqual(sum('camera_playback.audio_bridge' in getattr(a, 'cmd', [])
                             for a in description.actions if isinstance(a, SimpleNamespace)), 3)

    def test_low_memory_refuses_launch_before_controllers_are_constructed(self):
        with patch('camera_playback.resource_preflight.check_memory',
                   side_effect=RuntimeError('low memory')):
            command, process, service, _, code = self.launch(['--hardware'])
        self.assertEqual(code, 'low memory')
        command.assert_not_called()
        process.assert_not_called()
        service.run.assert_not_called()


if __name__ == '__main__':
    unittest.main()
