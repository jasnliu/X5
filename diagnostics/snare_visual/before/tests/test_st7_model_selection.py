"""Offline ST7 version-selection checks; no microphone, ROS processes or CAN."""
from pathlib import Path
import runpy
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch, call

from camera_playback.audio_bridge import detector_command, ST7_MODEL_VERSION, HIHAT_MODEL_VERSION


ROOT = Path(__file__).resolve().parents[1]
ASSETS = (".venv/bin/python", "cymbal.py", "models/v2/config.yaml",
          "models/v2/best.pt", "models/v2/normality/reference.npz")


class ModelSelectionTests(unittest.TestCase):
    def assets(self, root):
        for name in ASSETS:
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()

    def test_explicit_matching_v2_assets_not_baseline_defaults(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.assets(root)
            self.assertEqual(ST7_MODEL_VERSION, "v2")
            self.assertEqual(detector_command(root, "pipewire"), [
                str(root / ".venv/bin/python"), str(root / "cymbal.py"),
                "--config", str(root / "models/v2/config.yaml"), "listen",
                "--checkpoint", str(root / "models/v2/best.pt"),
                "--normality-artifact", str(root / "models/v2/normality/reference.npz"),
                "--device", "pipewire",
            ])

    def test_missing_v2_never_falls_back_to_existing_older_assets(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.assets(root)
            for name in ("configs/baseline.yaml", "runs/cymbal-fmn-tcn/best.pt",
                         "runs/cymbal-normality/reference.npz", "models/v1/best.pt"):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.touch()
            for name in ASSETS:
                with self.subTest(missing=name):
                    path = root / name
                    path.unlink()
                    with self.assertRaisesRegex(RuntimeError, "ST7 v2 is incomplete") as error:
                        detector_command(root, "pipewire")
                    self.assertIn(str(path), str(error.exception))
                    path.touch()

    def test_hihat_explicit_matching_assets_and_no_ride_fallback(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp);self.assets(root)
            self.assertEqual(HIHAT_MODEL_VERSION, 'hihat_v1')
            with self.assertRaisesRegex(RuntimeError, 'hihat_v1 is incomplete'):
                detector_command(root, 'pipewire', 'hihat')
            for name in ('config.yaml', 'best.pt', 'normality/reference.npz'):
                path = root/'models/hihat_v1'/name
                path.parent.mkdir(parents=True, exist_ok=True);path.touch()
            command = detector_command(root, 'pipewire', 'hihat')
            self.assertEqual(command[3], str(root/'models/hihat_v1/config.yaml'))
            self.assertEqual(command[6], str(root/'models/hihat_v1/best.pt'))
            self.assertEqual(command[8], str(root/'models/hihat_v1/normality/reference.npz'))

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

    def test_all_audio_modes_preflight_v2_and_pass_sibling_root(self):
        for mode in ([], ['--hardware'], ['--hardwaretest']):
            with self.subTest(mode=mode):
                command, _, service, description, code = self.launch(mode)
                self.assertEqual(code, 0)
                self.assertEqual(command.call_args_list, [
                    call(ROOT.parent / 'st7', 'pipewire'),
                    call(ROOT.parent / 'st7', 'pipewire', 'hihat')])
                audio = [a for a in description.actions if isinstance(a, SimpleNamespace)
                         and 'camera_playback.audio_bridge' in getattr(a, 'cmd', [])]
                self.assertEqual(len(audio), 2)
                self.assertEqual(len({a.cmd[a.cmd.index('--socket')+1] for a in audio}), 2)
                self.assertNotIn('--instrument', audio[0].cmd)  # default ride
                self.assertEqual(audio[1].cmd[audio[1].cmd.index('--instrument')+1], 'hihat')
                for a in audio:
                    self.assertEqual(a.cmd[a.cmd.index('--root')+1], str(ROOT.parent / 'st7'))
                app = next(a for a in description.actions if isinstance(a, SimpleNamespace)
                           and 'camera_playback.app' in getattr(a, 'cmd', []))
                self.assertEqual(app.cmd[app.cmd.index('--hihat-audio-socket')+1],
                                 audio[1].cmd[audio[1].cmd.index('--socket')+1])
                service.run.assert_called_once()

    def test_pure_test_mode_does_not_require_or_launch_st7(self):
        command, _, _, description, code = self.launch(['--test'], RuntimeError('missing V2'))
        self.assertEqual(code, 0)
        command.assert_not_called()
        self.assertFalse(any(isinstance(a, SimpleNamespace)
                             and 'camera_playback.audio_bridge' in getattr(a, 'cmd', [])
                             for a in description.actions))

    def test_missing_v2_stops_before_any_process_is_constructed(self):
        command, process, service, _, code = self.launch(['--hardware'], RuntimeError('missing V2'))
        command.assert_called_once()
        process.assert_not_called()
        service.run.assert_not_called()
        self.assertEqual(code, 'missing V2')

    def test_missing_optional_hihat_does_not_gate_ride_workflow(self):
        def preflight(root, device, instrument='ride'):
            if instrument == 'hihat':raise RuntimeError('missing optional hi-hat')
        _, _, service, _, code = self.launch(['--hardware'], preflight)
        self.assertEqual(code, 0)
        service.run.assert_called_once()


if __name__ == '__main__':
    unittest.main()
