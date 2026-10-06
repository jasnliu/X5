"""Offline ST7 version-selection checks; no microphone, ROS processes or CAN."""
from pathlib import Path
import runpy
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from camera_playback.audio_bridge import detector_command, ST7_MODEL_VERSION


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
                command.assert_called_once_with(ROOT.parent / 'st7', 'pipewire')
                audio = [a for a in description.actions if isinstance(a, SimpleNamespace)
                         and 'camera_playback.audio_bridge' in getattr(a, 'cmd', [])]
                self.assertEqual(len(audio), 1)
                self.assertEqual(audio[0].cmd[audio[0].cmd.index('--root')+1],
                                 str(ROOT.parent / 'st7'))
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


if __name__ == '__main__':
    unittest.main()
