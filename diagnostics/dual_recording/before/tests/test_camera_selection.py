"""Camera selection and launcher routing checks; no physical camera or motors."""
import argparse
import ast
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from camera_playback.camera_selection import choose_camera, list_cameras, print_cameras
from camera_search.device import camera_argument, open_camera, resolve_camera


ROOT = Path(__file__).resolve().parents[1]
LOGITECH = '/dev/v4l/by-id/usb-Test_Webcam_SERIAL-video-index0'


class CameraSelectionTests(unittest.TestCase):
    def test_argument_accepts_local_device_paths_and_rejects_non_live_inputs(self):
        for value in ('/dev/video0', LOGITECH, '/dev/v4l/by-path/pci-usb-video-index0'):
            self.assertEqual(camera_argument(value), value)
        for value in ('movie.mp4', 'rtsp://camera/live', '/tmp/image.jpg', '/dev/ttyUSB0'):
            with self.assertRaises(argparse.ArgumentTypeError): camera_argument(value)

    def test_path_resolves_to_camera_number_for_existing_capture_controls(self):
        with patch.object(Path, 'resolve', return_value=Path('/dev/video14')) as resolve:
            self.assertEqual(resolve_camera(LOGITECH), 14)
        resolve.assert_called_once_with(strict=True)

    def test_missing_or_non_video_device_path_cannot_fall_back(self):
        with patch.object(Path, 'resolve', side_effect=FileNotFoundError('missing')):
            with self.assertRaisesRegex(RuntimeError, 'path unavailable'): resolve_camera(LOGITECH)
        with patch.object(Path, 'resolve', return_value=Path('/tmp/frame.jpg')):
            with self.assertRaisesRegex(RuntimeError, 'local /dev/videoN'): resolve_camera(LOGITECH)

    def test_listing_excludes_metadata_and_identifies_orbbec_rgb(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            sysfs, dev = root/'sys', root/'dev'
            sysfs.mkdir(); dev.mkdir()
            config = root/'camera.json'
            config.write_text(json.dumps(dict(vendor_id='2bc5', product_id='0670', serial='GEMINI', usb_interface='04')))
            for number, interface, serial, vendor, product, name, index in [
                (0, '00', 'WEBCAM', '046d', '1234', 'Webcam', '0'),
                (1, '00', 'WEBCAM', '046d', '1234', 'Webcam metadata', '1'),
                (2, '02', 'GEMINI', '2bc5', '0670', 'Gemini', '0'),
                (4, '04', 'GEMINI', '2bc5', '0670', 'Gemini', '0'),
            ]:
                usb = root/serial
                face = usb/interface
                face.mkdir(parents=True, exist_ok=True)
                for key, value in [('idVendor', vendor), ('idProduct', product), ('serial', serial)]:
                    (usb/key).write_text(value)
                (face/'bInterfaceNumber').write_text(interface)
                entry = sysfs/f'video{number}'
                entry.mkdir(); (entry/'device').symlink_to(face)
                (entry/'name').write_text(name); (entry/'index').write_text(index)
                (dev/f'video{number}').touch()
            aliases = dev/'v4l/by-id'
            aliases.mkdir(parents=True)
            (aliases/'webcam').symlink_to(dev/'video0')
            rows = list_cameras(sysfs=sysfs, dev=dev, config_path=config)
            self.assertEqual([r['index'] for r in rows], [0, 2, 4])
            self.assertTrue(rows[2]['default'])
            self.assertTrue(rows[1]['non_rgb'])
            self.assertFalse(rows[0]['non_rgb'])
            self.assertEqual(rows[0]['aliases'], [str(aliases/'webcam')])
            lines = []
            print_cameras(rows, output=lines.append)
            self.assertIn('Webcam', '\n'.join(lines))
            self.assertIn('depth/IR', '\n'.join(lines))
            self.assertIn('--camera orbbec', '\n'.join(lines))

    def choices(self):
        return [dict(index=0, device='/dev/video0', name='Webcam', serial='W',
                     default=False, non_rgb=False, aliases=[LOGITECH]),
                dict(index=4, device='/dev/video4', name='Gemini', serial='G',
                     default=True, non_rgb=False, aliases=[])]

    def test_picker_prefers_stable_identity_after_explicit_number_choice(self):
        with patch('sys.stdin.isatty', return_value=True):
            self.assertEqual(choose_camera(cameras=self.choices(), input_fn=lambda _: '0', output=lambda _: None), LOGITECH)
            self.assertEqual(choose_camera(cameras=self.choices(), input_fn=lambda _: '4', output=lambda _: None), 'orbbec')

    def test_picker_reprompts_for_invalid_or_metadata_input(self):
        answers = iter(['nonsense', '1', '0'])
        messages = []
        with patch('sys.stdin.isatty', return_value=True):
            chosen = choose_camera(cameras=self.choices(), input_fn=lambda _: next(answers), output=messages.append)
        self.assertEqual(chosen, LOGITECH)
        self.assertEqual(sum('Invalid camera selection' in line for line in messages), 2)

    def test_picker_cancel_eof_and_no_terminal_do_not_launch_anything(self):
        with patch('sys.stdin.isatty', return_value=True):
            for fn in (lambda _: 'q', Mock(side_effect=EOFError), Mock(side_effect=KeyboardInterrupt)):
                with self.assertRaisesRegex(SystemExit, 'no controllers were launched'):
                    choose_camera(cameras=self.choices(), input_fn=fn, output=lambda _: None)
        with patch('sys.stdin.isatty', return_value=False):
            with self.assertRaisesRegex(RuntimeError, 'interactive terminal'):
                choose_camera(cameras=self.choices())

    def test_path_pointing_to_gemini_preserves_full_fov_enforcement(self):
        with patch('camera_search.device.resolve_camera', return_value=4), \
                patch('camera_search.device.is_configured_camera', return_value=True):
            with self.assertRaisesRegex(ValueError, 'maximum RGB FOV'):
                open_camera('/dev/video4', cv2=Mock(), width=640, height=480)


class LauncherCameraSelectionTests(unittest.TestCase):
    def run_prefix(self, args):
        """Execute real command construction, stopping before any ROS action exists."""
        tree = ast.parse((ROOT/'launch_right_camera_playback.py').read_text())
        body = []
        for node in tree.body:
            if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'camera_env' for t in node.targets):
                break
            if isinstance(node, ast.ImportFrom) and node.module.startswith(('launch', 'launch_ros')):
                continue
            body.append(node)
        namespace = dict(__file__=str(ROOT/'launch_right_camera_playback.py'))
        with patch('sys.argv', ['launch_right_camera_playback.py', *args]), \
                patch.object(Path, 'is_file', return_value=True):
            exec(compile(ast.Module(body=body, type_ignores=[]), 'launcher-prefix', 'exec'), namespace)
        return namespace

    def test_index_or_path_forwarded_in_every_camera_using_mode(self):
        for mode in ([], ['--hardware'], ['--hardwaretest']):
            for source in ('0', '/dev/video2', LOGITECH):
                namespace = self.run_prefix([*mode, '--camera', source])
                command = namespace['camera_command']
                self.assertEqual(command[command.index('--camera')+1], source)
                self.assertTrue(namespace['perception_enabled'])

    def test_picker_runs_before_launch_and_forwards_choice(self):
        for mode in ([], ['--hardware'], ['--hardwaretest']):
            with patch('camera_playback.camera_selection.choose_camera', return_value=LOGITECH) as choose:
                namespace = self.run_prefix([*mode, '--select-camera'])
            choose.assert_called_once_with()
            command = namespace['camera_command']
            self.assertEqual(command[command.index('--camera')+1], LOGITECH)

    def test_listing_exits_without_perception_dependencies_or_picker(self):
        with patch('camera_playback.camera_selection.list_cameras', return_value=[]) as listing, \
                patch('camera_playback.camera_selection.print_cameras') as printer, \
                patch('camera_playback.camera_selection.choose_camera') as choose:
            with self.assertRaises(SystemExit) as caught:
                self.run_prefix(['--list-cameras'])
        self.assertEqual(caught.exception.code, 0)
        listing.assert_called_once_with(); printer.assert_called_once_with([])
        choose.assert_not_called()

    def test_test_mode_remains_camera_free_and_rejects_interactive_picker(self):
        with patch('camera_playback.camera_selection.list_cameras') as listing, \
                patch('camera_playback.camera_selection.choose_camera') as choose:
            namespace = self.run_prefix(['--test', '--camera', '/dev/video0'])
        self.assertFalse(namespace['perception_enabled'])
        listing.assert_not_called(); choose.assert_not_called()
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit): self.run_prefix(['--test', '--select-camera'])

    def test_explicit_camera_and_picker_are_mutually_exclusive(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit): self.run_prefix(['--camera', '0', '--select-camera'])


if __name__ == '__main__':
    unittest.main()
