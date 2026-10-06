"""Offline camera selection tests; never open a physical camera or arm."""
import argparse
import ast
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from camera_search.device import (
    DEFAULT_CAMERA, add_camera_argument, camera_argument, open_camera, resolve_camera,
)
from camera_search.camera import _parser


ROOT = Path(__file__).resolve().parents[1]


class CameraDeviceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.sysfs = self.root/'video4linux'
        self.sysfs.mkdir()
        self.config = self.root/'camera.json'
        self.config.write_text(json.dumps(dict(name='Test Orbbec RGB', vendor_id='2bc5',
                                              product_id='0670', serial='camera-A', usb_interface='04')))

    def node(self, number, interface='04', index='0', serial='camera-A', vendor='2bc5'):
        usb = self.root/f'usb-{serial}-{vendor}'
        device = usb/f'interface-{interface}'
        device.mkdir(parents=True, exist_ok=True)
        for name, value in [('idVendor', vendor), ('idProduct', '0670'), ('serial', serial)]:
            (usb/name).write_text(value+'\n')
        (device/'bInterfaceNumber').write_text(interface+'\n')
        entry = self.sysfs/f'video{number}'
        entry.mkdir()
        (entry/'device').symlink_to(device)
        (entry/'index').write_text(index+'\n')

    def resolve(self, source=DEFAULT_CAMERA):
        return resolve_camera(source, sysfs=self.sysfs, config_path=self.config)

    def test_selects_rgb_not_depth_ir_metadata_or_another_camera(self):
        self.node(0, interface='00')
        self.node(2, interface='02')
        self.node(4)
        self.node(5, index='1')
        self.node(6, serial='another-Orbbec')
        self.node(8, vendor='1234')
        self.assertEqual(self.resolve(), 4)

    def test_selection_survives_video_number_change(self):
        self.node(14)
        self.assertEqual(self.resolve(), 14)

    def test_missing_camera_does_not_fall_back(self):
        self.node(0, serial='not-the-selected-camera')
        with self.assertRaisesRegex(RuntimeError, 'No fallback camera'):
            self.resolve()

    def test_metadata_only_and_ambiguous_capture_are_rejected(self):
        self.node(5, index='1')
        with self.assertRaises(RuntimeError): self.resolve()
        self.node(4)
        self.node(8)
        with self.assertRaises(RuntimeError): self.resolve()

    def test_numeric_override_does_not_enumerate_default_camera(self):
        self.config.unlink()
        self.assertEqual(self.resolve('0'), 0)
        self.assertEqual(self.resolve(7), 7)

    def test_invalid_camera_argument(self):
        for value in ('-1', 'garbage', '1.5'):
            with self.assertRaises(argparse.ArgumentTypeError): camera_argument(value)

    def test_all_camera_parsers_share_default_and_preserve_override(self):
        parsers = [_parser(ROOT)]
        # Load only parse_args(), never execute either ROS launcher.
        for name in ('launch_right_camera_cartesian.py', 'launch_right_camera_playback.py'):
            tree = ast.parse((ROOT/name).read_text())
            fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'parse_args')
            namespace = dict(argparse=argparse, Path=Path, ROOT=ROOT,
                             DEFAULT_TONOR_SOURCE='test', DEFAULT_ESP_PORT='test',
                             add_camera_argument=add_camera_argument)
            exec(compile(ast.Module(body=[fn], type_ignores=[]), name, 'exec'), namespace)
            with patch('sys.argv', [name]):
                self.assertEqual(namespace['parse_args']().camera, DEFAULT_CAMERA)
            with patch('sys.argv', [name, '--camera', '2']):
                self.assertEqual(namespace['parse_args']().camera, 2)
        for parser in parsers:
            self.assertEqual(parser.parse_args(['--socket', '/tmp/test']).camera, DEFAULT_CAMERA)
            self.assertEqual(parser.parse_args(['--socket', '/tmp/test', '--camera', '1']).camera, 1)

    def fake_cv2(self):
        cap = Mock()
        cap.isOpened.return_value = cap.set.return_value = True
        return SimpleNamespace(VideoCapture=Mock(return_value=cap),
                               CAP_V4L2=200, CAP_PROP_FOURCC=6, CAP_PROP_FRAME_WIDTH=3,
                               CAP_PROP_FRAME_HEIGHT=4, CAP_PROP_FPS=5,
                               VideoWriter_fourcc=Mock(return_value=1196444237)), cap

    def test_default_opens_resolved_rgb_with_mjpeg_and_requested_dimensions(self):
        cv2, cap = self.fake_cv2()
        with patch('camera_search.device.resolve_camera', return_value=14):
            actual, index = open_camera(cv2=cv2, width=1280, height=720)
        self.assertIs(actual, cap)
        self.assertEqual(index, 14)
        cv2.VideoCapture.assert_called_once_with(14, cv2.CAP_V4L2)
        cap.set.assert_any_call(cv2.CAP_PROP_FOURCC, 1196444237)
        cap.set.assert_any_call(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set.assert_any_call(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set.assert_any_call(cv2.CAP_PROP_FPS, 30)

    def test_numeric_override_retains_original_backend_and_format_behavior(self):
        cv2, cap = self.fake_cv2()
        open_camera(2, cv2=cv2)
        cv2.VideoCapture.assert_called_once_with(2)
        cv2.VideoWriter_fourcc.assert_not_called()

    def test_failed_open_or_configuration_releases_camera(self):
        cv2, cap = self.fake_cv2()
        cap.isOpened.return_value = False
        with self.assertRaises(RuntimeError): open_camera(2, cv2=cv2)
        cap.release.assert_called_once()
        cv2, cap = self.fake_cv2()
        cap.set.return_value = False
        with patch('camera_search.device.resolve_camera', return_value=4):
            with self.assertRaises(RuntimeError): open_camera(cv2=cv2)
        cap.release.assert_called_once()

    def test_standalone_video_uses_shared_default_and_reports_open_errors(self):
        from smooth_playback.hardware import Video
        with patch('camera_search.device.open_camera', side_effect=RuntimeError('Missing Orbbec')):
            video = Video(self.root, lambda: 'CAMERA TEST ONLY')
            video.start()
            video.close()
        self.assertEqual(video.error, 'Missing Orbbec')
        self.assertTrue(video.ready.is_set())


if __name__ == '__main__':
    unittest.main()
