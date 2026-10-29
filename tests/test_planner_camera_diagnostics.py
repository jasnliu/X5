"""No devices: preserve the SDK cause, identify holders without taking ownership."""
import io
import json
import os
from pathlib import Path
import socket
import threading
import time

import pytest

from drum_planner import camera_diagnostics as diagnostics, depth


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    def denied(*args, **kwargs):
        pytest.fail('No physical devices or subprocesses in these tests')
    monkeypatch.setattr(socket, 'socket', denied)
    monkeypatch.setattr(depth.subprocess, 'Popen', denied)
    monkeypatch.setattr(depth, 'build_depth', denied)


@pytest.fixture
def system(tmp_path):
    sys_root, proc_root, dev_root = [tmp_path / s for s in ('sys', 'proc', 'dev')]
    usb = sys_root / 'bus/usb/devices/7-3'
    usb.mkdir(parents=True)
    for key, value in dict(serial='TEST-SERIAL', busnum='7', devnum='42').items():
        (usb / key).write_text(value)
    video = sys_root / 'class/video4linux/video9'
    video.mkdir(parents=True)
    (video / 'device').symlink_to(usb / '7-3:1.2', target_is_directory=True)
    proc_root.mkdir()
    def process(pid, node, exe='/test/DT1/rgbd_capture'):
        p = proc_root / str(pid)
        (p / 'fd').mkdir(parents=True)
        (p / 'fd/12').symlink_to(dev_root / node)
        (p / 'exe').symlink_to(exe)
        (p / 'cmdline').write_text('SECRET-ARGUMENT')
        return p
    return dict(sys_root=sys_root, proc_root=proc_root, dev_root=dev_root), process


def test_resolves_serial_not_historical_usb_or_video_number(system):
    roots, process = system
    process(12001, 'bus/usb/007/042')
    process(12002, 'video9', '/test/video_app')
    process(12003, 'bus/usb/005/010', '/test/unrelated_camera')
    holders = diagnostics.camera_holders('TEST-SERIAL', **roots)
    assert holders == ['PID 12001: /test/DT1/rgbd_capture', 'PID 12002: /test/video_app']
    assert 'SECRET' not in str(holders)
    assert diagnostics.camera_holders('OTHER-SERIAL', **roots) == []


def test_ignores_self_and_bridge_and_tolerates_vanished_descriptors(system):
    roots, process = system
    process(os.getpid(), 'video9')
    p = process(12004, 'video9')
    (p / 'fd/12').unlink()
    process(12005, 'video9')
    assert diagnostics.camera_holders('TEST-SERIAL', ignored_pids=(12005,), **roots) == []


def test_missing_sysfs_proc_and_executable_are_best_effort(system):
    roots, process = system
    p = process(12006, 'video9')
    (p / 'exe').unlink()
    (p / 'comm').write_text('video-reader\n')
    assert diagnostics.camera_holders('TEST-SERIAL', **roots) == ['PID 12006: video-reader']
    roots['proc_root'] = roots['proc_root'] / 'missing'
    assert diagnostics.camera_holders('TEST-SERIAL', **roots) == []
    roots['sys_root'] = roots['sys_root'] / 'missing'
    assert diagnostics.camera_holders('TEST-SERIAL', **roots) == []


def test_busy_exposes_owner_sdk_error_and_recovery(tmp_path, monkeypatch):
    (tmp_path / 'depth_stderr.log').write_text('Orbbec: uvc_open failed: [Path: 5-1-10.0, Return Code: -6]\n')
    monkeypatch.setattr(diagnostics, 'camera_holders', lambda *a, **k: ['PID 123: /dt1/rgbd_capture'])
    message = diagnostics.failure_message('Depth stream ended', tmp_path, 'SERIAL')
    for text in ('is busy', 'PID 123', '/dt1/rgbd_capture', 'NEW scene',
                 'No other process was stopped', 'Return Code: -6', 'Depth stream ended',
                 str(tmp_path / 'depth_stderr.log')):
        assert text in message


def test_busy_without_visible_owner_does_not_invent_application(tmp_path, monkeypatch):
    (tmp_path / 'depth_stderr.log').write_text('uvc_open failed: [Return Code: -6]')
    monkeypatch.setattr(diagnostics, 'camera_holders', lambda *a, **k: [])
    message = diagnostics.failure_message('EOF', tmp_path, 'SERIAL')
    assert 'No other owner was visible' in message and 'PID' not in message


@pytest.mark.parametrize('detail,expected', [
    ('uvc_open failed: [Return Code: -3]', 'USB access denied'),
    ('No depth frames for five seconds', 'No depth frames for five seconds'),
    ('Calibrated Gemini 2 not found or busy', 'Calibrated Gemini 2 not found or busy'),
    ('uvc_open failed: [Return Code: -60]', 'Return Code: -60'),
    ('', 'Wrong camera identity'),
])
def test_other_causes_preserved_not_mislabeled_busy(tmp_path, monkeypatch, detail, expected):
    (tmp_path / 'depth_stderr.log').write_text(detail)
    monkeypatch.setattr(diagnostics, 'camera_holders', lambda *a, **k: pytest.fail('No busy error'))
    message = diagnostics.failure_message('Wrong camera identity', tmp_path, 'SERIAL')
    assert expected in message and 'Wrong camera identity' in message
    assert 'is busy (UVC error -6)' not in message


def test_missing_and_large_non_utf8_log_do_not_hide_original_error(tmp_path):
    assert 'original failure' in diagnostics.failure_message('original failure', tmp_path, 'S')
    (tmp_path / 'depth_stderr.log').write_bytes(b'x' * 10000 + b'\xff\x1bproblem')
    message = diagnostics.failure_message('original failure', tmp_path, 'S')
    assert 'original failure' in message and 'problem' in message
    assert len(message) < 2200 and '\x1b' not in message


def bridge(monkeypatch, tmp_path, output=b'', stderr=b''):
    class Process:
        pid = 123456
        stdout = io.BytesIO(output)
        def poll(self): return 1 if stderr else 0
    def popen(command, *, cwd, stdout, stderr):
        assert command[:2] == ['/fake/depth_capture', 'SERIAL']
        stderr.write(error_log)
        stderr.flush()
        return Process()
    error_log = stderr
    monkeypatch.setattr(depth, 'build_depth', lambda: Path('/fake/depth_capture'))
    monkeypatch.setattr(depth.subprocess, 'Popen', popen)
    monkeypatch.setattr(diagnostics, 'camera_holders', lambda *a, **k: ['PID 7: /dt1/rgbd_capture'])
    camera = depth.DepthCamera(tmp_path, {'camera': {'serial': 'SERIAL'}}, single_frame=True)
    camera.thread.join(timeout=3)
    assert not camera.thread.is_alive()
    camera.close()
    return camera


def test_real_depth_reader_surfaces_busy_bridge_exit(tmp_path, monkeypatch):
    camera = bridge(monkeypatch, tmp_path, stderr=b'Orbbec: uvc_open failed: [Return Code: -6]\n')
    assert camera.count == 0 and camera.meta is None
    assert 'is busy' in camera.error and '/dt1/rgbd_capture' in camera.error


def test_reader_retains_malformed_header_error(tmp_path, monkeypatch):
    camera = bridge(monkeypatch, tmp_path, output=b'{"protocol":1}')
    assert 'header incomplete' in camera.error and camera.count == 0


def test_successful_capture_unchanged(tmp_path, monkeypatch):
    monkeypatch.setattr(depth, 'check_metadata', lambda meta, cal: None)
    header = dict(n=1280*800*2, value_scale_m=.001, sequence=1,
                  device_timestamp_us=100, received_monotonic=time.monotonic())
    output = b'{}\n' + json.dumps(header).encode() + b'\n' + b'\x01\x00' * (1280*800)
    camera = bridge(monkeypatch, tmp_path, output=output)
    assert camera.error is None and camera.count == 1
    assert camera.latest[1].shape == (800, 1280)
