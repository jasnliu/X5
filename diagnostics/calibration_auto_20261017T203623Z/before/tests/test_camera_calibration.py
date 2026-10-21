"""Offline-only tests: synthetic geometry/images and mocked hardware endpoints."""
import copy
import io
import json
import socket
import subprocess
import time

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from camera_calibration import core
from camera_calibration.core import Board, transform, inverse, distance, solve, metrics, passes, camera_parameters
from camera_calibration.io import Session, load_session
from camera_calibration.observer import QuerySocket, stationary_capture
from safe_zone.encoder import request_frame, decode, FRAME, EFF, encoder_to_joint
from safe_zone.geometry import Model

REAL_POPEN = subprocess.Popen  # Only used by the explicit device-free SDK math test.


@pytest.fixture(scope='session', autouse=True)
def build_strict_decoder_without_devices():
    # Build before individual tests replace Popen; compilation never opens a camera.
    from camera_calibration.jpeg import build_decoder
    build_decoder()


@pytest.fixture(autouse=True)
def no_real_hardware(monkeypatch):
    def denied(*args, **kwargs):
        raise AssertionError('Tests must not open a real socket/device')
    monkeypatch.setattr(socket, 'socket', denied)
    monkeypatch.setattr(subprocess, 'Popen', denied)


@pytest.fixture
def synthetic():
    board = Board()
    K = np.array([[900., 0, 640], [0, 900., 360], [0, 0, 1.]])
    D = np.zeros(5)
    C = transform(Rotation.from_euler('xyz', [.2, -.15, .25]).as_matrix(), [.22, .18, 1.05])
    X = transform(Rotation.from_euler('xyz', [.3, .1, -.25]).as_matrix(), [.03, .02, .16])
    rng = np.random.default_rng(307)
    samples = []
    for i in range(25):
        B = transform(Rotation.from_rotvec(rng.uniform(-.5, .5, 3)).as_matrix(),
                      rng.uniform([-.19, -.12, .65], [-.03, .01, .9]))
        A = C @ B @ inverse(X)
        uv = cv2.projectPoints(board.points, Rotation.from_matrix(B[:3, :3]).as_rotvec(), B[:3, 3], K, D)[0].reshape(-1, 2)
        samples.append(dict(id=i+1, phase='training' if i < 20 else 'validation',
                            T_base_hand=A.tolist(), T_color_board=B.tolist(),
                            corners_px=uv.tolist(), corner_ids=list(range(15))))
    return board, K, D, C, X, samples


def metadata(K):
    return dict(protocol=2, serial='SYNTHETIC-NOT-HARDWARE', depth_stream_enabled=False, color_encoding='mjpeg',
                color=dict(width=1280, height=720, fps=15, format_enum=5, intrinsic_width=1280, intrinsic_height=720,
                           K=K.tolist(), D_opencv_order=[0.]*8, distortion_model=3),
                depth=dict(width=1280, height=800, fps=10),
                T_color_depth_m=transform(t=[.025, 0, 0]).tolist())


def test_unknown_board_mount_and_transform_direction(synthetic):
    board, K, D, C, X, samples = synthetic
    fit = solve(samples[:20], K, D, board)
    assert fit['training_passed']
    assert distance(C, np.array(fit['T_base_color']))[0] < 1e-8
    assert distance(X, np.array(fit['T_hand_board']))[1] < 1e-6
    assert passes(metrics(samples[20:], np.array(fit['T_base_color']), np.array(fit['T_hand_board']), K, D, board.points))
    # An inverted camera transform must not accidentally pass validation.
    assert not passes(metrics(samples[20:], inverse(C), X, K, D, board.points))


def test_noisy_poses_and_separate_holdout(synthetic):
    board, K, D, C, X, samples = synthetic
    rng = np.random.default_rng(2)
    for s in samples[:20]:
        B = np.asarray(s['T_color_board'])
        B[:3, 3] += rng.normal(0, .0003, 3)
        B[:3, :3] = Rotation.from_rotvec(rng.normal(0, .0007, 3)).as_matrix() @ B[:3, :3]
        s['T_color_board'] = B.tolist()
    fit = solve(samples[:20], K, D, board)
    assert fit['training_passed']
    assert distance(C, np.array(fit['T_base_color']))[0] < .003
    assert passes(metrics(samples[20:], np.array(fit['T_base_color']), np.array(fit['T_hand_board']), K, D, board.points))


def test_degenerate_and_too_few_rejected(synthetic):
    board, K, D, _, _, samples = synthetic
    with pytest.raises(ValueError, match='at least 15'):
        solve(samples[:14], K, D, board)
    for s in samples:
        s['T_base_hand'] = np.eye(4).tolist()
    with pytest.raises(ValueError, match='rotation diversity'):
        solve(samples, K, D, board)


def test_single_axis_motion_rejected(synthetic):
    board, K, D, _, _, samples = synthetic
    for i, s in enumerate(samples):
        s['T_base_hand'] = transform(Rotation.from_rotvec([0, 0, i*.06]).as_matrix(), [i*.01, 0, 0]).tolist()
    with pytest.raises(ValueError, match='rotation diversity'):
        solve(samples, K, D, board)


def test_bad_sample_not_silently_dropped(synthetic):
    board, K, D, _, _, samples = synthetic
    samples[4]['T_base_hand'][0][3] += .06
    fit = solve(samples[:20], K, D, board)
    assert not fit['training_passed']
    assert len(fit['training_metrics']['samples']) == 20


def test_camera_parameters_reject_guesses(synthetic):
    _, K, *_ = synthetic
    m = metadata(K)
    k, d = camera_parameters(m)
    assert np.array_equal(k, K) and len(d) == 5
    for model in (1, 2, 5, 99):
        m['color']['distortion_model'] = model
        with pytest.raises(ValueError, match='Unsupported distortion'):
            camera_parameters(m)
    m = metadata(K); m['color']['width'] = 640
    with pytest.raises(ValueError, match='uncropped'):
        camera_parameters(m)
    m = metadata(K); m['color']['intrinsic_height'] = 480
    with pytest.raises(ValueError, match='Intrinsics'):
        camera_parameters(m)


def render_board(board, B, K, D):
    # Rasterized printed board projected into a synthetic 1280x720 camera image.
    if hasattr(board.board, 'draw'):
        pattern = board.board.draw((1200, 800))
    else:
        pattern = board.board.generateImage((1200, 800))
    obj = np.array([[0, 0, 0], [.24, 0, 0], [.24, .16, 0], [0, .16, 0]], dtype=float)
    uv = cv2.projectPoints(obj, Rotation.from_matrix(B[:3, :3]).as_rotvec(), B[:3, 3], K, D)[0].reshape(-1, 2)
    H = cv2.getPerspectiveTransform(np.array([[0, 0], [1199, 0], [1199, 799], [0, 799]], np.float32), uv.astype(np.float32))
    image = cv2.warpPerspective(pattern, H, (1280, 720), borderValue=255)
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def test_rendered_target_detection(synthetic):
    board, K, D, _, _, samples = synthetic
    B = np.asarray(samples[0]['T_color_board'])
    image = render_board(board, B, K, D)
    result = board.detect(image, K, D)
    assert result['corner_count'] >= 10 and result['reprojection_rms_px'] < 1
    d, a = distance(B, np.asarray(result['T_color_board']))
    assert d < .004 and a < 1


def test_end_to_end_pixels_to_heldout_calibration(synthetic):
    board, K, D, C, X, samples = synthetic
    for s in samples:
        B = np.asarray(s['T_color_board'])
        s.update(board.detect(render_board(board, B, K, D), K, D))
    f = solve(samples[:20], K, D, board)
    assert f['training_passed']
    estimated = np.asarray(f['T_base_color'])
    assert distance(C, estimated)[0] < .005
    assert distance(C, estimated)[1] < .5
    assert passes(metrics(samples[20:], estimated, np.asarray(f['T_hand_board']), K, D, board.points))


def test_existing_pdf_pattern_and_manifest_match():
    board = Board()
    spec = json.loads(core.TARGET.read_text())
    assert len(board.points) == 15
    assert spec['legacy_pattern'] is False and spec['square_length_m'] == .04
    # Verify the actual pattern generated with the supplied PDFs, not only a newly drawn board.
    path = core.ROOT / 'tmp/pdfs/charuco/pattern.png'
    if path.exists():
        assert core.digest(path) == spec['pattern_png_sha256']
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        corners, ids, _ = cv2.aruco.detectMarkers(image, board.dictionary)
        assert set(ids.ravel()) == set(range(12))


def test_empty_image_rejected(synthetic):
    board, K, D, *_ = synthetic
    with pytest.raises(ValueError, match='markers'):
        board.detect(np.full((720, 1280, 3), 255, np.uint8), K, D)


def stationary_data():
    frames = []
    for i, t in enumerate(np.linspace(10, 11.5, 16)):
        frames.append(dict(header=dict(received_monotonic=float(t), sequence=i),
                           detection=dict(corner_ids=list(range(10)), corners_px=[[float(k), 3.] for k in range(10)]),
                           image=np.zeros((2, 2, 3), np.uint8)))
    joints = [dict(start=float(t)-.002, end=float(t)+.002, joints=dict.fromkeys(core.JOINTS, 0.))
              for t in np.linspace(9.9, 11.6, 35)]
    return frames, joints


def test_stationary_capture_accepts_bracketed_quiet_data():
    frames, joints = stationary_data()
    f, q, evidence = stationary_capture(frames, joints, 10, 11.5)
    assert f['header']['sequence'] == 8 and q[core.JOINTS[0]] == 0
    assert evidence['nearest_query_skew_s'] < .05


@pytest.mark.parametrize('problem', ['moving_joint', 'moving_board', 'missing_frames', 'stale_encoders', 'unbracketed'])
def test_stationary_capture_rejects_motion_and_stale_data(problem):
    frames, joints = stationary_data()
    if problem == 'moving_joint':
        joints[15]['joints'][core.JOINTS[2]] = .1
    elif problem == 'moving_board':
        frames[5]['detection']['corners_px'][0][0] += 20
    elif problem == 'missing_frames':
        frames = frames[:4] + frames[12:]
    elif problem == 'stale_encoders':
        joints = joints[:10]
    else:
        joints = joints[5:]
    with pytest.raises(ValueError):
        stationary_capture(frames, joints, 10, 11.5)


def test_query_allowlist_and_logging():
    class Sock:
        sent = []
        def send(self, data):
            self.sent.append(data); return len(data)
    raw = Sock(); log = io.StringIO(); q = QuerySocket(raw, log)
    for motor in range(1, 9):
        assert q.send(request_frame(motor)) == 16
    with pytest.raises(RuntimeError, match='Blocked'):
        q.send(b'\0'*16)
    forbidden = FRAME.pack(EFF | (3 << 24) | (0xfd << 8) | 1, 8, b'\0'*8)
    with pytest.raises(RuntimeError, match='Blocked'):
        q.send(forbidden)
    assert len(raw.sent) == len(log.getvalue().splitlines()) == 8
    assert all(json.loads(row)['interface'] == 'can0' for row in log.getvalue().splitlines())


def test_enabled_or_faulted_motors_rejected():
    payload = b'\x80\x00' + b'\0'*6
    for bits in (1 << 22, 1 << 16):
        data = FRAME.pack(EFF | (2 << 24) | (1 << 8) | bits, 8, payload)
        with pytest.raises(RuntimeError):
            decode(data)


def test_observer_is_right_only_and_cleanup_has_no_commands(monkeypatch, tmp_path):
    from camera_calibration import observer as module
    created = []
    class FakeSocket:
        def __init__(self):
            self.sent = []; self.closed = False
        def send(self, frame):
            self.sent.append(frame); return 16
        def close(self):
            self.closed = True
    class FakeObserver:
        def __init__(self, **kwargs):
            self.args = kwargs; self.socket = FakeSocket(); self.raw = self.socket; self.n = 0; created.append(self)
        def sample(self):
            if self.n:
                raise RuntimeError('mock end-of-feedback')
            self.n += 1
            for i in range(1, 9):
                self.socket.send(request_frame(i))
            return dict.fromkeys(core.JOINTS, 0.)
        def close(self):
            self.socket.close()
    monkeypatch.setattr(module, 'SingleArmObserver', FakeObserver)
    o = module.Observer(tmp_path); o.thread.join(timeout=2); o.close()
    assert created[0].args == dict(side='right', bus='can0')
    assert created[0].raw.sent == [request_frame(i) for i in range(1, 9)]
    assert created[0].raw.closed and o.error == 'mock end-of-feedback'


def test_fk_reuses_known_model_and_joint_convention():
    model = Model(core.URDF)
    q = {name: encoder_to_joint('right', i, .1) for i, name in enumerate(core.JOINTS, 1)}
    assert q['openarmx_right_joint5'] == pytest.approx(-.1 - np.radians(8.5))
    T = core.checked_transform(model.transforms(q)[core.HAND])
    q['openarmx_right_finger_joint1'] = .044
    assert np.array_equal(T, model.transforms(q)[core.HAND])  # Sliding fingers do not define mount frame.


def test_session_freeze_holdout_export_and_bad_validation(tmp_path, synthetic):
    board, K, D, C, X, samples = synthetic
    s = Session(tmp_path); s.data['camera'] = metadata(K)
    image = np.zeros((20, 30, 3), np.uint8)
    for sample in samples[:20]:
        s.append(sample, image)
    s.fit(board)
    frozen = copy.deepcopy(s.data['fit'])
    with pytest.raises(ValueError, match='frozen'):
        s.fit(board)
    with pytest.raises(ValueError, match='at least 5'):
        s.validate(board)
    for sample in samples[20:]:
        s.append(sample, image)
    output = s.validate(board)
    assert output['validated'] and not output['motion_authorized'] and not output['depth_mapping_verified']
    assert output['validation']['used_for_fitting'] is False
    assert s.data['fit'] == frozen
    assert np.allclose(output['T_base_depth'], C @ np.array(metadata(K)['T_color_depth_m']), atol=1e-7)
    loaded = load_session(s.path)
    assert loaded.data['fit'] == frozen
    loaded.data['samples'][-1]['T_base_hand'][0][3] += .1
    bad = loaded.validate(board)
    assert not bad['validated']
    assert not json.loads((s.path/'calibration.json').read_text())['validated']


def test_hash_mismatch_rejected(tmp_path):
    s = Session(tmp_path)
    (s.path/'target.json').write_text('{}')
    with pytest.raises(ValueError, match='hash mismatch'):
        load_session(s.path)


def test_session_rejects_wrong_phase_and_persistence_failure(tmp_path, monkeypatch, synthetic):
    board, K, _, _, _, samples = synthetic
    s = Session(tmp_path); s.data['camera'] = metadata(K)
    image = np.zeros((20, 30, 3), np.uint8)
    with pytest.raises(ValueError, match='Fit training first'):
        s.append(samples[-1], image)
    def fail():
        raise OSError('mock disk full')
    monkeypatch.setattr(s, 'save', fail)
    with pytest.raises(OSError, match='disk full'):
        s.append(samples[0], image)
    assert not s.data['samples']


@pytest.mark.parametrize('problem', [None, 'wrong_serial', 'stale', 'truncated', 'backward_timestamp'])
def test_mock_camera_protocol_and_cleanup(tmp_path, monkeypatch, problem):
    from camera_calibration import io as module
    K = np.array([[900., 0, 640], [0, 900., 360], [0, 0, 1.]])
    meta = metadata(K)
    meta['serial'] = 'WRONG' if problem == 'wrong_serial' else module.SERIAL
    ok, encoded = cv2.imencode('.jpg', np.full((720, 1280, 3), 255, np.uint8))
    assert ok
    payload = encoded.tobytes()
    now = time.monotonic()
    header = dict(n=len(payload), sequence=1, device_timestamp_us=1000,
                  system_timestamp_us=1000, received_monotonic=now - (2 if problem == 'stale' else 0))
    raw = json.dumps(meta).encode() + b'\n' + json.dumps(header).encode() + b'\n'
    raw += payload[:-100] if problem == 'truncated' else payload
    if problem == 'backward_timestamp':
        header['sequence'] = 2
        raw += json.dumps(header).encode() + b'\n' + payload
    class FakeProcess:
        def __init__(self, *args, **kwargs):
            self.stdout = io.BytesIO(raw); self.terminated = False
            assert args[0][1] == module.SERIAL
        def poll(self):
            return 0 if self.terminated else None
        def terminate(self):
            self.terminated = True
        def wait(self, **kwargs):
            return 0
    monkeypatch.setattr(module, 'build_bridge', lambda: '/not-a-real-device-bridge')
    monkeypatch.setattr(module.subprocess, 'Popen', FakeProcess)
    camera = module.Camera(tmp_path, fps=15, color_format='mjpeg'); camera.thread.join(timeout=2); camera.close()
    assert camera.process.terminated
    if problem is None:
        assert camera.latest[1].shape == (720, 1280, 3)
        assert 'stream ended' in camera.error  # Fixture ends after one frame.
    else:
        expected = {'wrong_serial': 'identity', 'stale': 'Stale', 'truncated': 'Truncated',
                    'backward_timestamp': 'timestamp'}[problem]
        assert expected in camera.error


def test_preview_never_constructs_an_observer(monkeypatch):
    # GUI logic driven with fake widgets; no display, camera process or CAN needed.
    from camera_calibration.app import App
    from camera_calibration import app as module
    class Value:
        def set(self, *args):
            pass
    class FakeSession:
        data = dict(preview_only=True, camera=None)
        def save(self):
            pass
    class FakeCamera:
        error = None; metadata = metadata_for_test = None; latest = None
    camera = FakeCamera(); camera.metadata = metadata(np.array([[900., 0, 640], [0, 900., 360], [0, 0, 1.]]))
    a = App.__new__(App); a.failure = None; a.camera = camera; a.parameters = None
    a.session = FakeSession(); a.status = Value(); a.observer = None; a.started = time.monotonic()
    def denied(*args, **kwargs):
        raise AssertionError('Preview must not open CAN')
    monkeypatch.setattr(module, 'Observer', denied)
    a._tick()
    assert a.observer is None and a.parameters is not None


def test_actual_sdk_extrinsic_math_without_devices(tmp_path, monkeypatch):
    from camera_calibration.io import SDK
    # Allow only this test's compiler/pure-math executable. No camera binary is run.
    monkeypatch.setattr(subprocess, 'Popen', REAL_POPEN)
    source = core.ROOT / 'tests/camera_calibration_sdk_math.cpp'
    binary = tmp_path / 'sdk_math_test'
    subprocess.run(['g++', '-std=c++17', '-O2', str(source), '-I'+str(SDK/'include'),
                    '-L'+str(SDK/'lib'), '-Wl,-rpath,'+str(SDK/'lib'), '-lOrbbecSDK', '-pthread',
                    '-o', str(binary)], check=True, capture_output=True, text=True, timeout=60)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True, timeout=10)
    cases = json.loads(result.stdout)
    assert len(cases) == 3
    for case in cases:
        T = core.checked_transform(case['matrix'])
        for point in case['points']:
            predicted = (T @ np.r_[point['input_m'], 1])[:3]
            assert np.allclose(predicted, point['output_m'], atol=2e-7, rtol=0)
    assert np.allclose(cases[0]['matrix'], np.eye(4), atol=1e-9)
    assert np.allclose(np.array(cases[1]['matrix'])[:3, 3], [.01, -.02, .03], atol=1e-9)
    assert np.allclose(np.array(cases[2]['matrix'])[:3, 3], [-.02425, .00175, -.0035], atol=2e-7)


def test_bridge_failure_detail_reaches_camera_error(tmp_path, monkeypatch):
    from camera_calibration import io as module
    class FailedProcess:
        def __init__(self, *args, **kwargs):
            kwargs['stderr'].write(b'SDK could not transform the depth/color basis\n')
            self.stdout = io.BytesIO(b'{"protocol":1,')  # Old partial-header failure.
        def poll(self):
            return 1
    monkeypatch.setattr(module, 'build_bridge', lambda: '/not-a-real-device-bridge')
    monkeypatch.setattr(module.subprocess, 'Popen', FailedProcess)
    camera = module.Camera(tmp_path, fps=15, color_format='mjpeg'); camera.thread.join(timeout=2); camera.close()
    assert 'SDK detail: SDK could not transform the depth/color basis' in camera.error
    assert 'disconnected' not in camera.error


def test_missing_stderr_does_not_hide_original_failure(tmp_path):
    from camera_calibration.io import camera_failure_message
    assert camera_failure_message(ValueError('example failure'), tmp_path) == 'example failure'


def test_sdk_internal_drop_diagnostics_are_retained_and_bounded(tmp_path):
    from camera_calibration.io import camera_failure_message
    logs = tmp_path / 'orbbec_sdk_logs'; logs.mkdir()
    (logs/'OrbbecSDK.log.txt').write_text('old event\n'*1000 + 'JPEG frame boundary verification failed\n')
    reason = camera_failure_message(RuntimeError('stream ended'), tmp_path)
    assert reason.startswith('stream ended')
    assert 'Recent SDK log:' in reason
    assert 'JPEG frame boundary verification failed' in reason
    assert len(reason) < 1600


def test_strict_jpeg_matches_clean_decode():
    from camera_calibration.jpeg import JpegDecoder
    rng = np.random.default_rng(74)
    source = rng.integers(0, 256, (48, 64, 3), dtype=np.uint8)
    ok, jpeg = cv2.imencode('.jpg', source)
    assert ok
    expected = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)
    actual = JpegDecoder().decode(jpeg.tobytes(), width=64, height=48)
    assert np.array_equal(actual, expected)


def test_native_yuyv_conversion_is_full_size_and_correct_color():
    from camera_calibration.io import decode_rgb_frame
    raw = np.empty((720, 1280, 2), dtype=np.uint8)
    raw[:, :, 0] = 16; raw[:, :, 1] = 128  # Black, neutral chroma.
    raw[0, :2, 0] = 235  # White pixel pair.
    raw[0, 2:4] = [[81, 90], [81, 240]]  # Red pixel pair: Y0/U/Y1/V.
    image = decode_rgb_frame(raw.tobytes(), 'yuyv')
    assert image.shape == (720, 1280, 3) and image.dtype == np.uint8
    assert image[10, 10].tolist() == [0, 0, 0]
    assert image[0, 0].tolist() == [255, 255, 255]
    assert image[0, 2, 0] <= 1 and image[0, 2, 1] <= 1 and image[0, 2, 2] >= 254


@pytest.mark.parametrize('size', [0, 100, 1843199, 1843201])
def test_partial_or_oversized_yuyv_is_never_repaired(size):
    from camera_calibration.io import decode_rgb_frame
    with pytest.raises(RuntimeError, match='Incomplete YUYV frame'):
        decode_rgb_frame(b'\0' * size, 'yuyv')


@pytest.mark.parametrize('problem', [None, 'encoding', 'format_enum', 'fps', 'partial'])
def test_raw_camera_protocol_and_profile_mismatch(tmp_path, monkeypatch, problem):
    from camera_calibration import io as module
    meta = metadata(np.array([[900., 0, 640], [0, 900., 360], [0, 0, 1.]]))
    meta.update(serial=module.SERIAL, color_encoding='yuyv')
    meta['color']['format_enum'] = 0
    if problem == 'encoding': meta['color_encoding'] = 'mjpeg'
    if problem == 'format_enum': meta['color']['format_enum'] = 5
    if problem == 'fps': meta['color']['fps'] = 10
    payload = bytes([16,128,16,128]) * (1280*720//2)
    if problem == 'partial': payload = payload[:-2]
    header = dict(n=len(payload), sequence=1, device_timestamp_us=1000,
                  received_monotonic=time.monotonic())
    stream = json.dumps(meta).encode()+b'\n'+json.dumps(header).encode()+b'\n'+payload
    class FakeProcess:
        def __init__(self, *args, **kwargs):
            assert args[0][-2:] == ['15', 'yuyv']
            self.stdout = io.BytesIO(stream)
        def poll(self):
            return 0
    def forbidden_jpeg_decoder():
        raise AssertionError('Raw color must not enter the JPEG decoder')
    monkeypatch.setattr(module, 'build_bridge', lambda: '/not-a-real-bridge')
    monkeypatch.setattr(module.subprocess, 'Popen', FakeProcess)
    monkeypatch.setattr(module, 'JpegDecoder', forbidden_jpeg_decoder)
    camera = module.Camera(tmp_path); camera.thread.join(timeout=2); camera.close()
    if problem is None:
        assert camera.latest[1].shape == (720,1280,3)
        assert not np.any(camera.latest[1])
        assert camera.latest[0]['frame_decode_policy'] == 'require_full_yuyv_frame'
        assert camera.latest[0]['color_encoding'] == 'yuyv'
        assert camera.latest[0]['frames_checked_total'] == 1
        assert 'jpeg_checked_total' not in camera.latest[0]
        assert 'stream ended' in camera.error  # Test stream intentionally ends.
    else:
        assert camera.latest is None
        assert ('Incomplete YUYV frame' if problem == 'partial' else 'mismatch') in camera.error


@pytest.mark.parametrize('damage', ['no_eoi', 'short_scan_with_eoi', 'not_jpeg', 'empty'])
def test_strict_jpeg_rejects_partial_images_without_stderr(damage, capfd):
    from camera_calibration.jpeg import JpegDecoder, CorruptJpegError
    rng = np.random.default_rng(77)
    source = rng.integers(0, 256, (48, 64, 3), dtype=np.uint8)
    _, encoded = cv2.imencode('.jpg', source)
    jpeg = encoded.tobytes()
    if damage == 'no_eoi':
        jpeg = jpeg[:-2]
    elif damage == 'short_scan_with_eoi':
        jpeg = jpeg[:-300] + b'\xff\xd9'
    elif damage == 'not_jpeg':
        jpeg = b'not a JPEG' * 100
    else:
        jpeg = b''
    decoder = JpegDecoder()
    for _ in range(20):
        with pytest.raises(CorruptJpegError):
            decoder.decode(jpeg, width=64, height=48)
    assert capfd.readouterr().err == ''


def test_strict_jpeg_dimensions_cannot_overrun_output(capfd):
    from camera_calibration.jpeg import JpegDecoder
    _, encoded = cv2.imencode('.jpg', np.zeros((48, 64, 3), dtype=np.uint8))
    with pytest.raises(RuntimeError, match='dimensions'):
        JpegDecoder().decode(encoded.tobytes(), width=32, height=24)
    assert capfd.readouterr().err == ''


def test_camera_skips_bad_jpeg_and_resumes_clean_frames(tmp_path, monkeypatch, capfd):
    from camera_calibration import io as module
    K = np.array([[900., 0, 640], [0, 900., 360], [0, 0, 1.]])
    meta = metadata(K); meta['serial'] = module.SERIAL
    rng = np.random.default_rng(76)
    _, encoded = cv2.imencode('.jpg', rng.integers(0, 256, (720, 1280, 3), dtype=np.uint8))
    valid = encoded.tobytes(); bad = valid[:-1000] + b'\xff\xd9'
    stream = json.dumps(meta).encode() + b'\n'
    for i, payload in enumerate([valid, bad, valid], 1):
        h = dict(n=len(payload), sequence=i, device_timestamp_us=i*1000,
                 system_timestamp_us=i*1000, received_monotonic=time.monotonic())
        stream += json.dumps(h).encode() + b'\n' + payload
    class FakeProcess:
        def __init__(self, *args, **kwargs):
            self.stdout = io.BytesIO(stream)
        def poll(self):
            return 0
    monkeypatch.setattr(module, 'build_bridge', lambda: '/not-a-real-bridge')
    monkeypatch.setattr(module.subprocess, 'Popen', FakeProcess)
    camera = module.Camera(tmp_path, fps=15, color_format='mjpeg'); camera.thread.join(timeout=3); camera.close()
    assert camera.latest[0]['sequence'] == 3
    assert camera.latest[0]['jpeg_rejected_total'] == 1
    assert camera.latest[0]['frame_decode_policy'] == 'reject_decoder_warnings'
    assert camera.jpeg_report['received'] == 3 and camera.jpeg_report['accepted'] == 2
    saved = json.loads((tmp_path/'camera_decode_report.json').read_text())
    assert saved['rejected'] == 1 and saved['consecutive_rejected'] == 0
    assert saved['rejection_examples'][0]['sequence'] == 2
    assert capfd.readouterr().err == ''


def test_persistent_jpeg_corruption_stops_instead_of_publishing(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from camera_calibration import io as module
    from camera_calibration.jpeg import CorruptJpegError
    meta = metadata(np.array([[900., 0, 640], [0, 900., 360], [0, 0, 1.]]))
    meta['serial'] = module.SERIAL
    data = b'bad frame'*100
    h = dict(n=len(data), sequence=1, device_timestamp_us=1000, received_monotonic=103.1)
    stream = json.dumps(meta).encode()+b'\n'+json.dumps(h).encode()+b'\n'+data
    class FakeProcess:
        def __init__(self, *args, **kwargs):
            self.stdout=io.BytesIO(stream)
        def poll(self):
            return 0
    class BadDecoder:
        def decode(self, data):
            raise CorruptJpegError('synthetic corrupt JPEG')
    times = iter([100., 103.1, 103.1, 103.1])
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: next(times)))
    monkeypatch.setattr(module, 'JpegDecoder', BadDecoder)
    monkeypatch.setattr(module, 'build_bridge', lambda: '/not-a-real-bridge')
    monkeypatch.setattr(module.subprocess, 'Popen', FakeProcess)
    camera = module.Camera(tmp_path, fps=15, color_format='mjpeg'); camera.thread.join(timeout=2); camera.close()
    assert camera.latest is None
    assert 'No fresh usable RGB frames' in camera.error
    assert 'synthetic corrupt JPEG' in camera.error


def test_historical_jpeg_warnings_are_not_reported_as_cause_of_a_stall():
    from camera_calibration.io import Camera
    camera = Camera.__new__(Camera)
    camera.jpeg_report = dict(last_received_monotonic=100., last_accepted_monotonic=100.,
                              rejected=5, consecutive_rejected=0, last_rejection='old JPEG warning')
    reason = camera.unusable_frame_message(now=103.2)
    assert 'RGB frame delivery stalled' in reason
    assert '3.2s' in reason
    assert 'Earlier JPEGs skipped: 5' in reason
    assert 'no current JPEG rejection streak' in reason
    assert 'old JPEG warning' not in reason


def test_current_jpeg_rejection_streak_is_distinct_from_no_delivery():
    from camera_calibration.io import Camera
    camera = Camera.__new__(Camera)
    camera.jpeg_report = dict(last_received_monotonic=103., last_accepted_monotonic=100.,
                              rejected=5, consecutive_rejected=3, last_rejection='current JPEG warning')
    reason = camera.unusable_frame_message(now=103.2)
    assert 'consecutive damaged JPEGs rejected: 3' in reason
    assert 'current JPEG warning' in reason
    assert 'delivery stalled' not in reason


@pytest.fixture
def fake_gui_with_camera(monkeypatch):
    from types import SimpleNamespace
    from camera_calibration import app as module
    class Widget:
        def __init__(self):
            self.value = ''; self.options = {}
        def set(self, value):
            self.value = value
        def get(self):
            return self.value
        def configure(self, **kwargs):
            self.options.update(kwargs)
    class FakeCamera:
        metadata = None; error = None; closed_reason = None
        jpeg_report = dict(rejected=5)
        latest = (dict(sequence=1, received_monotonic=98.5), np.zeros((48, 64, 3), dtype=np.uint8))
        def unusable_frame_message(self, now):
            return 'RGB frame delivery stalled'
        def close(self, reason=None):
            self.closed_reason = reason
    def board_missing(*args):
        raise ValueError('Show at least four board markers')
    app = module.App.__new__(module.App)
    app.camera = FakeCamera(); app.observer = None
    app.parameters = (np.eye(3), np.zeros(5)); app.failure = None; app.camera_waiting = False
    app.session = SimpleNamespace(data=dict(preview_only=True, fit=None, validation=None))
    app.pending = dict(frames=['must not survive an outage']); app.sequence = 1; app.started = 90.
    app.board = SimpleNamespace(detect=board_missing)
    for name in ('status', 'quality', 'canvas', 'train_button', 'val_button', 'fit_button',
                 'export_button', 'undo_button'):
        setattr(app, name, Widget())
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: 100.))
    monkeypatch.setattr(module.ImageTk, 'PhotoImage', lambda image: object())
    return app


def test_short_frame_gap_pauses_capture_without_killing_session(fake_gui_with_camera):
    app = fake_gui_with_camera
    app.sequence = 0  # A not-yet-displayed image is still forbidden if stale.
    def must_not_detect_stale_image(*args):
        raise AssertionError('A stale image must not reach the board detector')
    app.board.detect = must_not_detect_stale_image
    app._tick()
    assert app.failure is None and app.camera_waiting
    assert app.pending is None
    assert app.camera.closed_reason is None
    assert app.train_button.options['state'] == 'disabled'
    assert app.val_button.options['state'] == 'disabled'
    assert 'Capture canceled' in app.status.get()
    assert 'paused' in app.quality.get()
    assert app.sequence == 0


@pytest.mark.parametrize('phase', ['preview', 'training', 'validation', 'finalized'])
def test_clean_frame_recovers_only_appropriate_capture_buttons(fake_gui_with_camera, phase):
    from types import SimpleNamespace
    app = fake_gui_with_camera
    if phase != 'preview':
        app.observer = SimpleNamespace(error=None)
        app.session.data['preview_only'] = False
    if phase in ('validation', 'finalized'):
        app.session.data['fit'] = dict(training_passed=True)
    if phase == 'finalized':
        app.session.data['validation'] = dict(passed=True)
    app._tick()  # Temporary outage cancels a pending pose.
    app.camera.latest = (dict(sequence=2, received_monotonic=99.95), np.zeros((48, 64, 3), dtype=np.uint8))
    app._tick()
    assert not app.camera_waiting and app.failure is None
    assert app.pending is None and app.sequence == 2
    assert app.train_button.options['state'] == ('normal' if phase == 'training' else 'disabled')
    assert app.val_button.options['state'] == ('normal' if phase == 'validation' else 'disabled')
    assert 'restored' in app.status.get()
    assert 'Damaged JPEGs skipped: 5' in app.quality.get()


def test_persistent_frame_gap_still_stops_and_records_reason(fake_gui_with_camera):
    app = fake_gui_with_camera
    app.camera.latest[0]['received_monotonic'] = 96.9
    app._tick()
    assert app.failure == 'RGB frame delivery stalled'
    assert app.camera.closed_reason == app.failure
    assert app.pending is None
    assert 'CAPTURE STOPPED' in app.status.get()


def test_capture_hotkey_rejects_stale_frame_even_before_gui_tick(fake_gui_with_camera):
    from types import SimpleNamespace
    app = fake_gui_with_camera
    app.observer = SimpleNamespace(error=None)
    app.pending = None
    with pytest.raises(ValueError, match='Wait for fresh RGB frames'):
        app.capture('training')
    assert app.pending is None


@pytest.mark.parametrize('command', ['preview', 'capture'])
def test_default_cli_uses_uncompressed_full_fov_without_opening_devices(monkeypatch, command):
    from camera_calibration import app as app_module, cli
    seen = []
    class FakeApp:
        def __init__(self, args):
            seen.append(args)
        def run(self):
            pass
    monkeypatch.setattr(app_module, 'App', FakeApp)
    assert cli.main([command]) == 0
    assert seen[0].color_format == 'yuyv' and seen[0].fps == 15
