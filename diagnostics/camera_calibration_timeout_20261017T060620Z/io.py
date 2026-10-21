"""Durable sessions and a private SDK bridge. Importing this opens no devices."""
import datetime as dt
import json
import os
from pathlib import Path
import subprocess
import threading
import time
import uuid

import cv2

from .core import ROOT, TARGET, URDF, HAND, Board, camera_parameters, checked_transform, digest, metrics, passes, solve, MIN_VALIDATION
from .jpeg import JpegDecoder, CorruptJpegError

SDK = Path('/home/jason/Applications/Gemini2/sdk')
BUILD = ROOT / 'camera_calibration/build'
SERIAL = 'AY6R463004K'


def camera_failure_message(error, folder):
    """Surface the bridge's actual failure, not a generic disconnected-camera claim."""
    message = str(error)
    try:
        with (Path(folder) / 'camera_stderr.log').open('rb') as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - 4096))
            lines = f.read().decode('utf-8', errors='replace').splitlines()
        detail = ' | '.join(line.strip() for line in lines[-4:] if line.strip())[-1500:]
        if detail:
            message += '\nSDK detail: ' + detail
    except OSError:
        pass
    return message


def atomic_json(path, data):
    path = Path(path)
    temp = path.with_name(path.name + '.tmp')
    with temp.open('w') as f:
        json.dump(data, f, indent=2, allow_nan=False)
        f.write('\n'); f.flush(); os.fsync(f.fileno())
    os.replace(temp, path)


def build_bridge():
    source = ROOT / 'camera_calibration/orbbec_capture.cpp'
    binary = BUILD / 'orbbec_capture'
    signature = digest(source) + str(SDK.resolve())
    stamp = BUILD / 'signature.txt'
    if binary.is_file() and stamp.is_file() and stamp.read_text() == signature:
        return binary
    if not (SDK / 'include/libobsensor/ObSensor.hpp').is_file():
        raise RuntimeError(f'Orbbec SDK headers not found at {SDK}; no automatic installation attempted')
    BUILD.mkdir(parents=True, exist_ok=True)
    temp = binary.with_name(f'.bridge-{os.getpid()}-{uuid.uuid4().hex}')
    try:
        result = subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', str(source),
                                 '-I' + str(SDK / 'include'), '-L' + str(SDK / 'lib'),
                                 '-Wl,-rpath,' + str(SDK / 'lib'), '-lOrbbecSDK', '-pthread', '-o', str(temp)],
                                text=True, capture_output=True, timeout=120)
        if result.returncode:
            raise RuntimeError('SDK bridge compilation failed:\n' + result.stderr)
        os.replace(temp, binary); stamp.write_text(signature)
    finally:
        temp.unlink(missing_ok=True)
    return binary


class Camera:
    def __init__(self, folder, serial=SERIAL, fps=15):
        self.folder = Path(folder); self.serial = serial; self.fps = fps
        self.metadata = None; self.latest = None; self.error = None
        self.jpeg_report = dict(policy='reject_decoder_warnings', received=0, accepted=0, rejected=0,
                                consecutive_rejected=0, last_rejection=None, rejection_examples=[])
        self.stop = threading.Event(); self.process = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            binary = build_bridge()
            decoder = JpegDecoder()
            if self.stop.is_set():
                return
            with (self.folder / 'camera_stderr.log').open('wb') as log:
                self.process = subprocess.Popen([str(binary), self.serial, str(self.fps)],
                                                stdout=subprocess.PIPE, stderr=log, cwd=self.folder)
                stream = self.process.stdout

                def line():
                    raw = stream.readline(65537)
                    if not raw or len(raw) > 65536 or not raw.endswith(b'\n'):
                        raise RuntimeError('SDK camera stream ended or returned an incomplete header; see camera_stderr.log')
                    return json.loads(raw)

                meta = line()
                if meta.get('protocol') != 1 or meta.get('serial') != self.serial:
                    raise RuntimeError('Camera identity/protocol mismatch')
                camera_parameters(meta)
                meta['jpeg_decode_policy'] = self.jpeg_report['policy']
                self.metadata = meta
                previous = 0; previous_device = -1
                last_valid_time = time.monotonic()
                while not self.stop.is_set():
                    header = line(); size = header['n']
                    if not isinstance(size, int) or not 100 <= size <= 10_000_000:
                        raise RuntimeError('Invalid JPEG size from SDK')
                    data = stream.read(size)
                    if len(data) != size:
                        raise RuntimeError('Truncated camera frame')
                    if header['sequence'] <= previous or header['device_timestamp_us'] <= previous_device:
                        raise RuntimeError('Repeated or backward camera timestamp')
                    previous = header['sequence']; previous_device = header['device_timestamp_us']
                    age = time.monotonic() - header['received_monotonic']
                    if not -.01 <= age <= .5:
                        raise RuntimeError('Stale SDK frame / host clock mismatch; restart camera session')
                    self.jpeg_report['received'] += 1
                    try:
                        frame = decoder.decode(data)
                    except CorruptJpegError as e:
                        self.jpeg_report['rejected'] += 1
                        self.jpeg_report['consecutive_rejected'] += 1
                        self.jpeg_report['last_rejection'] = str(e)
                        if len(self.jpeg_report['rejection_examples']) < 20:
                            self.jpeg_report['rejection_examples'].append(dict(sequence=header['sequence'], reason=str(e)))
                        if time.monotonic() - last_valid_time > 3:
                            raise RuntimeError(self.unusable_frame_message()) from e
                        # Do not publish, reuse, or silently repair this image.
                        continue
                    last_valid_time = time.monotonic()
                    self.jpeg_report['accepted'] += 1
                    self.jpeg_report['consecutive_rejected'] = 0
                    header['jpeg_checked_total'] = self.jpeg_report['received']
                    header['jpeg_rejected_total'] = self.jpeg_report['rejected']
                    header['jpeg_decode_policy'] = self.jpeg_report['policy']
                    self.latest = (header, frame)
        except Exception as e:
            if not self.stop.is_set():
                self._terminate()
                self.error = camera_failure_message(e, self.folder)
        finally:
            self._terminate()
            try:
                atomic_json(self.folder / 'camera_decode_report.json', self.jpeg_report)
            except OSError:
                pass  # Best-effort diagnostic; accepted frame headers also carry counters.

    def unusable_frame_message(self):
        if self.jpeg_report['rejected']:
            return ('No fresh usable RGB frames; damaged JPEGs rejected: '
                    + str(self.jpeg_report['rejected']) + '. Last decoder error: '
                    + str(self.jpeg_report['last_rejection'])
                    + '. Stop and check camera/USB streaming before collecting calibration poses.')
        return 'Camera frame stream became stale'

    def _terminate(self):
        p = self.process
        if p is not None:
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    p.kill(); p.wait(timeout=2)

    def close(self):
        self.stop.set(); self._terminate(); self.thread.join(timeout=4)


class Session:
    def __init__(self, parent, preview=False):
        stamp = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self.path = Path(parent).expanduser().resolve() / (stamp + '-' + uuid.uuid4().hex[:8])
        self.path.mkdir(parents=True)
        (self.path / 'images').mkdir()
        (self.path / 'target.json').write_bytes(TARGET.read_bytes())
        (self.path / 'model.urdf').write_bytes(URDF.read_bytes())
        self.data = dict(schema='openarmx-eye-to-hand-session-v1', created_utc=stamp,
                         preview_only=preview, base_frame='world', hand_frame=HAND,
                         camera_frame='gemini2_color_optical_frame', units='metres',
                         model_sha256=digest(URDF), target_sha256=digest(TARGET),
                         encoder_source_sha256=digest(ROOT / 'safe_zone/encoder.py'),
                         sdk_directory=str(SDK.resolve()),
                         encoder_convention='safe_zone.encoder.encoder_to_joint; right J5 offset -8.5 degrees',
                         opencv_version=cv2.__version__, camera=None, confirmations=None,
                         samples=[], excluded=[], fit=None, validation=None)
        self.save()

    def save(self):
        atomic_json(self.path / 'session.json', self.data)

    def append(self, sample, image):
        sample = dict(sample)
        if sample.get('phase') not in ('training', 'validation'):
            raise ValueError('Unknown sample phase')
        frozen = self.data['fit'] is not None
        if (sample['phase'] == 'training' and frozen) or (sample['phase'] == 'validation' and not frozen):
            raise ValueError('Training/validation phase mismatch: fit is frozen' if frozen else 'Fit training first')
        if self.data['validation'] is not None:
            raise ValueError('Session is finalized; begin a new session')
        sample['captured_utc'] = dt.datetime.now(dt.timezone.utc).isoformat()
        sample['id'] = len(self.data['samples']) + len(self.data['excluded']) + 1
        relative = f"images/{sample['id']:04d}.png"
        temp = self.path / 'images' / '.capture.png'
        if not cv2.imwrite(str(temp), image):
            raise RuntimeError('Failed to save raw capture')
        os.replace(temp, self.path / relative)
        sample['image'] = relative; sample['image_sha256'] = digest(self.path / relative)
        self.data['samples'].append(sample)
        try:
            self.save()
        except Exception:
            self.data['samples'].pop()
            raise
        return sample

    def fit(self, board):
        if self.data['preview_only'] or self.data['camera'] is None:
            raise ValueError('Collect a capture-mode session with camera metadata before solving')
        if self.data['fit'] is not None:
            raise ValueError('Training is frozen; begin a new session to change the fit')
        K, D = camera_parameters(self.data['camera'])
        training = [s for s in self.data['samples'] if s['phase'] == 'training']
        fit = solve(training, K, D, board)
        atomic_json(self.path / 'training_report.json', fit)
        if not fit['training_passed']:
            m = fit['training_metrics']
            raise ValueError(f"Training consistency failed: {m['translation_mm']['rms']:.1f}mm, "
                             f"{m['rotation_deg']['rms']:.2f}deg, {m['reprojection_px']['rms']:.1f}px RMS. "
                             'See training_report.json. Check board rigidity, dimensions and joint zeros.')
        fit['frozen_utc'] = dt.datetime.now(dt.timezone.utc).isoformat()
        self.data['fit'] = fit
        try:
            self.save()
        except Exception:
            self.data['fit'] = None
            raise
        return fit

    def validate(self, board):
        # A failed revalidation must never leave an older passing result in place.
        export_path = self.path / 'calibration.json'
        if export_path.exists():
            atomic_json(export_path, dict(schema='openarmx-eye-to-hand-calibration-v1', validated=False,
                                          motion_authorized=False, depth_mapping_verified=False,
                                          error='Revalidation pending or failed; see session/training reports'))
        fit = self.data['fit']
        if not fit or not fit['training_passed']:
            raise ValueError('Solve and pass training first')
        heldout = [s for s in self.data['samples'] if s['phase'] == 'validation']
        if set(s['id'] for s in heldout) & set(fit['training_ids']):
            raise ValueError('Validation samples overlap the training fit')
        if len(heldout) < MIN_VALIDATION:
            raise ValueError(f'Capture at least {MIN_VALIDATION} new validation poses after solving')
        K, D = camera_parameters(self.data['camera'])
        C, X = checked_transform(fit['T_base_color']), checked_transform(fit['T_hand_board'])
        report = metrics(heldout, C, X, K, D, board.points)
        self.data['validation'] = dict(passed=passes(report), metrics=report,
                                       used_for_fitting=False, fit_training_ids=fit['training_ids'])
        self.save()
        output = dict(schema='openarmx-eye-to-hand-calibration-v1', units='metres',
                      transform_convention='p_destination = T_destination_source @ p_source (column vectors)',
                      base_frame='world', hand_frame=HAND, color_frame='gemini2_color_optical_frame',
                      depth_frame='gemini2_depth_optical_frame', T_base_color=C.tolist(),
                      T_hand_board=X.tolist(),
                      T_base_depth=(C @ checked_transform(self.data['camera']['T_color_depth_m'])).tolist(),
                      camera=self.data['camera'], model_sha256=self.data['model_sha256'],
                      encoder_source_sha256=self.data['encoder_source_sha256'],
                      sdk_directory=self.data['sdk_directory'],
                      target_sha256=self.data['target_sha256'], confirmations=self.data['confirmations'],
                      training=fit, validation=self.data['validation'],
                      validated=passes(report), motion_authorized=False, depth_mapping_verified=False,
                      warning='Consistency validation only, not certified physical accuracy or collision safety. '
                              'Raw-depth transform uses factory depth-to-color extrinsics; depth geometry still needs verification.')
        # Always overwrite the result, including failures: never leave a stale passing export.
        atomic_json(self.path / 'calibration.json', output)
        return output


def load_session(folder):
    s = Session.__new__(Session); s.path = Path(folder).expanduser().resolve()
    s.data = json.loads((s.path / 'session.json').read_text())
    if s.data.get('schema') != 'openarmx-eye-to-hand-session-v1':
        raise ValueError('Unknown session format')
    if digest(s.path / 'model.urdf') != s.data['model_sha256'] or digest(s.path / 'target.json') != s.data['target_sha256']:
        raise ValueError('Session model/target hash mismatch')
    for sample in s.data['samples']:
        image = (s.path / sample['image']).resolve()
        if not image.is_relative_to(s.path / 'images') or digest(image) != sample['image_sha256']:
            raise ValueError('Capture image path/hash mismatch')
    return s
