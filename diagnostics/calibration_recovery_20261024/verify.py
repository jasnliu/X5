"""Recheck saved evidence only. Never opens a camera/CAN connection or moves hardware."""
import hashlib
import json
from pathlib import Path
import struct
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from drum_planner.common import DEFAULT_CALIBRATION, calibration

D = Path(__file__).resolve().parent
SOURCE = ROOT/'calibration_sessions/20261024T211629Z-1227b7b1'
OLD = ROOT/'calibration_sessions/20261017T211949Z-762f9908-reprocessed/calibration.json'


def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path): return json.loads(Path(path).read_text())


def depth_planes(folder):
    """Dominant measured planes, not fitted to robot/calibration board observations.

    Fixed random seed and 8 mm inlier threshold, no enforced horizontal normal.
    Quantized occupied points are sufficient for an indicative level check, not
    submillimetre absolute accuracy. Old/new comparison uses the SAME fresh data.
    """
    a = np.fromfile(folder/'points_base.f32', dtype=np.float32).reshape(-1, 3)
    rng = np.random.default_rng(245)
    a = a[rng.choice(len(a), min(len(a), 30000), replace=False)]
    Rold = np.array(read(OLD)['T_base_depth'])[:3, :3]
    Rnew = np.array(calibration()['T_base_depth'])[:3, :3]
    rows = []
    for _ in range(5):
        best = np.zeros(len(a), bool)
        for _ in range(500):
            p0, p1, p2 = a[rng.choice(len(a), 3, replace=False)]
            normal = np.cross(p1-p0, p2-p0); norm = np.linalg.norm(normal)
            if norm < 1e-6: continue
            normal /= norm; mask = abs((a-p0)@normal) < .008
            if mask.sum() > best.sum(): best = mask
        points = a[best]; center = points.mean(axis=0)
        normal = np.linalg.svd(points-center, full_matrices=False)[2][-1]
        if normal[2] < 0: normal = -normal
        old_normal = Rold @ Rnew.T @ normal
        rows.append(dict(points=len(points), center=center.tolist(), normal=normal.tolist(),
            new_tilt_deg=float(np.degrees(np.arccos(abs(normal[2])))),
            old_tilt_deg=float(np.degrees(np.arccos(abs(old_normal[2])))),
            rms_mm=float(np.sqrt(np.mean(((points-center)@normal)**2))*1000),
            bounds=[points.min(axis=0).tolist(), points.max(axis=0).tolist()]))
        a = a[~best]
    return rows


def main():
    cal = calibration()
    source, copied = read(SOURCE/'session.json'), read(DEFAULT_CALIBRATION.parent/'session.json')
    assert source['samples'] == copied['samples'] and source['excluded'] == copied['excluded']
    assert source['fit'] is None and cal['validated']
    images = source['samples'] + source['excluded']
    for row in images:
        assert digest(SOURCE/row['image']) == row['image_sha256']
        assert digest(DEFAULT_CALIBRATION.parent/row['image']) == row['image_sha256']
    training = [s['id'] for s in source['samples'] if s['phase'] == 'training']
    heldout = [s['id'] for s in source['samples'] if s['phase'] == 'validation']
    assert training == cal['training']['training_ids'] and heldout == cal['validation']['heldout_ids']
    assert len(training) == 35 and heldout == [1,2,3,4,5] and not set(training)&set(heldout)
    before = read(D/'before_hashes.json')
    changed = [name for name, sha in before.items() if digest(ROOT/name) != sha]
    allowed = {'camera_calibration/io.py','camera_calibration/kinematics.py','drum_planner/common.py',
        'CAMERA_CALIBRATION.md','DRUM_PLANNER.md','DRUM_PLANNER_EXPERIMENTS.md',
        'tests/test_experiment_checkpoints.py','tests/test_hardware_workspace.py','tests/test_singlearc_arms.py',
        'tests/test_baseline_workspace.py','tests/test_sparse_hardware.py'}
    assert set(changed) <= allowed, changed
    assert all(digest(ROOT/p) == sha for p,sha in before.items() if p.startswith('calibration_sessions/'))
    live = {}
    for side in ('right','left'):
        folder = D/('live_'+side); context = read(folder/'context.json')
        report = read(folder/'acquisition/acquisition_report.json')
        scene = read(folder/'acquisition/camera/scene.json')
        assert context['calibration'] == str(DEFAULT_CALIBRATION)
        assert scene['calibration_sha256'] == digest(DEFAULT_CALIBRATION)
        assert scene['T_base_depth'] == cal['T_base_depth']
        assert report['passed'] and report['camera_released'] and not report['cached_inputs_used']
        assert report['telemetry']['thread_stopped'] and report['telemetry']['observer']['closed']
        rows = [json.loads(line) for line in (folder/'acquisition/can_query_audit.jsonl').read_text().splitlines()]
        sent = [r for r in rows if r['event'] == 'sent']
        for row in rows:
            cid,dlc,data = struct.unpack('=IB3x8s', bytes.fromhex(row['packet_hex']))
            assert row['operation'] == 'state_request_only' and row['bus'] in ('can0','can1')
            assert cid == 0x8200fd00|row['motor'] and 1 <= row['motor'] <= 8
            assert dlc == 8 and data == bytes.fromhex('0100000000000000')
        for row in (json.loads(line) for line in (folder/'acquisition/can_rx_audit.jsonl').read_text().splitlines()):
            cid,dlc,data = struct.unpack('=IB3x8s', bytes.fromhex(row['packet_hex']))
            assert ((cid>>22)&3) == 0 and ((cid>>16)&63) == 0  # disabled, no reported motor faults
        live[side] = dict(passed=True,calibration_sha256=scene['calibration_sha256'],
            elapsed_s=report['elapsed_s'],query_frames=len(sent),motor_commands=0,
            camera_released=True,encoders_closed=True,all_reported_motors_disabled=True,
            scene_sha256=digest(folder/'acquisition/camera/scene.json'))
    planes = depth_planes(D/'live_right/acquisition/camera')
    (D/'depth_planes.json').write_text(json.dumps(planes, indent=2)+'\n')
    result = dict(passed=True,source=str(SOURCE),source_session_sha256=digest(SOURCE/'session.json'),
        calibration=str(DEFAULT_CALIBRATION),calibration_sha256=digest(DEFAULT_CALIBRATION),
        original_and_copied_images_identical=len(images),training_ids=training,heldout_ids=heldout,
        all_original_capture_and_previous_calibration_files_unchanged=True,
        checked_original_files=len(before),changed_original_files=changed,
        training=cal['training']['training_metrics'],crossvalidation=cal['training']['internal_crossvalidation'],
        validation=cal['validation'],live=live,dominant_floor_plane=planes[0],
        physical_absolute_accuracy_verified=False,physical_motion_tested=False,
        source_sha256={p:digest(ROOT/p) for p in ['camera_calibration/bundle.py','camera_calibration/kinematics.py',
            'camera_calibration/io.py','camera_calibration/core.py','drum_planner/common.py']})
    (D/'verification.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('passed','calibration_sha256','original_and_copied_images_identical','live','changed_original_files')},indent=2))


if __name__ == '__main__': main()
