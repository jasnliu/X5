"""Verify saved FOV evidence offline; no camera, motor or ROS imports."""
import ast
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[1]


def read(name):
    return json.loads((BASE/name).read_text())


protected = read('protected_before.json')
for name, expected in protected.items():
    assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == expected, name

coverage = read('relative_coverage.json')
old = next(row for row in coverage if row['size'] == [640, 480])
assert .66 < old['relative_width'] < .68
assert .88 < old['relative_height'] < .90
for row in coverage:
    assert row['inliers'] > 100
    if row['size'] != [640, 480]:
        assert abs(row['relative_width']-1) < .005
        assert abs(row['relative_height']-1) < .005

modes = read('mode_comparison.json')
assert len(modes) == 4 and all(row['fps'] > 28 for row in modes)
for name in ('search', 'playback'):
    result = read(f'{name}_live.json')
    assert result['exit_code'] == 0 and result['can_forbidden']
    assert result['image_shape'] == [720, 1280, 3]
    assert result['display_frames'] > 250 and result['processed_frame_messages'] > 100
    assert 'full FOV, 1280x720' in (BASE/f'{name}_live.log').read_text()
standalone = read('standalone_video/result.json')
assert standalone['error'] is None and standalone['frames'] > 60
assert standalone['image_shape'] == [720, 1280, 3]
assert read('numeric_override.json')['image_shape'] == [720, 1280, 3]
assert read('width_only.json')['image_shape'] == [360, 640, 3]
log = (BASE/'regressions.log').read_text()
assert 'Ran 402 tests' in log and log.rstrip().endswith('OK')

names = ['camera_search/device.py', 'config/camera.json', 'camera_search/camera.py',
         'launch_right_camera_cartesian.py', 'launch_right_camera_playback.py',
         'tests/test_camera_device.py', 'tests/test_playback_transport.py', 'CAMERA_DEFAULT.md']
hashes, diffs = {}, []
for name in names:
    path = ROOT/name
    if path.suffix == '.py':
        ast.parse(path.read_text(), filename=name)
    hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    before = BASE/'before'/name
    if before.exists():
        diffs.append(subprocess.run(['diff', '-u', str(before), str(path)],
                                    capture_output=True, text=True).stdout)
(BASE/'changes.diff').write_text('\n'.join(diffs))
report = dict(verified_at_utc=datetime.now(timezone.utc).isoformat(), offline_audit=True,
              default_size=[1280, 720], maximum_rgb_fov=True,
              nominal_manufacturer_rgb_fov_degrees=[86, 55],
              source='https://www.orbbec.com.cn/index/Product/info.html?cate=38&id=51',
              angular_calibration_performed=False, relative_coverage=coverage,
              live_modes=modes, live_pipelines={n: read(f'{n}_live.json') for n in ('search', 'playback')},
              standalone_video=standalone, regressions_passed=402,
              final_scene_lighting=read('lighting_check.json'),
              arm_movement=False, protected_unchanged=protected, final_hashes=hashes)
(BASE/'verification.json').write_text(json.dumps(report, indent=2)+'\n')
print('PASS: full RGB FOV, live pipeline/recording checks, 402 regressions, protected files unchanged')
