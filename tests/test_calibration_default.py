"""New calibration routing; no actual hardware owners or camera connections."""
import json
import hashlib
import socket
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from camera_calibration import core, kinematics
from drum_planner import cli, common, model, planning
from drum_planner.experiment import app
from safe_zone.geometry import Model

OLD = common.ROOT / 'calibration_sessions/20261017T211949Z-762f9908-reprocessed/calibration.json'


def test_common_only_changes_the_calibration_default():
    source = (common.ROOT/'drum_planner/common.py').read_bytes()
    normalized = source.replace(b'20261024T211629Z-1227b7b1-reprocessed', b'20261017T211949Z-762f9908-reprocessed')
    assert hashlib.sha256(normalized).hexdigest() == '079cccfdeca5a75b681bd40506bd04f06c16ca2d38a26bcb5d500cc702ef98f7'


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    def denied(*a, **kw): pytest.fail('Routing test attempted hardware/process access')
    monkeypatch.setattr(socket, 'socket', denied)
    monkeypatch.setattr(subprocess, 'Popen', denied)


@pytest.mark.parametrize('side', ['right', 'left'])
@pytest.mark.parametrize('command', ['--hardware', '--hardware-sparse', 'hardware', 'hardware-sparse', 'singlearc', 'baseline', 'experiment'])
def test_both_arms_receive_current_calibration(monkeypatch, tmp_path, command, side):
    calls = []
    if command == 'experiment':
        monkeypatch.setattr(app, 'prepare', lambda folder,state,scene,calibration,**kw: calls.append((calibration, kw['side'])))
        monkeypatch.setattr(app, 'show', lambda *a, **kw: None)
    else:
        from drum_planner.baseline import workspace
        monkeypatch.setattr(workspace, 'show', lambda folder,cal_path,**kw: calls.append((cal_path, kw['moving_side'])))
    assert cli.main([command, '--'+side, '--output', str(tmp_path / 'run')]) == 0
    assert calls == [(str(common.DEFAULT_CALIBRATION), side)]
    assert '20261024T211629Z' in calls[0][0]


@pytest.mark.parametrize('command,module,function,argv,index', [
    ('camera', 'depth', 'capture', [], 3),
    ('plan', 'planning', 'plan', ['--state','state','--scene','scene'], 3),
    ('preview', 'preview', 'preview', ['plan'], None),
    ('hardware-check', 'hardware_check', 'run', ['--headless'], None),
    ('hardware-view', 'alignment_view', 'show', ['folder'], None),
    ('planned-view', 'planned_view', 'show', ['plan'], None),
    ('live', 'current_plan', 'run', ['--headless'], None),
])
def test_other_calibrated_commands_route_new_default(monkeypatch, tmp_path, command, module, function, argv, index):
    import importlib
    calls=[]; saved=tmp_path/'plan.json';saved.write_text('{"status":"planned"}')
    def fake(*a, **kw):
        calls.append(a[index] if index is not None else kw['cal_path'])
        if command == 'plan': return {'status':'planned'}
        if command == 'live': return saved
        return None
    monkeypatch.setattr(importlib.import_module('drum_planner.'+module), function, fake)
    assert cli.main([command, *argv]) == 0
    assert calls == [str(common.DEFAULT_CALIBRATION)]


@pytest.mark.parametrize('side', ['right', 'left'])
def test_new_model_uses_new_calibration_without_global_joint_changes(tmp_path, side):
    cal = common.calibration()
    manifest = model.generate(tmp_path, moving_side=side)
    assert manifest['calibration_sha256'] == core.digest(common.DEFAULT_CALIBRATION)
    original = Model(core.URDF); generated = Model(tmp_path/'robot.urdf')
    sample = json.loads((common.ROOT/'calibration_sessions/20261024T211629Z-1227b7b1/session.json').read_text())['samples'][0]
    q = dict(sample['joints_rad'])
    q.update({j:0. for j in common.LEFT_JOINTS})
    corrected = kinematics.corrected_joint_values(q, cal['kinematic_correction'])
    assert np.allclose(generated.transforms(q)[core.HAND], original.transforms(corrected)[core.HAND], atol=1e-12)
    assert np.allclose(generated.transforms(q)['openarmx_left_hand'], original.transforms(q)['openarmx_left_hand'], atol=1e-12)
    a, b = ET.parse(core.URDF).getroot(), ET.parse(tmp_path/'robot.urdf').getroot()
    for name, offset in cal['kinematic_correction']['offsets_rad'].items():
        old = a.find(f"joint[@name='{name}']/limit"); new = b.find(f"joint[@name='{name}']/limit")
        for key in ('lower', 'upper'):
            assert float(new.get(key)) == pytest.approx(float(old.get(key))-offset)
        for key in ('effort', 'velocity'): assert old.get(key) == new.get(key)


def test_historical_scene_is_not_silently_reinterpreted(tmp_path):
    scene = tmp_path/'old.json'
    scene.write_text(json.dumps({'schema':'x5-native-depth-scene-v1', 'synthetic':False, 'calibration_sha256':core.digest(OLD)}))
    with pytest.raises(ValueError, match='different calibration'):
        planning.scene(scene)
