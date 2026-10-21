"""Git housekeeping tests. No hardware, network, or removal of local data."""
import builtins
import io
import json
import os
from pathlib import Path
import socket
import subprocess

import pytest

from scripts import git_artifacts as artifacts


def test_repository_allowlist_and_generated_ignore_rules_match():
    result = artifacts.check()
    assert result['curated_files'] > 0
    assert result['curated_bytes'] < 250 * 2**20


def make_policy(tmp_path, names):
    data = {'schema': 'x5-git-artifact-policy-v1', 'generated_roots': ['runs'],
            'artifacts': [{'path': n, 'reason': 'fixture'} for n in names]}
    file = tmp_path / 'policy.json'
    file.write_text(json.dumps(data))
    return file


@pytest.mark.parametrize('name', ['/runs/x', 'runs/../secret', 'runs//x',
                                  'runs/*.json', 'source/x', 'runs/x\nother'])
def test_bad_manifest_paths_rejected(tmp_path, name):
    with pytest.raises(ValueError):
        artifacts.policy(make_policy(tmp_path, [name]))


def test_duplicate_paths_rejected(tmp_path):
    with pytest.raises(ValueError, match='Duplicate'):
        artifacts.policy(make_policy(tmp_path, ['runs/x', 'runs/x']))


def test_git_ignores_future_output_but_keeps_explicit_fixtures(tmp_path):
    manifest = make_policy(tmp_path, ['runs/saved/deep/input.json'])
    fixture = tmp_path / 'runs/saved/deep/input.json'
    fixture.parent.mkdir(parents=True)
    fixture.write_text('{}')
    (tmp_path / '.gitignore').write_text('__pycache__/\n')
    artifacts.update_ignore(tmp_path, manifest, write=True)
    original = (tmp_path / '.gitignore').read_bytes()
    artifacts.update_ignore(tmp_path, manifest, write=True)
    assert (tmp_path / '.gitignore').read_bytes() == original
    subprocess.run(['git', 'init', '-q', str(tmp_path)], check=True)
    names = ['runs/saved/deep/input.json', 'runs/saved/deep/new.log',
             'runs/new-session/raw_depth.npz', 'runs/new-session/video.avi',
             'source.py', '__pycache__/cached.pyc']
    result = subprocess.run(['git', '-C', str(tmp_path), 'check-ignore', '--no-index', '-z', '--stdin'],
                            input=('\0'.join(names) + '\0').encode(), capture_output=True)
    assert result.returncode == 0
    assert set(result.stdout.decode().rstrip('\0').split('\0')) == set(names[1:4] + names[5:])
    assert fixture.read_text() == '{}'


def test_required_inputs_work_without_access_to_untracked_run_files(monkeypatch):
    """Fail any read of uncurated output, even though old files remain on disk."""
    from drum_planner import common, planning, depth
    from drum_planner.experiment.config import saved_inputs
    from drum_planner.trajectory_view import PlanDisplay
    from smooth_playback.trajectory import ROOT as playback_root

    data, names = artifacts.policy()
    allowed = set(names)
    real_socket = socket.socket

    def socket_guard(family=socket.AF_INET, *args, **kwargs):
        if family == socket.AF_CAN:
            raise AssertionError('No CAN during Git housekeeping')
        return real_socket(family, *args, **kwargs)

    def denied(*args, **kwargs):
        raise AssertionError('No camera during Git housekeeping')

    def guard(original):
        def checked(file, *args, **kwargs):
            if isinstance(file, (str, bytes, os.PathLike)):
                p = Path(os.fsdecode(file)).resolve()
                if p.is_relative_to(artifacts.ROOT):
                    rel = p.relative_to(artifacts.ROOT).as_posix()
                    if rel.split('/')[0] in data['generated_roots']:
                        assert rel in allowed, 'Depends on untracked output: ' + rel
            return original(file, *args, **kwargs)
        return checked

    monkeypatch.setattr(socket, 'socket', socket_guard)
    monkeypatch.setattr(depth, 'DepthCamera', denied)
    monkeypatch.setattr(builtins, 'open', guard(builtins.open))
    monkeypatch.setattr(io, 'open', guard(io.open))
    assert common.calibration()['validated']
    state, scene = saved_inputs()
    common.read_state(state)
    assert planning.scene(scene)['source_scene_sha256'] == common.digest(scene)
    # Every retained demo/real-plan fixture can load its own calibrated model.
    plans = [p for p in names if p.endswith('/plan.json')]
    assert len(plans) >= 5
    for name in plans:
        display = PlanDisplay(common.ROOT / name)
        display.frames(0.)
        assert display.obj['hardware_execution_available'] is False
    timing = json.loads((playback_root / 'playback_results/legacy_gui_timing.json').read_text())
    assert timing['commands']
