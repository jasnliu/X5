"""Prepare-only launcher/worker isolation; no real devices in these tests."""
import ast
import json
from pathlib import Path
from unittest.mock import Mock
import pytest
from camera_playback import generated_prepare as prepare
from drum_planner.common import ROOT


def test_query_only_then_same_planning_worker_and_no_execution(tmp_path,monkeypatch):
    calls=[]
    class Job:
        def __init__(self,command,folder,**options):
            self.command=command;self.closed=False;calls.append((self,options));self.log_path=folder/'worker.log'
        def poll(self):return 0
        def close(self):self.closed=True
    monkeypatch.setattr(prepare,'Job',Job)
    from camera_playback import generated_planning
    loaded=Mock(return_value=({'duration_s':{'left':3.,'right':4.}},{}))
    monkeypatch.setattr(generated_planning,'load',loaded)
    mission=prepare.run(tmp_path/'run')
    assert calls[0][0].command[2]=='camera_playback.generated_acquisition'
    assert calls[0][1]==dict(timeout=60,allow_can=True)
    assert calls[1][0].command[2]=='camera_playback.generated_planning'
    assert calls[1][1]==dict(timeout=300,allow_can=False)
    assert all(job.closed for job,_ in calls)
    assert '--capture' in calls[1][0].command and mission==tmp_path/'run/plan/mission.json'
    loaded.assert_called_once_with(mission)
    proof=json.loads((tmp_path/'run/prepare_only.json').read_text())
    assert proof['status']=='ready' and proof['motor_commands']==0 and not proof['esp32_access']


@pytest.mark.parametrize('error',[RuntimeError('camera failed'),KeyboardInterrupt()])
def test_failed_acquisition_stops_before_planning_and_closes(tmp_path,monkeypatch,error):
    job=Mock();job.poll.side_effect=error
    factory=Mock(return_value=job);monkeypatch.setattr(prepare,'Job',factory)
    with pytest.raises(type(error)):prepare.run(tmp_path/'run')
    factory.assert_called_once();job.close.assert_called_once()
    assert json.loads((tmp_path/'run/prepare_only.json').read_text())['status']=='failed'


def launcher_prefix(args,monkeypatch):
    tree=ast.parse((ROOT/'launch_right_camera_playback.py').read_text());body=[]
    for node in tree.body:
        if isinstance(node,ast.If) and ast.unparse(node.test)=='args.list_cameras':break
        if isinstance(node,ast.ImportFrom) and node.module.startswith(('launch','launch_ros')):continue
        body.append(node)
    monkeypatch.setattr('sys.argv',['start_beat.sh',*args])
    return compile(ast.Module(body=body,type_ignores=[]),'launcher-prefix','exec')


def test_actual_launcher_dispatches_before_app_audio_or_esp(tmp_path,monkeypatch):
    called=Mock(return_value=0);monkeypatch.setattr(prepare,'main',called)
    code=launcher_prefix(['--hardware','--prepare-only'],monkeypatch)
    with pytest.raises(SystemExit) as exc:exec(code,{'__file__':str(ROOT/'launch_right_camera_playback.py')})
    assert exc.value.code==0;called.assert_called_once()


@pytest.mark.parametrize('args',[
    ['--prepare-only'],['--test','--prepare-only'],['--hardwaretest','--prepare-only'],
    ['--hardware','--prepare-only','--recording-only'],['--hardware','--prepare-only','--verify-swing'],
    ['--hardware','--prepare-only','--list-cameras'],
])
def test_prepare_only_cannot_turn_into_motion_mode(args,monkeypatch):
    called=Mock();monkeypatch.setattr(prepare,'main',called)
    code=launcher_prefix(args,monkeypatch)
    with pytest.raises(SystemExit) as exc:exec(code,{'__file__':str(ROOT/'launch_right_camera_playback.py')})
    assert exc.value.code==2;called.assert_not_called()
