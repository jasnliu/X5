"""Startup routing/acquisition/cleanup: fake devices only, no motor authority."""
import json
from pathlib import Path

import pytest

from drum_planner import cli
from drum_planner.common import arm_joints, atomic_json
from drum_planner.experiment import acquisition, app


@pytest.mark.parametrize('side',['right','left'])
@pytest.mark.parametrize('options,fresh',[
    ([],True),(['--capture-start'],True),(['--offline'],False),
    (['--state','state.json','--scene','scene.json'],False),
    (['--offline','--state','state.json'],False)])
def test_cli_fresh_default_and_explicit_offline(monkeypatch,tmp_path,side,options,fresh):
    calls=[]
    monkeypatch.setattr(app,'prepare',lambda *a,**kw:calls.append((a,kw)))
    monkeypatch.setattr(app,'headless',lambda *a:{'workspace_curve':{'status':'planned'}})
    assert cli.main(['experiment','--'+side,'--headless','--output',str(tmp_path),*options])==0
    assert len(calls)==1 and calls[0][1]['fresh'] is fresh and calls[0][1]['side']==side


@pytest.mark.parametrize('options',[
    ['--state','state.json'],['--scene','scene.json'],['--offline','--capture-start'],
    ['--capture-start','--state','state.json','--scene','scene.json']])
def test_cli_rejects_mixed_acquisition_before_devices(monkeypatch,options,capsys):
    monkeypatch.setattr(app,'prepare',lambda *a,**k:pytest.fail('Invalid inputs reached preparation'))
    assert cli.main(['experiment',*options])==1
    assert 'Planner stopped (no motor commands)' in capsys.readouterr().err


def test_list_methods_is_device_free(monkeypatch):
    monkeypatch.setattr(app,'prepare',lambda *a,**k:pytest.fail('Listing methods opened devices'))
    assert cli.main(['experiment','--list-methods'])==0


@pytest.fixture
def jobs(monkeypatch):
    calls=[]
    monkeypatch.setattr(app,'no_can',lambda:None)
    monkeypatch.setattr(app,'saved_inputs',lambda *a:pytest.fail('Fresh startup loaded saved defaults'))
    class Job:
        def __init__(self,command,folder,**kwargs):
            self.command=command;self.kwargs=kwargs;calls.append(self)
    def wait(job):
        cmd=job.command;folder=Path(cmd[cmd.index('--folder')+1]);folder.mkdir(exist_ok=True)
        if 'drum_planner.experiment.acquisition' in cmd:
            atomic_json(folder/'acquisition_report.json',dict(passed=True,folder=str(folder)))
        elif 'initialize' in cmd:
            atomic_json(folder/'context.json',dict(state=cmd[cmd.index('--state')+1],
                                                  scene=cmd[cmd.index('--scene')+1]))
    monkeypatch.setattr(app,'Job',Job);monkeypatch.setattr(app,'wait',wait)
    return calls,wait


@pytest.mark.parametrize('side',['right','left'])
def test_every_launch_acquires_once_then_disconnected_initialize(tmp_path,jobs,side):
    calls,_=jobs;contexts=[]
    for i in range(2):contexts.append(app.prepare(tmp_path/str(i),side=side,fresh=True))
    assert len(calls)==6
    for i,context in enumerate(contexts):
        build,capture,initialize=calls[i*3:i*3+3]
        assert 'build' in build.command and not build.kwargs.get('allow_can',False)
        assert 'drum_planner.experiment.acquisition' in capture.command
        assert capture.kwargs['allow_can'] and capture.kwargs['timeout']<=45
        assert capture.command[capture.command.index('--side')+1]==side
        assert 'initialize' in initialize.command and not initialize.kwargs.get('allow_can',False)
        assert context['input_origin']=='fresh_startup' and context['startup_acquisition']['passed']
        assert context['state']==str(tmp_path/str(i)/'acquisition/measured_start.json')
        assert context['scene']==str(tmp_path/str(i)/'acquisition/camera/scene.json')
    assert contexts[0]['scene']!=contexts[1]['scene']
    assert contexts[0]['state']!=contexts[1]['state']


def test_capture_failure_has_no_old_input_fallback_or_initialization(tmp_path,jobs,monkeypatch):
    calls,wait=jobs
    def failed(job):
        if 'drum_planner.experiment.acquisition' in job.command:
            target=tmp_path/'session/acquisition';target.mkdir()
            atomic_json(target/'acquisition_report.json',dict(passed=False,error='camera disconnected'))
            raise RuntimeError('worker failed')
        wait(job)
    monkeypatch.setattr(app,'wait',failed)
    with pytest.raises(RuntimeError,match='camera disconnected.*No saved-input fallback'):
        app.prepare(tmp_path/'session',fresh=True)
    assert len(calls)==2 and not (tmp_path/'session/context.json').exists()


def test_fresh_prepare_rejects_saved_override_before_jobs(tmp_path,jobs):
    with pytest.raises(ValueError,match='mixed'):
        app.prepare(tmp_path/'session',state='old.json',fresh=True)
    assert not jobs[0] and not (tmp_path/'session').exists()


@pytest.fixture
def devices(monkeypatch):
    events=[];workers=[]
    class Telemetry:
        def __init__(self,folder):
            self.closed=False;self.error=None;workers.append(self);events.append('encoders_opened')
        def close(self):self.closed=True;events.append('encoders_closed')
        def report(self):return dict(thread_stopped=self.closed,error=self.error,motor_commands=0)
    def capture_current(folder,worker,cal_path,*,stationary_joints):
        assert not worker.closed
        events.append(('camera_capture',stationary_joints))
        atomic_json(folder/'measured_start.json',dict(right_rad=[1.]*7,left_rad=[2.]*7))
        (folder/'camera').mkdir();atomic_json(folder/'camera/scene.json',dict(fresh=True))
        events.append('camera_closed')
    monkeypatch.setattr(acquisition,'Telemetry',Telemetry)
    monkeypatch.setattr(acquisition,'capture_current',capture_current)
    return events,workers


@pytest.mark.parametrize('side',['right','left'])
def test_acquisition_measures_both_arms_but_only_selected_stillness(tmp_path,devices,side):
    events,workers=devices;folder=tmp_path/'new'
    report=acquisition.capture(folder,side)
    assert events==['encoders_opened',('camera_capture',arm_joints(side)),'camera_closed','encoders_closed']
    assert report['passed'] and report['camera_released'] and report['telemetry']['thread_stopped']
    assert not report['cached_inputs_used'] and report['measured_sides']==['right','left']
    assert report['motor_commands']==report['mode_changes']==report['gripper_commands']==report['relax_commands']==0
    assert not report['motion_authorized']
    assert json.loads((folder/'acquisition_report.json').read_text())==report


@pytest.mark.parametrize('failure',['capture','telemetry','close','interrupt'])
def test_device_failures_cleanup_and_never_mark_success(tmp_path,devices,monkeypatch,failure):
    events,workers=devices
    def failed(folder,worker,*a,**k):
        if failure=='telemetry':worker.error='feedback lost'
        elif failure=='close':
            def bad_close():worker.closed=True;raise RuntimeError('close failed')
            worker.close=bad_close
        elif failure=='interrupt':raise KeyboardInterrupt()
        else:raise RuntimeError('camera startup failed')
    monkeypatch.setattr(acquisition,'capture_current',failed)
    with pytest.raises(KeyboardInterrupt if failure=='interrupt' else RuntimeError):
        acquisition.capture(tmp_path/'bad')
    assert workers[0].closed
    report=json.loads((tmp_path/'bad/acquisition_report.json').read_text())
    assert not report['passed'] and report['error']
    assert not report['cached_inputs_used']
