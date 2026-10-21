"""Current pose planning: all device access is simulated; MoveIt tests are real."""
import copy
import json
import socket
from types import SimpleNamespace

import numpy as np
import pytest

from drum_planner import common, current_plan as live, depth, planning, planned_view, cli


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    original=socket.socket
    def guarded(family=socket.AF_INET,*a,**kw):
        if family==socket.AF_CAN:raise AssertionError('Offline test attempted CAN access')
        return original(family,*a,**kw)
    monkeypatch.setattr(socket,'socket',guarded)
    monkeypatch.setattr(depth,'DepthCamera',lambda *a,**k:pytest.fail('Real camera forbidden'))


@pytest.fixture
def rig(tmp_path,monkeypatch):
    clock=SimpleNamespace(t=100.)
    monkeypatch.setattr(live.time,'monotonic',lambda:clock.t)
    monkeypatch.setattr(live.time,'sleep',lambda dt:setattr(clock,'t',clock.t+dt))
    state=common.read_state(common.ROOT/'drum_planner/config/example_start.json')
    objects=[];cameras=[]
    class Worker:
        def __init__(self,folder):
            self.began=clock.t;self.error=None;self.closed=False;self.drift=0.;objects.append(self)
        @property
        def count(self):return len(self.samples())
        def samples(self):
            result=[]
            for t in np.arange(self.began,clock.t-.01,.05):
                q=dict(zip(common.JOINTS+common.LEFT_JOINTS,state['right_rad']+state['left_rad']))
                q[common.JOINTS[0]]+=self.drift
                q['openarmx_left_finger_joint1']=0.
                result.append(dict(start=float(t),end=float(t+.01),joints=q,all_16_motors_disabled=True,
                    motor8_feedback={},feedback={}))
            return result
        def close(self):self.closed=True
        def report(self):return dict(thread_stopped=self.closed,mode_changes=0)
    class Camera:
        def __init__(self,folder,cal):
            self.meta=copy.deepcopy(cal['camera']);self.meta.update(protocol=1,alignment='native_depth',
                depth_stream_enabled=True,color_stream_enabled=False)
            self.error=None;self.thread=SimpleNamespace(is_alive=lambda:False)
            self.process=SimpleNamespace(poll=lambda:0);self.closed=False
            self.frames=[(dict(sequence=i+1,received_monotonic=clock.t-.4+i*.1,value_scale_m=.001,
                 device_timestamp_us=1000000+i*100000),np.full((800,1280),2000,dtype='<u2')) for i in range(5)]
            cameras.append(self)
        def close(self):self.closed=True
    monkeypatch.setattr(live,'Telemetry',Worker);monkeypatch.setattr(live,'DepthCamera',Camera)
    return SimpleNamespace(clock=clock,state=state,workers=objects,cameras=cameras,Worker=Worker,Camera=Camera)


def test_short_capture_five_frames_all_bracketed_no_motor_changes(tmp_path,rig):
    worker=rig.Worker(tmp_path)
    state,world=live.capture_current(tmp_path,worker)
    assert state['right_rad']==rig.state['right_rad'] and state['left_rad']==rig.state['left_rad']
    assert state['measured'] and state['motor_commands']==0
    sync=state['synchronization']
    assert sync['planning_frame_count']==5 and len(sync['all_five_frames'])==4
    for proof in [sync]+sync['all_five_frames']:
        assert proof['nearest_midpoint_skew_s']<=.1 and len(proof['bracketing_indices'])==2
    assert world['schema']=='x5-native-depth-scene-v1'
    assert world['measured_state_sha256']==common.digest(tmp_path/'measured_start.json')
    assert rig.cameras[0].closed and rig.clock.t-100<3
    assert not worker.closed  # retain ownership through planning
    assert planning.scene(tmp_path/'camera/scene.json')['synthetic'] is False


@pytest.mark.parametrize('bad',['sequence','gap','stale','camera_error','thread_alive'])
def test_bad_depth_cannot_be_planned(tmp_path,rig,monkeypatch,bad):
    class Bad(rig.Camera):
        def __init__(self,*a):
            super().__init__(*a)
            if bad=='sequence':self.frames[2][0]['sequence']=1
            if bad=='gap':self.frames[0][0]['received_monotonic']-=.4
            if bad=='stale':
                for h,_ in self.frames:h['received_monotonic']-=1
            if bad=='camera_error':self.error='USB fault'
            if bad=='thread_alive':self.thread=SimpleNamespace(is_alive=lambda:True)
    monkeypatch.setattr(live,'DepthCamera',Bad)
    with pytest.raises(RuntimeError):live.capture_current(tmp_path,rig.Worker(tmp_path))
    assert rig.cameras[0].closed
    assert not (tmp_path/'measured_start.json').exists()


def fake_plan(folder,state_path,scene_path,cal_path):
    folder.mkdir()
    state=common.read_state(state_path)
    result=dict(status='planned',start_state=state,provenance=dict(state=str(state_path),scene=str(scene_path)),
        physical_execution_blockers=['Actual resting and static left-arm states must be measured','physical model'],
        motion_authorized=False,hardware_execution_available=False)
    common.atomic_json(folder/'plan.json',result)
    return result


def test_pipeline_passes_exact_capture_to_planner_then_closes_devices(tmp_path,rig,monkeypatch):
    seen=[]
    def planner(*a):
        seen.append(a)
        assert not rig.workers[0].closed and rig.cameras[0].closed
        return fake_plan(*a)
    monkeypatch.setattr(planning,'plan',planner)
    p=live.run(tmp_path/'run',headless=True)
    result=json.loads(p.read_text())
    assert len(seen)==1 and len(seen[0])==4 # no occupied-only override
    assert result['actual_current_robot_state_verified'] and not result['motion_authorized']
    assert result['physical_execution_blockers']==['physical model']
    assert rig.workers[0].closed
    assert json.loads((p.parent.parent/'current_plan_report.json').read_text())['passed']


@pytest.mark.parametrize('bad',['drift','feedback_error','stale'])
def test_changed_state_invalidates_even_successful_plan(tmp_path,rig,monkeypatch,bad):
    def planner(*a):
        result=fake_plan(*a)
        if bad=='drift':rig.workers[0].drift=.05
        if bad=='feedback_error':rig.workers[0].error='Lost feedback'
        if bad=='stale':
            old=rig.workers[0].samples();rig.workers[0].samples=lambda:old;rig.clock.t+=1
        return result
    monkeypatch.setattr(planning,'plan',planner)
    with pytest.raises((ValueError,RuntimeError)):live.run(tmp_path/'run',headless=True)
    result=json.loads((tmp_path/'run/plan/plan.json').read_text())
    assert result['status']=='invalidated_acquisition'
    assert not result['actual_current_robot_state_verified'] and rig.workers[0].closed


def test_planner_exception_releases_query_ownership(tmp_path,rig,monkeypatch):
    def fail(*a):raise RuntimeError('MoveIt failed')
    monkeypatch.setattr(planning,'plan',fail)
    with pytest.raises(RuntimeError,match='MoveIt failed'):live.run(tmp_path/'run',headless=True)
    assert rig.workers[0].closed
    report=json.loads((tmp_path/'run/current_plan_report.json').read_text())
    assert not report['passed'] and report['motor_commands']==0


def test_real_moveit_checks_measured_start_and_matching_scene(tmp_path,rig):
    p=live.run(tmp_path/'run',headless=True)
    result=json.loads(p.read_text());req=json.loads((p.parent/'backend_request.json').read_text())
    assert req['start']==rig.state['right_rad'] and req['left']==rig.state['left_rad']
    assert req['goal']==common.endpoint().tolist()
    assert not req['occupied_only_preview'] and not req['scene']['synthetic']
    # This artificial depth image cannot certify the real start. Preserve the
    # block, never fall back to synthetic/occupied-only/recorded trajectories.
    assert result['status'] in ('blocked_start','blocked_goal','no_path','blocked_after_smoothing')
    from drum_planner.trajectory_view import PlanDisplay
    display=PlanDisplay(p);packet,mesh=planned_view.load(display,common.DEFAULT_CALIBRATION)
    assert not display.playable and len(mesh.faces)>0 and packet.sequence==5
    with (p.parent.parent/'camera/depth_m.bin').open('ab') as f:f.write(b'bad')
    with pytest.raises(ValueError,match='hash'):planned_view.load(display,common.DEFAULT_CALIBRATION)


def test_cli_defaults_preserved_and_no_unknown_space_override():
    with pytest.raises(SystemExit):cli.main(['live','--occupied-only-preview'])
    assert cli.main(['live','--execute'])==1  # requires review before any device access


def test_headless_rejects_display_timer_before_devices(tmp_path,rig):
    with pytest.raises(ValueError):live.run(tmp_path/'run',headless=True,seconds=2)
    assert not rig.workers and not (tmp_path/'run').exists()
