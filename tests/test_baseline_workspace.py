"""New production baseline UI. All CAN forbidden; no physical motor commands."""
import copy,json,socket,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from drum_planner.common import ROOT,JOINTS,LEFT_JOINTS,DEFAULT_CALIBRATION,atomic_json
from drum_planner.baseline import feedback

PLAN=ROOT/'planner_runs/20261019T225936Z-baseline-a256d6/plan/result/plan.json'

@pytest.fixture(autouse=True)
def no_can(monkeypatch):
    old=socket.socket
    def guard(family=socket.AF_INET,*a,**k):
        if family==socket.AF_CAN:raise AssertionError('No physical CAN in UI tests')
        return old(family,*a,**k)
    monkeypatch.setattr(socket,'socket',guard)
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')

@pytest.fixture
def view(tmp_path,monkeypatch):
    from drum_planner.baseline.workspace import Workspace
    from drum_planner import snapshot_preview
    from drum_planner.baseline import workspace
    # Keep tests small; native saved real-scene GUI test covers full mesh too.
    packet,mesh=snapshot_preview.prepare_scene(dict(sequence=5,received_monotonic=time.monotonic(),value_scale_m=.001),
        np.full((40,48),1000,dtype=np.uint16),{'depth':{'K':[[500,0,20],[0,500,16],[0,0,1]]}},np.eye(4))
    monkeypatch.setattr(workspace,'load_scene',lambda *a:(packet,mesh))
    v=Workspace(tmp_path,simulate_plan=PLAN,autoscene=False)
    v.poll()
    try:yield v
    finally:v.close()

def enabled(v):assert all(b.isEnabled() for b in v.control.buttons.values())


def test_exactly_three_windows_and_no_automatic_plan(view):
    v=view
    assert len([w for w in v.app.topLevelWidgets() if w.isVisible()])==3
    assert v.plan is None and v.preview.plan is None and v.job is None and not v.owner_info
    assert set(v.control.buttons)=={'calculate','start','hold','reverse','cancel'}
    for b in v.control.buttons.values():assert b.window() is v.control
    enabled(v)


def test_live_motion_does_not_change_disconnected_window(view):
    v=view;q=v.preview.robot.last_positions.copy()
    v.sim_joints[JOINTS[1]]+=.2;v.poll()
    assert not np.allclose(v.live.robot.last_positions,q)
    assert np.array_equal(v.preview.robot.last_positions,q)
    v.action(v.calculate);v.preview.seek(1.);q=v.preview.robot.last_positions.copy()
    v.sim_joints[JOINTS[2]]+=.2;v.poll()
    assert np.array_equal(v.preview.robot.last_positions,q)
    assert not v.plan_valid and 'moved since Calculate' in v.control.status.text()
    enabled(v)


def test_preview_play_pause_seek_speed_never_commands_or_moves_live_model(view,monkeypatch):
    v=view;v.action(v.calculate)
    pose=v.live.robot.last_positions.copy();duration=v.preview.plan.duration
    monkeypatch.setattr(v,'launch_owner',lambda:pytest.fail('Preview must never launch motors'))
    v.preview.rate.setValue(.5)
    assert v.preview.plan.duration==pytest.approx(2*(duration-2)+2)
    v.preview.play.click();assert v.preview.clock.playing
    v.preview.pause.click();assert not v.preview.clock.playing
    v.preview.timeline.setValue(round(v.preview.plan.obj['outward'][-1]['time_s']*1000))
    assert np.allclose(v.preview.robot.last_positions,v.plan.req['goal'],atol=.001)
    assert np.array_equal(v.live.robot.last_positions,pose)
    enabled(v)


@pytest.mark.parametrize('action',['start','hold','reverse'])
def test_missing_prerequisites_explain_without_disabling_or_throwing(view,action):
    v=view;v.action(getattr(v,action));enabled(v)
    assert v.control.status.text()
    v.action(v.calculate);assert v.plan_valid
    enabled(v)


def test_failed_movement_then_calculate_again_same_process(view):
    v=view;original=copy.deepcopy(v.sim_joints)
    v.action(v.calculate);v.sim_joints[JOINTS[0]]+=.02;v.poll()
    assert not v.plan_valid and v.anchor is None
    v.sim_joints=original;v.poll();assert not v.plan_valid
    v.action(v.calculate);assert v.plan_valid;enabled(v)


def test_start_sent_gap_is_active_even_with_old_ready_status(view):
    v=view;v.owner_info={'test':True};v.owner_status={'phase':'READY','safe_to_close':True};v.start_sent=True
    assert v.active
    v.action(v.calculate);assert v.owner_info and v.plan is None
    v.start_sent=False;v.owner_info=None


class Client:
    def __init__(self,status):self.status=status;self.messages=[];self.sent=[];self.closed=False
    def send(self,op,**kw):self.sent.append((op,kw))
    def poll(self):return self.status
    def close(self):self.closed=True

@pytest.mark.parametrize('phase',['RELAXED','REJECTED','DISABLED_EXTERNALLY'])
def test_finished_owner_is_released_and_calculation_can_restart(view,phase):
    v=view;v.action(v.calculate)
    client=Client(dict(phase=phase,safe_to_close=True,error='test rejection' if phase=='REJECTED' else ''))
    v.owner_info={'old':True};v.client=client;v.start_sent=True
    v.poll_owner(time.monotonic())
    assert client.closed and v.client is None and v.owner_info is None and not v.active
    v.action(v.calculate);assert v.plan_valid;enabled(v)


def test_reverse_is_one_click_hold_then_return_after_confirmation(view):
    v=view;c=Client(dict(phase='OUTWARD',safe_to_close=False));v.client=c;v.owner_info={'active':True}
    v.action(v.reverse);assert c.sent[-1][0]=='hold' and v.reverse_intent
    c.status=dict(phase='HELD',safe_to_close=False,hold_confirmed=True)
    v.poll_owner(v.reverse_intent['after']+.01)
    assert [op for op,_ in c.sent]==['hold','reverse'] and v.reverse_intent is None
    assert not any(op=='relax' for op,_ in c.sent)
    enabled(v)


def test_reverse_fault_does_not_queue_motion_after_communication_returns(view):
    v=view;c=Client(dict(phase='FAULT_HOLD',safe_to_close=False,hold_confirmed=False));v.client=c;v.owner_info={'active':True}
    v.action(v.reverse);assert c.sent[-1][0]=='retry_hold'
    c.messages=[dict(reply='retry_hold',error='CAN fault')];v.poll_owner(time.monotonic())
    assert v.reverse_intent is None
    c.status['hold_confirmed']=True;v.poll_owner(time.monotonic()+3)
    assert not any(op=='reverse' for op,_ in c.sent);enabled(v)
    v.action(v.reverse);v.poll_owner(v.reverse_intent['after']+.01)
    assert c.sent[-1][0]=='reverse'


def test_worker_failure_is_retryable_without_restarting(view,tmp_path):
    v=view;folder=tmp_path/'failed';folder.mkdir();atomic_json(folder/'error.json',dict(error='Test planning failure'))
    closed=[];v.job=SimpleNamespace(folder=folder,poll=lambda:1,close=lambda:closed.append(True),log_path=folder/'worker.log')
    v.job_kind='calculate';v.want_start=True
    v.poll_job();assert v.job is None and not v.want_start and closed and not v.plan_valid
    v.action(v.calculate);assert v.plan_valid;enabled(v)


def test_calculation_guard_latches_out_and_back_motion_in_reader_thread(tmp_path):
    joints=dict.fromkeys(JOINTS+LEFT_JOINTS,0.)|{'openarmx_left_finger_joint1':0.}
    anchor=feedback.normalized_pose(joints,time.monotonic())
    class Observer:
        def __init__(self,folder):self.i=0
        def sample(self):
            self.i+=1;q=joints.copy()
            if self.i==3:q[JOINTS[0]]=.1
            return dict(start=time.monotonic(),joints=q)
        def close(self):pass
        def report(self):return {}
    r=feedback.DisplayTelemetry(tmp_path,Observer);r.set_guard(anchor)
    try:
        deadline=time.monotonic()+2
        while r.count<5 and time.monotonic()<deadline:time.sleep(.01)
        assert r.snapshot()['joints'][JOINTS[0]]==0 and 'moved' in r.movement_error()
        r.set_guard(None);assert not r.movement_error()
    finally:r.close()


def test_monitored_worker_publishes_motion_failure_and_closes_query_only(tmp_path,monkeypatch):
    from drum_planner.baseline.work import MonitoredObserver
    from drum_planner.read_only import ReadOnlyObserver
    joints=dict.fromkeys(JOINTS+LEFT_JOINTS,0.)|{'openarmx_left_finger_joint1':0.}
    anchor=feedback.normalized_pose(joints,time.monotonic());q=joints.copy();q[JOINTS[0]]=.1
    monkeypatch.setattr(ReadOnlyObserver,'__init__',lambda *a,**k:None)
    monkeypatch.setattr(ReadOnlyObserver,'sample',lambda *a:dict(start=time.monotonic(),joints=q))
    out=tmp_path/'feedback.json';observer=MonitoredObserver(tmp_path,anchor,out)
    with pytest.raises(ValueError,match='moved'):observer.sample()
    assert 'moved' in json.loads(out.read_text())['error']


def test_start_check_never_replans_a_different_route(monkeypatch,tmp_path):
    from drum_planner.baseline import work
    # Assert the production wrapper opts out of the older prepare auto-replan.
    import inspect
    assert 'allow_replan=False' in inspect.getsource(work.calculate)


def test_cli_no_longer_calls_automatic_initial_planner(monkeypatch,tmp_path):
    from drum_planner import cli
    from drum_planner.baseline import app,workspace
    monkeypatch.setattr(app,'initial',lambda *a:pytest.fail('Old auto-planning entry point must not run'))
    calls=[];monkeypatch.setattr(workspace,'show',lambda *a,**k:calls.append((a,k)))
    assert cli.main(['baseline','--output',str(tmp_path/'run')])==0
    assert calls and calls[0][1]['simulate_plan'] is None


def test_cancel_work_does_not_change_motor_state_and_stale_completion_cannot_start(view,tmp_path):
    v=view;closed=[];folder=tmp_path/'cancel';folder.mkdir()
    v.job=SimpleNamespace(folder=folder,close=lambda:closed.append(True));v.job_kind='check';v.want_start=True
    v.action(v.cancel_calculation)
    assert closed and v.job is None and not v.want_start and v.anchor is None
    v.poll_job();assert v.owner_info is None
    v.action(v.calculate);assert v.plan_valid;enabled(v)


def test_cancel_is_not_a_physical_stop_or_relax(view):
    v=view;c=Client(dict(phase='OUTWARD',safe_to_close=False));v.client=c;v.owner_info={'active':True};v.start_sent=True
    v.action(v.cancel_calculation)
    assert c.sent==[] and 'Emergency Hold' in v.control.status.text();enabled(v)


def test_hidden_previews_can_reopen_without_new_windows(view):
    v=view;v.preview.close();v.app.processEvents();assert not v.preview.isVisible()
    actions=v.control.menu.actions()[0].menu().actions();actions[0].trigger();v.app.processEvents()
    assert v.preview.isVisible() and len([w for w in v.app.topLevelWidgets() if w.isVisible()])==3


def test_close_active_workspace_sends_hold_never_relax(view):
    v=view;c=Client(dict(phase='OUTWARD',safe_to_close=False));v.client=c;v.owner_info={'active':True};v.start_sent=True
    v.close();assert c.closed and c.sent[-1][0]=='hold'
    assert not any(op=='relax' for op,_ in c.sent)


def test_disconnected_controller_never_creates_second_owner(view,monkeypatch):
    v=view;v.owner_info={'known':'owner'};v.owner_status={};v.start_sent=True
    monkeypatch.setattr(v,'connect_owner',lambda:None)
    v.poll_owner(time.monotonic());v.action(v.calculate);v.action(v.start);v.action(v.reverse)
    assert v.owner_info=={'known':'owner'} and v.client is None
    enabled(v);v.owner_info=None;v.start_sent=False


@pytest.mark.parametrize('failure',[None,'camera','planning_movement','start_check','final_batch'])
def test_guarded_work_wrapper_keeps_observer_through_planning_and_always_closes(tmp_path,monkeypatch,failure):
    from drum_planner.baseline import work
    holder={};calls=[]
    class Telemetry:
        def __init__(self,*a,**kw):self.error='';self.closed=False;holder['reader']=self
        def close(self):
            self.closed=True;calls.append('close')
            if failure=='final_batch':self.error='Arm moved during final batch'
        def report(self):return dict(error=self.error,thread_stopped=self.closed)
    monkeypatch.setattr(work,'Telemetry',Telemetry)
    def capture(folder,worker,cal,*,stationary_joints=None):
        assert stationary_joints==JOINTS
        assert not worker.closed;calls.append('capture')
        if failure=='camera':raise ValueError('camera unavailable')
        (folder/'camera').mkdir();atomic_json(folder/'camera/scene.json',dict(test=True))
        atomic_json(folder/'measured_start.json',dict(test=True))
        return {},{}
    monkeypatch.setattr(work,'capture_current',capture)
    def verify(worker,state,*,stationary_joints=None):
        assert stationary_joints==JOINTS
        assert not worker.closed
        if worker.error:raise ValueError(worker.error)
        return dict(test=True)
    monkeypatch.setattr(work,'verify_unchanged',verify)
    def make_plan(*a):
        assert not holder['reader'].closed;calls.append('plan')
        if failure=='planning_movement':holder['reader'].error='Arm moved during planning'
        return tmp_path/'test_plan.json'
    def prepare(*a,**kw):
        assert not kw['allow_replan'];calls.append('check')
        if failure=='start_check':raise ValueError('Path blocked')
    monkeypatch.setattr(work.tasks,'make_plan',make_plan);monkeypatch.setattr(work.tasks,'prepare',prepare)
    if failure:
        with pytest.raises((ValueError,RuntimeError)):work.calculate(tmp_path,DEFAULT_CALIBRATION,{},plan='test.json' if failure=='start_check' else None)
        assert not (tmp_path/'result.json').exists()
    else:
        work.calculate(tmp_path,DEFAULT_CALIBRATION,{})
        assert (tmp_path/'result.json').exists() and calls==['capture','plan','check','close']
    assert holder['reader'].closed and (tmp_path/'query_report.json').exists()


def test_failed_reconnect_closes_its_socket_every_time(monkeypatch):
    from drum_planner.baseline import service
    closed=[]
    class Sock:
        def settimeout(self,*a):pass
        def connect(self,*a):raise ConnectionRefusedError('offline')
        def close(self):closed.append(True)
    monkeypatch.setattr(service.socket,'socket',lambda *a:Sock())
    for _ in range(10):
        with pytest.raises(ConnectionRefusedError):service.Client({'socket':'absent'})
    assert len(closed)==10


def test_terminal_owner_close_failure_does_not_require_restart(view):
    v=view;c=Client(dict(phase='REJECTED',safe_to_close=True,error='Rejected start'))
    def failed(*a,**kw):raise RuntimeError('Close IPC lost')
    c.send=failed;v.client=c;v.owner_info={'test':True};v.start_sent=True
    v.poll_owner(time.monotonic());assert not v.owner_info and not v.client and c.closed
    v.action(v.calculate);assert v.plan_valid;enabled(v)


def test_repeated_identical_feedback_does_not_recompute_robot_geometry(view,monkeypatch):
    v=view;calls=[];original=v.live.set_pose
    monkeypatch.setattr(v.live,'set_pose',lambda p:calls.append(p) or original(p))
    for _ in range(10):v.set_pose(feedback.normalized_pose(v.sim_joints,time.monotonic()))
    assert not calls
    v.sim_joints[JOINTS[0]]+=.02;v.set_pose(feedback.normalized_pose(v.sim_joints,time.monotonic()))
    assert len(calls)==1


def test_long_error_messages_cannot_push_controls_off_window(view):
    from PyQt5 import QtCore
    v=view;v.control.resize(380,740)
    v.message('Collision/connection failure: '+('long diagnostic detail ' * 150),error=True)
    v.control.run_status.setText('Feedback unavailable: '+('detail ' * 100));v.app.processEvents()
    rects=[QtCore.QRect(b.mapTo(v.control,QtCore.QPoint()),b.size()) for b in v.control.buttons.values()]
    for i,r in enumerate(rects):
        assert v.control.rect().contains(r)
        assert not any(r.intersects(other) for other in rects[i+1:])
    enabled(v)


@pytest.mark.parametrize('joint',LEFT_JOINTS+['openarmx_left_finger_joint1'])
def test_passive_left_movement_keeps_plan_and_only_updates_live_view(view,joint):
    v=view;v.calculate();preview=v.preview.robot.last_positions.copy()
    v.sim_joints[joint]+=.01 if 'finger' in joint else .08
    v.poll()
    assert v.plan_valid and v.anchor is not None and not feedback.movement_error(v.anchor,v.pose)
    assert np.array_equal(v.preview.robot.last_positions,preview)
    assert v.live.pose['joints'][joint]==v.sim_joints[joint]
    enabled(v)


def test_real_start_uses_calculated_packet_without_camera_or_check_worker(view,monkeypatch,tmp_path):
    v=view;v.calculate();v.set_pose(feedback.normalized_pose(v.sim_joints,time.monotonic()))
    v.simulated=False;v.prepared=tmp_path/'prepared.json'
    launched=[]
    monkeypatch.setattr(v,'launch_owner',lambda:launched.append((v.prepared,v.plan.path,v.start_rate)))
    monkeypatch.setattr(v,'launch_job',lambda *a:pytest.fail('Start must not capture or recheck a new scene'))
    v.start()
    assert launched==[(v.prepared,v.plan.path,v.preview.rate.value())]
    assert v.job is None and v.plan_valid and v.want_start
    enabled(v)


def test_calculate_result_keeps_checked_packet_for_direct_start(view,tmp_path,monkeypatch):
    v=view;v.calculate();v.plan_valid=False
    atomic_json(tmp_path/'result.json',dict(plan=str(PLAN),prepared=str(tmp_path/'checked/prepared.json'),scene='test_scene'))
    v.job=SimpleNamespace(folder=tmp_path,poll=lambda:0,close=lambda:None);v.job_kind='calculate'
    monkeypatch.setattr(v,'set_scene',lambda *a:None)
    v.poll_job()
    assert v.plan_valid and v.prepared==tmp_path/'checked/prepared.json' and v.job is None


def test_query_monitor_reports_but_does_not_reject_passive_left_motion(tmp_path,monkeypatch):
    from drum_planner.baseline.work import MonitoredObserver
    from drum_planner.read_only import ReadOnlyObserver
    joints=dict.fromkeys(JOINTS+LEFT_JOINTS,0.)|{'openarmx_left_finger_joint1':0.}
    anchor=feedback.normalized_pose(joints,time.monotonic());q=joints.copy()
    q[LEFT_JOINTS[0]]=.12;q['openarmx_left_finger_joint1']=.005
    monkeypatch.setattr(ReadOnlyObserver,'__init__',lambda *a,**k:None)
    monkeypatch.setattr(ReadOnlyObserver,'sample',lambda *a:dict(start=time.monotonic(),joints=q))
    out=tmp_path/'feedback.json';observer=MonitoredObserver(tmp_path,anchor,out)
    assert observer.sample()['joints']==q
    assert not json.loads(out.read_text())['error']


def test_offline_local_pose_does_not_raise_a_fake_stale_telemetry_error(view):
    v=view;v.pose['stamp']-=5
    v.calculate();assert v.plan_valid
    v.pose['stamp']-=5
    assert time.monotonic()-v.fresh_pose()['stamp']<.1
    v.sim_joints[JOINTS[0]]+=.1;v.fresh_pose()
    assert not v.plan_valid  # still detects actual changes to the local state


def test_physical_feedback_is_never_restamped_to_hide_staleness(view):
    v=view;v.simulated=False;v.pose['stamp']-=5
    with pytest.raises(ValueError,match='No fresh arm feedback'):v.fresh_pose()
