"""Workspace hardware selections/mission playback, exclusively fake devices."""
from test_checkpoint_performance import OPTIMIZED
import copy,json,socket,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from test_experiment_checkpoints import sequence
from drum_planner.common import atomic_json
from drum_planner.experiment import checkpoints,config
from drum_planner.baseline import paths,motion
from drum_planner.baseline.curve import Curve
from drum_planner.baseline.sequence import PlanDisplay
from drum_planner.baseline.simulation import SimulatedBus


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    old=socket.socket
    def guard(family=socket.AF_INET,*a,**kw):
        if family==socket.AF_CAN:raise AssertionError('No physical CAN during software verification')
        return old(family,*a,**kw)
    monkeypatch.setattr(socket,'socket',guard)
    from drum_planner import depth
    monkeypatch.setattr(depth,'DepthCamera',lambda *a,**kw:pytest.fail('No actual camera permitted'))
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')


@pytest.fixture
def mission(sequence):
    context,leg,_,folder=sequence
    obj=checkpoints.plan_sequence(context,folder,'workspace_curve',config.settings(),5,leg)
    req=json.loads((folder/'backend_request.json').read_text());req['execution_joint_limits']=[[-4,4]]*7
    return obj,req,folder/'plan.json'


@pytest.mark.parametrize('rate',[.25,.6,1.])
def test_sequence_preview_exact_clock_and_fixed_holds(mission,rate):
    from drum_planner.baseline.windows import PlanWindow
    from PyQt5.QtWidgets import QApplication
    app=QApplication.instance() or QApplication([])
    obj,req,path=mission;curve=Curve(obj);paths.validate_timing(curve)
    p=PlanDisplay(path);window=PlanWindow(p,1.)
    try:
        window.install_plan(p);window.rate.setValue(rate);display=window.plan
        assert display.duration==pytest.approx(4*1.7/rate+6.)
        for t in np.linspace(0,display.duration,100):
            assert np.allclose(display.positions(t),curve.at(display.source_time(t)),atol=1e-10)
        for i,(start,arrival,source_start,source_arrival) in enumerate(display.display_parts):
            if i==3:break
            for dt in (.001,.5,1.999):
                assert np.allclose(display.positions(arrival+dt),curve.at(source_arrival))
                assert 'HOLD' in display.phase(arrival+dt)
        assert np.allclose(display.positions(display.duration),req['start'])
        assert p.duration==pytest.approx(12.8) and p.preview_rate==1.
    finally:window.shutdown()


def test_mission_batched_preparation_keeps_every_scalar_target_bit_exact(mission):
    obj,_,_=mission;curve=Curve(obj)
    times=np.r_[np.arange(0,curve.duration,.002),curve.duration]
    assert np.array_equal(curve.at_many(times),np.array([curve.at(t) for t in times]))


@pytest.mark.parametrize('rate',[.25,1.])
@pytest.mark.parametrize('stop',['none','second_leg','checkpoint_hold','return'])
def test_normal_sequence_and_fault_recovery_only_release_at_original_start(mission,rate,stop):
    obj,req,_=mission;clock=SimpleNamespace(t=100.)
    bus=SimulatedBus(req,now=lambda:clock.t);c=motion.Controller(bus,obj,req,now=lambda:clock.t,rate=rate)
    def tick():clock.t+=.01;c.step()
    def until(predicate):
        for _ in range(40000):
            if predicate():return
            assert c.phase not in motion.TERMINAL,(c.phase,c.error)
            tick()
        pytest.fail('Mission did not complete')
    c.start()
    if stop!='none':
        predicates={'second_leg':lambda:c.phase=='OUTWARD' and c.leg_index==1 and c.tau>4.,
                    'checkpoint_hold':lambda:c.phase=='CHECKPOINT_HOLD' and c.leg_index==1,
                    'return':lambda:c.phase=='RETURN' and c.tau>11.5}
        until(predicates[stop]);c.halt('injected test fault',fault=True)
        for _ in range(80):tick()
        assert c.hold_confirmed and not bus.releases
        request=c.request_return(True)
        spec=paths.recovery(c.curve,request['held'],request['tau'],rate)
        spec['proof']={'all_valid':True};spec[c.snapshot_key]=request[c.snapshot_key]
        for _ in range(70):tick()
        assert c.accept_return(spec,request['generation'])
        until(lambda:c.phase=='RELAXED')
    else:until(lambda:c.phase=='RELAXED')
    assert bus.releases==1 and np.allclose(bus.q,req['start'],atol=1e-10)
    if stop=='none':
        events=list(c.events);holds=[(i,e) for i,e in enumerate(events) if e['phase']=='CHECKPOINT_HOLD']
        assert len(holds)==3
        for i,event in holds:
            assert events[i+1]['monotonic']-event['monotonic']==pytest.approx(2.,abs=.015)
        assert sum(e['phase']=='RETURN' for e in events)==1
        assert c.tau==pytest.approx(c.curve.duration)


def test_mission_law_tamper_and_incomplete_return_rejected(mission):
    obj,_,_=mission
    bad=copy.deepcopy(obj);bad['cycle'][-20]['positions_rad'][0]+=.02
    with pytest.raises(ValueError,match='samples'):Curve(bad)
    bad=copy.deepcopy(obj);bad['checkpoint_sequence']['return_leg']['goal'][0]+=.01
    with pytest.raises(ValueError):Curve(bad)
    bad=copy.deepcopy(obj);bad['checkpoint_sequence']['segments'][1]['start_time_s']+=.01
    with pytest.raises(ValueError,match='mission'):Curve(bad)


def test_hardware_flag_and_arm_aliases_dispatch_only_to_workspace(monkeypatch,tmp_path):
    from drum_planner import cli
    from drum_planner.baseline import workspace
    calls=[];monkeypatch.setattr(workspace,'show',lambda *a,**kw:calls.append(kw))
    for i,command in enumerate(['--hardware','hardware','singlearc','baseline']):
        assert cli.main([command,'--left','--output',str(tmp_path/str(i))])==0
        assert calls[-1]['moving_side']=='left'
    assert len(calls)==4


def test_selector_changes_require_calculate_and_cannot_change_active_run(mission,tmp_path,monkeypatch):
    from drum_planner.baseline.workspace import Workspace
    from drum_planner.experiment import destinations
    obj,req,plan=mission
    # Current simulated fixture destinations only; never modify the user's files.
    files=[Path(s['destination_source']) for s in obj['checkpoint_sequence']['segments']]
    monkeypatch.setattr(destinations,'files',lambda side:files)
    v=Workspace(tmp_path/'ui',simulate_plan=plan,autoscene=False,moving_side=req.get('moving_side','right'))
    try:
        v.poll();selector=v.control.destinations
        assert v.plan is None and v.job is None
        selector.boxes[0].setCurrentIndex(2)
        assert v.job is None and not v.plan_valid
        selector.add.click();assert len(selector.boxes)==2 and v.job is None
        assert selector.boxes[1].findData(selector.boxes[0].currentData())<0
        selector.boxes[1].setCurrentIndex(1);selector.add.click()
        assert len(selector.boxes)==3 and not selector.add.isEnabled() and v.job is None
        selector.boxes[2].setCurrentIndex(1)
        captured=[];monkeypatch.setattr(v,'launch_simulation_job',lambda anchor,binding:captured.append(binding))
        v.control.buttons['calculate'].click();assert len(captured)==1
        assert len(captured[0]['checkpoints'])==3
        before=selector.paths();v.owner_info={'fake':True};v.start_sent=True
        selector.boxes[0].setCurrentIndex(0)
        assert selector.paths()==before and all(b.isEnabled() for b in v.control.buttons.values())
        v.owner_info=None;v.start_sent=False
        selector.remove.click();assert len(selector.boxes)==2 and len(captured)==1
    finally:v.owner_info=None;v.close()


def test_two_checkpoints_use_one_new_return(sequence):
    context,leg,_,folder=sequence
    context['checkpoints']=context['checkpoints'][:2]
    context['checkpoint_sha256']=checkpoints.signature(context['checkpoints'])
    obj=checkpoints.plan_sequence(context,folder,'workspace_curve',config.settings(),5,leg)
    c=Curve(obj);paths.validate_timing(c)
    assert len(c.parts)==3 and c.duration==pytest.approx(3*1.7+4)
    assert np.allclose(c.at(c.duration),obj['outward'][0]['positions_rad'])


def test_new_destination_packet_integrity_and_goal_binding(mission,sequence,tmp_path,monkeypatch):
    # Test-only acquisition flags in an ephemeral fixture, always revoked. CAN
    # is blocked by the autouse fixture. Native full-curve proof is tested by
    # hardware_workspace_ui_check, not by this packet-plumbing mock.
    from drum_planner.baseline import tasks
    from drum_planner.common import DEFAULT_CALIBRATION,digest
    obj,req,path=mission;context,_,_,_=sequence
    original=copy.deepcopy(obj)
    capture=tmp_path/'capture';capture.mkdir()
    state_path=Path(context['state']);state=json.loads(state_path.read_text())
    state.update(measured=True,offline_test_fixture=True)
    atomic_json(capture/'measured_start.json',state)
    obj['provenance'].update(state=str(capture/'measured_start.json'),state_sha256=digest(capture/'measured_start.json'))
    (capture/'camera').symlink_to(Path(context['scene']).parent,target_is_directory=True)
    obj.update(baseline_hardware_session=True,actual_current_robot_state_verified=True,experiment_only=False,
               offline_test_fixture=True,acquisition_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
               destination_bindings=context['checkpoints'])
    atomic_json(path,obj)
    monkeypatch.setattr(tasks,'hard_check',lambda r,q,f:dict(samples=len(q),all_valid=True,**tasks.margin_contract(r)))
    try:
        tasks.prepare(tmp_path/'checked',path,capture,context['calibration'],allow_replan=False)
        packet,c=tasks.check_packet(tmp_path/'checked/prepared.json')
        assert len(c.parts)==4 and np.allclose(c.at(c.duration),req['start'])
        assert packet['proof']['samples']==len(np.arange(0,c.duration,.002))+1
        Path(context['checkpoints'][1]['goal_recording']).write_text('{}')
        with pytest.raises(ValueError,match='snapshot changed'):tasks.load_checked(path,context['calibration'])
    finally:atomic_json(path,original)


def test_shared_generators_timing_models_and_user_data_preserved():
    from drum_planner.common import ROOT,digest
    before=json.loads((ROOT/'diagnostics/hardware_workspace_20261022/before_hashes.json').read_text())
    changed={'drum_planner/cli.py'}|{'drum_planner/baseline/'+n+'.py' for n in
        ('__init__','planner','curve','motion','tasks','workspace','windows','work','computation','service','transport')}
    # Approved 5 mm map wiring; test_depth_resolution protects all generation/timing/data.
    changed.update({'drum_planner/depth.py','drum_planner/planning.py','drum_planner/cpp/moveit_plan.cpp','drum_planner/experiment/worker.py','drum_planner/experiment/algorithms.py','drum_planner/baseline/work.py','drum_planner/live_view.py'})
    for path,sha in before.items():
        if path not in changed|OPTIMIZED:assert digest(ROOT/path)==sha,path


def test_owner_preloads_exact_law_before_start_without_opening_transport(mission,tmp_path,monkeypatch):
    from drum_planner.baseline import service,simulation
    obj,req,path=mission
    server=service.Server.__new__(service.Server)
    server.info=dict(plan=str(path),simulated=True,rate=1.,moving_side=req.get('moving_side','right'))
    server.side=req.get('moving_side','right');server.folder=tmp_path;server.engine=None;server.bus=None
    server.start_error='';server.event=lambda *_:None
    opened=[]
    def fake_bus(request):
        opened.append(True);return SimulatedBus(request)
    monkeypatch.setattr(simulation,'SimulatedBus',fake_bus)
    server.prepare_motion()
    assert not opened and server.bus is None
    curve=server.motion_data[2]
    # The authenticated Start/heartbeat loop must not redo expensive polynomial
    # validation for all four legs; it uses the validated immutable law instead.
    monkeypatch.setattr(motion,'Curve',lambda *_:pytest.fail('Heavy curve rebuild on Start'))
    server.command(dict(op='start',confirmation='MOVE '+server.side.upper()+' ARM'))
    assert opened==[True] and server.engine.curve is curve
    assert server.engine.phase=='PREFLIGHT' and not server.engine.started
    assert server.bus.releases==0


def test_owner_rejects_changed_preloaded_input_before_transport(mission,tmp_path,monkeypatch):
    from drum_planner.baseline import service,simulation
    _,req,path=mission
    server=service.Server.__new__(service.Server)
    server.info=dict(plan=str(path),simulated=True,rate=1.,moving_side=req.get('moving_side','right'))
    server.side=req.get('moving_side','right');server.folder=tmp_path;server.engine=None;server.bus=None
    server.start_error='';server.prepare_motion()
    monkeypatch.setattr(simulation,'SimulatedBus',lambda *_:pytest.fail('Changed plan must not open bus'))
    path.write_text(path.read_text()+'\n')
    with pytest.raises(ValueError,match='inputs changed'):
        server.command(dict(op='start',confirmation='MOVE '+server.side.upper()+' ARM'))
    assert server.engine is None and server.bus is None


def test_owner_validation_finishes_before_accepting_ipc(tmp_path,monkeypatch):
    from drum_planner.baseline import service
    events=[]
    monkeypatch.setattr(service.Server,'prepare_motion',lambda _:events.append('validated'))
    class Socket:
        def __init__(self,*_):events.append('socket')
        def bind(self,path):assert events==['validated','socket'];Path(path).touch()
        def listen(self,*_):events.append('listening')
        def setblocking(self,*_):pass
    monkeypatch.setattr(service.socket,'socket',Socket)
    monkeypatch.setattr(service.signal,'signal',lambda *_:None)
    server=service.Server(dict(folder=str(tmp_path),socket=str(tmp_path/'ipc'),simulated=False,moving_side='right'))
    try:assert events==['validated','socket','listening'] and server.bus is None
    finally:server.log.close()
