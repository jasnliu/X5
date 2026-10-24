"""Selected-arm Single arc hardware workflow, entirely virtual wires/devices."""
from test_checkpoint_performance import OPTIMIZED
import copy,json,socket,struct,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from drum_planner import common,planning
from drum_planner.baseline import arms,feedback,motion,paths,planner,service
from drum_planner.baseline.simulation import SimulatedBus


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    original=socket.socket
    def guarded(family=socket.AF_INET,*args,**kw):
        if family==socket.AF_CAN:pytest.fail('Physical CAN forbidden')
        return original(family,*args,**kw)
    monkeypatch.setattr(socket,'socket',guarded)
    from drum_planner import depth
    monkeypatch.setattr(depth,'DepthCamera',lambda *a,**k:pytest.fail('Physical camera forbidden'))
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')


@pytest.fixture(params=['right','left'])
def arm(request):return request.param


def inputs(side):
    from drum_planner.experiment.config import saved_inputs
    state,scene=saved_inputs(side=side)
    return state,scene,common.read_state(state)


@pytest.fixture
def rig(arm):
    _,_,state=inputs(arm);other=arms.other_side(arm)
    outward,_,timing=planning.smooth_samples([state[arm+'_rad'],common.endpoint(side=arm)])
    obj=dict(outward=outward,timing=timing,moving_side=arm)
    req=dict(moving_side=arm,start=state[arm+'_rad'],left=state['left_rad'],right=state['right_rad'],
             left_opening_m=state['left_gripper_opening_m'],execution_joint_limits=[[-4,4]]*7)
    clock=SimpleNamespace(t=100.);bus=SimulatedBus(req,now=lambda:clock.t)
    engine=motion.Controller(bus,obj,req,now=lambda:clock.t)
    def tick(n=1):
        for _ in range(n):clock.t+=.01;engine.step()
    def until(phase):
        for _ in range(50000):
            if engine.phase==phase:return
            assert engine.phase not in motion.TERMINAL,(engine.phase,engine.error)
            tick()
        pytest.fail(str(engine.status()))
    return SimpleNamespace(side=arm,other=other,obj=obj,req=req,bus=bus,engine=engine,tick=tick,until=until)


@pytest.mark.parametrize('interruption',[None,'hold','CAN fault'])
def test_selected_cycle_passive_sway_and_recovery(rig,interruption):
    r=rig;c=r.engine;c.start();r.until('OUTWARD');r.tick(30)
    passive=copy.deepcopy(r.req[r.other]);r.req[r.other]=[v+.003 for v in passive];r.tick(5)
    assert c.phase=='OUTWARD' and r.bus.releases==0
    assert all(r.bus.states[r.other,i][1]==0 for i in range(1,9))
    if interruption:
        if interruption=='CAN fault':r.bus.fault='CAN failure';r.tick();assert c.phase=='FAULT_HOLD'
        else:c.halt()
        r.tick(80);assert r.bus.releases==0
        r.bus.fault='';r.tick(80)
        c.retry_hold();r.tick(80)
        request=c.request_return(True);r.tick(80)
        key=r.other+'_snapshot';assert request[key][r.other]==r.req[r.other]
        spec=paths.recovery(c.curve,request['held'],request['tau'],request['rate'])
        spec.update(proof={'all_valid':True},**{key:copy.deepcopy(request[key])})
        assert c.accept_return(spec,request['generation'])
    r.until('RELAXED')
    assert r.bus.releases==1 and np.allclose(r.bus.q,r.req['start'])
    assert all(r.bus.states[r.other,i][1]==0 for i in range(1,9))
    assert c.status()['moving_side']==r.side and r.side+'_rad' in c.status()
    assert c.events[-2]['phase']=='VERIFIED_START_POSITION'


def test_active_start_guard_ignores_only_opposite_arm(arm):
    q=dict.fromkeys(common.JOINTS+common.LEFT_JOINTS,0.)|{'openarmx_left_finger_joint1':0.}
    anchor=feedback.normalized_pose(q,1.);pose=copy.deepcopy(anchor)
    pose['joints'][common.arm_joints(arms.other_side(arm))[0]]+=.4
    assert feedback.movement_error(anchor,pose,arm)==''
    pose['joints'][common.arm_joints(arm)[0]]+=.02
    assert f'{arm.capitalize()} arm moved' in feedback.movement_error(anchor,pose,arm)


def test_left_closed_target_matches_accepted_model_and_existing_playback():
    from camera_playback.left_hold import LEFT_GRIPPER_TARGET
    from drum_planner.model import LEFT_CLOSED_MOTOR_RAW_RAD
    assert arms.closed_gripper('left')==LEFT_GRIPPER_TARGET==-LEFT_CLOSED_MOTOR_RAW_RAD
    assert arms.closed_gripper('right')==pytest.approx(np.radians(7))


def test_wrong_arm_controller_never_starts(rig):
    bad=copy.deepcopy(rig.obj);bad['moving_side']=rig.other
    with pytest.raises(ValueError,match='Arm mismatch'):motion.Controller(rig.bus,bad,rig.req)
    rig.bus.control_side=rig.other
    with pytest.raises(ValueError,match='transport arm mismatch'):motion.Controller(rig.bus,rig.obj,rig.req)
    assert not rig.bus.enable_attempted


def test_real_wire_adapter_only_selected_bus_exact_grip_and_rest_release(tmp_path,monkeypatch,rig):
    from drum_planner.baseline.transport import BaselineMotors
    from safe_zone import encoder
    from centering.motors import packet,parameter,RIGHT_PLAYBACK_SPEED
    created=[];original_open=encoder.os.open
    def opened(path,*a,**kw):
        if str(path).startswith('/tmp/openarmx-cartesian-'):path=tmp_path/Path(path).name
        return original_open(path,*a,**kw)
    monkeypatch.setattr(encoder.os,'open',opened)
    class Wire:
        def __init__(self,*a,**kw):self.sent=[];self.pending=[];self.mode={i:0 for i in range(1,9)};self.targets={};created.append(self)
        def bind(self,addr):
            self.side='left' if addr[0]=='can1' else 'right'
            q=rig.req['start'] if self.side==rig.side else rig.req[rig.other]
            self.raw={i:encoder.joint_to_motor(self.side,i,v) for i,v in enumerate(q,1)}
            self.raw[8]=-arms.closed_gripper(self.side)
        def setsockopt(self,*a):pass
        def setblocking(self,*a):pass
        def close(self):pass
        def send(self,frame):
            self.sent.append(frame);cid,dlc,data=encoder.FRAME.unpack(frame);kind=(cid>>24)&31;i=cid&255
            if kind==3:self.mode[i]=2;self.raw[i]=self.targets[i]
            if kind==4:self.mode[i]=0
            if kind==18 and data[:2]==b'\x16\x70':
                self.targets[i]=struct.unpack('<f',data[4:])[0]
                if self.mode[i]==2:self.raw[i]=self.targets[i]
            if kind==17:self.pending.append(encoder.FRAME.pack(encoder.EFF|(17<<24)|(i<<8)|0xfd,8,bytes.fromhex('0570000005000000')))
            if kind==2:
                count=round((self.raw[i]+12.57)/25.14*65535)
                self.pending.append(encoder.FRAME.pack(encoder.EFF|(2<<24)|(self.mode[i]<<22)|(i<<8)|0xfd,8,
                    count.to_bytes(2,'big')+bytes.fromhex('8000800000fa')))
            return 16
        def recv(self,n):
            if not self.pending:raise BlockingIOError()
            return self.pending.pop(0)
    monkeypatch.setattr(encoder.socket,'socket',Wire)
    b=BaselineMotors(tmp_path,rig.req['start'],moving_side=rig.side)
    try:
        for _ in range(6):b.poll();time.sleep(.015)
        for _ in b.begin_arm(rig.req['start']):b.poll();time.sleep(.002)
        active=next(w for w in created if w.side==rig.side);passive=next(w for w in created if w.side==rig.other)
        # The physical bus mapping, not just displayed FK, selects motion.
        assert b.control_side==rig.side and all(f in b.queries for f in passive.sent)
        first_enable=next(i for i,f in enumerate(active.sent) if f in [packet(3,j) for j in range(1,9)])
        assert all(parameter(i,0x7016,encoder.joint_to_motor(rig.side,i,rig.req['start'][i-1]) if i<=7 else -arms.closed_gripper(rig.side))
                   in active.sent[:first_enable] for i in range(1,9))
        assert all(parameter(i,0x7017,RIGHT_PLAYBACK_SPEED) in active.sent for i in range(1,8))
        grip=[f for f in active.sent if encoder.FRAME.unpack(f)[0]&255==8 and encoder.FRAME.unpack(f)[2][:2]==b'\x16\x70']
        assert grip and set(grip)=={parameter(8,0x7016,-arms.closed_gripper(rig.side))}
        assert not any(f==packet(4,i) for f in active.sent for i in range(1,9))
        for i in range(1,9):
            for f in [packet(3,i),packet(4,i),parameter(i,0x7016,0.)]:
                with pytest.raises(RuntimeError):b.sockets[rig.other].send(f)
                with pytest.raises(RuntimeError):b._send(rig.other,f)
        with pytest.raises(RuntimeError):b.send_control(parameter(8,0x7016,0.))
        b.set_positions(np.array(rig.req['start'])+.02)
        for _ in range(6):b.poll();time.sleep(.015)
        with pytest.raises(RuntimeError,match='Not at saved rest'):b.release_at_verified_rest(rig.req['start'])
        assert not any(f==packet(4,i) for f in active.sent for i in range(1,9))
        b.set_positions(rig.req['start'])
        for _ in range(6):b.poll();time.sleep(.015)
        b.release_at_verified_rest(rig.req['start'])
        assert [f for f in active.sent if f in [packet(4,i) for i in range(1,9)]]==[packet(4,i) for i in range(1,9)]
        assert all(f in b.queries for f in passive.sent)
    finally:b.close()


def test_fresh_startup_models_and_controls_match_side(tmp_path,monkeypatch,arm):
    from drum_planner.baseline import workspace
    from PyQt5 import QtCore
    monkeypatch.setattr(workspace,'DisplayTelemetry',lambda *a,**k:pytest.fail('No physical reads'))
    v=workspace.Workspace(tmp_path,autoscene=False,moving_side=arm)
    try:
        assert v.plan is None and v.job is None and v.owner_info is None
        for w in (v.preview,v.live):
            m=w.robot.plan
            assert m.side==arm and m.joints==common.arm_joints(arm) and m.stick_link==arm+'_drumstick'
        assert arm.upper() in v.control.windowTitle() and arm in v.control.buttons['start'].text()
        assert len([w for w in v.windows if w.isVisible()])==3
        assert all(b.isEnabled() for b in v.control.buttons.values())
        if arm=='left':assert v.preview.robot.plan.obj['model']['left_tool_geometry'].startswith('traditional')
    finally:
        v.close();QtCore.QCoreApplication.sendPostedEvents(None,QtCore.QEvent.DeferredDelete)


@pytest.mark.parametrize('command',['singlearc','baseline'])
def test_cli_side_alias_and_default_without_devices(tmp_path,monkeypatch,command,arm):
    from drum_planner import cli
    from drum_planner.baseline import workspace
    calls=[];monkeypatch.setattr(workspace,'show',lambda *a,**kw:calls.append((a,kw)))
    assert cli.main([command,'--'+arm,'--output',str(tmp_path/'session')])==0
    assert calls[0][1]['moving_side']==arm


def test_conflicting_cli_arms_are_rejected_before_startup(tmp_path):
    from drum_planner import cli
    with pytest.raises(SystemExit):cli.main(['singlearc','--left','--right','--output',str(tmp_path)])


def test_resume_side_binding_and_legacy_default():
    assert arms.side_of({})=='right'
    assert arms.side_of({'moving_side':'left'})=='left'
    with pytest.raises(ValueError,match='Arm mismatch'):arms.require_side({'moving_side':'left'},'right')
    with pytest.raises(ValueError):arms.side_of({'moving_side':'both'})


def test_backend_routes_goal_zone_model_and_timing_to_selected_arm(tmp_path,monkeypatch,arm):
    state,scene,data=inputs(arm);calls=[]
    data['measured']=True;state=tmp_path/'measured.json';common.atomic_json(state,data)
    def generate(req,context,*a):
        assert arms.side_of(req)==arm and context['moving_side']==arm
        assert req['goal']==common.endpoint(side=arm).tolist()
        assert req[arm+'_zone']['zone_file'].endswith('zone3.json' if arm=='right' else 'zone1.json')
        assert req[arm+'_zone']['tcp'].startswith('openarmx_'+arm)
        calls.append('search');return dict(path=[req['start'],req['goal']],timing={}),{'joint_limit_check':{'valid':True}}
    def timing(w,report,req,context,*a):
        assert arms.side_of(req)==arm;calls.append('timing')
        out,_,law=planning.smooth_samples(w['path'])
        return out,law,{'samples_checked':100},{'method':'test timing dispatch'}
    from drum_planner.timing import dispatch
    monkeypatch.setattr(planner.workspace_curve,'run',generate);monkeypatch.setattr(dispatch,'retime',timing)
    archive_cal=common.ROOT/'calibration_sessions/20261017T211949Z-762f9908-reprocessed/calibration.json'
    obj=planner.plan(tmp_path/'plan',state,scene,archive_cal,moving_side=arm)
    assert calls==['search','timing'] and obj['status']=='planned'
    assert obj['moving_side']==arm and obj['joint_order']==common.arm_joints(arm)
    assert obj['recording_sha256']==common.digest(common.goal_recording(arm))
    assert arm+'_zone_constraint' in obj
    assert obj['outward'][-1]['positions_rad']==common.endpoint(side=arm).tolist()
    assert not obj['actual_current_robot_state_verified'] and not obj['motion_authorized']


def test_algorithm_timing_models_recordings_and_shared_motors_preserved():
    manifest=json.loads((common.ROOT/'diagnostics/singlearc_arms_20261021/before.json').read_text())
    # Later authorized experiment destination dropdown changes only its UI and
    # endpoint routing. test_experiment_destinations checks that routing; keep
    # the historical manifest and all algorithm/timing/hardware hashes intact.
    destination_update={'drum_planner/experiment/app.py','drum_planner/experiment/worker.py'}
    # Authorized exact-output performance work; numerical parity and unchanged
    # generation policies are independently checked in test_planner_performance.
    performance_update={
        'drum_planner/experiment/algorithms.py','drum_planner/experiment/runtime.py',
        'drum_planner/timing/core.py','drum_planner/timing/adaptive.py',
        'drum_planner/timing/integration.py','drum_planner/timing/smooth_v2.py'}
    # Authorized fifth experiment choice; the new isolation audit verifies all
    # original generators, timing, native checks and actuator sources unchanged.
    workspace_update={'drum_planner/experiment/config.py','drum_planner/cpp/experiment_oracle.cpp'}
    # Approved occupancy-resolution change; separate unchanged-source audit retained.
    workspace_update.update({'drum_planner/depth.py','drum_planner/planning.py','drum_planner/cpp/moveit_plan.cpp','drum_planner/experiment/worker.py','drum_planner/experiment/algorithms.py','drum_planner/baseline/work.py','drum_planner/live_view.py'})
    for path,sha in manifest['protected'].items():
        if path not in destination_update|performance_update|workspace_update|OPTIMIZED:assert common.digest(common.ROOT/path)==sha,path


def test_guarded_worker_propagates_side_through_monitor_and_computation(tmp_path,monkeypatch,arm):
    from drum_planner.baseline import work
    holder={};calls=[]
    class Telemetry:
        def __init__(self,folder,observer_factory):
            holder['reader']=self;self.error='';self.closed=False
            holder['observer']=observer_factory(folder)
        def close(self):self.closed=True
        def report(self):return {'thread_stopped':self.closed}
    monkeypatch.setattr(work,'MonitoredObserver',lambda f,a,o,s:SimpleNamespace(side=s))
    monkeypatch.setattr(work,'Telemetry',Telemetry)
    def capture(folder,worker,cal,*,stationary_joints):
        assert stationary_joints==common.arm_joints(arm);calls.append('capture')
        (folder/'camera').mkdir();common.atomic_json(folder/'camera/scene.json',{})
        common.atomic_json(folder/'measured_start.json',{});return {},{}
    def verify(worker,state,*,stationary_joints):
        assert stationary_joints==common.arm_joints(arm);assert not worker.closed
        calls.append('verify');return {}
    def compute(folder,cal,worker,*,plan,moving_side):
        assert moving_side==arm and not worker.closed;calls.append('compute');return {}
    monkeypatch.setattr(work,'capture_current',capture);monkeypatch.setattr(work,'verify_unchanged',verify)
    monkeypatch.setattr(work.computation,'run',compute)
    work.calculate(tmp_path,common.DEFAULT_CALIBRATION,{},moving_side=arm)
    assert calls==['capture','verify','compute','verify']
    assert holder['reader'].closed and holder['observer'].side==arm


def test_launch_rejects_wrong_side_before_service_or_devices(tmp_path,monkeypatch):
    common.atomic_json(tmp_path/'backend_request.json',{'moving_side':'left'})
    monkeypatch.setattr(service.subprocess,'run',lambda *a,**kw:pytest.fail('Must not launch'))
    with pytest.raises(ValueError,match='Arm mismatch'):
        service.launch(tmp_path/'owner',simulate_plan=tmp_path/'plan.json',moving_side='right')
    assert not (tmp_path/'owner/control.json').exists()


def test_recovery_checker_uses_current_passive_side_snapshot(tmp_path,monkeypatch,rig):
    from drum_planner.baseline import tasks
    r=rig;request=copy.deepcopy(r.req);request['scene']={'obstacle_padding_m':.02}
    plan=tmp_path/'plan.json';common.atomic_json(plan,r.obj)
    packet=tmp_path/'prepared.json';common.atomic_json(packet,dict(plan=str(plan),calibration='test',fresh_scene='test',fresh_scene_sha256='scene'))
    monkeypatch.setattr(tasks,'load_checked',lambda *a:(r.obj,copy.deepcopy(request),r.engine.curve))
    monkeypatch.setattr(planning,'scene',lambda *a:{'source_scene_sha256':'scene'})
    passive={r.other:(np.asarray(r.req[r.other])+.01).tolist()}
    if r.other=='left':passive['left_opening_m']=.002
    seen=[]
    def check(req,q,folder):
        seen.append(req);assert req[r.other]==passive[r.other]
        assert req['start']==r.req['start'];return {'all_valid':True}
    monkeypatch.setattr(tasks,'hard_check',check)
    tau=r.engine.curve.duration*.3
    tasks.check_return(tmp_path/'return',packet,r.engine.curve.at(tau),tau,1.,**{r.other+'_snapshot':passive})
    result=json.loads((tmp_path/'return/return.json').read_text())
    assert result[r.other+'_snapshot']==passive and result['proof']['all_valid'] and len(seen)==1
    with pytest.raises(ValueError,match='wrong passive arm'):
        tasks.check_return(tmp_path/'wrong',packet,r.engine.curve.at(tau),tau,1.,**{r.side+'_snapshot':passive})


def test_owner_rejects_wrong_arm_start_authorization_before_transport(arm):
    server=service.Server.__new__(service.Server);server.info={'moving_side':arm};server.engine=None
    with pytest.raises(ValueError,match='Explicit MOVE'):
        server.command(dict(op='start',confirmation=f'MOVE {arms.other_side(arm).upper()} ARM'))
    assert server.engine is None
