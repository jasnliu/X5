"""Offline execution policy, exact polynomials and byte-level CSP transport tests.
No physical motor or camera is opened. Simulated tracking is not hardware proof.
"""
import copy
import io
import json
import socket
from types import SimpleNamespace

import numpy as np
import pytest

from drum_planner import common, planning, execution as ex, motor_transport as mt
from centering.motors import packet,parameter,RIGHT_GRIPPER_CLOSED
from safe_zone import encoder


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    original=socket.socket
    def guarded(family=socket.AF_INET,*a,**kw):
        if family==socket.AF_CAN:raise AssertionError('No real CAN in execution tests')
        return original(family,*a,**kw)
    monkeypatch.setattr(socket,'socket',guarded)


@pytest.fixture
def route():
    state=common.read_state(common.ROOT/'drum_planner/config/example_start.json')
    points,_,timing=planning.smooth_samples([state['right_rad'],common.endpoint().tolist()])
    obj=dict(outward=points,timing=timing)
    return obj,dict(start=state['right_rad'],left=state['left_rad'],left_opening_m=0.,
                    execution_joint_limits=[[-3.5,3.5]]*7)


@pytest.mark.parametrize('corners',[False,True])
def test_exact_polynomials_replay_saved_samples(corners,route):
    obj,req=route
    waypoints=[req['start'],((np.array(req['start'])+common.endpoint())/2).tolist(),common.endpoint().tolist()]
    points,_,timing=planning.smooth_samples(waypoints,stop_at_corners=corners)
    c=ex.Curve(dict(outward=points,timing=timing))
    assert np.allclose(c.at(0),req['start'],atol=1e-12)
    assert np.allclose(c.at(c.duration),common.endpoint(),atol=1e-12)
    for p in points:assert np.allclose(c.at(p['time_s']),p['positions_rad'],atol=1e-10)
    points[3]['positions_rad'][0]+=.01
    with pytest.raises(ValueError,match='differs'):ex.Curve(dict(outward=points,timing=timing))


def gate():
    bus=mt.PlannerMotors.__new__(mt.PlannerMotors)
    bus.expected=np.zeros(7);bus.active=True;bus.arming=False;bus.release_permitted=False
    bus.queries=frozenset(encoder.request_frame(i) for i in range(1,9));bus.audit=lambda *a,**k:None
    return bus


def test_wire_gate_closed_only_no_mit_zero_gain_or_blind_relax():
    b=gate()
    assert b.command_allowed(parameter(8,0x7016,-RIGHT_GRIPPER_CLOSED))
    for target in (0.,np.radians(3),-np.radians(6),float('nan')):
        assert not b.command_allowed(parameter(8,0x7016,target))
    for i in range(1,9):
        for frame in (packet(4,i),packet(3,i),packet(6,i),packet(1,i),parameter(i,0x701e,20.)):
            assert not b.command_allowed(frame)
    assert not b.command_allowed(parameter(1,0x7016,.1))
    assert b.command_allowed(parameter(5,0x7016,encoder.joint_to_motor('right',5,0.)))
    with pytest.raises(RuntimeError):b.relax()
    b.release_permitted=True;assert b.command_allowed(packet(4,8))


def test_left_bus_gate_only_queries_and_closing_sends_nothing():
    sent=[];sock=SimpleNamespace(send=lambda f:sent.append(f) or 16,close=lambda:None)
    g=mt.GateSocket(sock,'left',gate())
    g.send(encoder.request_frame(3))
    with pytest.raises(RuntimeError):g.send(parameter(8,0x7016,-RIGHT_GRIPPER_CLOSED))
    with pytest.raises(RuntimeError):g.send(packet(4,1))
    g.close();assert sent==[encoder.request_frame(3)]


@pytest.fixture
def simulation(route,tmp_path,monkeypatch):
    obj,req=route;clock=SimpleNamespace(t=100.)
    monkeypatch.setattr(ex.time,'monotonic',lambda:clock.t)
    monkeypatch.setattr(ex.time,'sleep',lambda t:setattr(clock,'t',clock.t+t))
    class Bus:
        def __init__(self):
            self.states={};self.targets=[];self.releases=0;self.armed=False;self.fault=None;self.runner=None
            self.q=np.array(req['start']);self.left=np.array(req['left']);self.stalled=False
            self.poll()
        def poll(self):
            if self.fault:raise RuntimeError(self.fault)
            for side,q in [('right',self.q),('left',self.left)]:
                for i,a in enumerate(q,1):self.states[side,i]=(a,2 if side=='right' and self.armed else 0,clock.t)
                self.states[side,8]=(RIGHT_GRIPPER_CLOSED if side=='right' else 0.,
                                     2 if side=='right' and self.armed else 0,clock.t)
        def fresh(self):return True
        def arm_at_start(self,start):self.armed=True;self.poll()
        def set_positions(self,q):
            self.targets.append((self.runner.phase,np.array(q).copy()))
            if not self.stalled:self.q=np.array(q).copy()
        def release_at_verified_rest(self,start):
            assert np.allclose(self.q,start,atol=ex.REST_TOL);self.releases+=1;self.armed=False
    bus=Bus();runner=ex.Runner(bus,obj,req,ex.Curve(obj),tmp_path,clock.t);bus.runner=runner
    faults=[]
    def fault_wait(e):faults.append(str(e));runner.operator_supported_exit=True
    monkeypatch.setattr(runner,'fault_wait',fault_wait)
    return SimpleNamespace(bus=bus,runner=runner,clock=clock,req=req,faults=faults)


def test_complete_cycle_measured_start_goal_hold_return_release(simulation):
    s=simulation;s.runner.run()
    assert s.bus.releases==1 and not s.faults and s.runner.disabled
    assert np.allclose(s.bus.targets[-1][1],s.req['start'])
    events=s.runner.events;phases=[p['phase'] for p in events]
    assert phases==['WAITING_FOR_DISABLED_FEEDBACK','ARMING_AT_CURRENT_POSE','OUTWARD',
                    'HOLD_AT_GOAL_2_SECONDS','RETURN_TO_SAVED_START','VERIFIED_SUPPORTED_START','RELAXED_AT_SAVED_START']
    a=next(p['monotonic'] for p in events if p['phase']=='HOLD_AT_GOAL_2_SECONDS')
    b=next(p['monotonic'] for p in events if p['phase']=='RETURN_TO_SAVED_START')
    assert b-a>=2
    assert all(np.allclose(q,common.endpoint()) for phase,q in s.bus.targets if phase=='HOLD_AT_GOAL_2_SECONDS')


def test_cancel_returns_exact_prefix_not_center(simulation,monkeypatch):
    s=simulation;send=s.bus.set_positions
    def cancel(q):
        send(q)
        if s.runner.phase=='OUTWARD' and len(s.bus.targets)>120:s.runner.cancel=True
    monkeypatch.setattr(s.bus,'set_positions',cancel)
    s.runner.run()
    assert s.bus.releases==1 and not s.faults
    assert 'HOLD_AT_GOAL_2_SECONDS' not in [p['phase'] for p in s.runner.events]
    assert np.allclose(s.bus.q,s.req['start'])


@pytest.mark.parametrize('fault',['stall','can','left_moved','motor_stopped'])
def test_faults_never_disable_or_recenter_blindly(simulation,monkeypatch,fault):
    s=simulation;send=s.bus.set_positions;poll=s.bus.poll
    def inject(q):
        send(q)
        if s.runner.phase=='OUTWARD' and len(s.bus.targets)>85:
            if fault=='stall':s.bus.stalled=True
            if fault=='can':s.bus.fault='CAN error'
            if fault=='left_moved':s.bus.left[0]+=.1
            if fault=='motor_stopped':s.bus.armed=False
    monkeypatch.setattr(s.bus,'set_positions',inject)
    s.runner.run()
    assert s.bus.releases==0 and s.faults and not s.runner.disabled
    assert 'RETURN_TO_SAVED_START' not in [p['phase'] for p in s.runner.events]


@pytest.mark.parametrize('bad',['moved','stale_scene','enabled'])
def test_recheck_before_enable_rejects_stale_start(simulation,bad):
    s=simulation
    if bad=='moved':s.bus.q[0]+=.1
    if bad=='stale_scene':s.runner.scene_stamp-=31
    if bad=='enabled':s.bus.armed=True
    with pytest.raises(RuntimeError):s.runner.run()
    assert not s.runner.started and not s.bus.targets and s.bus.releases==0


def test_commissioning_false_flags_and_wrong_support_rejected(tmp_path,route):
    obj,req=route
    for name in ('robot.urdf','robot.srdf'):(tmp_path/name).write_text(name)
    req.update(urdf=str(tmp_path/'robot.urdf'),srdf=str(tmp_path/'robot.srdf'))
    review=dict(schema='x5-drum-planner-commissioning-v1',calibration_sha256=common.digest(common.DEFAULT_CALIBRATION),
        urdf_sha256=common.digest(req['urdf']),srdf_sha256=common.digest(req['srdf']),
        recording_sha256=common.digest(common.DEFAULT_RECORDING),supported_right_rad=req['start'],
        verification_notes='SIMULATED REVIEW ONLY for offline unit testing; NOT physical verification')
    flags=['depth_alignment_verified','right_tool_mount_verified','left_attachment_clearance_reviewed',
           'supported_rest_verified','static_scene_access_controlled','emergency_stop_available']
    review.update({f:True for f in flags});path=tmp_path/'review.json'
    common.atomic_json(path,review);ex.commissioning(path,obj,req,common.DEFAULT_CALIBRATION)
    for flag in flags:
        bad=dict(review,**{flag:False});common.atomic_json(path,bad)
        with pytest.raises(ValueError,match='incomplete'):ex.commissioning(path,obj,req,common.DEFAULT_CALIBRATION)
    bad=dict(review,supported_right_rad=[0.]*7);common.atomic_json(path,bad)
    with pytest.raises(ValueError,match='supported rest'):ex.commissioning(path,obj,req,common.DEFAULT_CALIBRATION)


def test_demo_cannot_enter_execution_even_with_review(tmp_path):
    common.atomic_json(tmp_path/'plan.json',dict(status='planned',synthetic=True))
    common.atomic_json(tmp_path/'backend_request.json',{})
    with pytest.raises(ValueError,match='NEW measured'):ex.prepare(tmp_path/'plan.json',tmp_path/'absent.json')


@pytest.fixture
def execution_candidate(tmp_path,route):
    """Fabricated inputs for testing gates ONLY, never physical commissioning."""
    from drum_planner import depth
    obj,small_req=route;cal=common.calibration()
    state=common.read_state(common.ROOT/'drum_planner/config/example_start.json');state['measured']=True
    common.atomic_json(tmp_path/'state.json',state)
    camera=tmp_path/'camera';camera.mkdir()
    meta=copy.deepcopy(cal['camera']);meta.update(protocol=1,alignment='native_depth',depth_stream_enabled=True,color_stream_enabled=False)
    frames=[(dict(sequence=i,received_monotonic=ex.time.monotonic()-.5+.1*i,value_scale_m=.001),
             np.full((800,1280),2000,dtype='<u2')) for i in range(5)]
    depth.snapshot(camera,meta,frames,common.DEFAULT_CALIBRATION)
    world=planning.scene(camera/'scene.json');req,model=planning.request(tmp_path,state,world)
    common.atomic_json(tmp_path/'backend_request.json',req)
    obj.update(status='planned',synthetic=False,strict_unknown_space=False,collision_policy=planning.COLLISION_POLICY,
        actual_current_robot_state_verified=True,
        reverse_checked_on_same_static_scene=True,hold_at_goal_s=2.,calibration_sha256=common.digest(common.DEFAULT_CALIBRATION),
        recording_sha256=common.digest(common.DEFAULT_RECORDING),model=model,
        provenance=dict(state=str(tmp_path/'state.json'),state_sha256=common.digest(tmp_path/'state.json'),
                        scene=str(camera/'scene.json')))
    common.atomic_json(tmp_path/'plan.json',obj)
    review=dict(schema='x5-drum-planner-commissioning-v1',calibration_sha256=obj['calibration_sha256'],
        urdf_sha256=common.digest(req['urdf']),srdf_sha256=common.digest(req['srdf']),recording_sha256=obj['recording_sha256'],
        supported_right_rad=req['start'],verification_notes='UNIT TEST ONLY: synthetic depth and simulated measurements, NOT a physical review')
    review.update({f:True for f in ('depth_alignment_verified','right_tool_mount_verified','left_attachment_clearance_reviewed',
        'supported_rest_verified','static_scene_access_controlled','emergency_stop_available')})
    common.atomic_json(tmp_path/'review.json',review)
    return tmp_path,req,obj


def test_execution_rechecks_exact_path_with_extra_clearance(execution_candidate,monkeypatch):
    folder,req,obj=execution_candidate;calls=[]
    def checked(request,*a):
        calls.append(request)
        return dict(status='checked',collision_policy=request['collision_policy'],strict_unknown_space=False,
                    checks=[dict(valid=True) for _ in request['check_states']])
    monkeypatch.setattr(planning,'backend',checked)
    _,run_req,curve,_,_=ex.prepare(folder/'plan.json',folder/'review.json')
    assert len(calls)==1 and calls[0]['minimum_self_clearance_m']==.04
    assert calls[0]['scene']['obstacle_padding_m']==.05
    # Execution's extra world clearance must not enlarge the initial self-mask.
    assert calls[0]['drumstick_self_filter_padding_m']==req['drumstick_self_filter_padding_m']==.029
    assert not calls[0]['occupied_only_preview'] and not calls[0]['scene']['synthetic']
    assert calls[0]['collision_policy']=='observed_obstacles'
    assert len(run_req['execution_joint_limits'])==7
    assert np.allclose(calls[0]['check_states'][-1],common.endpoint())
    assert (folder/'execution_commissioning.json').is_file()


@pytest.mark.parametrize('bad',['clearance','incomplete_check','scene_hash','state_hash','goal','old_scene'])
def test_execution_gates_fail_before_motor_constructor(execution_candidate,monkeypatch,bad):
    folder,req,obj=execution_candidate
    def checked(request,*a):
        rows=[dict(valid=bad!='clearance') for _ in request['check_states']]
        if bad=='incomplete_check':rows.pop()
        return dict(status='checked',collision_policy=request['collision_policy'],strict_unknown_space=False,checks=rows)
    monkeypatch.setattr(planning,'backend',checked)
    if bad=='scene_hash':
        with (folder/'camera/depth_m.bin').open('ab') as f:f.write(b'x')
    if bad=='state_hash':
        with (folder/'state.json').open('a') as f:f.write(' ')
    if bad=='goal':
        obj['timing']['coefficients'][0][-1][-1]+=.1
        common.atomic_json(folder/'plan.json',obj)
    if bad=='old_scene':
        now=ex.time.monotonic();monkeypatch.setattr(ex.time,'monotonic',lambda:now+40)
    with pytest.raises(ValueError):ex.prepare(folder/'plan.json',folder/'review.json')


@pytest.mark.parametrize('bad',['missing_policy','legacy_preview','strict_result','request_policy',
                                'recheck_policy','recheck_visibility'])
def test_execution_policy_is_explicit_and_preserved_during_recheck(execution_candidate,monkeypatch,bad):
    folder,req,obj=execution_candidate;calls=[]
    def checked(request,*a):
        calls.append(request)
        return dict(status='checked',collision_policy='observed_free_space' if bad=='recheck_policy' else 'observed_obstacles',
                    strict_unknown_space=bad=='recheck_visibility',checks=[dict(valid=True) for _ in request['check_states']])
    monkeypatch.setattr(planning,'backend',checked)
    if bad=='missing_policy':obj.pop('collision_policy')
    if bad=='legacy_preview':req['occupied_only_preview']=True
    if bad=='strict_result':obj['strict_unknown_space']=True
    if bad=='request_policy':req['collision_policy']='observed_free_space'
    common.atomic_json(folder/'plan.json',obj);common.atomic_json(folder/'backend_request.json',req)
    with pytest.raises(ValueError):ex.prepare(folder/'plan.json',folder/'review.json')
    assert len(calls)==(1 if bad.startswith('recheck_') else 0)


def test_actual_moveit_minimum_self_clearance(route,tmp_path):
    obj,small_req=route;state=common.read_state(common.ROOT/'drum_planner/config/example_start.json')
    req,_=planning.request(tmp_path,state,dict(synthetic=True,obstacle_padding_m=.02,boxes=[]))
    req['check_states']=[req['start']]
    normal=planning.backend(req,tmp_path,'normal')
    assert normal['checks'][0]['valid']
    req['minimum_self_clearance_m']=.1
    strict=planning.backend(req,tmp_path,'clearance')
    assert strict['checks'][0]['reason']=='insufficient_self_clearance'
    assert strict['checks'][0]['distance_m']<.1


def test_transport_emulated_wire_no_left_actuation_no_initial_disable(tmp_path,route,monkeypatch):
    obj,req=route;created=[];real_open=encoder.os.open
    def opened(path,*a,**k):
        if str(path).startswith('/tmp/openarmx-cartesian-'):path=tmp_path/path.split('/')[-1]
        return real_open(path,*a,**k)
    monkeypatch.setattr(encoder.os,'open',opened)
    class Socket:
        def __init__(self,*a,**kw):self.sent=[];self.pending=[];self.mode={i:0 for i in range(1,9)};self.targets={};created.append(self)
        def bind(self,address):
            self.side='right' if address[0]=='can0' else 'left';q=req['start'] if self.side=='right' else req['left']
            self.raw={i:encoder.joint_to_motor(self.side,i,v) for i,v in enumerate(q,1)}
            self.raw[8]=-RIGHT_GRIPPER_CLOSED if self.side=='right' else 0.
        def setsockopt(self,*a):pass
        def setblocking(self,*a):pass
        def close(self):pass
        def send(self,frame):
            self.sent.append(frame);cid,dlc,data=encoder.FRAME.unpack(frame);kind=(cid>>24)&31;i=cid&255
            if kind==3:self.mode[i]=2;self.raw[i]=self.targets[i]
            if kind==4:self.mode[i]=0
            if kind==18 and data[:2]==b'\x16\x70':
                self.targets[i]=__import__('struct').unpack('<f',data[4:])[0]
                if self.mode[i]==2:self.raw[i]=self.targets[i]
            if kind==17:self.pending.append(encoder.FRAME.pack(encoder.EFF|(17<<24)|(i<<8)|0xfd,8,bytes.fromhex('0570000005000000')))
            if kind==2:
                count=int(round((self.raw[i]+12.57)/25.14*65535))
                self.pending.append(encoder.FRAME.pack(encoder.EFF|(2<<24)|(self.mode[i]<<22)|(i<<8)|0xfd,8,
                    count.to_bytes(2,'big')+bytes.fromhex('8000800000fa')))
            return 16
        def recv(self,n):
            if not self.pending:raise BlockingIOError()
            return self.pending.pop(0)
    monkeypatch.setattr(encoder.socket,'socket',Socket)
    b=mt.PlannerMotors(tmp_path,req['start'])
    for _ in range(5):b.poll();ex.time.sleep(.015)
    assert b.fresh();b.arm_at_start(req['start'])
    assert all(b.states['right',i][1]==2 for i in range(1,9))
    right=next(s for s in created if s.side=='right');left=next(s for s in created if s.side=='left')
    assert not any(((encoder.FRAME.unpack(f)[0]>>24)&31)==4 for f in right.sent)
    assert all(f in b.queries for f in left.sent)
    targets=[f for f in right.sent if (encoder.FRAME.unpack(f)[0]&255)==8 and encoder.FRAME.unpack(f)[2][:2]==b'\x16\x70']
    assert targets and all(f==parameter(8,0x7016,-RIGHT_GRIPPER_CLOSED) for f in targets)
    with pytest.raises(RuntimeError):b.send_control(packet(4,1))
    b.release_at_verified_rest(req['start']);assert all(b.states['right',i][1]==0 for i in range(1,9))
    n=len(right.sent)+len(left.sent);b.close();assert n==len(right.sent)+len(left.sent)
