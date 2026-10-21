"""Offline safety/state-machine/GUI contracts. No physical CAN or arm movement."""
import copy,json,socket,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from drum_planner import common,planning
from drum_planner.baseline import paths,motion,tasks
from drum_planner.baseline.simulation import SimulatedBus


@pytest.fixture(autouse=True)
def no_can(monkeypatch):
    original=socket.socket
    def guarded(family=socket.AF_INET,*a,**k):
        if family==socket.AF_CAN:raise AssertionError('Hardware tests must NEVER open CAN')
        return original(family,*a,**k)
    monkeypatch.setattr(socket,'socket',guarded)


@pytest.fixture
def rig():
    state=common.read_state(common.ROOT/'drum_planner/config/example_start.json')
    waypoints=[state['right_rad'],common.endpoint().tolist()]
    outward,_,timing=planning.smooth_samples(waypoints)
    obj=dict(outward=outward,timing=timing)
    req=dict(start=state['right_rad'],left=state['left_rad'],left_opening_m=0.,execution_joint_limits=[[-3.5,3.5]]*7)
    clock=SimpleNamespace(t=100.)
    bus=SimulatedBus(req,now=lambda:clock.t)
    commands=[];send=bus.set_positions
    def record(q):commands.append(np.asarray(q).copy());send(q)
    bus.set_positions=record
    engine=motion.Controller(bus,obj,req,now=lambda:clock.t)
    def tick(n=1):
        for _ in range(n):clock.t+=.01;engine.step()
    def until(phase,limit=150000):
        for _ in range(limit):
            if engine.phase==phase:return
            if engine.phase in motion.TERMINAL:raise AssertionError((engine.phase,engine.error))
            tick()
        raise AssertionError(('timeout',engine.status()))
    return SimpleNamespace(bus=bus,engine=engine,tick=tick,until=until,clock=clock,req=req,obj=obj,commands=commands)


@pytest.mark.parametrize('rate',[.25,.6,1.])
def test_complete_cycle_only_relaxes_at_original_supported_start(rig,rate):
    r=rig;r.engine.rate=rate;r.engine.start();r.until('RELAXED')
    assert r.bus.releases==1 and r.engine.disabled
    assert np.allclose(r.bus.q,r.req['start'])
    phases=[x['phase'] for x in r.engine.events]
    assert phases==['PREFLIGHT','ARMING','SETTLING_START','OUTWARD','SETTLING_GOAL','GOAL_HOLD','RETURN','SETTLING_REST','VERIFIED_START_POSITION','RELAXED']
    hold=next(x for x in r.engine.events if x['phase']=='GOAL_HOLD')['monotonic']
    returned=next(x for x in r.engine.events if x['phase']=='RETURN')['monotonic']
    assert returned-hold>=2 and returned-hold<2.03
    assert all(np.isfinite(q).all() for q in r.commands)


def held(r,phase='OUTWARD'):
    r.engine.start();r.until(phase);r.tick(30);r.engine.halt();r.tick(70)
    assert r.engine.phase=='HELD' and r.engine.hold_confirmed and r.bus.releases==0


def return_from_hold(r):
    request=r.engine.request_return(True);r.tick(70)
    spec=paths.recovery(r.engine.curve,request['held'],request['tau'],request['rate'])
    spec['proof']=dict(all_valid=True,simulation_only=True)
    assert r.engine.accept_return(spec,request['generation'])
    r.until('RELAXED');assert r.bus.releases==1
    return spec


@pytest.mark.parametrize('phase',['SETTLING_START','OUTWARD','SETTLING_GOAL','GOAL_HOLD','RETURN','SETTLING_REST'])
def test_hold_stays_indefinite_then_reverses_prefix(rig,phase):
    r=rig;held(r,phase);tau=r.engine.tau;pose=r.bus.q.copy();sent=len(r.commands)
    r.tick(600)
    assert r.engine.tau==tau and np.array_equal(pose,r.bus.q) and len(r.commands)==sent and r.bus.releases==0
    spec=return_from_hold(r)
    assert spec['tau']<=tau and np.allclose(r.bus.q,r.req['start'])
    assert 'GOAL_HOLD' not in [x['phase'] for x in r.engine.events] if phase in ('OUTWARD','SETTLING_START') else True


@pytest.mark.parametrize('fault',['can','thermal','contact','motor_disabled','left_moved','scheduler'])
def test_faults_hold_never_auto_reverse_or_relax(rig,fault):
    r=rig;r.engine.start();r.until('OUTWARD');r.tick(50)
    if fault=='can':r.bus.fault='CAN failure'
    elif fault=='thermal':r.bus.fault='motor too hot'
    elif fault=='contact':r.bus.contact=True
    elif fault=='motor_disabled':r.bus.enabled=False
    elif fault=='left_moved':r.bus.req=copy.deepcopy(r.bus.req);r.bus.req['left'][0]+=.1
    elif fault=='scheduler':r.clock.t+=.1
    for _ in range(600):
        r.tick()
        if r.engine.phase=='FAULT_HOLD':break
    assert r.engine.phase=='FAULT_HOLD';r.tick(600)
    assert not r.engine.disabled and r.bus.releases==0
    assert 'RECOVERY' not in [x['phase'] for x in r.engine.events]
    if fault in ('can','thermal','motor_disabled','left_moved'):
        with pytest.raises((RuntimeError,ValueError)):r.engine.request_return(True)
        assert not r.engine.hold_confirmed
    r.bus.fault='';r.bus.contact=False;r.bus.enabled=True;r.bus.req=r.req
    r.tick(70)
    if r.engine.held is None:
        with pytest.raises(ValueError,match='Fresh hold'):r.engine.request_return(True)
        r.tick(70)
    return_from_hold(r)


def test_hold_during_partial_enable_does_not_enable_remaining_drives(rig):
    r=rig;r.engine.start();r.until('ARMING');r.tick()
    assert r.bus.enable_attempted
    r.engine.halt();r.tick(100)
    assert r.engine.phase=='HELD' and r.bus.releases==0


def test_cancel_before_enable_sends_no_targets(rig):
    r=rig;r.engine.start();r.engine.halt();r.tick(200)
    assert r.engine.phase=='REJECTED' and not r.bus.enable_attempted and not r.commands and r.bus.releases==0


def test_stale_or_wrong_initial_pose_never_enables(rig):
    r=rig;r.bus.q[0]+=.1;r.engine.start();r.tick()
    assert r.engine.phase=='REJECTED' and not r.bus.enable_attempted and not r.commands


def test_reverse_needs_new_operator_action_and_stationarity(rig):
    r=rig;r.engine.start();r.until('OUTWARD');r.tick(50);r.engine.halt()
    with pytest.raises(ValueError,match='explicit operator'):r.engine.request_return(False)
    with pytest.raises(ValueError,match='stationary'):r.engine.request_return(True)
    assert r.bus.releases==0


def test_hold_cancels_pending_return_approval(rig):
    r=rig;held(r);request=r.engine.request_return(True);r.tick(70)
    spec=paths.recovery(r.engine.curve,request['held'],request['tau'],request['rate']);spec['proof']=dict(all_valid=True)
    r.engine.halt()
    assert not r.engine.accept_return(spec,request['generation']) and r.bus.releases==0


def test_hold_during_recovery_can_be_recovered_again(rig):
    r=rig;held(r,'RETURN');request=r.engine.request_return(True);r.tick(70)
    spec=paths.recovery(r.engine.curve,request['held'],request['tau'],request['rate']);spec['proof']=dict(all_valid=True)
    r.engine.accept_return(spec,request['generation']);r.tick(90);r.engine.halt();r.tick(70)
    return_from_hold(r)


def test_no_drift_following_while_held(rig):
    r=rig;held(r);anchor=r.engine.held.copy();n=len(r.commands)
    r.bus.q[0]+=.03;r.tick(70);r.engine.halt('another fault',fault=True);r.tick(70)
    assert not r.engine.hold_confirmed and np.array_equal(r.engine.held,anchor) and len(r.commands)==n
    with pytest.raises(ValueError):r.engine.request_return(True)
    assert r.bus.releases==0


def test_supported_exit_never_sends_disable(rig):
    r=rig;held(r)
    r.engine.supported_exit()
    assert r.engine.phase=='CONTROLLER_EXIT' and r.bus.releases==0 and r.bus.enabled


@pytest.mark.parametrize('phase',['PREFLIGHT','ARMING','OUTWARD','HELD','FAULT_HOLD','CHECKING_RETURN','RECOVERY','GOAL_HOLD','RETURN'])
def test_manual_relax_is_immediate_without_return_or_targets(rig,phase):
    r=rig;r.engine.start()
    if phase=='PREFLIGHT':pass
    elif phase in ('HELD','FAULT_HOLD','CHECKING_RETURN','RECOVERY'):
        r.until('OUTWARD');r.tick(50);r.engine.halt(fault=phase=='FAULT_HOLD');r.tick(70)
        if phase in ('CHECKING_RETURN','RECOVERY'):
            request=r.engine.request_return(True)
            if phase=='RECOVERY':
                r.tick(70);spec=paths.recovery(r.engine.curve,request['held'],request['tau'],request['rate'])
                spec['proof']=dict(all_valid=True);r.engine.accept_return(spec,request['generation']);r.tick(20)
    else:r.until(phase)
    before=len(r.commands);q=r.bus.q.copy();generation=r.engine.generation
    r.engine.relax_now()
    assert r.bus.releases==1 and not r.bus.enabled  # disable is sent in the click, not a later return
    assert r.engine.phase=='RELAXING' and not r.engine.disabled  # confirmation still pending
    assert r.engine.generation==generation+1 and r.engine.return_spec is None and r.engine.arm_steps is None
    r.tick(200)
    assert r.engine.phase=='RELAXED' and r.engine.disabled
    assert len(r.commands)==before and np.array_equal(r.bus.q,q)
    r.engine.relax_now();r.tick(100);assert r.bus.releases==1
    with pytest.raises(ValueError):r.engine.start()


def test_manual_relax_failure_does_not_claim_hold_or_retry_or_resume(rig,monkeypatch):
    r=rig;held(r);before=len(r.commands);requests=[]
    def broken():requests.append(True);raise OSError('CAN unavailable')
    monkeypatch.setattr(r.bus,'release_on_operator_request',broken)
    r.engine.relax_now();r.tick(200)
    assert r.engine.phase=='RELAX_UNCONFIRMED' and not r.engine.disabled and not r.engine.hold_confirmed
    assert len(requests)==1 and len(r.commands)==before and r.bus.releases==0
    r.engine.halt('GUI disconnected',fault=True);r.tick(100)
    assert r.engine.phase=='RELAX_UNCONFIRMED' and len(requests)==1 and len(r.commands)==before
    assert not r.engine.status()['can_return'] and not r.engine.status()['safe_to_close']
    with pytest.raises(ValueError):r.engine.request_return(True)
    monkeypatch.setattr(r.bus,'release_on_operator_request',lambda:(setattr(r.bus,'enabled',False),r.bus.poll()))
    r.engine.relax_now();r.tick();assert r.engine.phase=='RELAXED'


def test_relax_needs_all_eight_fresh_disabled_acknowledgements(rig,monkeypatch):
    r=rig;held(r);old={k:(q,0,t-1) for k,(q,mode,t) in r.bus.states.items()}
    monkeypatch.setattr(r.bus,'release_on_operator_request',lambda:None)
    monkeypatch.setattr(r.bus,'poll',lambda:None)
    r.bus.states=old;r.engine.relax_now();r.tick(150)
    assert r.engine.phase=='RELAX_UNCONFIRMED' and not r.engine.disabled
    for i in range(1,9):r.bus.states['right',i]=(0.,0,r.clock.t)
    r.bus.states['right',8]=(0.,2,r.clock.t);r.tick()
    assert not r.engine.disabled
    r.bus.states['right',8]=(0.,0,r.clock.t);r.tick();assert r.engine.disabled


def test_manual_relax_cancels_pending_recovery_and_survives_log_failure(rig):
    r=rig;held(r);request=r.engine.request_return(True);r.tick(70)
    spec=paths.recovery(r.engine.curve,request['held'],request['tau'],request['rate']);spec['proof']=dict(all_valid=True)
    r.engine.sink=lambda _:(_ for _ in ()).throw(OSError('disk full'))
    before=len(r.commands);r.engine.relax_now();r.tick()
    assert r.engine.phase=='RELAXED' and r.bus.releases==1 and len(r.commands)==before
    assert not r.engine.accept_return(spec,request['generation'])


@pytest.mark.parametrize('bad',[0,.1,1.01,2,float('nan'),float('inf')])
def test_speed_cannot_exceed_validated_limits(bad):
    with pytest.raises(ValueError):paths.speed(bad)


def test_recovery_only_visited_prefix_and_tiny_connector(rig):
    c=rig.engine.curve;tau=c.duration*.6;held_q=c.at(tau-.03)+np.array([.0005,0,0,0,0,0,0])
    spec=paths.recovery(c,held_q,tau,1.)
    assert spec['tau']<=tau and spec['connector_l1_rad']<=motion.TRACK_L1
    samples=paths.recovery_samples(c,spec)
    assert np.allclose(samples[0],held_q) and np.allclose(samples[-1],rig.req['start'])
    for t in np.linspace(spec['bridge_s'],spec['bridge_s']+spec['return_s'],101):
        q,u,_=paths.recovery_at(c,spec,t)
        assert 0<=u<=tau and np.allclose(q,c.at(u))
    # Dense numerical check independent of analytic time-warp bounds.
    t=np.linspace(0,spec['bridge_s']+spec['return_s'],20001);dt=t[1]-t[0]
    q=np.array([paths.recovery_at(c,spec,x)[0] for x in t])
    for limit in (.6,1.2,6.):
        q=np.gradient(q,dt,axis=0)
        assert np.max(abs(q[5:-5]))<=limit*1.02
    with pytest.raises(ValueError,match='too far'):paths.recovery(c,held_q+1,tau,1.)


def test_baseline_plan_inputs_not_experiment_cache():
    # Loader rejects even a perfectly valid experiment route as a hardware
    # artifact: only newly measured baseline sessions may be armed.
    examples=sorted((common.ROOT/'diagnostics').glob('planner_zone3_*/actual_camera/plans/baseline-*/plan.json'))
    if not examples:pytest.skip('No saved real-scene software baseline fixture')
    with pytest.raises(ValueError,match='newly acquired'):tasks.load_checked(examples[-1],common.DEFAULT_CALIBRATION)


def test_prepare_or_execute_has_no_call_on_import(monkeypatch):
    from drum_planner.baseline import app,service,transport
    assert callable(app.show) and callable(service.launch) and issubclass(transport.BaselineMotors,__import__('drum_planner.motor_transport',fromlist=['PlannerMotors']).PlannerMotors)


def test_cli_unsafe_flags_rejected_without_devices():
    from drum_planner.cli import main
    assert main(['baseline','--seconds','1'])==1
    assert main(['baseline','--resume','x','--simulate','y'])==1


def test_job_can_opt_in_is_only_for_query_capture(monkeypatch,tmp_path):
    from drum_planner.baseline import app
    calls=[]
    monkeypatch.setattr(app,'Job',lambda *a,**k:calls.append((a,k)))
    for action in ('capture','plan','prepare','return'):app.job(action,tmp_path/action)
    assert [k['allow_can'] for _,k in calls]==[True,False,False,False]


def test_fresh_start_through_worker_entrypoints_preserves_logs_and_new_plan_directory(monkeypatch,tmp_path):
    """Exercise the launcher -> Job -> tasks.main path, not a saved-plan view.

    Only the devices/native solver and systemd transport are simulated. Match
    Job's real precreated log directory and planning.plan's exclusive mkdir.
    """
    import sys
    from drum_planner.baseline import app
    calls=[]
    def capture(folder,cal):
        common.atomic_json(folder/'acquisition.json',dict(proof=dict(test=True),boot_id='unit-test'))
    def plan(folder,state,scene,cal):
        assert folder.name=='result' and (folder.parent/'worker.log').read_text()=='worker log retained'
        folder.mkdir(parents=True,exist_ok=False)
        common.atomic_json(folder/'backend_request.json',dict(test=True))
        return dict(status='planned')
    class Job:
        def __init__(self,cmd,folder,**kw):
            self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True)
            self.log_path=self.folder/'worker.log';self.log_path.write_text('worker log retained')
            self.process=SimpleNamespace(returncode=0);self.closed=False;calls.append(self)
            with monkeypatch.context() as m:
                m.setattr(sys,'argv',[cmd[2],*cmd[3:]]);tasks.main()
        def poll(self):return 0
        def close(self):self.closed=True
    monkeypatch.setattr(app,'Job',Job);monkeypatch.setattr(tasks,'acquire',capture)
    monkeypatch.setattr(planning,'plan',plan)
    root=tmp_path/'session';path=app.initial(root,common.DEFAULT_CALIBRATION)
    assert path==root/'plan/result/plan.json' and path.is_file()
    obj=json.loads(path.read_text())
    assert obj['baseline_hardware_session'] and obj['actual_current_robot_state_verified']
    assert obj['current_state_check']==dict(test=True)
    assert (path.parent/'backend_request.json').is_file()
    assert all(j.closed for j in calls) and [j.folder.name for j in calls]==['capture','plan']
    before=path.read_bytes()
    with pytest.raises(FileExistsError):tasks.make_plan(root/'plan',root/'capture',common.DEFAULT_CALIBRATION)
    assert path.read_bytes()==before and (root/'plan/worker.log').read_text()=='worker log retained'


def test_baseline_startup_error_does_not_claim_controller_is_active(monkeypatch,capsys,tmp_path):
    from drum_planner import cli
    from drum_planner.baseline import app
    def fail(*a,**kw):raise ValueError('test startup failure')
    monkeypatch.setattr(app,'initial',fail)
    assert cli.main(['baseline','--output',str(tmp_path/'startup')])==1
    error=capsys.readouterr().err
    assert 'test startup failure' in error and 'If a controller is already active' in error


@pytest.mark.parametrize('world,self_gap',[(.02,None),(.02,0.),(.06,.015)])
def test_hardware_check_preserves_exact_baseline_margins(monkeypatch,tmp_path,world,self_gap):
    req=dict(scene=dict(obstacle_padding_m=world),collision_policy='observed_obstacles',right_zone=dict(enabled=True))
    if self_gap is not None:req['minimum_self_clearance_m']=self_gap
    original=copy.deepcopy(req)
    calls=[]
    def backend(request,*a):
        calls.append(request)
        return dict(status='checked',collision_policy='observed_obstacles',strict_unknown_space=False,checks=[dict(valid=True) for _ in request['check_states']])
    monkeypatch.setattr(planning,'backend',backend)
    proof=tasks.hard_check(req,np.zeros((3,7)),tmp_path)
    assert proof==dict(all_valid=True,samples=3,collision_margin_policy='baseline_request',
                       obstacle_padding_m=world,minimum_self_clearance_m=self_gap or 0.)
    assert calls[0]['scene']['obstacle_padding_m']==world
    assert calls[0].get('minimum_self_clearance_m')==self_gap
    assert calls[0]['right_zone']==req['right_zone'] and calls[0]['check_path'] and calls[0]['stop_on_invalid']
    assert req==original
    monkeypatch.setattr(planning,'backend',lambda *a,**k:dict(status='checked',collision_policy='observed_obstacles',strict_unknown_space=False,checks=[dict(valid=False)]))
    with pytest.raises(ValueError,match='Baseline collision'):tasks.hard_check(req,np.zeros((3,7)),tmp_path)


@pytest.mark.parametrize('world,self_gap',[(.019,0.),(.101,0.),(float('nan'),0.),(.02,-.001),(.02,.101),(.02,float('nan'))])
def test_invalid_baseline_margins_rejected_before_backend(monkeypatch,tmp_path,world,self_gap):
    def forbidden(*a,**kw):raise AssertionError('Invalid margins reached native backend')
    monkeypatch.setattr(planning,'backend',forbidden)
    with pytest.raises(ValueError,match='Invalid Baseline collision margins'):
        tasks.hard_check(dict(scene=dict(obstacle_padding_m=world),minimum_self_clearance_m=self_gap),np.zeros((3,7)),tmp_path)


def test_transient_bus_fault_can_establish_hold_without_power_cycle(rig):
    r=rig;r.engine.start();r.until('OUTWARD');r.tick(70)
    r.bus.fault='CAN unavailable';r.tick()
    assert r.engine.phase=='FAULT_HOLD' and r.engine.held is None
    with pytest.raises(RuntimeError):r.engine.retry_hold()
    r.bus.fault='';r.tick(70);r.engine.retry_hold();r.tick(70)
    assert r.engine.hold_confirmed;return_from_hold(r)


def test_event_log_failure_before_enable_aborts(rig):
    r=rig
    def broken(_):raise OSError('disk full')
    r.engine.sink=broken;r.engine.start();r.tick(100)
    assert r.engine.phase=='REJECTED' and not r.bus.enable_attempted and r.bus.releases==0


def test_event_log_failure_in_motion_holds_without_release(rig):
    r=rig;r.engine.start();r.until('OUTWARD');r.tick(30)
    r.engine.sink=lambda _:(_ for _ in ()).throw(OSError('disk full'))
    r.engine.emit('RETURN');r.tick(30)
    assert r.engine.phase=='FAULT_HOLD' and r.bus.releases==0


def test_can_audit_failure_does_not_block_hold_packet(rig):
    r=rig;r.engine.start();r.until('OUTWARD');r.tick(40)
    r.bus.audit_error='disk full';r.tick()
    assert r.engine.phase=='FAULT_HOLD' and r.engine.hold_confirmed
    r.tick(80)
    with pytest.raises(RuntimeError):r.engine.request_return(True)
    r.bus.audit_error='';r.tick(80);return_from_hold(r)


# Reuse the synthetic execution fixture but REMOVE its legacy physical review.
# Baseline no longer consumes or invents reviews. AF_CAN is forbidden throughout.
from test_planner_execution import execution_candidate,route

@pytest.fixture
def prepared_fixture(execution_candidate,monkeypatch):
    folder,req,obj=execution_candidate
    from drum_planner.right_zone import metadata
    obj.update(schema='x5-moveit-result-v1',motion_authorized=False,hardware_execution_available=False,
               baseline_hardware_session=True,acquisition_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
               right_zone_constraint=metadata(req['right_zone']),cycle=planning.cycle(obj['outward']))
    common.atomic_json(folder/'plan.json',obj)
    (folder/'measured_start.json').write_bytes((folder/'state.json').read_bytes())
    monkeypatch.setattr(tasks,'hard_check',lambda request,q,target:dict(all_valid=True,samples=len(q),**tasks.margin_contract(request)))
    (folder/'review.json').unlink()
    tasks.prepare(folder/'prepared',folder/'plan.json',folder,common.DEFAULT_CALIBRATION)
    return folder/'prepared/prepared.json'


def test_prepared_packet_is_bound_to_path_scene_model_zone_without_review(prepared_fixture):
    packet,curve=tasks.check_packet(prepared_fixture)
    assert packet['motion_authorized'] is False and curve.duration>0
    assert packet['schema']=='x5-baseline-prepared-v2'
    assert not any('review' in k or 'support' in k for k in packet)
    assert not (prepared_fixture.parent.parent/'review.json').exists()


@pytest.mark.parametrize('bad',['expired','fake_stamp','changed_plan','old_schema','authorized','request','proof',
    'self_margin','missing_self_margin','margin_policy','missing_margin_policy','proof_samples','proof_valid','zone'])
def test_prepared_packet_changes_rejected(prepared_fixture,monkeypatch,bad):
    p=prepared_fixture;data=json.loads(p.read_text())
    if bad=='expired':monkeypatch.setattr(tasks.time,'monotonic',lambda:data['scene_stamp']+31)
    if bad=='fake_stamp':data['scene_stamp']+=1000
    if bad=='changed_plan':
        with Path(data['plan']).open('a') as f:f.write(' ')
    if bad=='old_schema':data['schema']='x5-baseline-prepared-v1'
    if bad=='authorized':data['motion_authorized']=True
    if bad=='request':data['request']['start'][0]+=.01
    if bad=='proof':data['proof']['obstacle_padding_m']=.05
    if bad=='self_margin':data['proof']['minimum_self_clearance_m']=.04
    if bad=='missing_self_margin':data['proof'].pop('minimum_self_clearance_m')
    if bad=='margin_policy':data['proof']['collision_margin_policy']='hardware_extra_clearance'
    if bad=='missing_margin_policy':data['proof'].pop('collision_margin_policy')
    if bad=='proof_samples':data['proof']['samples']-=1
    if bad=='proof_valid':data['proof']['all_valid']=False
    if bad=='zone':data['request']['right_zone']['buffer_m']=.1
    common.atomic_json(p,data)
    with pytest.raises(ValueError):tasks.check_packet(p)


def test_prepare_rejects_changed_arm_pose_before_any_motor(prepared_fixture,tmp_path):
    packet=json.loads(prepared_fixture.read_text());base=prepared_fixture.parent.parent
    q=json.loads((base/'measured_start.json').read_text());q['right_rad'][2]+=.02
    common.atomic_json(base/'measured_start.json',q)
    with pytest.raises(ValueError,match='Arm changed'):tasks.prepare(base/'again',packet['plan'],base,packet['calibration'])


@pytest.fixture
def start_view(tmp_path,monkeypatch):
    """Real UI callbacks, fake widgets/jobs/service; cannot access any hardware."""
    from drum_planner.baseline import app
    class Widget:
        def __init__(self):self.enabled=True;self.text=''
        def setEnabled(self,v):self.enabled=v
        def setText(self,v):self.text=v
    class Job:
        def __init__(self,action,folder,**kw):
            self.action=action;self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True)
            self.kw=kw;self.code=None;self.cancelled=False;self.closed=False;self.log_path=self.folder/'worker.log'
            jobs.append(self)
        def poll(self):return self.code
        def cancel(self):self.cancelled=True
        def close(self):self.closed=True
    jobs=[];launches=[]
    def launch(folder,**kw):
        launches.append((folder,kw))
        return dict(socket=str(folder/'not-created.sock'))
    v=app.View.__new__(app.View)
    v.session=tmp_path;v.cal_path=common.DEFAULT_CALIBRATION;v.simulated=False
    v.client=None;v.client_info=None;v.hardware_status={};v.job=None;v.stage=None
    v.prepared=None;v.pending_start=False;v.start_sent=False;v.prepare_folder=None;v.start_when_ready=False
    v.locked=False;v.last_error='';v.connection_failed=False;v.requested_rate=.75;v.close_requested=False
    v.plan=SimpleNamespace(path=tmp_path/'plan.json',playable=True,obj=dict(provenance={}),req={})
    v.clock=SimpleNamespace(pause=lambda:None)
    for attr in ('rate_box','execute_button','prepare_button','hardware_label','status',
                 'reverse_button','support_button','relax_button','retry_hold_button','reconnect_button'):
        setattr(v,attr,Widget())
    v.set_scene=lambda *a:None
    monkeypatch.setattr(app,'job',Job);monkeypatch.setattr(app.service,'launch',launch)
    monkeypatch.setattr(app,'load',lambda *a:(None,None))
    def finish_capture():
        jobs[-1].code=0;v.poll_hardware();assert jobs[-1].action=='prepare'
    def finish_check(age=0,code=0):
        common.atomic_json(jobs[-1].folder/'prepared.json',dict(scene_stamp=time.monotonic()-age,
            fresh_scene='mock-scene.json',request=dict(scene={})))
        jobs[-1].code=code;v.poll_hardware()
    return SimpleNamespace(v=v,jobs=jobs,launches=launches,finish_capture=finish_capture,finish_check=finish_check)


def test_direct_start_waits_for_all_automatic_checks_once_without_dialog(start_view):
    x=start_view;v=x.v
    assert not hasattr(v,'physical_review')
    v.execute_clicked()
    assert v.start_when_ready and v.locked and not v.rate_box.enabled and not v.execute_button.enabled
    assert x.jobs[0].action=='capture' and not x.launches and not v.active
    v.execute_clicked();assert len(x.jobs)==1  # duplicate click is not queued
    x.finish_capture()
    assert x.jobs[0].closed and not x.launches and 'review' not in x.jobs[1].kw
    x.finish_check()
    assert len(x.launches)==1 and not v.start_when_ready and v.pending_start and v.active
    assert x.launches[0][1]['rate']==.75 and x.launches[0][1]['simulate_plan'] is None
    assert all(j.closed for j in x.jobs) and not list(v.session.rglob('*review*'))
    v.poll_hardware();v.execute_clicked();assert len(x.launches)==1


def test_optional_readiness_never_authorizes_motion(start_view):
    x=start_view;v=x.v;v.prepare_clicked();x.finish_capture();x.finish_check()
    assert v.prepared and not v.start_when_ready and not v.active and not x.launches
    assert v.execute_button.enabled
    v.execute_clicked();assert len(x.launches)==1 and len(x.jobs)==2


@pytest.mark.parametrize('stage',['capture','prepare'])
@pytest.mark.parametrize('stop',['hold','close'])
def test_cancelled_start_cannot_start_from_late_worker_completion(start_view,stage,stop):
    x=start_view;v=x.v;v.execute_clicked()
    if stage=='prepare':x.finish_capture()
    if stop=='hold':v.emergency_hold()
    else:v.request_close()
    assert x.jobs[-1].cancelled and not v.start_when_ready
    x.jobs[-1].code=0  # worker races cancel and reports success: still no start
    v.poll_hardware()
    assert not x.launches and v.prepared is None and not v.active and not v.locked
    assert 'cancelled' in v.last_error
    v.poll_hardware();assert not x.launches


@pytest.mark.parametrize('stage',['capture','prepare'])
def test_automatic_check_failure_clears_start_intent(start_view,stage):
    x=start_view;v=x.v;v.execute_clicked()
    if stage=='prepare':x.finish_capture()
    common.atomic_json(x.jobs[-1].folder/'error.json',dict(error='test failed check'))
    x.jobs[-1].code=1;v.poll_hardware()
    assert not v.start_when_ready and not x.launches and not v.locked and v.execute_button.enabled
    assert v.last_error=='test failed check'
    # A later readiness-only success cannot inherit the failed Start intent.
    v.prepare_clicked();x.finish_capture();x.finish_check();assert not x.launches


def test_check_expiring_while_start_pending_does_not_retry_or_move(start_view):
    x=start_view;v=x.v;v.execute_clicked();x.finish_capture();x.finish_check(age=31)
    assert not x.launches and not v.start_when_ready and not v.prepared and not v.job
    assert 'expired' in v.last_error and v.execute_button.enabled


@pytest.mark.parametrize('poll_expiry',[False,True])
def test_start_refreshes_expired_readiness_without_manual_reprepare(start_view,poll_expiry):
    x=start_view;v=x.v;v.prepare_clicked();x.finish_capture();x.finish_check()
    packet=json.loads(v.prepared.read_text());packet['scene_stamp']=time.monotonic()-31
    common.atomic_json(v.prepared,packet)
    if poll_expiry:v.poll_hardware();assert v.execute_button.enabled and not v.locked
    v.execute_clicked()
    assert len(x.jobs)==3 and x.jobs[-1].action=='capture' and v.start_when_ready and not x.launches


def test_start_worker_launch_failure_resets_ui_and_authorization(start_view,monkeypatch):
    from drum_planner.baseline import app
    x=start_view;v=x.v
    def fail(*a,**kw):raise OSError('worker unavailable')
    monkeypatch.setattr(app,'job',fail);v.execute_clicked()
    assert not v.job and not v.start_when_ready and not v.locked and v.execute_button.enabled
    assert v.last_error=='worker unavailable' and not x.launches


def test_simulated_start_button_needs_no_camera_review_or_typed_phrase(start_view):
    x=start_view;x.v.simulated=True;x.v.execute_clicked()
    assert len(x.launches)==1 and not x.jobs and x.launches[0][1]['simulate_plan']==x.v.plan.path


def test_hold_before_owner_ready_cancels_deferred_start(start_view):
    x=start_view;v=x.v;v.simulated=True;v.execute_clicked();v.emergency_hold()
    sent=[]
    v.client=SimpleNamespace(poll=lambda:dict(phase='READY',safe_to_close=True),messages=[],
                             send=lambda op,**kw:sent.append(op))
    for _ in range(5):v.poll_hardware()
    assert not sent and not v.pending_start and not v.start_sent


def test_start_is_disabled_for_unplayable_or_closing_view(start_view):
    x=start_view;v=x.v;v.plan.playable=False;v.execute_clicked();v.prepare_clicked()
    assert not x.launches and not x.jobs
    v.plan.playable=True;v.close_requested=True;v.execute_clicked();v.prepare_clicked()
    assert not x.launches and not x.jobs


def test_button_callbacks_send_one_action_without_dialogs(start_view):
    v=start_view.v;sent=[]
    v.client=SimpleNamespace(send=lambda op,**kw:sent.append((op,kw)))
    v.hardware_status=dict(can_return=True,can_relax=True)
    v.reverse_clicked();v.relax_clicked();v.supported_clicked()
    assert sent==[('reverse',{}),('relax',{}),('supported_exit',{})]
    assert not v.start_when_ready and not v.pending_start


def test_old_controller_reverse_exit_do_not_reintroduce_typed_dialogs(start_view):
    v=start_view.v;sent=[]
    v.client=SimpleNamespace(send=lambda op,**kw:sent.append((op,kw)))
    v.hardware_status=dict(phase='HELD',started=True,can_return=True)
    v.reverse_clicked();v.supported_clicked()
    assert sent==[('reverse',dict(scene_confirmed=True)),('supported_exit',dict(confirmation='ARM PHYSICALLY SUPPORTED'))]


def test_close_window_requests_hold_but_closes_without_relaxing(start_view):
    v=start_view.v;sent=[]
    v.client=SimpleNamespace(send=lambda op,**kw:sent.append(op));v.start_sent=True
    v.hardware_status=dict(phase='OUTWARD',safe_to_close=False)
    v.request_close()
    assert v.close_requested and sent==['hold'] and not v.pending_start


def test_service_relax_cancels_checker_without_recovery_or_confirmation(rig):
    from drum_planner.baseline.service import Server
    r=rig;held(r);s=Server.__new__(Server);s.engine=r.engine;s.pending={'test':True}
    closed=[];s.job=SimpleNamespace(close=lambda:closed.append(True))
    s.command(dict(op='relax'));r.tick()
    assert r.engine.phase=='RELAXED' and s.pending is None and s.job is None and closed==[True]


def test_service_reverse_and_exit_accept_plain_button_commands(rig):
    from drum_planner.baseline.service import Server
    r=rig;held(r);s=Server.__new__(Server);s.engine=r.engine;s.info=dict(simulated=True);s.quit=False
    s.command(dict(op='reverse'));assert s.pending and r.engine.phase=='CHECKING_RETURN'
    r.engine.halt();s.command(dict(op='supported_exit'))
    assert s.quit and r.engine.phase=='CONTROLLER_EXIT' and r.bus.releases==0


@pytest.mark.parametrize('phase',['CONTROLLER_EXIT','SUPPORTED_EXIT'])
def test_client_accepts_acknowledged_controller_exit_then_eof(phase):
    from drum_planner.baseline.service import Client
    c=Client.__new__(Client);c.latest={};c.messages=[];c.buffer=b'';c.last_received=time.monotonic();c.last_heartbeat=time.monotonic()
    incoming=[(json.dumps(dict(status=dict(phase=phase,safe_to_close=True),reply='supported_exit'))+'\n').encode(),b'']
    c.socket=SimpleNamespace(recv=lambda _:incoming.pop(0))
    assert c.poll()['phase']==phase


def test_service_bad_start_confirmation_never_constructs_bus(tmp_path):
    from drum_planner.baseline.service import Server
    s=Server.__new__(Server);s.engine=None;s.bus=None;s.info=dict(simulated=False)
    with pytest.raises(ValueError,match='confirmation'):s.command(dict(op='start',confirmation='yes'))
    assert s.bus is None


def test_real_service_cannot_inject_mock_faults():
    from drum_planner.baseline.service import Server
    s=Server.__new__(Server);s.info=dict(simulated=False)
    with pytest.raises(ValueError,match='unavailable'):s.command(dict(op='inject_fault',kind='clear'))


def test_service_close_active_means_hold_not_quit(rig):
    from drum_planner.baseline.service import Server
    r=rig;r.engine.start();r.until('OUTWARD');r.tick(50)
    s=Server.__new__(Server);s.engine=r.engine;s.quit=False
    with pytest.raises(ValueError,match='Controller is holding'):s.command(dict(op='close'))
    assert not s.quit and r.engine.phase=='HELD' and r.bus.releases==0


def test_operator_disable_attempts_all_right_motors_even_if_one_send_fails():
    from drum_planner.baseline.transport import BaselineMotors
    from centering.motors import packet
    b=BaselineMotors.__new__(BaselineMotors);b.release_permitted=False;b.active=True
    sent=[]
    def send(frame):
        assert b.command_allowed(frame)
        sent.append(frame)
        if frame==packet(4,3):raise OSError('test failed write')
    b.send_control=send
    with pytest.raises(RuntimeError,match='right motor 3'):b.release_on_operator_request()
    assert sent==[packet(4,i) for i in range(1,9)]
    assert not b.release_permitted and not b.active
    assert all(not b.command_allowed(packet(4,i)) for i in range(1,9))


def test_transport_begin_arm_uses_original_wire_gate(tmp_path,rig,monkeypatch):
    from drum_planner.baseline.transport import BaselineMotors
    from safe_zone import encoder
    from centering.motors import packet,parameter,RIGHT_GRIPPER_CLOSED
    created=[];original_open=encoder.os.open
    def opened(path,*a,**k):
        if str(path).startswith('/tmp/openarmx-cartesian-'):path=tmp_path/Path(path).name
        return original_open(path,*a,**k)
    monkeypatch.setattr(encoder.os,'open',opened)
    class Wire:
        def __init__(self,*a,**k):self.sent=[];self.pending=[];self.mode={i:0 for i in range(1,9)};self.targets={};created.append(self)
        def bind(self,addr):
            self.side='right' if addr[0]=='can0' else 'left';q=rig.req['start'] if self.side=='right' else rig.req['left']
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
                self.pending.append(encoder.FRAME.pack(encoder.EFF|(2<<24)|(self.mode[i]<<22)|(i<<8)|0xfd,8,count.to_bytes(2,'big')+bytes.fromhex('8000800000fa')))
            return 16
        def recv(self,n):
            if not self.pending:raise BlockingIOError()
            return self.pending.pop(0)
    monkeypatch.setattr(encoder.socket,'socket',Wire)
    b=BaselineMotors(tmp_path,rig.req['start'])
    try:
        for _ in range(6):b.poll();time.sleep(.015)
        for _ in b.begin_arm(rig.req['start']):b.poll();time.sleep(.002)
        right=next(w for w in created if w.side=='right');left=next(w for w in created if w.side=='left')
        assert all(f in b.queries for f in left.sent)
        assert not any(((encoder.FRAME.unpack(f)[0]>>24)&31)==4 for f in right.sent)
        grip=[f for f in right.sent if encoder.FRAME.unpack(f)[0]&255==8 and encoder.FRAME.unpack(f)[2][:2]==b'\x16\x70']
        assert grip and all(f==parameter(8,0x7016,-RIGHT_GRIPPER_CLOSED) for f in grip)
        # Fresh-fault barrier: a rejected CAN frame must not be followed by a
        # hold packet justified only with old cached healthy telemetry.
        b.failed_at=time.monotonic()+100
        with pytest.raises(RuntimeError,match='complete fresh'):b.poll()
        b.failed_at=None
        b.audit_bytes=8*1024*1024;b.audit('tx_attempt','right',next(iter(b.queries)))
        assert b.audit_rotations==1 and (tmp_path/'execution_can_audit.jsonl.1').exists()
        # Real Baseline adapter, fake wires: explicit release at a NON-rest pose
        # sends only eight right disable packets, including the closed gripper.
        b.set_positions(np.array(rig.req['start'])+.03)
        left_count=len(left.sent);right_count=len(right.sent)
        b.release_on_operator_request()
        assert right.sent[right_count:]==[packet(4,i) for i in range(1,9)]
        assert len(left.sent)==left_count and not b.release_permitted and not b.active
        assert all(not b.command_allowed(packet(4,i)) for i in range(1,9))
    finally:
        count=sum(len(w.sent) for w in created);b.close();assert sum(len(w.sent) for w in created)==count
