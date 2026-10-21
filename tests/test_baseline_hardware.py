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
    spec['proof']=dict(all_valid=True,simulation_only=True);spec['left_snapshot']=request['left_snapshot']
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


@pytest.mark.parametrize('fault',['can','thermal','contact','motor_disabled','scheduler'])
def test_faults_hold_never_auto_reverse_or_relax(rig,fault):
    r=rig;r.engine.start();r.until('OUTWARD');r.tick(50)
    if fault=='can':r.bus.fault='CAN failure'
    elif fault=='thermal':r.bus.fault='motor too hot'
    elif fault=='contact':r.bus.contact=True
    elif fault=='motor_disabled':r.bus.enabled=False
    elif fault=='scheduler':r.clock.t+=.1
    for _ in range(600):
        r.tick()
        if r.engine.phase=='FAULT_HOLD':break
    assert r.engine.phase=='FAULT_HOLD';r.tick(600)
    if fault=='motor_disabled':
        assert r.engine.phase=='DISABLED_EXTERNALLY' and r.engine.disabled and r.bus.releases==0
        assert not r.engine.status()['can_return']
        assert r.engine.events[-1]['motor_commands_sent'] is False
        return  # recognizing an external disable is not sending a relaxation
    assert not r.engine.disabled and r.bus.releases==0
    assert 'RECOVERY' not in [x['phase'] for x in r.engine.events]
    if fault in ('can','thermal','motor_disabled'):
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
    spec=paths.recovery(r.engine.curve,request['held'],request['tau'],request['rate']);spec['proof']=dict(all_valid=True);spec['left_snapshot']=request['left_snapshot']
    r.engine.halt()
    assert not r.engine.accept_return(spec,request['generation']) and r.bus.releases==0


def test_hold_during_recovery_can_be_recovered_again(rig):
    r=rig;held(r,'RETURN');request=r.engine.request_return(True);r.tick(70)
    spec=paths.recovery(r.engine.curve,request['held'],request['tau'],request['rate']);spec['proof']=dict(all_valid=True);spec['left_snapshot']=request['left_snapshot']
    r.engine.accept_return(spec,request['generation']);r.tick(90);r.engine.halt();r.tick(70)
    return_from_hold(r)


def test_no_drift_following_while_held(rig):
    r=rig;held(r);anchor=r.engine.held.copy();n=len(r.commands)
    r.bus.q[0]+=.03;r.tick(70);r.engine.halt('another fault',fault=True);r.tick(70)
    assert not r.engine.hold_confirmed and np.array_equal(r.engine.held,anchor) and len(r.commands)==n
    with pytest.raises(ValueError):r.engine.request_return(True)
    assert r.bus.releases==0


def test_external_disable_requires_continuous_fresh_all_motor_feedback(rig,monkeypatch):
    r=rig;held(r);count=len(r.commands);r.bus.enabled=False
    r.tick(20);assert r.engine.phase=='HELD' and not r.engine.disabled
    r.bus.enabled=True;r.tick();r.bus.enabled=False;r.tick(20)
    assert not r.engine.disabled
    poll=r.bus.poll
    monkeypatch.setattr(r.bus,'poll',lambda:None);r.tick(60)
    assert not r.engine.disabled  # disabled-looking but stale is NOT enough
    monkeypatch.setattr(r.bus,'poll',poll);r.tick(70)
    assert r.engine.phase=='DISABLED_EXTERNALLY' and r.engine.disabled
    assert r.bus.releases==0 and len(r.commands)==count


@pytest.mark.parametrize('problem',['one_enabled','one_missing','fault','bus_error','temperature'])
def test_external_disable_does_not_release_on_incomplete_or_bad_feedback(rig,monkeypatch,problem):
    r=rig;held(r);r.bus.enabled=False;poll=r.bus.poll
    def bad():
        poll()
        if problem=='one_enabled':r.bus.states['right',8]=(0.,2,r.clock.t)
        if problem=='one_missing':r.bus.states.pop(('left',8))
        if problem=='fault':r.bus.health={('right',2):dict(fault=1,temperature=30.)}
        if problem=='bus_error':r.bus.failed_at=r.clock.t
        if problem=='temperature':r.bus.health={('right',2):dict(fault=0,temperature=80.)}
    monkeypatch.setattr(r.bus,'poll',bad);r.tick(100)
    assert r.engine.phase=='HELD' and not r.engine.disabled and r.bus.releases==0


@pytest.mark.parametrize('connected',[False,True])
@pytest.mark.parametrize('phase',['RELAXED','DISABLED_EXTERNALLY','REJECTED','CONTROLLER_EXIT'])
def test_finished_owner_releases_bus_without_disable_commands(rig,phase,connected):
    from drum_planner.baseline.service import Server
    r=rig;closed=[];r.bus.close=lambda:closed.append(True)
    r.engine.phase=phase;s=Server.__new__(Server);s.engine=r.engine;s.bus=r.bus;s.quit=False
    s.owner=object() if connected else None
    s.release_finished_bus();s.release_finished_bus()
    assert closed==[True] and s.bus is None and r.bus.releases==0
    assert s.quit is (not connected)


@pytest.mark.parametrize('phase',['OUTWARD','HELD','FAULT_HOLD','CHECKING_RETURN','RELAX_UNCONFIRMED'])
def test_unfinished_owner_keeps_bus_and_hold_ownership(rig,phase):
    from drum_planner.baseline.service import Server
    r=rig;closed=[];r.bus.close=lambda:closed.append(True)
    r.engine.phase=phase;s=Server.__new__(Server);s.engine=r.engine;s.bus=r.bus;s.quit=False;s.owner=None
    s.release_finished_bus()
    assert not closed and s.bus is r.bus and not s.quit and r.bus.releases==0


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
                spec['proof']=dict(all_valid=True);spec['left_snapshot']=request['left_snapshot'];r.engine.accept_return(spec,request['generation']);r.tick(20)
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
    spec=paths.recovery(r.engine.curve,request['held'],request['tau'],request['rate']);spec['proof']=dict(all_valid=True);spec['left_snapshot']=request['left_snapshot']
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
    from drum_planner import bus_ownership
    monkeypatch.setattr(bus_ownership,'check_available',lambda:None)  # isolated worker test, not live lock diagnosis
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
    from drum_planner.baseline import workspace
    def fail(*a,**kw):raise ValueError('test startup failure')
    monkeypatch.setattr(workspace,'show',fail)
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


@pytest.mark.parametrize('when',['before_start','outward'])
def test_event_log_failure_warns_but_completes_cycle(rig,when):
    r=rig
    def broken(_):raise OSError('disk full')
    if when=='before_start':r.engine.sink=broken
    r.engine.start();r.until('OUTWARD')
    r.engine.sink=broken;r.until('RELAXED')
    assert r.bus.releases==1 and np.allclose(r.bus.q,r.req['start'])
    assert 'Event log unavailable' in r.engine.status()['warnings'][0]
    assert not r.engine.error and 'FAULT_HOLD' not in [x['phase'] for x in r.engine.events]


def test_can_audit_failure_warns_without_interrupting_playback_or_return(rig):
    r=rig;r.engine.start();r.until('OUTWARD');r.tick(40)
    r.bus.audit_error='disk full';r.tick()
    assert r.engine.phase=='OUTWARD' and not r.engine.error
    assert any('CAN audit unavailable' in s for s in r.engine.status()['warnings'])
    r.engine.halt();r.tick(80)
    assert r.engine.hold_confirmed
    return_from_hold(r)
    assert r.bus.releases==1


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


@pytest.mark.parametrize('bad',['fake_stamp','changed_plan','old_schema','authorized','request','proof',
    'self_margin','missing_self_margin','margin_policy','missing_margin_policy','proof_samples','proof_valid','zone'])
def test_prepared_packet_changes_rejected(prepared_fixture,monkeypatch,bad):
    p=prepared_fixture;data=json.loads(p.read_text())
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


@pytest.mark.parametrize('rate',[.5,1.])
def test_dynamic_recording_playback_accepts_realistic_csp_following_lag(rig,rate):
    r=rig;r.engine.rate=rate
    # Seven independent firmware servos follow, not teleport to the command.
    target=r.bus.q.copy();last=[r.clock.t];poll=r.bus.poll
    def command(q):
        nonlocal target
        target=np.asarray(q).copy();r.commands.append(target.copy())
    def delayed_poll():
        dt=max(0.,r.clock.t-last[0]);last[0]=r.clock.t
        r.bus.q+=(target-r.bus.q)*(1.-np.exp(-dt/.08))
        poll()
    r.bus.set_positions=command;r.bus.poll=delayed_poll
    r.engine.start();r.until('RELAXED')
    assert np.sum(r.engine.peak_tracking_error)>motion.TRACK_L1
    assert r.bus.releases==1 and r.engine.disabled
    assert not any(x['phase']=='FAULT_HOLD' for x in r.engine.events)
    assert np.max(abs(r.bus.q-r.req['start']))<=motion.REST_TOL


def test_target_step_limit_is_per_joint_not_summed_across_axes(rig):
    r=rig;r.engine.start();r.until('OUTWARD')
    target=r.engine.anchor+np.full(7,.004)
    assert sum(abs(target-r.engine.anchor))>.025
    r.engine.send(target)
    assert np.array_equal(r.commands[-1],target)
    with pytest.raises(RuntimeError,match='discontinuity'):
        r.engine.send(target+.1)


def test_new_dynamic_monitor_never_turns_off_a_stalled_arm(rig):
    r=rig;r.engine.start();r.until('OUTWARD');r.bus.contact=True
    for _ in range(1000):
        r.tick()
        if r.engine.phase=='FAULT_HOLD':break
    assert r.engine.phase=='FAULT_HOLD'
    assert 'tracking error' in r.engine.error or 'STALL:' in r.engine.error
    assert r.engine.hold_confirmed and not r.engine.disabled and r.bus.releases==0
    r.tick(500)
    assert r.engine.phase=='FAULT_HOLD' and r.bus.releases==0


def test_window_close_does_not_poll_or_send_duplicate_close(monkeypatch):
    from drum_planner.baseline import app
    from drum_planner.snapshot_preview import SnapshotPlaybackView
    v=object.__new__(app.View);v.start_when_ready=False;v.pending_start=False
    v.start_sent=True;v.hardware_status={'phase':'RELAXED','safe_to_close':True}
    v.job=None;v.client_info={};v.close_requested=False;sent=[]
    v.client=SimpleNamespace(close=lambda:sent.append('socket.close'))
    v.send=lambda op:sent.append(op)
    v.poll_hardware=lambda:(_ for _ in ()).throw(AssertionError('Do not poll a closing connection'))
    monkeypatch.setattr(SnapshotPlaybackView,'poll',lambda *a:ord('q'))
    monkeypatch.setattr(SnapshotPlaybackView,'close',lambda *a:None)
    v.request_close();assert v.close_requested and not sent
    assert v.poll()==ord('q') and not sent
    v.close();assert sent==['close','socket.close']


@pytest.mark.parametrize('phase',['READY','OUTWARD','HELD','FAULT_HOLD','CHECKING_RETURN','RELAXED','DISABLED_EXTERNALLY'])
def test_reverse_request_stays_clickable_even_when_owner_reports_not_ready(start_view,phase):
    v=start_view.v;sent=[]
    status=dict(phase=phase,error='test fault',feedback_error='hold unconfirmed',can_return=False,hold_confirmed=False,can_relax=False)
    v.client=SimpleNamespace(poll=lambda:status,messages=[],send=lambda op,**kw:sent.append(op))
    for _ in range(5):v.poll_hardware()
    assert v.reverse_button.enabled
    v.reverse_clicked()
    assert sent==['reverse']  # stale/not-ready status never swallows the click


def test_reverse_disconnected_reconnects_and_reports_missing_owner(start_view):
    v=start_view.v;sent=[];v.client_info={'known_owner':True}
    def reconnect():v.client=SimpleNamespace(send=lambda op,**kw:sent.append(op))
    v.reconnect=reconnect;v.reverse_clicked();assert sent==['reverse']
    v.client=None;v.client_info=None;v.reverse_clicked()
    assert 'no connected active trajectory' in v.last_error and v.reverse_button.enabled


def test_reverse_rejection_reason_stays_visible_between_status_updates(start_view):
    v=start_view.v
    v.client=SimpleNamespace(poll=lambda:dict(phase='FAULT_HOLD',error='old fault',feedback_error='',can_relax=True),
        messages=[dict(reply='reverse',error='Right drive not running')])
    v.poll_hardware();assert 'Right drive not running' in v.last_error
    v.poll_hardware();v.poll_hardware()
    assert 'Reverse not started: Right drive not running' in v.hardware_label.text
    assert v.reverse_button.enabled


@pytest.mark.parametrize('change',['arm','gripper'])
def test_relaxed_left_change_can_return_without_restoring_original_left_pose(rig,change):
    r=rig;original=copy.deepcopy(r.req);r.engine.start();r.until('OUTWARD');r.tick(50)
    r.bus.req=copy.deepcopy(r.req)
    if change=='arm':r.bus.req['left'][5]-=np.radians(.4)
    else:r.bus.req['left_opening_m']=.003
    r.tick(70)
    assert r.engine.phase=='OUTWARD'  # passive left sway does not interrupt playback
    r.engine.halt();r.tick(70)  # explicit operator hold still permits checked recovery
    spec=return_from_hold(r)
    assert spec['left_snapshot']['left']==r.bus.req['left']
    assert spec['left_snapshot']['left_opening_m']==pytest.approx(r.bus.req['left_opening_m'])
    assert r.engine.return_left==spec['left_snapshot'] and r.req==original
    assert r.bus.releases==1 and np.allclose(r.bus.q,r.req['start'])


def test_fault_without_confirmed_hold_still_allows_request_and_fresh_retry(rig):
    r=rig;r.engine.start();r.until('OUTWARD');r.tick(20);r.bus.fault='CAN failure';r.tick()
    assert r.engine.phase=='FAULT_HOLD' and not r.engine.hold_confirmed and r.engine.status()['can_return']
    with pytest.raises(RuntimeError,match='CAN failure'):r.engine.request_return(True)
    assert r.bus.releases==0 and r.engine.phase=='FAULT_HOLD'
    r.bus.fault='';r.tick(70)
    # Nothing automatically moves after communication returns.
    assert r.engine.phase=='FAULT_HOLD' and r.bus.releases==0
    with pytest.raises(ValueError,match='Fresh hold established'):r.engine.request_return(True)
    r.tick(70);return_from_hold(r)


@pytest.mark.parametrize('problem',['new_left_shift','new_gripper_shift','wrong_proof_snapshot'])
def test_recovery_accepts_passive_left_motion_but_rejects_wrong_proof(rig,problem):
    r=rig;held(r);request=r.engine.request_return(True);r.tick(70)
    spec=paths.recovery(r.engine.curve,request['held'],request['tau'],request['rate'])
    spec.update(proof=dict(all_valid=True),left_snapshot=copy.deepcopy(request['left_snapshot']))
    r.bus.req=copy.deepcopy(r.req)
    if problem=='new_left_shift':r.bus.req['left'][0]+=.02
    elif problem=='new_gripper_shift':r.bus.req['left_opening_m']=.004
    else:spec['left_snapshot']['left'][0]+=.02
    if problem=='wrong_proof_snapshot':
        with pytest.raises(ValueError,match='snapshot does not match'):r.engine.accept_return(spec,request['generation'])
        assert r.bus.releases==0 and r.engine.phase=='CHECKING_RETURN'
    else:
        assert r.engine.accept_return(spec,request['generation'])
        r.until('RELAXED');assert r.bus.releases==1


def test_failed_return_checker_does_not_stick_in_checking_state(rig,monkeypatch):
    from drum_planner.baseline.service import Server
    r=rig;held(r);s=Server.__new__(Server)
    s.engine=r.engine;s.info={'simulated':True};s.job=None;s.pending=None
    def fail(*a,**kw):raise ValueError('test collision checker failed')
    monkeypatch.setattr(paths,'recovery',fail)
    with pytest.raises(ValueError,match='checker failed'):s.command({'op':'reverse'})
    assert r.engine.phase=='FAULT_HOLD' and r.engine.status()['can_return'] and r.bus.releases==0
    assert s.job is None and s.pending is None


def test_native_recovery_checker_receives_current_left_pose(rig,tmp_path,monkeypatch):
    r=rig;packet=tmp_path/'prepared.json'
    plan=tmp_path/'plan.json';plan.write_text('{}')
    common.atomic_json(packet,dict(plan=str(plan),calibration='unit',fresh_scene='unit-scene',fresh_scene_sha256='digest'))
    monkeypatch.setattr(tasks,'load_checked',lambda *a:(r.obj,copy.deepcopy(r.req),r.engine.curve))
    monkeypatch.setattr(planning,'scene',lambda *a:dict(source_scene_sha256='digest'))
    calls=[]
    def check(req,q,folder):calls.append((copy.deepcopy(req),q));return dict(all_valid=True,samples=len(q))
    monkeypatch.setattr(tasks,'hard_check',check)
    left=dict(left=(np.array(r.req['left'])+.01).tolist(),left_opening_m=.004)
    tasks.check_return(tmp_path/'return',packet,r.req['start'],0,1,left)
    result=json.loads((tmp_path/'return/return.json').read_text())
    assert calls[0][0]['left']==left['left'] and calls[0][0]['left_opening_m']==.004
    assert result['left_snapshot']==left and result['proof']['all_valid']
    assert calls[0][0]['start']==r.req['start']


@pytest.mark.parametrize('stage',['capture','prepare','owner_connect'])
def test_reverse_click_cancels_deferred_start_without_enabling(start_view,stage):
    x=start_view;v=x.v
    if stage=='owner_connect':
        v.simulated=True;v.execute_clicked();assert v.pending_start
        v.reconnect=lambda:None  # owner not yet listening
        v.reverse_clicked();assert not v.pending_start and not v.start_sent
    else:
        v.execute_clicked()
        if stage=='prepare':x.finish_capture()
        v.reverse_clicked();assert x.jobs[-1].cancelled and not v.start_when_ready
        x.jobs[-1].code=0;v.poll_hardware();assert not x.launches and not v.pending_start
    assert v.reverse_button.enabled


def test_owner_display_reports_measured_pose_not_target_and_keeps_original_stamps(rig):
    from drum_planner.baseline.feedback import owner_pose
    r=rig;held(r)
    r.engine.anchor=r.engine.q+1.  # display must not follow this commanded target
    pose=r.engine.status()['measured_pose']
    assert np.allclose([pose['joints'][j] for j in common.JOINTS],r.bus.q)
    assert np.allclose([pose['joints'][j] for j in common.LEFT_JOINTS],r.req['left'])
    stamp=pose['stamp'];r.clock.t+=2.
    assert r.engine.status()['measured_pose']['stamp']==stamp
    r.bus.states['right',1]=(float('nan'),2,stamp)
    assert owner_pose(r.bus.states) is None  # no exception on owner's fault/status channel


def test_display_telemetry_latest_only_and_releases_without_motor_commands(tmp_path):
    from drum_planner.baseline.feedback import DisplayTelemetry
    class Observer:
        def __init__(self,folder):self.n=0;self.closed=False
        def sample(self):
            self.n+=1
            return dict(start=time.monotonic(),joints=dict.fromkeys(common.JOINTS+common.LEFT_JOINTS, self.n*.01)|
                        {'openarmx_left_finger_joint1':.01})
        def close(self):self.closed=True
        def report(self):return dict(closed=self.closed,motion_commands=0)
    reader=DisplayTelemetry(tmp_path,Observer)
    try:
        until=time.monotonic()+2
        while reader.count<3 and time.monotonic()<until:time.sleep(.01)
        assert reader.count>=3 and not reader.error
        assert not hasattr(reader,'history')
        assert reader.snapshot()['joints'][common.JOINTS[0]]>=.03
    finally:reader.close()
    assert not reader.thread.is_alive() and reader.observer.closed
    report=json.loads((tmp_path/'display_query_report.json').read_text())
    assert report['history_capacity']==1 and report['motor_commands']==0


def test_idle_display_audits_are_bounded_across_handoffs(tmp_path):
    from drum_planner.baseline.feedback import RollingAudit
    p=tmp_path/'audit.jsonl'
    for session in range(3):
        log=RollingAudit(p,limit=512)
        for i in range(70):log.write(json.dumps(dict(query=i,session=session))+'\n')
        log.flush();log.close()
    assert len(list(tmp_path.iterdir()))==2
    assert all(f.stat().st_size<=512 for f in tmp_path.iterdir())
    assert json.loads(p.read_text().splitlines()[-1])==dict(query=69,session=2)


def test_readiness_replans_once_without_changing_margins_or_authorizing(prepared_fixture,monkeypatch):
    packet=json.loads(prepared_fixture.read_text());base=prepared_fixture.parent.parent
    calls=[];plans=[];normal=tasks.hard_check
    def check(req,q,folder):
        calls.append((copy.deepcopy(req),folder))
        if len(calls)==1:raise tasks.PathBlocked([dict(sample_index=3,reason='collision',contacts=[['gemini_native_depth','right_drumstick']])])
        return normal(req,q,folder)
    monkeypatch.setattr(tasks,'hard_check',check)
    monkeypatch.setattr(tasks,'make_plan',lambda *a:plans.append(a) or packet['plan'])
    target=base/'replan'
    tasks.prepare(target,packet['plan'],base,packet['calibration'])
    new=json.loads((target/'prepared.json').read_text())
    assert len(plans)==1 and len(calls)==2
    assert tasks.margin_contract(calls[0][0])==tasks.margin_contract(calls[1][0])
    assert new['replanned']['first_blocked_time_s']==.006
    assert new['replanned']['explicit_start_required'] is True and new['motion_authorized'] is False
    assert new['proof']['all_valid'] and (target/'replan_reason.json').exists()


def test_replan_never_loops_or_ignores_invalid_replacement(prepared_fixture,monkeypatch):
    packet=json.loads(prepared_fixture.read_text());base=prepared_fixture.parent.parent;plans=[]
    def blocked(*a):raise tasks.PathBlocked([dict(sample_index=0,reason='collision')])
    monkeypatch.setattr(tasks,'hard_check',blocked)
    monkeypatch.setattr(tasks,'make_plan',lambda *a:plans.append(a) or packet['plan'])
    target=base/'blocked'
    with pytest.raises(tasks.PathBlocked):tasks.prepare(target,packet['plan'],base,packet['calibration'])
    assert len(plans)==1 and not (target/'prepared.json').exists()


def test_readiness_backend_error_is_not_a_collision_or_replanning_request(prepared_fixture,monkeypatch):
    packet=json.loads(prepared_fixture.read_text());base=prepared_fixture.parent.parent
    def failed(*a):raise ValueError('Backend incomplete')
    monkeypatch.setattr(tasks,'hard_check',failed)
    monkeypatch.setattr(tasks,'make_plan',lambda *a:pytest.fail('Do not replan after backend errors'))
    with pytest.raises(ValueError,match='Backend incomplete'):
        tasks.prepare(base/'incomplete',packet['plan'],base,packet['calibration'])


@pytest.mark.parametrize('age',[0,31])
def test_replacement_never_executes_pending_start_and_is_previewable(start_view,age):
    x=start_view;v=x.v;adopted=[]
    v.adopt_replanned=lambda p:adopted.append(p)
    v.execute_clicked();x.finish_capture()
    common.atomic_json(x.jobs[-1].folder/'prepared.json',dict(scene_stamp=time.monotonic()-age,
        fresh_scene='new-scene.json',request=dict(scene={}),plan='new-plan.json',replanned=dict(explicit_start_required=True)))
    x.jobs[-1].code=0;v.poll_hardware()
    assert adopted and not v.start_when_ready and not v.pending_start and not x.launches
    assert v.execute_button.enabled and v.prepare_button.enabled
    assert ('REPLACEMENT' in v.hardware_label.text) and not v.last_error
    assert bool(v.prepared)==(age==0)


@pytest.mark.parametrize('action',['prepare','start'])
def test_query_reader_releases_before_other_can_owner_starts(start_view,monkeypatch,action):
    x=start_view;v=x.v;events=[]
    v.telemetry=SimpleNamespace(close=lambda:events.append('reader_closed'))
    if action=='prepare':
        v.prepare_clicked()
        assert x.jobs[-1].action=='capture'
    else:
        v.prepared=Path('prepared.json');v.start_controller()
        assert x.launches
    assert events==['reader_closed'] and v.telemetry is None


def test_unreleased_display_reader_cannot_start_another_can_owner(start_view):
    x=start_view;v=x.v
    def failed():raise RuntimeError('Reader cannot release CAN')
    v.telemetry=SimpleNamespace(close=failed)
    v.execute_clicked()
    assert not x.jobs and not x.launches and not v.start_when_ready
    assert 'release CAN' in v.last_error


def test_live_display_uses_existing_owner_without_opening_second_reader(start_view,monkeypatch):
    from drum_planner.baseline import feedback
    v=start_view.v
    joints=dict.fromkeys(common.JOINTS+common.LEFT_JOINTS, .1)|{'openarmx_left_finger_joint1':.01}
    pose=feedback.normalized_pose(joints,time.monotonic())
    v.client_info={'owner':True};v.hardware_status={'measured_pose':pose,'phase':'FAULT_HOLD'}
    stopped=[];v.telemetry=SimpleNamespace(close=lambda:stopped.append(True))
    v.live_display=False;v.display_pose=None;v.display_error='';v.next_telemetry=0
    v.feedback_label=v.hardware_label
    monkeypatch.setattr(feedback,'DisplayTelemetry',lambda *a:pytest.fail('Must not compete with motor owner'))
    v.poll_feedback()
    assert stopped and v.telemetry is None and v.display_pose==pose
    assert 'MEASURED encoders' in v.feedback_label.text
    pose['stamp']-=2.;v.poll_feedback()
    assert 'unavailable/stale' in v.feedback_label.text


def test_simulated_view_never_opens_idle_can_reader(start_view,monkeypatch):
    from drum_planner.baseline import feedback
    v=start_view.v;v.simulated=True;v.live_display=False;v.display_pose=None
    v.display_error='';v.feedback_label=v.hardware_label
    monkeypatch.setattr(feedback,'DisplayTelemetry',lambda *a:pytest.fail('Simulated UI must not open CAN'))
    v.poll_feedback()
    assert 'no CAN access' in v.feedback_label.text


@pytest.mark.parametrize('rate',[.25,.6,1.])
def test_continuously_swinging_left_arm_and_gripper_do_not_interrupt_cycle(rig,rate):
    r=rig;r.engine.rate=rate;r.bus.req=copy.deepcopy(r.req);r.engine.start()
    for n in range(20000):
        r.bus.req['left']=(np.array(r.req['left'])+np.radians(5)*np.sin(n*.05+np.arange(7))).tolist()
        r.bus.req['left_opening_m']=.01*(1+np.sin(n*.07))
        r.tick()
        assert r.engine.phase not in {'FAULT_HOLD','REJECTED'},r.engine.error
        if r.engine.phase=='RELAXED':break
    assert r.engine.phase=='RELAXED' and r.bus.releases==1
    assert np.allclose(r.bus.q,r.req['start'])


def test_prepared_frozen_scene_does_not_expire_while_previewing(prepared_fixture,monkeypatch):
    data=json.loads(prepared_fixture.read_text())
    monkeypatch.setattr(tasks.time,'monotonic',lambda:data['scene_stamp']+3600)
    packet,curve=tasks.check_packet(prepared_fixture)
    assert packet['plan']==data['plan'] and curve.duration>0


def test_prepare_does_not_reject_passive_left_sway(prepared_fixture):
    packet=json.loads(prepared_fixture.read_text());base=prepared_fixture.parent.parent
    q=json.loads((base/'measured_start.json').read_text());q['left_rad'][2]+=.12;q['left_gripper_opening_m']=.012
    common.atomic_json(base/'measured_start.json',q)
    tasks.prepare(base/'left_sway',packet['plan'],base,packet['calibration'],allow_replan=False)
    prepared,_=tasks.check_packet(base/'left_sway/prepared.json')
    assert prepared['plan']==packet['plan'] and prepared['proof']['all_valid']


def test_status_file_failure_warns_without_control_halt(rig,tmp_path,monkeypatch):
    from drum_planner.baseline import service
    r=rig;r.engine.start();r.until('OUTWARD')
    server=service.Server.__new__(service.Server);server.folder=tmp_path;server.engine=r.engine
    server.info=dict(simulated=True);server.job=None;server.connection_error='';server.bus=r.bus
    monkeypatch.setattr(service,'atomic_json',lambda *a:(_ for _ in ()).throw(OSError('disk full')))
    server.save_status();assert 'disk full' in server.status()['warnings'][-1]
    r.tick(30);assert r.engine.phase=='OUTWARD' and not r.engine.error
    r.engine.halt();r.tick(70);return_from_hold(r)
