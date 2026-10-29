"""Current hardware planner parity, immutable profiles and DT1; no real devices."""
import copy,json,socket
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.interpolate import PPoly
from drum_planner.common import ROOT,DEFAULT_CALIBRATION,atomic_json,digest
from drum_planner.baseline import settings,targets,planner,current,paths,computation,work,tasks
from drum_planner.baseline.curve import Curve
from drum_planner.timing import core
from drum_planner.experiment import motion_settings,stationary,rgbd


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    old=socket.socket
    def guard(family=socket.AF_INET,*a,**kw):
        if family==socket.AF_CAN:raise AssertionError('Physical CAN forbidden')
        return old(family,*a,**kw)
    monkeypatch.setattr(socket,'socket',guard)
    from drum_planner import depth
    monkeypatch.setattr(depth,'DepthCamera',lambda *a,**kw:pytest.fail('Physical camera forbidden'))
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')


def example(side='right',limits=None,stationary=False):
    profile=settings.settings(side)
    if limits is not None:profile.update(dict(zip(settings.FIELDS,limits)))
    limits=settings.matrix(profile)
    delta=np.zeros(7) if stationary else np.repeat(.1,7)
    c=np.array([6*delta,-15*delta,10*delta,np.zeros(7),np.zeros(7),np.zeros(7)])[:,None,:]
    law=dict(schema='x5-fixed-path-timing-v1',kind='joint_polynomial',curve=core.encode(PPoly(c,[0,1])),
             duration_s=0. if stationary else 1.,limits=limits.tolist(),stationary=stationary)
    ev=core.Evaluator(law);law['peaks']=core.polynomial_peaks(ev.p)[0].tolist()
    obj=dict(moving_side=side,timing=law,outward=[dict(time_s=0.,positions_rad=[0.]*7)],
        hardware_planning_pipeline=settings.PIPELINE,hardware_motion_profile=profile,
        hardware_motion_profile_sha256=settings.identity(profile))
    for key,row in zip(('velocity_limit_rad_s','acceleration_limit_rad_s2','jerk_limit_rad_s3'),limits):obj[key]=row.tolist()
    if not stationary:obj['outward'].append(dict(time_s=1.,positions_rad=delta.tolist()))
    return obj


@pytest.mark.parametrize('side',['right','left'])
def test_profile_defaults_roundtrip_and_experiment_import(tmp_path,side):
    p=settings.settings(side);assert np.array_equal(settings.matrix(p),settings.DEFAULT_LIMITS)
    assert settings.for_optimizer(p)==motion_settings.settings(side)
    assert settings.settings(side,motion_settings.settings(side))==p
    file=tmp_path/'profile.json';settings.save(file,p);assert settings.load(file,side)==p
    assert p['firmware_settings_changed'] is False and p['hardware_limits_verified'] is False
    with pytest.raises(ValueError):settings.load(file,'right' if side=='left' else 'left')


@pytest.mark.parametrize('changes',[{'timing_quality':'unknown'},{'hardware_limits_verified':True},
    {'firmware_settings_changed':True},{'velocity_rad_s':[0]*7},{'side':'left'},{'extra':1}])
def test_invalid_profiles_rejected(changes):
    with pytest.raises(ValueError):settings.settings('right',dict(settings.settings(),**changes))


def test_hardware_default_routes_to_same_current_engine(monkeypatch):
    seen=[];monkeypatch.setattr(current,'plan',lambda *a,**kw:seen.append(kw) or 'result')
    assert planner.plan('a','b','c','d',moving_side='left',motion_profile={'test':True})=='result'
    assert seen[0]['moving_side']=='left' and seen[0]['motion_profile']=={'test':True}


@pytest.mark.parametrize('tamper',['profile','law','reported','unknown'])
def test_profile_hash_and_limits_bound_to_curve(tamper):
    obj=example()
    if tamper=='profile':obj['hardware_motion_profile']['velocity_rad_s'][0]*=2
    elif tamper=='law':obj['timing']['limits'][0][0]*=2
    elif tamper=='reported':obj['velocity_limit_rad_s'][0]*=2
    else:obj['hardware_planning_pipeline']='unknown'
    with pytest.raises(ValueError):Curve(obj)


def test_actual_extrema_not_forged_metadata():
    obj=example(limits=[[.1]*7,[1.2]*7,[6]*7]);obj['timing']['peaks']=[[0]*7]*3
    with pytest.raises(ValueError,match='exceeds'):paths.validate_timing(Curve(obj))


def test_unchanged_drive_cap_is_not_claimed_as_motor_rating():
    obj=example(limits=[[2]*7,[20]*7,[60]*7])
    c=Curve(obj);c._peaks[0]=1.
    with pytest.raises(ValueError,match='drive cap'):paths.validate_drive_speed(c,1.)
    paths.validate_drive_speed(c,.5)


def test_recovery_honors_lower_profile_without_accelerating_higher_profiles():
    default=Curve(example());p=default.plan
    # The same geometric curve under a larger envelope must not speed recovery.
    high=Curve(example(limits=(settings.DEFAULT_LIMITS*2).tolist()))
    low=Curve(example(limits=(settings.DEFAULT_LIMITS*.5).tolist()))
    args=(default.at(.6),.6,1.)
    a=paths.recovery(default,*args);b=paths.recovery(high,*args);c=paths.recovery(low,*args)
    assert a==b and c['return_s']>a['return_s']


def test_checked_stationary_hold_and_reject_zero_time_jump():
    obj=example(stationary=True);c=Curve(obj);paths.validate_timing(c)
    assert c.duration==0 and np.array_equal(c.at(0),np.zeros(7))
    obj['timing']['stationary']=False
    with pytest.raises(ValueError,match='Zero-duration'):Curve(obj)
    obj=example();obj['timing']['duration_s']=0;obj['timing']['stationary']=True
    with pytest.raises(ValueError,match='Zero-duration'):Curve(obj)


def test_dt1_selection_freezes_saved_but_not_generated_goal(tmp_path):
    paths_=[str(ROOT/'destinations/right/destinationR1.json'),'dt1']
    obj=targets.freeze(paths_,'right',tmp_path)
    assert targets.count(obj)==2 and targets.has_dt1(obj)
    assert obj['entries'][1]==dict(kind='dt1')
    assert obj['entries'][0]['destination_sha256']==digest(paths_[0])
    with pytest.raises(ValueError,match='right-arm'):targets.freeze(['dt1'],'left',tmp_path)
    with pytest.raises(ValueError,match='Consecutive'):targets.freeze(['dt1','dt1'],'right',tmp_path)
    obj['entries'].reverse()
    with pytest.raises(ValueError,match='changed'):targets.resolve({'moving_side':'right'},obj,tmp_path/'result')


@pytest.mark.parametrize('backend',['current','adaptive','sparse'])
def test_subprocess_forwards_profile_and_backend(tmp_path,monkeypatch,backend):
    seen=[]
    def run(command,worker):seen.append(command);atomic_json(tmp_path/'computation/result.json',{'ok':True});return 0
    monkeypatch.setattr(computation,'_run',run)
    p=settings.settings('left') if backend=='current' else None
    assert computation.run(tmp_path,DEFAULT_CALIBRATION,SimpleNamespace(error=None),moving_side='left',
                           motion_profile=p,timing_backend=backend)=={'ok':True}
    assert ('--timing-backend' in seen[0])==(backend!='current')
    assert ('--motion-profile' in seen[0])==(backend=='current')
    if p:assert json.loads((tmp_path/'motion_profile.json').read_text())==p


@pytest.mark.parametrize('side,backend',[('right','current'),('left','current'),('right','sparse'),('left','sparse')])
def test_calculate_camera_routing_and_same_monitor_lifetime(tmp_path,monkeypatch,side,backend):
    seen=[]
    class Telemetry:
        error=''
        def __init__(self,*a,**kw):self.closed=False
        def close(self):self.closed=True;seen.append('closed')
        def report(self):return {'closed':self.closed}
    monkeypatch.setattr(work,'Telemetry',Telemetry)
    def capture(folder,*a,**kw):
        folder.mkdir();seen.append('paired');atomic_json(folder/'scene.json',{});atomic_json(folder.parent/'measured_start.json',{})
    def legacy(folder,*a,**kw):
        (folder/'camera').mkdir();seen.append('legacy');atomic_json(folder/'camera/scene.json',{});atomic_json(folder/'measured_start.json',{});return {},{}
    monkeypatch.setattr(rgbd,'capture',capture);monkeypatch.setattr(work,'capture_current',legacy)
    monkeypatch.setattr(work,'verify_unchanged',lambda *a,**kw:dict(test=True))
    def compute(folder,cal,reader,**kw):
        assert not reader.closed and kw['moving_side']==side;seen.append('compute');return {}
    monkeypatch.setattr(computation,'run',compute)
    work.calculate(tmp_path,DEFAULT_CALIBRATION,{},moving_side=side,timing_backend=backend)
    assert seen==['paired' if side=='right' and backend=='current' else 'legacy','compute','closed']


def test_acquisition_changed_rejected_before_planning(tmp_path,monkeypatch):
    capture=tmp_path/'capture';(capture/'camera').mkdir(parents=True);atomic_json(capture/'measured_start.json',{});atomic_json(capture/'camera/scene.json',{})
    atomic_json(capture/'acquisition.json',dict(state=str(capture/'measured_start.json'),state_sha256='wrong'))
    monkeypatch.setattr(planner,'plan',lambda *a,**kw:pytest.fail('No calculation on changed acquisition'))
    with pytest.raises(ValueError,match='changed'):tasks.make_plan(tmp_path/'plan',capture,DEFAULT_CALIBRATION)


@pytest.mark.parametrize('side,backend,has_dt1',[('right','current',True),('left','current',False),('right','sparse',False)])
def test_ui_choices_no_calculation_on_open_or_edit(tmp_path,side,backend,has_dt1):
    from drum_planner.baseline.workspace import Workspace
    view=Workspace(tmp_path,moving_side=side,timing_backend=backend,autoscene=False)
    try:
        box=view.control.destinations.boxes[0]
        assert (box.findData('dt1')>=0)==has_dt1
        assert view.plan is None and view.job is None and view.owner_info is None
        assert not hasattr(view.control,'start_box')
        if has_dt1:
            box.setCurrentIndex(box.findData('dt1'));assert view.job is None and not view.plan_valid
        if backend=='current':
            controls=view.control.planning;controls.quality.setCurrentIndex(controls.quality.findData('fast'))
            assert view.motion_profile['timing_quality']=='fast' and view.job is None
            dialog=controls.dialog();dialog.fields[0,0].setText('-1');dialog.buttons['apply'].click()
            assert dialog.error_label.text() and view.motion_profile['velocity_rad_s'][0]==.6
            dialog.fields[0,0].setText('.5');dialog.buttons['apply'].click()
            assert view.motion_profile['velocity_rad_s'][0]==.5 and not view.plan_valid
            view.owner_info={'unit':True};view.owner_status={}
            with pytest.raises(ValueError,match='active run'):view.apply_motion_settings(settings.settings(side))
            view.owner_info=None
            dialog.deleteLater()
    finally:view.close()


@pytest.mark.parametrize('fixture_folder',[
    'diagnostics/hardware_rest_transition_20261028/verified/latest_dt1/hardware',
    'diagnostics/right_j2_operating_limit_20261028/final_fault/hardware',
    'diagnostics/cymbal_goal_reserve_20261027/hardware_fault/hardware',
    'diagnostics/hardware_current_20261026/native_dt1_v2/hardware'])
def test_native_prepared_packet_with_explicitly_mocked_freshness(tmp_path,monkeypatch,fixture_folder):
    """Actual saved-scene/FCL proof; freshness is MOCKED, never a motion test.

    Only the temp test plan is labelled as a fake hardware acquisition. It is
    deleted with pytest's tmp data and is not published as an executable route.
    """
    # Current reserved goals AND immutable old v4 missions/recovery stay readable.
    fixture=ROOT/fixture_folder
    if not (fixture/'plan.json').exists():pytest.skip('Run hardware_current_native_check.py dt1 first')
    import shutil
    # Historical capture predates the explicit synchronization-label field.
    # Only the TEST reader supplies this fake freshness label; its measured
    # state hash and all scene payloads still undergo real verification.
    reader=tasks.validated_scene
    monkeypatch.setattr(tasks,'validated_scene',lambda *a:dict(reader(*a),pose_synchronization='encoder_bracketed'))
    obj=json.loads((fixture/'plan.json').read_text());obj.update(experiment_only=False,offline_simulation=False,
        baseline_hardware_session=True,actual_current_robot_state_verified=True,
        acquisition_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        current_state_check={'TEST_ONLY_FAKE_FRESHNESS':True})
    atomic_json(tmp_path/'plan.json',obj);shutil.copyfile(fixture/'backend_request.json',tmp_path/'backend_request.json')
    capture=Path(obj['provenance']['state']).parent
    from drum_planner.joint_position_limits import policy
    if obj['model'].get('joint_position_policy')!=policy():
        # Legacy missions remain loadable for checked prefix recovery, but
        # cannot become NEW forward motion under the obsolete J2 interval.
        loaded,req,curve=tasks.load_checked(tmp_path/'plan.json',DEFAULT_CALIBRATION)
        assert loaded==obj and curve.duration>0
        with pytest.raises(ValueError,match='J2 operating limit changed'):
            tasks.prepare(tmp_path/'checked',tmp_path/'plan.json',capture,DEFAULT_CALIBRATION,allow_replan=False)
        return
    tasks.prepare(tmp_path/'checked',tmp_path/'plan.json',capture,DEFAULT_CALIBRATION,allow_replan=False)
    packet,curve=tasks.check_packet(tmp_path/'checked/prepared.json')
    assert packet['proof']['all_valid'] and packet['motion_authorized'] is False
    assert packet['object']['current_state_check']['TEST_ONLY_FAKE_FRESHNESS']
    assert curve.duration==obj['duration_outward_s']
    # Simulated owner validates the same exact law but cannot open physical CAN.
    from drum_planner.baseline import service
    holder=SimpleNamespace(info=dict(simulated=True,plan=str(tmp_path/'plan.json')),side='right')
    service.Server.prepare_motion(holder)
    assert holder.motion_data[0]==obj
    # DT1 source provenance is independently enforced, not just its saved pose.
    bad=copy.deepcopy(obj);bad['dt1_generation']['identity']['scene_sha256']='other scene'
    atomic_json(tmp_path/'plan.json',bad)
    with pytest.raises(ValueError,match='DT1'):tasks.load_checked(tmp_path/'plan.json',DEFAULT_CALIBRATION)


def test_clock_integration_endpoint_roundoff_not_a_false_failure():
    obj=example();obj['outward'][-1]['time_s']=np.nextafter(1.,np.inf)
    c=Curve(obj);np.testing.assert_allclose(c.at(obj['outward'][-1]['time_s']),[.1]*7,atol=1e-15)
    with pytest.raises(ValueError,match='out of range'):c.at(1.+1e-8)


@pytest.mark.parametrize('side',['right','left'])
@pytest.mark.parametrize('stationary_case',[False,True])
@pytest.mark.parametrize('interrupt',[False,True])
def test_new_profile_exact_law_fake_cycle_and_recovery(side,stationary_case,interrupt):
    from drum_planner.baseline.motion import Controller
    from drum_planner.baseline.simulation import SimulatedBus
    obj=example(side,limits=[[.25]*7,[1.2]*7,[6.]*7],stationary=stationary_case)
    other='right' if side=='left' else 'left'
    req=dict(moving_side=side,start=[0.]*7,left_opening_m=0.,execution_joint_limits=[[-3.5,3.5]]*7,**{other:[0.]*7})
    clock=SimpleNamespace(t=100.);bus=SimulatedBus(req,now=lambda:clock.t)
    c=Controller(bus,obj,req,now=lambda:clock.t)
    def tick():clock.t+=.01;c.step()
    def until(phase):
        for _ in range(5000):
            if c.phase==phase:return
            tick()
            assert c.phase not in ('REJECTED','FAULT_HOLD'),c.error
        pytest.fail('Timeout '+c.phase)
    c.start()
    if interrupt:
        until('GOAL_HOLD' if stationary_case else 'OUTWARD')
        for _ in range(25):tick()
        c.halt();[tick() for _ in range(70)]
        assert bus.releases==0 and c.hold_confirmed
        r=c.request_return(True);spec=paths.recovery(c.curve,r['held'],r['tau'],r['rate'])
        spec[c.snapshot_key]=r[c.snapshot_key];spec['proof']=dict(all_valid=True,simulation_only=True)
        [tick() for _ in range(70)];assert c.accept_return(spec,r['generation'])
    until('RELAXED');assert bus.releases==1 and np.array_equal(bus.q,req['start'])


def test_paired_camera_emits_hash_bound_five_frame_hardware_sync(tmp_path,monkeypatch):
    """Fake camera/encoder sources; run the real capture adapter and depth fusion."""
    import shutil
    from drum_planner import read_only,hardware_check
    from drum_planner.common import arm_joints
    source=ROOT/'diagnostics/experiment_dt1_20261026/fresh_right/acquisition'
    if not source.exists():pytest.skip('Saved paired capture fixture unavailable')
    folder=tmp_path/'camera';seen=[]
    class CameraChild:
        returncode=0
        def __init__(self,*a,**kw):
            for name in ('paired_capture.json','paired_depths.npz','pair.npz'):
                shutil.copyfile(source/'camera'/name,folder/name)
        def poll(self):return 0
    monkeypatch.setattr(rgbd.subprocess,'Popen',CameraChild)
    state=json.loads((source/'measured_start.json').read_text())
    def select(samples,stamp,**kw):
        assert samples=='FAKE_ENCODERS' and kw['stationary_joints']==arm_joints('right')
        seen.append(stamp);return [],{'TEST_ONLY':True,'stamp':stamp}
    monkeypatch.setattr(read_only,'select_stationary',select)
    monkeypatch.setattr(hardware_check,'measured_state',lambda *a:copy.deepcopy(state))
    clock=SimpleNamespace(t=1e8)
    def now():clock.t+=.5;return clock.t
    monkeypatch.setattr(rgbd,'time',SimpleNamespace(monotonic=now,sleep=lambda _:None))
    worker=SimpleNamespace(count=16,error=None,samples=lambda:'FAKE_ENCODERS')
    obj=rgbd.capture(folder,DEFAULT_CALIBRATION,worker,stationary_joints=arm_joints('right'))
    assert len(seen)==5 and obj['pose_synchronization']=='encoder_bracketed'
    assert obj['measured_state_sha256']==digest(tmp_path/'measured_start.json')
    world=rgbd.scene(folder/'scene.json',DEFAULT_CALIBRATION)
    assert world['source_scene_sha256']==digest(folder/'scene.json')


def test_explicit_profile_cannot_silently_load_a_different_simulated_clock(tmp_path):
    from drum_planner.baseline.workspace import Workspace
    plan=ROOT/'diagnostics/hardware_current_20261026/native_dt1_v2/hardware/plan.json'
    if not plan.exists():pytest.skip('Native fixture unavailable')
    profile=settings.settings(values={'timing_quality':'thorough'})
    v=Workspace(tmp_path,simulate_plan=plan,autoscene=False,motion_profile=profile)
    try:
        assert v.simulation_recalculate and v.motion_profile==profile
        assert v.job is None and v.plan is None
    finally:v.close()
