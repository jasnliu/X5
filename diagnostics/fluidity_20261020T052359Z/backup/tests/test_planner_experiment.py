"""Device-free tests for experiment contracts, risk statistics and containment."""
import json,subprocess,socket,sys
from pathlib import Path
import numpy as np
import pytest
from drum_planner.experiment import config,risk,algorithms,runtime
from drum_planner.common import ROOT


def test_nine_methods_no_learning_or_recorded_route():
    assert set(config.METHODS)=={'baseline','clearance','robust','chance','cvar','families','iris_gcs','chomp','stomp'}
    for path in (ROOT/'drum_planner/experiment').glob('*.py'):
        text=path.read_text()
        assert 'motor_transport' not in text and 'Telemetry(' not in text
        assert "['samples']" not in text or path.name=='algorithms.py'
    assert 'native MoveIt' in config.DESCRIPTIONS['chomp'] or 'Installed MoveIt CHOMP' in config.DESCRIPTIONS['chomp']


@pytest.mark.parametrize('key,value',[('depth_mm',-1),('joint_deg',float('nan')),('scenarios',15),('scenarios',32.2),
    ('seed',True),('chance_limit',0),('cvar_alpha',1),('tool_mm',31),('unknown',1)])
def test_settings_reject_invalid(key,value):
    with pytest.raises(ValueError):config.settings({key:value})


@pytest.mark.parametrize('value',[-1,0,4,float('nan'),float('inf'),601])
def test_invalid_budgets(value):
    with pytest.raises(ValueError):config.budget(value)


def test_presets_are_independent():
    p=config.settings();p['depth_mm']=20
    assert config.settings()['depth_mm']==3
    for v in config.PRESETS.values():assert config.settings(v)['scenarios']==64


def test_cvar_fractional_boundary_not_just_top_ceil():
    assert risk.cvar([0,1,2,10],.5)==6
    assert risk.cvar([0,1,2,10],.625)==pytest.approx(22/3)
    assert risk.cvar([0,1,2,10],.99)==pytest.approx(10)
    with pytest.raises(ValueError):risk.cvar([], .9)


def test_wilson_never_reports_zero_certainty():
    assert .03<risk.wilson_upper(0,64)<.05
    assert risk.wilson_upper(10,64)>10/64
    assert risk.wilson_upper(64,64)==pytest.approx(1)


def test_scenarios_bounded_reproducible_temporally_correlated():
    p=config.settings();a=list(risk.scenarios(p,101,[0,0,.8]));b=list(risk.scenarios(p,101,[0,0,.8]))
    assert len(a)==64
    for (dq,kw),(dq2,kw2) in zip(a,b):
        assert np.array_equal(dq,dq2) and kw==kw2
        assert np.max(abs(dq))<=np.radians(p['joint_deg'])+1e-12
        T=np.asarray(kw['world_transform']);assert np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3))
        assert np.linalg.norm(T[:3,3])<.01
        assert np.max(abs(np.diff(dq,axis=0)))<np.radians(p['joint_deg'])/10


def test_envelope_zero_and_monotonic():
    p=config.settings({key:0 for key in config.BOUND_FIELDS});e=risk.robust_envelope(p)
    assert e['world_m']==e['self_m']==e['joint_rad']==0
    p['joint_deg']=.1;f=risk.robust_envelope(p)
    assert f['world_m']>0 and f['self_m']>f['world_m']


def test_risk_scores_and_budget_fail_closed():
    class Oracle:
        def check(self,q,**kwargs):return dict(valid=[True]*len(q))
    p=config.settings(dict(scenarios=16,chance_limit=.5));q=np.zeros((11,7))
    r=risk.evaluate(Oracle(),q,p,[0,0,0])
    assert r['accepted'] and r['scenarios']==16 and r['cvar']==0 and r['wilson_one_sided_95_upper']>0
    with pytest.raises(TimeoutError):risk.evaluate(Oracle(),q,p,[0,0,0],deadline=0)


def test_resampling_endpoints_and_rdp():
    q=np.linspace(np.zeros(7),np.ones(7),33);q[16,2]+=.2
    a=algorithms.resample(q,101);b=algorithms.thin(q)
    assert a.shape==(101,7) and np.array_equal(a[[0,-1]],q[[0,-1]])
    assert np.any(np.all(b==q[16],axis=1))
    assert len(algorithms.thin(np.linspace(np.zeros(7),np.ones(7),20)))==2


def test_dense_check_batches_keep_boundary_edges_and_stop_on_failure():
    class Oracle:
        def __init__(self):self.calls=[]
        def check(self,q,**kw):self.calls.append(q.copy());return dict(valid=[True]*len(q))
    q=np.linspace(np.zeros(7),np.ones(7),9001);o=Oracle();ok,r=algorithms.dense_check(o,q)
    assert ok and len(o.calls)==3
    assert np.array_equal(o.calls[0][-1],o.calls[1][0])
    assert np.array_equal(o.calls[1][-1],o.calls[2][0])


def test_clearance_is_lexicographic_not_weighted_compromise():
    def candidate(clearance,length):return dict(metrics=dict(transit_min_clearance_m=clearance,
         transit_low_decile_clearance_m=clearance,transit_proximity=0,workspace_length_m=length),timing=dict(duration_s=length))
    assert algorithms.clearance_key(candidate(.09,10))<algorithms.clearance_key(candidate(.05,.1))


def test_can_audit_guard_in_child_no_global_test_side_effects():
    code='from drum_planner.experiment.config import no_can; import socket; no_can(); socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)'
    p=subprocess.run([sys.executable,'-c',code],capture_output=True,text=True)
    assert p.returncode and 'Experiment mode forbids CAN sockets' in p.stderr


def test_latest_pose_is_saved_success_not_example(tmp_path):
    with pytest.raises(ValueError,match='No successful'):config.latest_state(tmp_path)
    assert config.latest_state().name=='measured_start.json'


def test_job_command_isolated_and_cancellable(tmp_path,monkeypatch):
    calls=[]
    class Process:
        def __init__(self,args,**kwargs):calls.append(args);self.returncode=None
        def poll(self):return self.returncode
        def wait(self,timeout):self.returncode=0;return 0
    monkeypatch.setattr(runtime.subprocess,'Popen',Process)
    monkeypatch.setattr(runtime.subprocess,'run',lambda *a,**kw:type('R',(),{'stdout':''})())
    job=runtime.Job(['/bin/true'],tmp_path,timeout=25)
    args=calls[0]
    assert 'MemoryMax=3G' in args and 'MemorySwapMax=128M' in args and 'KillMode=control-group' in args
    assert 'RestrictAddressFamilies=AF_UNIX AF_INET AF_INET6 AF_NETLINK' in args
    assert '--user' in args and '--wait' in args
    job.cancel();assert job.cancelled;job.close()


def test_shared_scenarios_score_all_candidates_without_losing_checks():
    class Oracle:
        def __init__(self):self.counts=[]
        def call(self,op,**kw):
            assert op=='scenarios'
            self.counts.append(len(kw['scenarios']))
            return dict(rows=[[dict(failed=False,endpoint_failed=False,loss=float(k)) for k,_ in enumerate(s['paths'])] for s in kw['scenarios']])
    p=config.settings(dict(scenarios=16));o=Oracle()
    result=risk.evaluate_many(o,[np.zeros((101,7)),np.ones((101,7))],p,[0,0,0])
    assert o.counts==[8,8] and len(result)==2
    assert result[0]['cvar']==0 and result[1]['cvar']==1
    assert result[0]['confidence_candidates']==2
    assert result[0]['wilson_one_sided_95_upper']>risk.wilson_upper(0,16)


def test_immediate_cancel_retries_service_registration_race(tmp_path,monkeypatch):
    calls=[]
    class Process:
        def __init__(self,*args,**kwargs):self.returncode=None
        def poll(self):return self.returncode
        def wait(self,timeout):self.returncode=0;return 0
    monkeypatch.setattr(runtime.subprocess,'Popen',Process)
    def run(args,**kwargs):
        calls.append(args)
        return type('R',(),{'stdout':''})()
    monkeypatch.setattr(runtime.subprocess,'run',run)
    job=runtime.Job(['/bin/true'],tmp_path)
    job.cancel();job.last_stop=-1.;job.poll()
    stops=[c for c in calls if 'stop' in c]
    assert len(stops)==2 and job.cancelled
    job.close()


@pytest.fixture
def queued_view(tmp_path,monkeypatch):
    """Actual controller methods with device-free workers/widgets, no solver/UI."""
    from collections import deque
    from types import SimpleNamespace
    from drum_planner.experiment import app
    from drum_planner.snapshot_preview import SnapshotPlaybackView
    monkeypatch.setattr(SnapshotPlaybackView,'poll',lambda self:-1)
    monkeypatch.setattr(SnapshotPlaybackView,'close',lambda self:None)
    class Widget:
        def __init__(self):self.enabled=True;self.text='';self.index=list(config.METHODS).index('clearance');self.items={}
        def setEnabled(self,value):self.enabled=value
        def setText(self,value):self.text=value
        def currentData(self):return list(config.METHODS)[self.index]
        def blockSignals(self,value):pass
        def setItemText(self,index,text):self.items[index]=text
    class Clock:
        playing=False
        def pause(self):self.playing=False
    jobs=[]
    class FakeJob:
        def __init__(self,folder,method):
            assert not any(j.process.returncode is None for j in jobs),'Concurrent workers!'
            self.process=SimpleNamespace(returncode=None);self.cancelled=False;self.closed=False
            self.method=method;self.log_path=folder/'worker.log';jobs.append(self)
        def poll(self):return self.process.returncode
        def cancel(self):self.cancelled=True;self.process.returncode=0
        def close(self):
            self.closed=True
            if self.process.returncode is None:self.process.returncode=0
    def start(folder,method,params,seconds):
        target=tmp_path/(method+'-'+str(len(jobs)));target.mkdir()
        (target/'plan.json').write_text(json.dumps(dict(status='no_feasible_route' if method=='robust' else 'planned')))
        return target,FakeJob(target,method)
    monkeypatch.setattr(app,'start_plan',start)
    view_class=app.make_view_class();v=view_class.__new__(view_class)
    v.session=tmp_path;v.context=dict(scene_sha256='scene',state_sha256='state')
    v.params=config.settings();v.planning_seconds=60.;v.cache={};v.pending_methods=deque()
    v.method_states={key:'not queued' for key in config.METHODS};v.closing=False
    v.job=None;v.job_kind=None;v.job_method=None;v.job_signature=None;v.job_target=None
    v.active_method='clearance';v.displayed_method=None;v.events=deque(maxlen=256);v.generation_count=0
    v.plan=SimpleNamespace(playable=False);v.clock=Clock();v.autoplay_result=False
    for name in ('method_box','generate_button','cache_all_button','budget_spin','settings_button',
                 'recapture_button','cancel_button','play_button','timeline','queue_label','status','plan_label','method_description'):
        setattr(v,name,Widget())
    v.window=SimpleNamespace(findChildren=lambda kind:[]);v.QtWidgets=SimpleNamespace(QPushButton=Widget)
    def install(path,method):
        v.plan=SimpleNamespace(playable=bool(method and json.loads(Path(path).read_text())['status']=='planned'))
        v.displayed_method=method;v.clock=Clock()
        if method:v.active_method=method;v.method_box.index=list(config.METHODS).index(method)
        v.locked(bool(v.job))
    v.install_plan=install
    def finish(code=0):
        assert v.job is not None
        v.job.process.returncode=code;v.poll()
    def choose(method):
        v.method_box.index=list(config.METHODS).index(method);v.select_method()
    v.finish=finish;v.choose=choose;v.test_jobs=jobs
    return v


def test_cache_all_runs_every_method_without_selection_and_serializes_workers(queued_view):
    v=queued_view;v.cache_all()
    assert v.job_method=='clearance' and len(v.pending_methods)==8
    while v.job:v.finish()
    assert [j.method for j in v.test_jobs]==['clearance',*[m for m in config.METHODS if m!='clearance']]
    assert len(v.cache)==len(config.METHODS)==v.generation_count
    assert v.method_states['robust']=='no feasible route' and '9/9' in v.queue_label.text
    assert v.displayed_method=='clearance' and all(j.closed for j in v.test_jobs)
    v.cache_all();assert not v.job and v.generation_count==9


def test_selection_prioritizes_pending_without_stealing_worker_result(queued_view):
    v=queued_view;v.cache_all();first=v.job
    v.choose('stomp')
    assert v.job is first and v.job_method=='clearance' and v.pending_methods[0]=='stomp'
    assert len(v.pending_methods)==8 and len(set(v.pending_methods))==8
    v.finish()
    assert 'clearance' in v.cache and 'stomp' not in v.cache
    assert v.active_method=='stomp' and v.displayed_method is None and v.job_method=='stomp'
    v.finish();assert v.displayed_method=='stomp' and v.play_button.enabled


def test_cached_playback_stays_usable_and_uninterrupted_during_other_workers(queued_view):
    v=queued_view;v.cache_all();v.finish()
    assert v.job_method=='baseline' and v.method_box.enabled and v.play_button.enabled and v.timeline.enabled
    assert not v.settings_button.enabled and not v.recapture_button.enabled
    clock=v.clock;clock.playing=True;v.finish()
    assert v.clock is clock and v.clock.playing and v.displayed_method=='clearance'
    job=v.job;generations=v.generation_count
    v.choose('baseline')
    assert v.job is job and v.generation_count==generations and v.displayed_method=='baseline'


def test_cancel_stops_entire_queue_keeps_cache_and_cache_all_resumes(queued_view):
    v=queued_view;v.cache_all();v.finish();v.cancel_job();v.poll()
    assert not v.job and not v.pending_methods and set(v.cache)=={'clearance'}
    count=v.generation_count;v.poll();assert v.generation_count==count
    assert v.play_button.enabled and v.settings_button.enabled
    v.cache_all();assert v.job_method=='baseline' and 'clearance' not in v.pending_methods


def test_worker_failure_does_not_block_remaining_automatic_methods(queued_view):
    v=queued_view;v.cache_all();v.finish(7)
    assert v.method_states['clearance']=='failed' and 'clearance' not in v.cache
    assert v.job_method=='baseline'
    while v.job:v.finish()
    assert len(v.cache)==8
    v.choose('clearance');assert v.job_method=='clearance'
    v.finish();assert len(v.cache)==9


def test_stale_worker_cannot_be_cached_under_changed_settings(queued_view):
    v=queued_view;v.cache_all();v.params['depth_mm']+=1;v.finish()
    assert 'clearance' not in v.cache and v.method_states['clearance']=='failed'
    assert v.job_method=='baseline' and v.job_signature==v.signature()
    assert not v.plan.playable


def test_budget_change_rebuilds_automatic_queue_and_invalidates_cache(queued_view):
    v=queued_view;v.cache_all()
    while v.job:v.finish()
    v.budget_changed(30)
    assert not v.cache and not v.plan.playable and v.job_method=='clearance'
    assert v.job_signature[-1]==30 and len(v.pending_methods)==8


def test_explicit_generate_replaces_only_selected_cached_result(queued_view):
    v=queued_view;v.cache_all()
    while v.job:v.finish()
    v.generate()
    assert v.job_method=='clearance' and len(v.cache)==8 and not v.plan.playable and not v.pending_methods
    v.finish();assert len(v.cache)==9 and not v.job


def test_close_stops_active_worker_and_never_launches_queued_jobs(queued_view):
    v=queued_view;v.cache_all();job=v.job;v.close()
    assert job.closed and v.closing and not v.pending_methods and not v.job
    v.start_next();v.cache_all();assert len(v.test_jobs)==1
