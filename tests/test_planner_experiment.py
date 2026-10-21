"""Device-free gesture metrics, shared-pool contracts and experiment UI queue."""
import json,subprocess,socket,sys
from pathlib import Path
import numpy as np
import pytest
from drum_planner.experiment import config,fluidity,algorithms,runtime,search,gesture
from drum_planner.common import ROOT


def test_four_survivors_no_learning_or_recorded_route():
    assert list(config.METHODS)==['progress','sweep','single_arc','clearance']
    for path in (ROOT/'drum_planner/experiment').glob('*.py'):
        text=path.read_text()
        assert 'motor_transport' not in text and 'Telemetry(' not in text
    for name in ('risk.py','regions.py'):assert not (ROOT/'drum_planner/experiment'/name).exists()
    assert 'recordings' in config.DESCRIPTIONS['single_arc']
    assert not set(config.METHODS)&{'baseline','reversals','coordination','spatial','combined'}


@pytest.mark.parametrize('key,value',[('backtrack_weight',-1),('corridor_deg',float('nan')),
    ('corridor_deg',0),('candidates',1),('candidates',2.2),('seed',True),('seed',float('inf')),
    ('unknown',1),('depth_mm',3),('backtrack_weight','bad')])
def test_settings_reject_invalid(key,value):
    with pytest.raises(ValueError):config.settings({key:value})


@pytest.mark.parametrize('value',[-1,0,4,float('nan'),float('inf'),601])
def test_invalid_budgets(value):
    with pytest.raises(ValueError):config.budget(value)


def test_presets_are_independent():
    p=config.settings();p['backtrack_weight']=8
    assert config.settings()['backtrack_weight']==4
    for v in config.PRESETS.values():assert config.settings(v)['candidates']==6


def test_reversal_deadband_count_and_size():
    def m(x):return fluidity.reversals(np.column_stack([x,*[np.zeros(len(x)) for _ in range(6)]]),.02)
    assert m([0,.1,.2,.3])['count']==0
    r=m([0,.3,.1,.4]);assert r['count']==2 and r['excess_travel_rad']==pytest.approx(.4)
    assert m([0,.3,.299,.4])['count']==0
    assert m([0,.4,.1,.5])['score']>r['score']
    assert m([0,0,0])['score']==0


def test_coordination_separates_sequential_from_simultaneous_without_reversals():
    straight=np.column_stack([np.linspace(0,1,101)]*2+[np.zeros(101)]*5)
    serial=np.zeros((101,7));serial[:51,0]=np.linspace(0,1,51);serial[51:,0]=1;serial[50:,1]=np.linspace(0,1,51)
    a=fluidity.curve_shape(straight,.05);b=fluidity.curve_shape(serial,.05)
    assert fluidity.reversals(straight,.02)['count']==fluidity.reversals(serial,.02)['count']==0
    assert a['bending_energy']<1e-8 and b['bending_energy']>1


def test_spatial_curve_rejects_zigzags_and_is_sampling_stable():
    t=np.linspace(0,1,801)
    arc=np.column_stack([t,.2*np.sin(np.pi*t),t*0])
    zig=np.column_stack([t,.2*np.sin(5*np.pi*t),t*0])
    a=fluidity.curve_shape(arc,.05);b=fluidity.curve_shape(zig,.05)
    assert b['bending_energy']>a['bending_energy']*5
    assert b['curvature_variation']>a['curvature_variation']*5
    duplicated=np.repeat(arc,3,axis=0)
    assert fluidity.curve_shape(duplicated,.05)==pytest.approx(a)
    assert fluidity.curve_shape(arc[::4],.05)['bending_energy']==pytest.approx(a['bending_energy'],rel=.01)


def ranking_rows():
    rows=[]
    for i,source in enumerate(('corridor','progress','sweep','single_arc')):
        q=np.linspace(np.zeros(7),np.ones(7),81)
        x=q[:,:3].copy();x[:,1]+=(4-i)*.1*np.sin(np.linspace(0,np.pi,81))
        g=gesture.geometry(q,dict(wrist_xyz_m=x,tip_xyz_m=x*1.5))
        rows.append(dict(index=i+1,source=source,duration_s=10+i,gesture=g,
            metrics=dict(baseline=dict(components=dict(obstacle_proximity=.1*i)))))
    return rows


def test_family_eligibility_and_no_demonstration_or_duration_filter():
    rows=ranking_rows()
    for method,sources in [('compact',{'corridor'}),('progress',{'corridor','progress'}),
                          ('sweep',{'corridor','sweep'}),('single_arc',{'corridor','single_arc'}),
                          ('clearance',{'corridor','progress','sweep','single_arc'})]:
        ranked,meta=search.rank(rows,method)
        assert {r['source'] for r in ranked}==sources
        assert meta['duration_ratio_filter'] is None and not meta['uses_recording_similarity']
        assert not meta['uses_elapsed_time']


def test_gesture_winners_do_not_change_when_candidate_durations_change():
    rows=ranking_rows()
    for method in config.METHODS:
        a,_=search.rank(rows,method)
        changed=json.loads(json.dumps(rows))
        for i,row in enumerate(changed):row['duration_s']=(i+1)**3
        b,_=search.rank(changed,method)
        assert [(r['index'],r['score']) for r in a]==[(r['index'],r['score']) for r in b]


def test_editable_weights_affect_objectives_not_collision_policy():
    rows=ranking_rows()
    a,_=search.rank(rows,'clearance',dict(clearance_weight=0.))
    b,_=search.rank(rows,'clearance',dict(clearance_weight=10.))
    assert a[0]['index']!=b[0]['index']
    assert config.saved_inputs()[0].name=='measured_start.json'


def test_incomplete_or_tampered_pool_not_reused(tmp_path):
    from drum_planner.common import atomic_json,digest
    assert algorithms.read_pool(tmp_path,'key') is None
    p=tmp_path/'candidate.json';atomic_json(p,dict(test=True))
    data=dict(schema=config.SCHEMA,key='key',complete=True,candidates=[dict(file=p.name,sha256=digest(p))])
    atomic_json(tmp_path/'pool.json',data);assert algorithms.read_pool(tmp_path,'key')==data
    p.write_text('{}')
    with pytest.raises(ValueError,match='hash mismatch'):algorithms.read_pool(tmp_path,'key')
    data['complete']=False;atomic_json(tmp_path/'pool.json',data)
    with pytest.raises(ValueError,match='incomplete'):algorithms.read_pool(tmp_path,'key')


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


def test_can_audit_guard_in_child_no_global_test_side_effects():
    code='from drum_planner.experiment.config import no_can; import socket; no_can(); socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)'
    p=subprocess.run([sys.executable,'-c',code],capture_output=True,text=True)
    assert p.returncode and 'Experiment mode forbids CAN sockets' in p.stderr


def test_latest_pose_is_saved_success_not_example(tmp_path):
    with pytest.raises(ValueError,match='No successful'):config.latest_state(tmp_path)
    assert config.latest_state().name=='measured_start.json'


def test_default_prepare_uses_frozen_inputs_and_never_captures(tmp_path,monkeypatch):
    from drum_planner.experiment import app
    state=tmp_path/'saved_state.json';scene=tmp_path/'scene.json'
    monkeypatch.setattr(app,'saved_inputs',lambda:(state,scene))
    monkeypatch.setattr(app,'no_can',lambda:None)
    commands=[]
    class FakeJob:
        def __init__(self,command,folder,**kwargs):self.command=command;commands.append(command)
    def wait(job):
        if 'initialize' in job.command:
            folder=Path(job.command[job.command.index('--folder')+1])
            (folder/'context.json').write_text(json.dumps(dict(state=str(state),scene=str(scene))))
    monkeypatch.setattr(app,'Job',FakeJob);monkeypatch.setattr(app,'wait',wait)
    context=app.prepare(tmp_path/'session')
    assert context==dict(state=str(state),scene=str(scene))
    assert len(commands)==2 and all('capture' not in c for c in commands)
    assert str(state) in commands[1] and str(scene) in commands[1]


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
        def __init__(self):self.enabled=True;self.text='';self.index=list(config.METHODS).index('progress');self.items={}
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
        (target/'plan.json').write_text(json.dumps(dict(status='no_feasible_route' if method=='single_arc' else 'planned')))
        return target,FakeJob(target,method)
    monkeypatch.setattr(app,'start_plan',start)
    view_class=app.make_view_class();v=view_class.__new__(view_class)
    v.session=tmp_path;v.context=dict(scene_sha256='scene',state_sha256='state')
    v.params=config.settings();v.planning_seconds=60.;v.cache={};v.pending_methods=deque()
    v.method_states={key:'not queued' for key in config.METHODS};v.closing=False
    v.job=None;v.job_kind=None;v.job_method=None;v.job_signature=None;v.job_target=None
    v.active_method='progress';v.displayed_method=None;v.events=deque(maxlen=256);v.generation_count=0
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
    assert v.job_method=='progress' and len(v.pending_methods)==len(config.METHODS)-1
    while v.job:v.finish()
    assert [j.method for j in v.test_jobs]==['progress',*[m for m in config.METHODS if m!='progress']]
    assert len(v.cache)==len(config.METHODS)==v.generation_count
    assert v.method_states['single_arc']=='no feasible route' and '4/4' in v.queue_label.text
    assert v.displayed_method=='progress' and all(j.closed for j in v.test_jobs)
    v.cache_all();assert not v.job and v.generation_count==len(config.METHODS)


def test_selection_prioritizes_pending_without_stealing_worker_result(queued_view):
    v=queued_view;v.cache_all();first=v.job
    v.choose('clearance')
    assert v.job is first and v.job_method=='progress' and v.pending_methods[0]=='clearance'
    assert len(v.pending_methods)==len(config.METHODS)-1 and len(set(v.pending_methods))==len(config.METHODS)-1
    v.finish()
    assert 'progress' in v.cache and 'clearance' not in v.cache
    assert v.active_method=='clearance' and v.displayed_method is None and v.job_method=='clearance'
    v.finish();assert v.displayed_method=='clearance' and v.play_button.enabled


def test_cached_playback_stays_usable_and_uninterrupted_during_other_workers(queued_view):
    v=queued_view;v.cache_all();v.finish()
    assert v.job_method=='sweep' and v.method_box.enabled and v.play_button.enabled and v.timeline.enabled
    assert not v.settings_button.enabled and not v.recapture_button.enabled
    clock=v.clock;clock.playing=True;v.finish()
    assert v.clock is clock and v.clock.playing and v.displayed_method=='progress'
    job=v.job;generations=v.generation_count
    v.choose('sweep')
    assert v.job is job and v.generation_count==generations and v.displayed_method=='sweep'


def test_cancel_stops_entire_queue_keeps_cache_and_cache_all_resumes(queued_view):
    v=queued_view;v.cache_all();v.finish();v.cancel_job();v.poll()
    assert not v.job and not v.pending_methods and set(v.cache)=={'progress'}
    count=v.generation_count;v.poll();assert v.generation_count==count
    assert v.play_button.enabled and v.settings_button.enabled
    v.cache_all();assert v.job_method=='sweep' and 'progress' not in v.pending_methods


def test_worker_failure_does_not_block_remaining_automatic_methods(queued_view):
    v=queued_view;v.cache_all();v.finish(7)
    assert v.method_states['progress']=='failed' and 'progress' not in v.cache
    assert v.job_method=='sweep'
    while v.job:v.finish()
    assert len(v.cache)==len(config.METHODS)-1
    v.choose('progress');assert v.job_method=='progress'
    v.finish();assert len(v.cache)==len(config.METHODS)


def test_stale_worker_cannot_be_cached_under_changed_settings(queued_view):
    v=queued_view;v.cache_all();v.params['backtrack_weight']+=1;v.finish()
    assert 'progress' not in v.cache and v.method_states['progress']=='failed'
    assert v.job_method=='sweep' and v.job_signature==v.signature()
    assert not v.plan.playable


def test_budget_change_rebuilds_automatic_queue_and_invalidates_cache(queued_view):
    v=queued_view;v.cache_all()
    while v.job:v.finish()
    v.budget_changed(30)
    assert not v.cache and not v.plan.playable and v.job_method=='progress'
    assert v.job_signature[-1]==30 and len(v.pending_methods)==len(config.METHODS)-1


def test_explicit_generate_replaces_only_selected_cached_result(queued_view):
    v=queued_view;v.cache_all()
    while v.job:v.finish()
    v.generate()
    assert v.job_method=='progress' and len(v.cache)==len(config.METHODS)-1 and not v.plan.playable and not v.pending_methods
    v.finish();assert len(v.cache)==len(config.METHODS) and not v.job


def test_close_stops_active_worker_and_never_launches_queued_jobs(queued_view):
    v=queued_view;v.cache_all();job=v.job;v.close()
    assert job.closed and v.closing and not v.pending_methods and not v.job
    v.start_next();v.cache_all();assert len(v.test_jobs)==1
