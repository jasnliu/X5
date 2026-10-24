"""Single arc continuity preference; frozen live regression, no devices."""
from test_checkpoint_performance import OPTIMIZED
import ast,copy,hashlib,json,socket
from pathlib import Path
import pytest
from drum_planner.common import ROOT,atomic_json,digest
from drum_planner.experiment import algorithms,config,gesture,search

D=ROOT/'diagnostics/single_arc_priority_20261021'


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    real=socket.socket
    def guard(family=socket.AF_INET,*args,**kwargs):
        if family==socket.AF_CAN:pytest.fail('No CAN in selection tests')
        return real(family,*args,**kwargs)
    monkeypatch.setattr(socket,'socket',guard)
    from drum_planner import depth
    monkeypatch.setattr(depth,'DepthCamera',lambda *a,**k:pytest.fail('No camera in selection tests'))


def cases():return json.loads((D/'selection_cases.json').read_text())


def row(index,source='single_arc',stop=False,cost=1.):
    return dict(index=index,source=source,stop_at_corners=stop,duration_s=10.,
        gesture=dict(joint_excess_ratio=cost,workspace_detour=0.,workspace_overshoot=0.,
                     broad_turning_rad=0.,progress_disagreement=0.),
        metrics=dict(baseline=dict(components=dict(obstacle_proximity=0.))))


def test_live_regression_selects_continuous_arc_over_point_eight_percent_score_advantage():
    case=cases()['live_right'];before=copy.deepcopy(case)
    ranked,meta=search.rank(case['candidates'],'single_arc',case['parameters'])
    assert case==before and case['old_selected']==2
    assert ranked[0]['index']==12 and ranked[0]['selection_class']=='continuous_arc'
    assert ranked[0]['stop_at_corners'] is False
    old=next(r for r in ranked if r['index']==2)
    assert old['score']<ranked[0]['score']<old['score']*1.009
    assert meta['selection_policy']=='continuous_arc_first_v1' and not meta['fallback_used']
    assert meta['tier_counts']==dict(continuous_arc=3,continuous_fallback=1,segmented_fallback=5)


@pytest.mark.parametrize('case',['saved_right','saved_left'])
def test_saved_arm_pools_choose_valid_continuous_arc(case):
    value=cases()[case];ranked,meta=search.rank(value['candidates'],'single_arc',value['parameters'])
    assert ranked[0]['source']=='single_arc' and ranked[0]['stop_at_corners'] is False
    assert not meta['fallback_used']
    if case=='saved_right':assert ranked[0]['index']==value['old_selected']==14


@pytest.mark.parametrize('case',['live_right','saved_right','saved_left'])
@pytest.mark.parametrize('method',['compact','progress','sweep','clearance'])
def test_other_method_rankings_and_scores_are_unchanged(case,method):
    value=cases()[case];rows=value['candidates'];params=value['parameters']
    sources={'compact':('corridor','fallback'),'progress':('corridor','fallback','progress'),
             'sweep':('corridor','fallback','sweep'),
             'clearance':('corridor','fallback','progress','sweep','single_arc')}
    mode='economy' if method in ('compact','progress') else 'comfort' if method=='clearance' else 'arc'
    old=[dict(r,score=gesture.objective(r['gesture'],mode,r['metrics']['baseline']['components']['obstacle_proximity'],params))
         for r in rows if r['source'] in sources[method]]
    old.sort(key=lambda r:(round(r['score'],8),r['index']))
    actual,meta=search.rank(rows,method,params)
    assert actual==old and 'selection_policy' not in meta


def test_priority_is_not_a_small_weight_that_can_be_outvoted():
    rows=[row(1,'corridor',True,0.),row(2,'corridor',False,.01),row(3,cost=100.)]
    ranked,meta=search.rank(rows,'single_arc')
    assert [r['index'] for r in ranked]==[3,2,1]
    assert [r['continuity_tier'] for r in ranked]==[0,1,2]
    assert meta['score_scope']=='within_continuity_tier'


def test_no_arc_prefers_continuous_fallback_before_segmented_arc():
    ranked,meta=search.rank([row(1,stop=True,cost=0.),row(2,'corridor',False,10.)],'single_arc')
    assert ranked[0]['index']==2 and meta['selected_class']=='continuous_fallback' and meta['fallback_used']
    assert meta['tier_counts']['continuous_arc']==0


def test_only_segmented_candidates_remain_playable_and_explicitly_labelled():
    ranked,meta=search.rank([row(1,'corridor',True,2.),row(2,stop=True,cost=1.)],'single_arc')
    assert ranked[0]['index']==2 and meta['selected_class']=='segmented_fallback' and meta['fallback_used']


def test_missing_continuity_metadata_is_not_assumed_smooth():
    unknown=row(1,cost=0.);del unknown['stop_at_corners']
    ranked,meta=search.rank([unknown,row(2,'fallback',False,2.)],'single_arc')
    assert ranked[0]['index']==2 and ranked[1]['continuity_tier']==2


def test_selection_remains_time_independent_and_stable_within_tier():
    rows=[row(1,cost=1.),row(2,cost=1.-1e-10),row(3,cost=.5)]
    a,meta=search.rank(rows,'single_arc')
    for i,r in enumerate(rows):r['duration_s']=(i+1)**10
    b,_=search.rank(rows[::-1],'single_arc')
    assert [r['index'] for r in a]==[r['index'] for r in b]==[3,1,2]
    assert not meta['uses_elapsed_time'] and not meta['uses_recording_similarity']


@pytest.mark.parametrize('kind',['continuous_arc','continuous_fallback','segmented_fallback'])
def test_shared_runtime_records_and_logs_selection_or_fallback(tmp_path,monkeypatch,kind):
    rows={'continuous_arc':[row(1,cost=4.),row(2,'corridor',True,0.)],
          'continuous_fallback':[row(1,'corridor',False,4.),row(2,stop=True,cost=0.)],
          'segmented_fallback':[row(1,'corridor',True,0.)]}[kind]
    folder=tmp_path/'fixed_key';folder.mkdir()
    for r in rows:
        r['file']=f'candidate_{r["index"]}.json';atomic_json(folder/r['file'],dict(index=r['index']))
        r['sha256']=digest(folder/r['file'])
    atomic_json(folder/'pool.json',dict(schema=config.SCHEMA,key='fixed_key',complete=True,
        candidates=rows,attempts=[],elapsed_s=1.,joint_limit_check={'valid':True}))
    monkeypatch.setattr(algorithms,'pool_key',lambda *a:'fixed_key')
    from drum_planner.experiment import runtime
    monkeypatch.setattr(runtime,'Oracle',lambda *a:pytest.fail('Validated pool must be reused'))
    logs=[];winner,report=algorithms.run('single_arc',{}, {},tmp_path,tmp_path/'unused.json',config.settings(),120,logs.append)
    assert winner['index']==1 and report['pool_reused']
    assert report['normalization']['selected_class']==kind
    assert any('SINGLE ARC PRIORITY:' in line for line in logs)
    assert any('best continuity tier, then geometric score' in line for line in logs)
    fallback=[line for line in logs if 'SINGLE ARC FALLBACK:' in line]
    assert bool(fallback)==(kind!='continuous_arc')
    if fallback:assert kind in fallback[0]


@pytest.mark.parametrize('changed',['experiment/search.py','experiment/algorithms.py','experiment/config.py'])
def test_selection_source_changes_invalidate_disk_pool_cache(tmp_path,monkeypatch,changed):
    urdf=tmp_path/'robot.urdf';urdf.write_text('<robot name="test"/>')
    srdf=tmp_path/'robot.srdf';srdf.write_text('<robot name="test"/>')
    cal=tmp_path/'cal.json';cal.write_text('{}')
    req=dict(urdf=str(urdf),srdf=str(srdf),goal=[0.]*7,collision_policy='observed_obstacles')
    context=dict(scene_sha256='scene',state_sha256='state',calibration=str(cal))
    original=algorithms.pool_key(context,req,config.settings(),120)
    real=algorithms.digest;path=ROOT/'drum_planner'/changed
    monkeypatch.setattr(algorithms,'digest',lambda p:'changed' if Path(p)==path else real(p))
    assert algorithms.pool_key(context,req,config.settings(),120)!=original


def test_generation_timing_models_ui_and_hardware_policy_bytes_preserved():
    preserved=json.loads((D/'preserved_sources.json').read_text())
    # The later occupied-volume correction changes margin accounting/proofs,
    # not proposal generation, ranking, timing or actuator behavior. Keep the
    # historical hashes as evidence; test_voxel_clearance covers the new policy.
    margin_update={'drum_planner/planning.py','drum_planner/cpp/moveit_plan.cpp',
                   'drum_planner/cpp/experiment_oracle.cpp','drum_planner/baseline/tasks.py'}
    # Later user-authorized selected-arm Single arc UI/controller adaptation.
    # New test_singlearc_arms verifies both buses; original shared motors and
    # algorithm/timing bytes are independently preserved by its manifest audit.
    arm_update={'drum_planner/baseline/'+name+'.py' for name in
                ('planner','tasks','motion','transport','simulation','windows','workspace','feedback','work','service','computation')}
    arm_update.update({'drum_planner/cli.py','drum_planner/bus_ownership.py','plan_drums.sh'})
    # Later authorized experiment-only destination UI / selected endpoint routing.
    # New destination tests cover that contract; no generation/ranking/timing or
    # hardware source is exempted by this update. Retain the historical manifest.
    destination_update={'drum_planner/experiment/app.py','drum_planner/experiment/worker.py'}
    # Later authorized caching/numerical-overhead changes. Keep all historical
    # hashes and the function-level generation/validation assertions below.
    performance_update={'drum_planner/experiment/algorithms.py','drum_planner/experiment/runtime.py',
                        'drum_planner/timing/core.py','drum_planner/timing/adaptive.py',
                        'drum_planner/timing/integration.py','drum_planner/timing/smooth_v2.py'}
    for path,sha in preserved['files'].items():
        if path not in margin_update|arm_update|destination_update|performance_update|OPTIMIZED:assert digest(ROOT/path)==sha,path
    for key,sha in preserved['functions'].items():
        path,name=key.split(':');text=(ROOT/path).read_text()
        node=next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name==name)
        assert hashlib.sha256(ast.get_source_segment(text,node).encode()).hexdigest()==sha,key
