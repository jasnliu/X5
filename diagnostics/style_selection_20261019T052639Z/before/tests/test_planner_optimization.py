"""Best-of-N selection without devices; real MoveIt integration is in test_drum_planner."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from drum_planner import planning, cli, current_plan


@pytest.fixture
def search(tmp_path, monkeypatch):
    req=dict(seed=19,candidate_count=3,start=[0.]*7,goal=[1.]*7,
             left=[-.1]*7,scene=dict(synthetic=True),execute=False,
             collision_policy='observed_obstacles',occupied_only_preview=False)
    rig=SimpleNamespace(req=req,calls=[],checks=[],statuses={},bad={},
                        durations={1:(10.,20.),2:(4.,12.),3:(7.,14.)},
                        seed_durations={1:.1,2:9.,3:5.},endpoint_bad=set())

    def backend(trial, folder, name='backend'):
        index=1+(trial['seed']-19)//7919
        folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
        rig.calls.append((copy.deepcopy(trial),name))
        common=dict(schema='x5-moveit-result-v1',collision_policy='observed_obstacles',
                    strict_unknown_space=False,motion_authorized=False)
        if 'check_states' not in trial:
            raw=dict(common,status=rig.statuses.get(index,'planned'),
                     duration_outward_s=rig.seed_durations[index],
                     path_waypoints=[trial['start'],[float(index)]*7,trial['goal']])
            if index in rig.endpoint_bad:raw['path_waypoints'][-1]=[2.]*7
        else:
            assert trial['check_path'] and trial['stop_on_invalid']
            stop=name.endswith('_1');rig.checks.append((index,stop))
            rows=[dict(valid=True) for _ in trial['check_states']]
            raw=dict(common,status='checked',checks=rows)
            bad=rig.bad.get((index,stop))
            if bad=='collision':rows[0]=dict(valid=False,reason='collision')
            if bad=='empty':raw['checks']=[]
            if bad=='short':raw['checks']=rows[:-1]
            if bad=='missing_valid':rows[0]={}
            if bad=='truthy':rows[0]=dict(valid=1)
            if bad=='status':raw['status']='error'
            if bad=='policy':raw['collision_policy']='observed_free_space'
            if bad=='strict':raw['strict_unknown_space']=True
        (folder/(name+'_request.json')).write_text(json.dumps(trial))
        (folder/(name+'_result.json')).write_text(json.dumps(raw))
        return raw

    def smooth(path, stop=False):
        index=int(path[1][0]);duration=rig.durations[index][int(stop)]
        if isinstance(duration,Exception):raise duration
        points=[dict(time_s=t,positions_rad=q,velocities_rad_s=[0.]*7,
                     accelerations_rad_s2=[0.]*7) for t,q in [(0.,path[0]),(duration,path[-1])]]
        return points,np.array([path[0],path[-1]]),dict(duration_s=duration,stop_at_corners=stop)

    monkeypatch.setattr(planning,'backend',backend)
    monkeypatch.setattr(planning,'smooth_samples',smooth)
    rig.run=lambda:planning.fastest_plan(tmp_path,req)
    return rig


def test_rank_final_time_not_first_seed_or_totg_time(search,tmp_path):
    before=copy.deepcopy(search.req);result=search.run();opt=result['optimization']
    assert result['status']=='planned' and result['duration_outward_s']==4.
    assert result['moveit_seed_duration_s']==9.  # raw fastest seed was NOT the final fastest
    assert opt['selected_candidate']==2 and opt['selected_seed']==7938
    assert opt['searched_candidates']==3 and not opt['globally_time_optimal']
    assert search.checks==[(2,False)]  # slower variants cannot improve the winner
    assert search.req==before
    assert json.loads((tmp_path/'backend_request.json').read_text())['seed']==7938
    assert json.loads((tmp_path/'backend_result.json').read_text())['duration_outward_s']==9.
    assert json.loads((tmp_path/'optimization.json').read_text())==opt
    assert result['outward'][0]['positions_rad']==before['start']
    assert result['outward'][-1]['positions_rad']==before['goal']
    assert result['cycle'][-1]['positions_rad']==before['start']
    assert result['cycle'][-1]['time_s']==10. and result['hold_at_goal_s']==2.
    assert len({r['seed'] for r,n in search.calls if n=='backend'})==3
    for request,_ in search.calls:
        for field in ('start','goal','left','scene','execute','collision_policy','occupied_only_preview'):
            assert request[field]==before[field]


@pytest.mark.parametrize('bad',['collision','empty','short','missing_valid','truthy','status','policy','strict'])
def test_faster_invalid_or_incomplete_check_cannot_win(search,bad):
    search.bad[2,False]=bad
    result=search.run()
    assert search.checks==[(2,False),(3,False)]
    assert result['optimization']['selected_candidate']==3
    rejected=result['optimization']['candidates'][1]['variants'][0]
    assert rejected['status']=='rejected_collision_check' and not rejected['passed']


def test_corner_stop_also_competes_by_final_time(search):
    search.durations[2]=(8.,3.)
    result=search.run()
    assert result['duration_outward_s']==3.
    assert result['timing']['stop_at_corners']
    assert search.checks==[(2,True)]


@pytest.mark.parametrize('status',['no_path','blocked_after_smoothing','failed_time_parameterization','invalid_smoothed_goal'])
def test_one_seed_failure_does_not_cancel_other_candidates(search,status):
    search.statuses[1]=status
    assert search.run()['optimization']['selected_candidate']==2


def test_endpoint_changes_are_never_accepted(search):
    search.endpoint_bad.add(2)
    result=search.run()
    assert result['optimization']['selected_candidate']==3
    assert result['optimization']['candidates'][1]['status']=='invalid_path'
    assert search.checks==[(3,False)]


@pytest.mark.parametrize('bad',[ValueError('impractically slow'),float('nan'),float('inf'),0.])
def test_bad_timing_is_rejected_without_losing_other_candidates(search,bad):
    search.durations[2]=(bad,bad)
    result=search.run()
    assert result['optimization']['selected_candidate']==3
    assert all(v['status']=='rejected_timing' for v in result['optimization']['candidates'][1]['variants'])


@pytest.mark.parametrize('status',['blocked_start','blocked_goal','error'])
def test_common_input_failure_stops_without_wasting_seeds(search,status):
    search.statuses[1]=status
    result=search.run()
    assert result['status']==status and len(search.calls)==1
    assert result['optimization']['selected_candidate'] is None


def test_every_seed_no_path_remains_nonplayable(search):
    search.statuses={i:'no_path' for i in range(1,4)}
    result=search.run()
    assert result['status']=='no_path' and not search.checks
    assert result['optimization']['searched_candidates']==3
    assert 'cycle' not in result


def test_all_final_curves_colliding_yields_no_playable_route(search):
    search.bad={(i,stop):'collision' for i in range(1,4) for stop in (False,True)}
    result=search.run()
    assert result['status']=='blocked_interpolated_path'
    assert len(search.checks)==6 and 'outward' not in result and 'cycle' not in result
    assert result['optimization']['selected_candidate'] is None


def test_one_seed_option_and_deterministic_ties(search):
    search.req['candidate_count']=1
    result=search.run()
    assert result['optimization']['searched_candidates']==1
    assert result['duration_outward_s']==10.
    search.req['candidate_count']=3
    search.durations={i:(4.,4.) for i in range(1,4)}
    result=search.run()
    assert result['optimization']['selected_candidate']==1
    assert not result['optimization']['selected_stop_at_corners']


@pytest.mark.parametrize('value',[0,-1,33,1.5,'1.5',True,None,float('nan'),float('inf')])
def test_invalid_count_rejected_before_device_access(tmp_path,monkeypatch,value):
    monkeypatch.setattr(current_plan,'Telemetry',lambda *a:pytest.fail('No devices allowed'))
    with pytest.raises(ValueError,match='Candidate count'):
        current_plan.run(tmp_path/'never_created',headless=True,candidates=value)
    assert not (tmp_path/'never_created').exists()


@pytest.mark.parametrize('command',['plan','live','demo'])
def test_cli_validates_count_without_running(command):
    args=[command,'--candidates','0']
    if command=='plan':args+=['--scene','unused','--state','unused']
    with pytest.raises(SystemExit) as error:cli.main(args)
    assert error.value.code==2


def test_cli_saved_and_live_forward_candidate_count(tmp_path,monkeypatch):
    calls=[]
    monkeypatch.setattr(planning,'plan',lambda *args: calls.append(args) or dict(status='planned'))
    assert cli.main(['plan','--scene','saved_scene','--state','saved_state','--candidates','8'])==0
    assert calls[-1][-1]==8
    path=tmp_path/'plan.json';path.write_text('{"status":"planned"}')
    monkeypatch.setattr(current_plan,'run',lambda *a,**kw:calls.append(kw) or path)
    assert cli.main(['live','--headless','--candidates','9'])==0
    assert calls[-1]['candidates']==9
