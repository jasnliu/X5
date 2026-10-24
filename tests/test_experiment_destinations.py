"""Destination discovery, frozen inputs and experiment-only goal routing."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from drum_planner.common import ROOT,DEFAULT_CALIBRATION,endpoint,goal_recording,digest
from drum_planner.experiment import app,config,destinations,worker


@pytest.mark.parametrize('side',['right','left'])
def test_list_is_arm_specific_and_naturally_sorted(tmp_path,side):
    assert destinations.files(side,tmp_path)==[]
    folder=tmp_path/side;folder.mkdir()
    other=tmp_path/('left' if side=='right' else 'right');other.mkdir()
    (other/'wrong.json').write_text('{}')
    for name in ('destination10.json','destination2.json','destination1.json','notes.txt'):
        (folder/name).write_text('{}')
    assert [p.name for p in destinations.files(side,tmp_path)]==['destination1.json','destination2.json','destination10.json']


@pytest.mark.parametrize('side',['right','left'])
def test_selection_is_frozen_arm_validated_and_tamper_detected(tmp_path,side):
    source=tmp_path/'pose.json';source.write_bytes(goal_recording(side).read_bytes())
    context=destinations.freeze(source,side,tmp_path/'session');context['moving_side']=side
    before=destinations.goal(context)
    source.write_text('{}')
    assert destinations.goal(context)==before
    Path(context['goal_recording']).write_text('{}')
    with pytest.raises(ValueError,match='snapshot changed'):destinations.goal(context)
    with pytest.raises(ValueError,match='Incompatible'):
        destinations.freeze(goal_recording('right' if side=='left' else 'left'),side,tmp_path/'session')


def test_headless_legacy_contexts_keep_default_endpoints():
    for side in ('right','left'):
        path,values=destinations.goal(dict(moving_side=side))
        assert path==goal_recording(side)
        assert values==endpoint(side=side).tolist()


def test_start_plan_passes_private_context_to_worker(tmp_path,monkeypatch):
    original=dict(destination_sha256='one',goal_recording='frozen-one.json')
    (tmp_path/'context.json').write_text(json.dumps(original))
    calls=[]
    monkeypatch.setattr(app,'Job',lambda cmd,*a,**kw:calls.append(cmd) or SimpleNamespace())
    target,job=app.start_plan(tmp_path,'single_arc',config.settings(),5)
    (tmp_path/'context.json').write_text(json.dumps(dict(destination_sha256='two')))
    assert json.loads((target/'context.json').read_text())==original
    assert calls[0][calls[0].index('--context')+1]==str(target/'context.json')


@pytest.mark.parametrize('side',['right','left'])
def test_worker_uses_selected_destination_not_builtin_recording(tmp_path,monkeypatch,side):
    record=json.loads(goal_recording(side).read_text())
    record['samples'][-1]['positions_rad'][0]+=.025
    selected=tmp_path/'different.json';selected.write_text(json.dumps(record))
    state,scene=config.saved_inputs(side)
    context=dict(state=str(state),scene=str(scene),scene_sha256=digest(scene),state_sha256=digest(state),
                 calibration=str(DEFAULT_CALIBRATION),moving_side=side,pool_root=str(tmp_path/'pools'))
    context.update(destinations.freeze(selected,side,tmp_path))
    context[side+'_zone']=worker.zone_metadata(None)
    monkeypatch.setattr(worker.planning,'scene',lambda *a:dict(synthetic=True))
    req=dict(goal=endpoint(side=side).tolist(),scene=dict(synthetic=True),collision_policy='observed_obstacles')
    monkeypatch.setattr(worker.planning,'request',lambda *a,**kw:(dict(req),{}))
    seen=[]
    from drum_planner.experiment import algorithms
    monkeypatch.setattr(algorithms,'run',lambda method,request,*a:(seen.append(request['goal']) or None,dict(reason='test_no_route')))
    result=worker.plan(context,tmp_path/'result','single_arc',config.settings(),5)
    assert seen==[endpoint(selected,side=side).tolist()]
    assert result['recording_sha256']==digest(selected)
    assert result['experiment']['destination_sha256']==digest(selected)
    assert str(selected) in result['experiment']['goal_source']
    assert result['status']=='no_feasible_route'
    assert json.loads((tmp_path/'result/backend_request.json').read_text())['goal']==seen[0]
    assert not result['motion_authorized'] and not result['hardware_execution_available']
