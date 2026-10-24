"""Native FCL regression: moving a leg start must not move camera self-masking."""
import argparse
import copy
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np

from drum_planner.common import atomic_json,endpoint,ROOT
from drum_planner.experiment import runtime


def check(request,folder,old_binary=None):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    original=json.loads(Path(request).read_text())
    side=original.get('moving_side','right')
    goal=endpoint(side=side).tolist()
    states=np.linspace(original['start'],goal,31)
    def query(req,name,binary=None):
        directory=folder/name;directory.mkdir(exist_ok=True)
        atomic_json(directory/'request.json',req)
        binary=binary or runtime.build()
        with patch.object(runtime,'build',return_value=binary):
            oracle=runtime.Oracle(directory/'request.json',directory)
            try:return oracle.meta,oracle.check(states)
            finally:oracle.close()
    nominal,nominal_checks=query(original,'nominal')
    if old_binary:
        old,old_checks=query(original,'legacy',Path(old_binary))
        assert old==nominal and old_checks==nominal_checks,'Legacy request behavior changed'
    moved=copy.deepcopy(original);moved.update(start=goal,scene_robot_start=original['start'])
    later,later_checks=query(moved,'later_leg')
    mask=dict(later['self_filter']);assert mask.pop('checkpoint_snapshot_start')==original['start']
    assert mask==nominal['self_filter'],'Camera self-masking changed between legs'
    assert later_checks==nominal_checks,'Same frozen world gave different collision results'
    # Demonstrate that the test distinguishes the original incorrect behavior.
    moved.pop('scene_robot_start');wrong,_=query(moved,'incorrect_remasking')
    assert wrong['self_filter']!=nominal['self_filter'],'Fixture did not exercise self-mask movement'
    result=dict(side=side,legacy_native_parity=bool(old_binary),snapshot_mask_invariant=True,
                collision_queries_identical=len(states),incorrect_remasking_detected=True,
                camera_access=False,motor_commands=0)
    atomic_json(folder/'verification.json',result);print(result,flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--request',required=True);p.add_argument('--output',required=True)
    p.add_argument('--old-binary');a=p.parse_args();check(a.request,a.output,a.old_binary)
