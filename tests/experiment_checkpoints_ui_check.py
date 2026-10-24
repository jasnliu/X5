"""Native checkpoint UI and full contained planning/playback, device-free.

Left has only one user destination, so two additional small endpoint variations
are test fixtures under the run folder, never additions to destinations/left.
"""
import argparse
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch

import numpy as np

from drum_planner.common import atomic_json,endpoint
from drum_planner.experiment import app,destinations,checkpoints,config
from drum_planner.experiment.fast_config import METHODS


def main():
    p=argparse.ArgumentParser();p.add_argument('--side',required=True,choices=['right','left'])
    p.add_argument('--output',required=True,type=Path);p.add_argument('--reuse-session',action='store_true')
    a=p.parse_args();config.no_can()
    if not a.reuse_session:app.prepare(a.output,side=a.side)
    available=destinations.files(a.side)
    fixtures=[]
    if len(available)<3:
        for i in range(3-len(available)):
            obj=json.loads(available[0].read_text())
            obj['samples'][-1]['positions_rad'][0]+=.015*(i+1)
            path=a.output/f'test-only-left-variation-{i+1}.json';atomic_json(path,obj)
            fixtures.append(path.resolve())
    available=available+fixtures
    errors=[];old_hook=sys.excepthook;sys.excepthook=lambda t,v,b:errors.append(str(v))
    checks=[]
    with patch.object(destinations,'files',return_value=available):
        view=app.make_view_class()(a.output,method='workspace_curve')
        def pump(seconds=.1):
            until=time.monotonic()+seconds
            while time.monotonic()<until:
                view.poll();time.sleep(.01)
            assert not errors,errors
        def wait_all():
            deadline=time.monotonic()+2400
            while (view.job or view.pending_methods) and time.monotonic()<deadline:pump(.1)
            assert not view.job and not view.pending_methods,'Queue timeout'
            assert len(view.cache)==len(METHODS),(view.method_states,view.status.text())
            assert not any(e['event'] in ('worker_stopped','result_rejected','worker_start_failed') for e in view.events),list(view.events)
        def choose(index,path):
            box=view.checkpoint_boxes[index];value=box.findData(str(path))
            assert value>0,(index,str(path));box.setCurrentIndex(value)
        def button(text):
            return next(b for b in view.window.findChildren(view.QtWidgets.QPushButton) if b.text()==text)
        def playback(count):
            mesh=view.mesh;uploads=view.viewport.surface.geometry_uploads
            for method in METHODS:
                view.method_box.setCurrentIndex(list(METHODS).index(method));view.select_method();pump()
                plan=view.plan;assert view.displayed_method==method
                row=dict(method=method,checkpoints=count,status=plan.obj['status'],path=str(plan.path))
                if plan.playable:
                    segments=plan.obj.get('checkpoint_sequence',{}).get('segments',[])
                    assert len(segments)==(count if count>1 else 0)
                    for segment in segments:
                        arrival=segment['arrival_time_s'];view.seek(arrival+.5);pump()
                        assert np.array_equal(view.robot.last_positions,segment['goal'])
                        assert 'HOLD' in view.plan.phase(view.clock.position)
                    if count>1:
                        sequence=plan.obj['checkpoint_sequence'];return_leg,evaluator=plan.return_leg
                        assert sequence['return_policy']==checkpoints.RETURN_POLICY
                        assert return_leg['start']==segments[-1]['goal']
                        assert return_leg['goal']==plan.req['start']
                        request=json.loads((Path(return_leg['plan']).parent/'backend_request.json').read_text())
                        assert request['start']==return_leg['start'] and request['goal']==return_leg['goal']
                        assert request['scene_robot_start']==plan.req['start']
                        for t in np.linspace(.001,evaluator.duration-.001,10):
                            when=return_leg['start_time_s']+t
                            view.seek(when);pump(.01)
                            assert np.allclose(view.robot.last_positions,evaluator(t)[0],atol=1e-10)
                            assert plan.phase(when)=='RETURN TO ORIGINAL START'
                        row['new_return_checked']=True
                    button('Go to goal').click();pump()
                    assert np.array_equal(plan.positions(view.clock.position),plan.req['goal'])
                    view.seek(0);view.play_button.click();pump(.15);view.play_button.click()
                    pos=view.clock.position;pump();assert pos==view.clock.position and pos>0
                    view.timeline.setValue(int(plan.duration*500));pump()
                    for i,rate in enumerate((.5,1.,2.)):
                        view.speed.setCurrentIndex(i);assert view.clock.rate==rate
                    view.loop_box.setChecked(True);view.seek(plan.duration-.01);old=view.clock.completed_cycles
                    view.play_button.click();pump(.1);assert view.clock.completed_cycles>old
                    view.play_button.click();view.loop_box.setChecked(False)
                    view.seek(0);view.play_button.click();deadline=time.monotonic()+plan.duration/2+5
                    while view.clock.playing and time.monotonic()<deadline:pump(.05)
                    assert view.clock.position==plan.duration and not view.clock.playing
                    assert np.allclose(view.robot.last_positions,plan.req['start'],atol=1e-10)
                    view.speed.setCurrentIndex(1);row['complete_cycle_played']=True
                else:
                    assert view.generate_button.isEnabled() and view.method_box.isEnabled()
                assert view.mesh is mesh and view.viewport.surface.geometry_uploads==uploads
                checks.append(row)
            assert not view.viewport.errors
        try:
            pump(.4);assert view.job is None and view.generation_count==0
            assert len(view.checkpoint_boxes)==1 and not view.remove_checkpoint_button.isEnabled()
            # First selection starts; adding an empty optional checkpoint cancels it.
            choose(0,available[0]);old_job=view.job
            view.add_checkpoint_button.click();pump()
            assert not view.plan.playable and view.destination_path is None
            assert view.checkpoint_boxes[1].findData(str(available[0]))<0
            view.remove_checkpoint_button.click();wait_all()
            assert old_job.reaped
            playback(1)
            # Two checkpoints work; third can be added during an active calculation.
            view.add_checkpoint_button.click();choose(1,available[1]);superseded=view.job
            view.add_checkpoint_button.click();choose(2,available[2]);wait_all()
            assert superseded.reaped
            assert len(view.checkpoint_boxes)==3 and not view.add_checkpoint_button.isEnabled()
            view.add_checkpoint();assert len(view.checkpoint_boxes)==3
            assert view.checkpoint_boxes[2].findData(str(available[1]))<0
            assert view.checkpoint_boxes[2].findData(str(available[0]))>0
            playback(3)
            view.method_box.setCurrentIndex(list(METHODS).index('workspace_curve'));view.select_method();pump()
            view.viewport.save_window(view.window,a.output/'checkpoints.png')
            # No stale sequence with the same final endpoint can be installed.
            path=view.cache['workspace_curve'][1];original=view.context['checkpoint_sha256']
            view.context['checkpoint_sha256']='different-order'
            try:
                view.install_plan(path,'workspace_curve')
                raise AssertionError('Stale sequence was accepted')
            except ValueError as exc:assert 'different checkpoint' in str(exc)
            finally:view.context['checkpoint_sha256']=original
            # Refresh, cancellation and retry preserve the whole ordered selection.
            selection=view.checkpoint_selection();view.refresh_destinations_button.click()
            assert view.checkpoint_selection()==selection
            view.generate_button.click();view.cancel_button.click()
            deadline=time.monotonic()+12
            while view.job and time.monotonic()<deadline:pump()
            assert not view.job and not view.pending_methods
            view.cache_all_button.click();wait_all()
            # Removing checkpoint 3 returns to a genuinely two-leg plan.
            view.remove_checkpoint_button.click();wait_all();playback(2)
            # A previous selection becoming the next one clears the duplicate.
            choose(0,available[1]);pump()
            assert view.checkpoint_boxes[1].currentData() is None and view.destination_path is None
            # Empty choices remain recoverable, especially on the real left folder.
            with patch.object(destinations,'files',return_value=[available[1]]):
                view.refresh_destinations_button.click();pump()
                assert view.checkpoint_boxes[1].count()==1
                assert view.remove_checkpoint_button.isEnabled()
            view.destination_box.setCurrentIndex(0);pump()
            deadline=time.monotonic()+12
            while view.job and time.monotonic()<deadline:pump()
            assert not view.job and not view.pending_methods
            report=dict(side=a.side,camera_access=False,motor_commands=0,checks=checks,
                native_ui_errors=errors,test_only_destinations=list(map(str,fixtures)),
                controls='add/remove/max/exclusion/order/refresh/cancel/retry/play/pause/scrub/speed/loop/goal/start',
                scene_geometry_retained=True,report=view.report())
            atomic_json(a.output/'checkpoint_ui_verification.json',report)
            print('CHECKPOINT UI PASS',a.side,[(r['method'],r['checkpoints'],r['status']) for r in checks],flush=True)
        finally:view.close();sys.excepthook=old_hook


if __name__=='__main__':main()
