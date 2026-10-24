"""Real Qt/VTK, saved scene, contained solver; no CAN/camera or physical motion.

Run with sourced ROS and PYTHONPATH including the repo; --output must be new.
"""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import time
from unittest.mock import patch

import numpy as np

from drum_planner.common import atomic_json,endpoint
from drum_planner.experiment import app,destinations
from drum_planner.experiment.config import no_can
from drum_planner.experiment.fast_config import METHODS


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--side',choices=['right','left'],required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--second',action='store_true',help='Also solve the second saved destination')
    parser.add_argument('--all',action='store_true',help='Exercise every saved destination')
    parser.add_argument('--full-playback',action='store_true',help='Play each complete outward/hold/reverse cycle')
    parser.add_argument('--first-method',choices=METHODS,default=next(iter(METHODS)))
    parser.add_argument('--reuse-session',action='store_true',help='Reuse an existing software session and its checked caches')
    parser.add_argument('--cpu-only',action='store_true',help='Force software Vulkan in workers to verify visible CPU fallback')
    args=parser.parse_args();no_can()
    if args.cpu_only:
        original_command=app.command
        app.command=lambda *a,**kw:['/usr/bin/env','VK_DRIVER_FILES=/usr/share/vulkan/icd.d/lvp_icd.json',*original_command(*a,**kw)]
    if not args.reuse_session:app.prepare(args.output,side=args.side)
    view=app.make_view_class()(args.output,method=args.first_method)
    errors=[];old_hook=sys.excepthook
    sys.excepthook=lambda t,v,b:errors.append(str(v))
    checks=[]

    def pump(seconds=.1):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            key=view.poll()
            if key==ord('s'):view.save_view()
            time.sleep(.01)
        assert not errors,errors

    def wait_all():
        deadline=time.monotonic()+900
        while (view.job or view.pending_methods) and time.monotonic()<deadline:pump(.1)
        assert not view.job and not view.pending_methods,'Calculation did not finish'
        assert len(view.cache)==len(METHODS),(view.method_states,view.status.text())
        assert not any(e['event'] in ('worker_stopped','result_rejected','worker_start_failed') for e in view.events),list(view.events)

    def button(text):
        return next(b for b in view.window.findChildren(view.QtWidgets.QPushButton) if b.text()==text)

    def choose(path):
        index=view.destination_box.findData(str(path))
        assert index>0
        view.destination_box.setCurrentIndex(index)

    def exercise_methods(source):
        expected=endpoint(source,side=args.side)
        mesh=view.mesh;packet=view.packet;uploads=view.viewport.surface.geometry_uploads
        for index,method in enumerate(METHODS):
            view.method_box.setCurrentIndex(index);view.select_method();pump()
            assert view.displayed_method==method
            assert np.array_equal(view.plan.req['goal'],expected)
            assert view.plan.obj['recording_sha256']==view.context['destination_sha256']
            assert view.plan.obj['status'] in ('planned','no_feasible_route')
            if view.plan.playable:
                if args.cpu_only:
                    assert view.plan.obj['experiment']['retiming']['actual_backend']=='cpu'
                    assert 'CPU FALLBACK' in view.plan_label.text()
                button('Go to goal').click();pump()
                assert np.array_equal(view.plan.positions(view.clock.position),expected)
                button('Go to start').click();pump()
                view.play_button.click();pump(.2)
                assert view.clock.playing and view.clock.position>0
                view.play_button.click();position=view.clock.position;pump()
                assert view.clock.position==position
                view.timeline.setValue(int(view.plan.duration*400));pump()
                for i,rate in enumerate((.5,1.,2.)):
                    view.speed.setCurrentIndex(i);assert view.clock.rate==rate
                view.speed.setCurrentIndex(1)
                view.loop_box.setChecked(True);view.seek(view.plan.duration-.01)
                count=view.clock.completed_cycles;view.play_button.click();pump(.15)
                assert view.clock.completed_cycles>count
                view.play_button.click();view.loop_box.setChecked(False)
                view.seek(view.plan.duration)
                assert np.array_equal(view.plan.positions(view.plan.duration),view.plan.req['start'])
                if args.full_playback:
                    view.speed.setCurrentIndex(2);view.seek(0);view.play_button.click()
                    end=time.monotonic()+view.plan.duration/2+5
                    while view.clock.playing and time.monotonic()<end:pump(.05)
                    assert not view.clock.playing and view.clock.position==view.plan.duration
                    assert np.allclose(view.robot.last_positions,view.plan.req['start'],atol=1e-10)
                    view.speed.setCurrentIndex(1)
            assert view.mesh is mesh and view.packet is packet
            assert view.viewport.surface.geometry_uploads==uploads
            checks.append(dict(destination=str(source),method=method,status=view.plan.obj['status'],playback_tested=view.plan.playable))

    try:
        pump(1.)
        assert view.destination_box.currentData() is None
        assert view.destination_path is None and view.job is None and view.generation_count==0
        view.generate_button.click();view.cache_all_button.click();pump()
        assert view.generation_count==0
        with patch.object(destinations,'files',return_value=[]):
            view.refresh_destinations_button.click();pump()
            assert view.destination_box.count()==1 and view.destination_box.currentData() is None
        view.refresh_destinations_button.click()
        with tempfile.TemporaryDirectory(prefix='x5-invalid-destination-') as temporary:
            bad=Path(temporary)/'bad.json';bad.write_text('{not json')
            view.destination_box.addItem('Invalid test destination',str(bad))
            view.destination_box.setCurrentIndex(view.destination_box.count()-1);pump()
            assert 'Destination not loaded' in view.status.text()
            assert view.destination_path is None and view.job is None
        view.refresh_destinations_button.click();pump()
        available=destinations.files(args.side)
        assert available
        assert [view.destination_box.itemData(i) for i in range(1,view.destination_box.count())]==list(map(str,available))
        primary=available[0]
        # Rapidly supersede a just-starting worker; only the last selection wins.
        choose(primary);old_job=view.job
        assert old_job is not None
        if len(available)>1:choose(available[1])
        view.destination_box.setCurrentIndex(0)
        assert not view.plan.playable and not view.cache
        choose(primary);assert view.discard_job
        wait_all();assert old_job.reaped
        assert any(e['event']=='obsolete_job_discarded' for e in view.events)
        assert all(view.cached(m) for m in METHODS)
        assert all(json.loads(path.read_text())['status']=='planned' for _,path in view.cache.values())
        exercise_methods(primary)
        view.refresh_destinations_button.click()
        assert view.destination_box.currentData()==str(primary)
        # Usual viewer controls and camera navigation still work on the same mesh.
        for i in range(3):view.mode.setCurrentIndex(i);pump()
        for i in range(2):view.color.setCurrentIndex(i);pump()
        button('Fit arm + kit').click();button('Reset / fit view [R]').click();pump()
        if view.plan.playable:button('Go to goal').click();pump()
        view.save_button.click();pump()
        assert (args.output/'preview_view.png').exists()
        view.viewport.save_window(view.window,args.output/'destination_dropdown.png')
        if 'workspace_curve' in METHODS:
            # Display an explicit no-route fixture in the real widgets. It is
            # labelled blocked, never executed or counted as a planning pass.
            source=view.cache['workspace_curve'][1];obj=json.loads(source.read_text())
            obj.update(status='no_feasible_route',details={'reason':'no_valid_workspace_curve_within_budget'})
            obj.pop('outward',None);obj.pop('cycle',None)
            obj['experiment']['report'].update(candidates=[],selected=None)
            fixture=args.output/'no_route_ui_fixture';fixture.mkdir(exist_ok=True)
            atomic_json(fixture/'plan.json',obj)
            atomic_json(fixture/'backend_request.json',json.loads((source.parent/'backend_request.json').read_text()))
            view.install_plan(fixture/'plan.json','workspace_curve');pump()
            assert not view.plan.playable and not view.play_button.isEnabled()
            assert view.method_box.isEnabled() and view.generate_button.isEnabled()
            view.generate_button.click();wait_all()
            assert view.displayed_method=='workspace_curve' and view.plan.playable
        # Cancel/retry remain usable with the destination retained.
        view.generate_button.click();assert view.job is not None
        view.cancel_button.click()
        deadline=time.monotonic()+12
        while view.job and time.monotonic()<deadline:pump()
        assert view.job is None and not view.pending_methods
        view.cache_all_button.click();wait_all()
        for extra in available[1:] if args.all else available[1:2] if args.second else []:
            choose(extra);assert not view.cache and not view.plan.playable
            wait_all();exercise_methods(extra)
        view.destination_box.setCurrentIndex(0);pump()
        count=view.generation_count;view.method_box.setCurrentIndex(0);view.select_method();pump()
        assert view.job is None and view.generation_count==count and not view.plan.playable
        assert not view.viewport.errors
        report=view.report();report.update(tested_destinations=checks,native_ui_errors=errors,
            startup_unselected=True,rapid_selection_cancellation=True,refresh=True,
            cancel_resume=True,controls_tested=True,empty_folder_message=True,
            invalid_destination_recovery=True,no_route_ui_fixture_tested='workspace_curve' in METHODS,
            full_cycles_played=args.full_playback,camera_access=False,forced_cpu_fallback=args.cpu_only)
        atomic_json(args.output/'destination_ui_verification.json',report)
        print('PASS native destination UI:',args.side,checks,flush=True)
    finally:
        view.close();sys.excepthook=old_hook


if __name__=='__main__':main()
