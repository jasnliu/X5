"""Software GUI integration test; saved scene only, no camera or CAN.
Recapture success/failure are exercised using an explicit filesystem fixture.
Run in a sourced ROS environment with PYTHONPATH=. and the planner Python:
python diagnostics/timing_experiment_20261021/verify_ui.py SESSION REPORT_DIRECTORY.
"""
import json,sys,time
from pathlib import Path
import numpy as np
from PyQt5 import QtCore,QtWidgets,QtTest
from drum_planner.experiment.config import METHODS,no_can
from drum_planner.experiment.app import make_view_class
from drum_planner.experiment import app
from drum_planner.common import atomic_json,digest
import shutil


def main(session,report_folder):
    no_can();session=Path(session);out=Path(report_folder);out.mkdir(parents=True,exist_ok=True)
    v=make_view_class()(session);checks=[]
    original_command=app.command
    capture_failure=[False]
    def fixture_command(action,folder,**values):
        if action!='capture':return original_command(action,folder,**values)
        if capture_failure[0]:return [sys.executable,'-c','raise RuntimeError("Intentional offline capture-failure test")']
        source=Path(v.context['scene']).parent
        code='import shutil; shutil.copytree('+repr(str(source))+','+repr(str(Path(folder)/'camera'))+')'
        return [sys.executable,'-c',code]
    app.command=fixture_command
    def pump(seconds=.15):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            key=v.poll()
            if key==ord('s'):v.save_view()
            assert key!=ord('q'),'Unexpected closed window'
            time.sleep(.01)
    def wait_jobs(limit=240):
        end=time.monotonic()+limit
        while v.job or v.pending_methods:
            pump(.05)
            assert time.monotonic()<end,'Jobs timed out'
        assert not v.viewport.errors
        assert not any(s=='failed' for s in v.method_states.values()),v.method_states
    def click(button):
        assert button.isEnabled(),button.text()
        QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton);pump()
    def choose(method):
        v.method_box.setCurrentIndex(list(METHODS).index(method));pump()
        if v.displayed_method!=method:v.select_method();pump()
        assert v.displayed_method==method
    def button(text):return next(b for b in v.window.findChildren(QtWidgets.QPushButton) if b.text()==text)
    try:
        wait_jobs()
        assert set(v.cache)==set(METHODS) and v.generation_count==len(METHODS)
        v.viewport.save_window(v.window,out/'initial_ui.png')
        results={m:dict(plan=str(p),status=json.loads(p.read_text())['status']) for m,(_,p) in v.cache.items()}
        atomic_json(out/'results.json',results)
        reports=[json.loads(Path(r['plan']).read_text())['experiment']['report'] for r in results.values()]
        assert len({r['pool_key'] for r in reports})==1
        assert all(r['pool_reused'] for r in reports[1:])
        assert all(r['duration_ratio_filter'] is None for r in reports)
        assert all(r['selected'] in {c['index'] for c in r['candidates']} for r in reports)
        uploads=v.viewport.surface.geometry_uploads;generations=v.generation_count
        print('AUTO-CACHE: all survivors ready on one shared proposal pool',flush=True)
        for method in METHODS:
            choose(method)
            assert not v.job and v.generation_count==generations
            assert v.viewport.surface.geometry_uploads==uploads
            assert v.plan.playable,(method,v.plan.obj.get('details'))
            assert v.plan.retimed is not None and v.plan.obj['timing']['method']=='adaptive'
            assert f'{v.plan.obj["duration_outward_s"]:.2f}s' in v.plan_label.text()
            initial=v.plan.frames(0);other='left' if v.side=='right' else 'right'
            fixed=[n for n in initial if n.startswith('openarmx_'+other+'_') or n==other+'_drumstick']
            for t in np.linspace(0,v.plan.duration,25):
                for name in fixed:assert np.array_equal(v.plan.frames(t)[name],initial[name]),name
            click(button('Go to goal'));assert abs(v.clock.position-v.plan.obj['duration_outward_s'])<1e-8
            click(button('Go to start'));assert v.clock.position==0
            v.timeline.setValue(500);pump();assert abs(v.clock.position-.5)<.001
            for index,rate in enumerate((.5,1.,2.)):
                v.speed.setCurrentIndex(index);pump(.05);assert v.clock.rate==rate
            click(button('Go to start'));click(v.play_button);pump(.2)
            click(v.play_button);paused=v.clock.position;pump(.15)
            assert not v.clock.playing and v.clock.position==paused
            click(v.play_button);end=time.monotonic()+v.plan.duration/2+3
            while v.clock.playing and time.monotonic()<end:pump(.05)
            assert not v.clock.playing and v.clock.completed_cycles==1
            assert np.allclose(v.robot.last_positions,v.plan.req['start'],atol=1e-10)
            checks.append(dict(method=method,played_entire_cycle=True,duration_s=v.plan.duration,
                               selected=v.plan.obj['experiment']['report']['selected']))
            print('PLAYBACK PASSED:',method,flush=True)
        v.loop_box.setChecked(True);v.seek(v.plan.duration-.1);click(v.play_button);pump(.3)
        assert v.clock.playing and v.clock.completed_cycles>=2
        click(v.play_button);v.loop_box.setChecked(False)
        for mode in (1,2,0):v.mode.setCurrentIndex(mode);pump()
        for color in (1,0):v.color.setCurrentIndex(color);pump()
        camera=v.viewport.renderer.GetActiveCamera();camera.Azimuth(30);camera.Elevation(15);camera.Dolly(1.15)
        v.draw_canvas();pump();v.reset_view();v.focus_arm();pump()
        assert v.viewport.surface.geometry_uploads==uploads
        click(v.save_button);assert (session/'preview_view.png').stat().st_size>1000
        click(button('Go to goal'))
        v.viewport.save_window(v.window,out/'gesture_ui.png')
        checks.append(dict(play_pause_seek_speed_loop_layers_color_navigation_save=True,mesh_uploads=uploads))
        old=dict(v.params);cache=dict(v.cache)
        def dismiss():
            d=QtWidgets.QApplication.activeModalWidget();assert d;d.reject()
        QtCore.QTimer.singleShot(100,dismiss);click(v.settings_button)
        assert v.params==old and v.cache==cache
        def accept():
            d=QtWidgets.QApplication.activeModalWidget();assert d
            d.findChild(QtWidgets.QComboBox).setCurrentIndex(2)
            d.findChild(QtWidgets.QSpinBox,'candidates').setValue(2)
            d.accept()
        QtCore.QTimer.singleShot(100,accept);click(v.settings_button)
        assert v.params['backtrack_weight']==6 and v.params['candidates']==2
        assert not v.cache and not v.plan.playable and v.job
        click(v.cancel_button);wait_jobs()
        assert not v.pending_methods and not v.job and v.settings_button.isEnabled()
        click(v.cache_all_button);wait_jobs()
        assert len(v.cache)==len(METHODS)
        # New budget invalidates the pool/queue; no old route remains playable.
        v.budget_spin.setValue(5);pump(.05);assert not v.cache and v.job
        wait_jobs();assert len(v.cache)==len(METHODS)
        key_before=json.loads(next(iter(v.cache.values()))[1].read_text())['experiment']['report']['pool_key']
        click(v.generate_button);wait_jobs()
        obj=json.loads(v.cache[v.active_method][1].read_text())
        assert obj['experiment']['report']['pool_reused'] and obj['experiment']['report']['pool_key']==key_before
        checks.append(dict(settings_accept_cancel_presets=True,cancel_resume=True,budget_invalidation=True,generate_reranks_shared_pool=True))
        oldscene=v.context['scene']
        capture_failure[0]=True;click(v.recapture_button);wait_jobs(100)
        assert v.context['scene']==oldscene and v.recapture_button.isEnabled()
        assert v.play_button.isEnabled()==v.plan.playable
        capture_failure[0]=False;click(v.recapture_button);wait_jobs(240)
        assert v.context['scene']!=oldscene and v.scene_generation==1 and len(v.cache)==len(METHODS)
        # This test deliberately copies the baseline snapshot, whose schema
        # does not contain the experiment capture worker's release annotation.
        # No camera ran: verify the exact fixture bytes instead.
        assert digest(v.context['scene'])==digest(oldscene)
        assert v.viewport.surface.geometry_uploads==uploads+1
        assert all(json.loads(p.read_text())['provenance']['scene']==v.context['scene'] for _,p in v.cache.values())
        checks.append(dict(offline_capture_fixture_and_requeue=True,capture_failure_recovery=True,actual_camera_opened=False))
        report=v.report();report.update(passed=True,feature_checks=checks)
        atomic_json(out/'ui_full.json',report)
        print('ALL GESTURE UI FEATURES PASSED',flush=True)
    finally:
        app.command=original_command;v.close();atomic_json(out/'ui_closed.json',v.report())


if __name__=='__main__':main(*sys.argv[1:])
