"""Attended-display software QA; actual camera recapture only, never CAN."""
import json,time,sys
from pathlib import Path
import numpy as np
from PyQt5 import QtWidgets,QtCore,QtTest
from drum_planner.experiment.app import make_view_class
from drum_planner.experiment.config import METHODS,no_can,settings,BOUND_FIELDS
from drum_planner.common import atomic_json,digest
no_can();root=Path(Path('/tmp/x5_experiment_task').read_text().strip());session=root/'actual_camera_all'
results=json.loads((session/'results.json').read_text());v=make_view_class()(session)
checks=[]
def pump(seconds=.2):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        key=v.poll()
        if key==ord('s'):v.save_view()
        time.sleep(.01)
def wait_job(limit=130):
    start=time.monotonic()
    while v.job:
        pump(.05)
        if time.monotonic()-start>limit:raise AssertionError('GUI job did not finish')
    assert not v.viewport.errors

def click(button):
    QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton);pump()
try:
    # Cache toggles use real method outputs, retaining the EXACT scene/model.
    for method,row in results.items():v.cache[method]=(v.signature(),Path(row['plan']))
    first_uploads=v.viewport.surface.geometry_uploads
    for method,row in results.items():
        index=list(METHODS).index(method)
        if v.method_box.currentIndex()==index:v.select_method()
        else:v.method_box.setCurrentIndex(index)
        pump();assert v.displayed_method==method
        assert v.job is None and v.viewport.surface.geometry_uploads==first_uploads
        if v.plan.playable:
            goal=next(b for b in v.window.findChildren(QtWidgets.QPushButton) if b.text()=='Go to goal')
            click(goal);assert abs(v.clock.position-v.plan.obj['outward'][-1]['time_s'])<1e-9
            start=next(b for b in v.window.findChildren(QtWidgets.QPushButton) if b.text()=='Go to start')
            click(start);assert v.clock.position==0
            # Run a complete real-time cycle at the user-selectable 2x speed.
            v.speed.setCurrentIndex(2);v.loop_box.setChecked(False)
            click(v.play_button);assert v.clock.playing
            pump(.1);click(v.play_button);paused=v.clock.position;pump(.15)
            assert not v.clock.playing and v.clock.position==paused
            click(v.play_button)
            end=time.monotonic()+v.plan.duration/2+2
            while v.clock.playing and time.monotonic()<end:pump(.04)
            assert not v.clock.playing and v.clock.completed_cycles==1
            assert np.allclose(v.robot.last_positions,v.plan.req['start'],atol=1e-10)
            checks.append(dict(method=method,played=True,duration=v.plan.duration,completed_cycles=v.clock.completed_cycles))
        else:
            assert not v.play_button.isEnabled() and not v.timeline.isEnabled()
            assert v.plan.obj['status']=='no_feasible_route'
            checks.append(dict(method=method,played=False,normal_blocked_result=v.plan.obj['details']['reason']))
    # All render controls and camera navigation preserve geometry.
    for mode in (1,2,0):v.mode.setCurrentIndex(mode);pump()
    for color in (1,0):v.color.setCurrentIndex(color);pump()
    v.viewport.renderer.GetActiveCamera().Azimuth(30);v.viewport.renderer.GetActiveCamera().Elevation(15);v.draw_canvas();pump()
    v.reset_view();v.focus_arm();pump()
    assert v.viewport.surface.geometry_uploads==first_uploads
    v.request_save();pump();assert (session/'preview_view.png').stat().st_size>1000
    # Loop wraps, timeline seek and speed update.
    successful=next(k for k,row in results.items() if json.loads(Path(row['plan']).read_text())['status']=='planned')
    v.method_box.setCurrentIndex(list(METHODS).index(successful));pump()
    v.loop_box.setChecked(True);v.seek(v.plan.duration-.1);click(v.play_button);pump(.4)
    assert v.clock.playing and v.clock.completed_cycles>=1
    click(v.play_button);v.loop_box.setChecked(False)
    v.timeline.setValue(1000);pump();assert abs(v.clock.position-1)<.001
    # Open actual uncertainty dialog, select preset, accept, prove invalidation.
    def accept_settings():
        dialog=QtWidgets.QApplication.activeModalWidget();assert dialog
        combo=dialog.findChild(QtWidgets.QComboBox);combo.setCurrentIndex(1)
        buttons=dialog.findChild(QtWidgets.QDialogButtonBox)
        QtTest.QTest.mouseClick(buttons.button(QtWidgets.QDialogButtonBox.Ok),QtCore.Qt.LeftButton)
    QtCore.QTimer.singleShot(100,accept_settings);click(v.settings_button)
    assert v.params['depth_mm']==1 and not v.plan.playable
    # Real generation -> cancel; process tree exits without any partial playback.
    click(v.generate_button);assert v.job is not None
    pump(1.5);click(v.cancel_button);wait_job()
    assert not v.plan.playable and 'CANCELLED' in v.status.text()
    # Successful generation through UI under explicitly nominal uncertainty,
    # exercises the robust success branch even when moderate bounds block rest.
    v.params=settings({key:0. for key in BOUND_FIELDS})
    v.method_box.setCurrentIndex(list(METHODS).index('robust'));wait_job()
    assert v.plan.playable, v.plan.obj
    v.speed.setCurrentIndex(2);click(v.play_button)
    end=time.monotonic()+v.plan.duration/2+2
    while v.clock.playing and time.monotonic()<end:pump(.04)
    assert v.clock.completed_cycles>=1
    checks.append(dict(method='robust_explicit_zero_error_control',played=True,plan=str(v.plan.path)))
    # Explicit actual-camera recapture: old routes/cache invalidated and camera
    # released. Cancelled/failed capture must not corrupt the previous scene.
    oldscene=v.context['scene'];oldhash=v.context['scene_sha256'];click(v.recapture_button);wait_job(50)
    assert v.context['scene']!=oldscene and v.context['scene_sha256']!=oldhash
    assert not v.cache and not v.plan.playable and v.scene_generation==1
    assert v.viewport.surface.geometry_uploads==first_uploads+1
    assert json.loads(Path(v.context['scene']).read_text())['camera_released']
    checks.append(dict(recapture=True,scene=v.context['scene']))
    # UI-generated clearance plan on the NEW scene, no stale overlay/cache.
    v.params=settings();v.method_box.setCurrentIndex(list(METHODS).index('clearance'));wait_job()
    assert v.displayed_method=='clearance'
    assert v.plan.obj['provenance']['scene']==v.context['scene']
    assert v.plan.obj['status'] in ('planned','no_feasible_route')
    if v.plan.playable:
        v.speed.setCurrentIndex(2);click(v.play_button)
        end=time.monotonic()+v.plan.duration/2+2
        while v.clock.playing and time.monotonic()<end:pump(.04)
        assert v.clock.completed_cycles>=1
    checks.append(dict(new_scene_generation=True,status=v.plan.obj['status']))
    v.focus_arm();v.viewport.save_window(v.window,root/'experiment_final_ui.png')
    report=v.report();report.update(passed=True,feature_checks=checks)
    atomic_json(root/'ui_full.json',report)
    print('FULL UI QA PASSED',json.dumps(checks),flush=True)
finally:v.close()
