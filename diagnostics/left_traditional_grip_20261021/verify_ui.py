import json,sys,time
from pathlib import Path
import numpy as np
from drum_planner.experiment.config import no_can,METHODS
no_can()
from drum_planner.experiment.app import make_view_class
from drum_planner.common import atomic_json
folder=Path(sys.argv[1]).resolve();results=json.loads((folder/'results.json').read_text())
view=make_view_class()(folder,precompute=False)
assert view.side=='left'
from drum_planner.model import LEFT_STICK_LENGTH
assert view.robot.stick_length==LEFT_STICK_LENGTH
errors=[];sys.excepthook=lambda t,v,b:errors.append(str(v))
checks=[]
def pump(duration=.12):
    until=time.monotonic()+duration
    while time.monotonic()<until:
        key=view.poll()
        if key==ord('s'):view.save_view()
        time.sleep(.01)
    assert not errors,errors

def button(label):return next(b for b in view.window.findChildren(view.QtWidgets.QPushButton) if b.text()==label)
try:
    view.window.resize(1440,960);pump()
    for method,row in results.items():
        assert row['status']=='planned'
        view.cache[method]=(view.signature(),Path(row['plan']));view.method_states[method]='ready'
    view.update_queue_status()
    mesh=view.mesh;packet=view.packet;uploads=view.viewport.surface.geometry_uploads
    for i,method in enumerate(METHODS):
        view.method_box.setCurrentIndex(i);view.select_method();pump()
        assert view.plan.side==view.side and view.plan.playable
        assert view.displayed_method==method and view.play_button.isEnabled()
        first=view.plan.frames(0);other='right' if view.side=='left' else 'left'
        fixed=[n for n in first if n.startswith('openarmx_'+other+'_') or n==other+'_drumstick']
        for t in np.linspace(0,view.plan.duration,35):
            view.timeline.setValue(int(round(t*1000)));view.draw_canvas()
            actual=view.plan.frames(view.clock.position)['left_drumstick']
            assert np.allclose(view.robot.last_tip,actual[:3,3]+actual[:3,2]*LEFT_STICK_LENGTH/2)
            for name in fixed:assert np.array_equal(view.plan.frames(t)[name],first[name]),name
        button('Go to goal').click();pump()
        assert np.array_equal(view.plan.positions(view.clock.position),view.plan.req['goal'])
        if method=='single_arc':
            view.viewport.save_window(view.window,folder/(view.side+'_single_arc_goal.png'))
        button('Go to start').click();assert view.clock.position==0
        view.play_button.click();pump(.25);assert view.clock.playing and view.clock.position>0
        view.play_button.click();position=view.clock.position;pump();assert view.clock.position==position
        for index,rate in enumerate((.5,1.,2.)):
            view.speed.setCurrentIndex(index);assert view.clock.rate==rate
        view.seek(view.plan.duration-.01);view.loop_box.setChecked(True);before=view.clock.completed_cycles
        view.play_button.click();pump(.12);assert view.clock.completed_cycles>before
        view.play_button.click();view.loop_box.setChecked(False)
        view.seek(view.plan.duration-.01);view.play_button.click();pump(.12)
        assert not view.clock.playing and view.clock.position==view.plan.duration
        assert np.array_equal(view.plan.positions(view.plan.duration),view.plan.req['start'])
        assert view.mesh is mesh and view.packet is packet and view.viewport.surface.geometry_uploads==uploads
        checks.append(dict(method=method,side=view.side,play_pause=True,seek=True,goal=True,exact_return=True,loop=True,speed=True,other_arm_unchanged=True))
    for index in range(3):view.mode.setCurrentIndex(index);pump()
    for index in range(2):view.color.setCurrentIndex(index);pump()
    view.viewport.camera.Azimuth(35);view.viewport.camera.Elevation(10);view.draw_canvas();pump()
    button('Fit arm + kit').click();button('Reset / fit view [R]').click();pump()
    view.save_button.click();pump();assert (folder/'preview_view.png').exists()
    # Open the actual settings dialog; cancel leaves parameters/cache untouched.
    def cancel_dialog():
        dialog=view.app.activeModalWidget();assert dialog is not None;dialog.reject()
    old=dict(view.params);view.QtCore.QTimer.singleShot(50,cancel_dialog);view.settings_button.click()
    assert view.params==old and len(view.cache)==4
    # A real contained worker is cancelled, then missing work is resumed.
    view.generate_button.click();assert view.job is not None
    view.cancel_button.click();end=time.monotonic()+12
    while view.job and time.monotonic()<end:pump(.05)
    assert view.job is None and not view.pending_methods and view.generate_button.isEnabled()
    view.cache_all_button.click();end=time.monotonic()+180
    while view.job and time.monotonic()<end:pump(.1)
    assert view.job is None and len(view.cache)==4 and all(view.cached(m) for m in METHODS)
    view.method_box.setCurrentIndex(2);view.select_method();button('Go to goal').click();pump()
    assert not view.viewport.errors
    report=view.report();report.update(tested_controls=checks,layer_and_color_switches=True,camera_navigation=True,
        settings_dialog_cancel=True,real_worker_cancel_and_resume=True,errors=errors,
        no_scene_geometry_reupload_on_method_switch=True)
    atomic_json(folder/'arm_ui_verification.json',report)
    view.viewport.save_window(view.window,folder/(view.side+'_verified_ui.png'))
    # Close-up at the real saved endpoint, using the same frozen mesh.
    camera=view.viewport.camera
    camera.SetFocalPoint(.24,.27,.19);camera.SetPosition(1.12,-.70,.80);camera.SetViewUp(0,0,1)
    view.viewport.renderer.ResetCameraClippingRange();view.draw_canvas();pump()
    view.viewport.save_window(view.window,folder/'left_traditional_goal_closeup.png')
    print(json.dumps(dict(side=view.side,methods=len(checks),errors=errors,renderer=report['renderer'],report=str(folder/'arm_ui_verification.json')),indent=2))
finally:view.close()
