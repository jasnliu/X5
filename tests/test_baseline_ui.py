"""No devices: regression for the stray native readiness-button window."""
import socket,time
import numpy as np
import pytest


@pytest.mark.parametrize('simulated',[False,True])
def test_controls_are_parented_before_show_and_do_not_overlap(monkeypatch,tmp_path,simulated):
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')
    from PyQt5 import QtWidgets,QtCore
    from drum_planner.baseline import app
    from drum_planner import common,snapshot_preview
    from drum_planner.trajectory_view import PlanDisplay
    original=socket.socket
    def guarded(family=socket.AF_INET,*a,**k):
        if family==socket.AF_CAN:raise AssertionError('UI test must never open CAN')
        return original(family,*a,**k)
    monkeypatch.setattr(socket,'socket',guarded)
    monkeypatch.setattr(PlanDisplay,'check_live_calibration',lambda *a:None)
    monkeypatch.setattr(app,'load',lambda *a:snapshot_preview.prepare_scene(
        dict(sequence=1,received_monotonic=time.monotonic(),value_scale_m=.001),
        np.full((40,48),1000,dtype=np.uint16),
        {'depth':{'K':[[500,0,20],[0,500,16],[0,0,1]]}},np.eye(4)))
    shown=[];set_visible=QtWidgets.QPushButton.setVisible
    def show(button,visible):
        if visible:
            assert button.parentWidget() is not None, 'Parentless button would open a second native window'
            shown.append(button.text())
        set_visible(button,visible)
    monkeypatch.setattr(QtWidgets.QPushButton,'setVisible',show)
    view=app.View(common.ROOT/'planner_runs/software_demo/plan.json',tmp_path,simulated=simulated)
    try:
        for width,height in ((1440,900),(1120,820)):
            view.window.resize(width,height);view.app.processEvents()
            buttons=[b for b in view.window.findChildren(QtWidgets.QPushButton) if b.isVisible()]
            rectangles=[QtCore.QRect(b.mapTo(view.window,QtCore.QPoint()),b.size()) for b in buttons]
            for i,(b,r) in enumerate(zip(buttons,rectangles)):
                assert view.window.rect().contains(r),b.text()
                assert not any(r.intersects(other) for other in rectangles[i+1:]),b.text()
        assert view.reverse_button.isEnabled()
        assert view.prepare_button.window() is view.window
        assert view.prepare_button.isVisible() is (not simulated)
        assert not view.viewport.errors
        # Measured transforms use BOTH arms and the exact same retained meshes.
        from drum_planner.baseline.feedback import normalized_pose
        from drum_planner.common import JOINTS,LEFT_JOINTS
        uploads=[layer.geometry_uploads for *_,layer in view.robot.mesh_entries]
        pose=dict(zip(JOINTS,view.plan.req['start']))|dict(zip(LEFT_JOINTS,view.plan.req['left']))
        pose['openarmx_left_finger_joint1']=.012
        pose[JOINTS[0]]+=.15;pose[LEFT_JOINTS[1]]+=.1
        view.display_pose=normalized_pose(pose,time.monotonic());view.set_live_display(True)
        assert np.allclose(view.robot.last_positions,[pose[j] for j in JOINTS])
        frames=view.plan.model.transforms(pose)
        for name,_,local,layer in view.robot.mesh_entries:
            assert np.allclose(layer._last_transform,frames[name]@local)
        assert 'ENCODER' in view.viewport.title or 'MEASURED' in view.viewport.title
        view.display_pose['stamp']-=2.;view.draw_canvas()
        assert 'STALE' in view.viewport.title
        view.toggle_play();assert not view.live_display and view.clock.playing
        view.seek(0);assert not view.clock.playing
        view.live_box.click();assert view.live_display
        assert uploads==[layer.geometry_uploads for *_,layer in view.robot.mesh_entries]
        assert view.viewport.surface.geometry_uploads==1
    finally:view.close()
