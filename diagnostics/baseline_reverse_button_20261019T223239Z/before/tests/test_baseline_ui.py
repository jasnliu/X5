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
        assert view.prepare_button.window() is view.window
        assert view.prepare_button.isVisible() is (not simulated)
        assert not view.viewport.errors
    finally:view.close()
