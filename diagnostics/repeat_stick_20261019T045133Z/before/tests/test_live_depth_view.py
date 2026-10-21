"""Live-view math, bounded work, GUI controls and capture regressions; no devices."""
import json
import socket
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from drum_planner import common, depth
from drum_planner.live_view import build_packet, sampled, LatestCloudWorker, LiveDepthView


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    real=socket.socket
    def guarded(family=socket.AF_INET,*args,**kwargs):
        if family==socket.AF_CAN:raise AssertionError('No CAN in viewer tests')
        return real(family,*args,**kwargs)
    monkeypatch.setattr(socket,'socket',guarded)
    def denied(*a,**kw):raise AssertionError('No camera in offline tests')
    monkeypatch.setattr(depth,'DepthCamera',denied)


def inputs(sequence=5):
    meta={'depth':{'K':[[2.,0,1.],[0,2.,1.],[0,0,1.]]}}
    frames=[]
    for n in range(5):
        a=np.full((4,5),1000,dtype='<u2');a[0,0]=0;a[0,1]=65535
        a[1,0]=149;a[1,1]=5001
        # A thin obstacle seen in the oldest frame must survive voxel fusion.
        a[2,3]=500 if n==0 else 0
        h=dict(sequence=sequence-4+n,value_scale_m=.001,received_monotonic=time.monotonic())
        frames.append((h,a))
    transform=np.eye(4);transform[:3,3]=[.1,.2,.3]
    return meta,frames,transform


def test_display_packet_preserves_units_transform_and_unknowns():
    meta,frames,T=inputs();p=build_packet(meta,frames,T,maximum=100)
    assert p.sequence==5 and p.raw_count==15
    assert p.raw_image[0,0].tolist()==[0,0,0]
    assert p.raw_image[0,1].tolist()==[0,0,0]
    assert p.raw_image[1,0].tolist()==[0,0,0]
    assert p.raw_image[1,1].tolist()==[0,0,0]
    assert np.allclose(p.raw_points[:,2],1.3)
    _,_,minimum,_=depth.depth_window(frames)
    expected=depth.occupied_voxels(depth.deproject(minimum,meta,T))
    assert np.array_equal(p.voxels,expected)
    assert np.min(p.voxels[:,2])<.81  # close thin return, absent from last raw image
    assert np.all(p.raw_image[2,3]==0) and np.any(p.voxel_image[2,3]>0)


def test_sampling_affects_only_display_and_keeps_budget():
    meta,frames,T=inputs()
    small=build_packet(meta,frames,T,maximum=3)
    full=build_packet(meta,frames,T,maximum=100)
    assert len(small.raw_points)==len(small.voxels)==3
    assert small.raw_count==full.raw_count and small.voxel_count==full.voxel_count
    assert np.array_equal(small.bounds,full.bounds)
    assert np.array_equal(small.voxels,sampled(full.voxels,3))
    assert np.array_equal(small.raw_image,full.raw_image)
    with pytest.raises(ValueError):sampled(full.voxels,0)


def test_empty_returns_remain_empty_display_not_fake_surface():
    meta,frames,T=inputs()
    for _,a in frames:a[:]=0
    p=build_packet(meta,frames,T)
    assert p.raw_count==p.voxel_count==0 and p.bounds is None
    assert p.raw_points.shape==p.voxels.shape==(0,3)
    assert not p.raw_image.any() and not p.voxel_image.any()


def test_integer_voxelization_exactly_matches_original_full_resolution_algorithm():
    points=np.random.default_rng(7).uniform(-5,5,(100000,3))
    points=np.r_[points,points[:500],[[0,0,0],[-.01,0,.01],[.02,-.02,.03]]]
    original=(np.unique(np.floor(points/.01).astype(np.int32),axis=0).astype(float)+.5)*.01
    assert np.array_equal(depth.occupied_voxels(points),original)
    assert depth.occupied_voxels(np.empty((0,3))).shape==(0,3)
    with pytest.raises(ValueError):depth.occupied_voxels(points,0)
    with pytest.raises(ValueError):depth.occupied_voxels(np.array([[np.nan,0,0]]))


def test_worker_has_one_job_and_no_backlog():
    gate=threading.Event();seen=[]
    def builder(meta,frames,T):
        gate.wait(2);seen.append(frames[-1][0]['sequence']);return seen[-1]
    meta,frames,T=inputs();w=LatestCloudWorker(T,builder)
    try:
        assert w.submit(meta,frames)
        assert not w.submit(meta,inputs(6)[1])
        assert not w.submit(meta,inputs(7)[1])
        assert w.take() is None
        gate.set();w.future.result(timeout=3)
        assert w.take()==5 and not w.submit(meta,frames)
        assert w.submit(meta,inputs(100)[1]);w.future.result(timeout=3)
        assert w.take()==100 and seen==[5,100]
        assert w.submissions==2
    finally:gate.set();w.close()


def test_worker_errors_are_not_hidden():
    def bad(*a):raise ValueError('display failure')
    meta,frames,T=inputs();w=LatestCloudWorker(T,bad)
    try:
        w.submit(meta,frames)
        with pytest.raises(ValueError,match='display failure'):w.future.result(timeout=3)
        with pytest.raises(ValueError,match='display failure'):w.take()
    finally:w.close()


@pytest.fixture
def gui(tmp_path,monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')
    v=LiveDepthView(common.calibration(),tmp_path)
    try:yield v
    finally:v.close()


def test_gui_freeze_keeps_view_but_resume_accepts_latest(gui):
    meta,frames,T=inputs();p=build_packet(meta,frames,T)
    assert gui.accept(p)
    gui.freeze_button.click();assert gui.frozen
    newer=build_packet(*inputs(50))
    assert not gui.accept(newer) and gui.packet.sequence==5
    camera=SimpleNamespace(count=150,latest=inputs(150)[1][-1],meta=meta,frames=inputs(150)[1])
    gui.poll(camera)
    assert '150 frames' in gui.status.text() and 'FROZEN' in gui.status.text()
    assert gui.worker.submissions==0
    gui.freeze_button.click();assert not gui.frozen
    assert gui.accept(newer) and gui.packet.sequence==50
    assert gui.report()['freeze_count']==gui.report()['resume_count']==1


def test_gui_mode_height_zoom_pan_view_persist_and_reset(gui,tmp_path):
    gui.accept(build_packet(*inputs()))
    gui.mode.setCurrentIndex(1);gui.color.setCurrentIndex(1)
    assert 'voxel' in gui.viewport.title and gui.viewport.color_label=='World height (m)'
    gui.viewport.set_angles(11,37);camera=gui.viewport.camera
    camera.SetPosition(*(np.array(camera.GetPosition())+[.2,-.1,0]))
    camera.SetFocalPoint(*(np.array(camera.GetFocalPoint())+[.2,-.1,0])) # panned by operator
    distance=camera.GetDistance();gui.viewport.zoom(1.2);assert camera.GetDistance()<distance
    pose=(camera.GetPosition(),camera.GetFocalPoint(),camera.GetViewUp())
    gui.accept(build_packet(*inputs(100)))
    assert pose==(camera.GetPosition(),camera.GetFocalPoint(),camera.GetViewUp())
    assert gui.report()['modes_rendered']==['raw','voxels']
    gui.reset_view();assert pose!=(camera.GetPosition(),camera.GetFocalPoint(),camera.GetViewUp())
    expected=np.array(camera.GetPosition());gui.viewport.set_angles(24,-55)
    assert np.allclose(camera.GetPosition(),expected)
    gui.window.grab().save(str(tmp_path/'viewer.png'))
    assert (tmp_path/'viewer.png').stat().st_size>1000


def test_gui_save_close_buttons_and_empty_cloud(gui):
    meta,frames,T=inputs()
    for _,a in frames:a[:]=0
    gui.accept(build_packet(meta,frames,T));assert gui.packet.raw_count==0
    gui.request_save()
    camera=SimpleNamespace(count=5,latest=frames[-1],meta=meta,frames=frames)
    assert gui.poll(camera)==ord('s') and not gui.save_requested
    gui.saved();assert 'LIVE full-resolution' in gui.save_status.text()
    gui.window.close();assert gui.poll(camera)==ord('q')


def test_gui_discards_inflight_prefreeze_packet(gui):
    meta,frames,T=inputs();gui.accept(build_packet(meta,frames,T))
    assert gui.worker.submit(meta,frames)
    gui.toggle_freeze();gui.toggle_freeze()
    gui.worker.future.result(timeout=3)
    camera=SimpleNamespace(count=50,latest=inputs(50)[1][-1],meta=meta,frames=inputs(50)[1])
    old_rendered=gui.rendered;gui.poll(camera)
    assert gui.rendered==old_rendered # obsolete job discarded, not flashed on resume
    assert gui.worker.last_sequence==50


def test_gui_space_and_reset_shortcuts(gui):
    from PyQt5 import QtCore, QtTest
    gui.accept(build_packet(*inputs()))
    gui.window.activateWindow();gui.window.setFocus();gui.app.processEvents()
    QtTest.QTest.keyClick(gui.window,QtCore.Qt.Key_Space);gui.app.processEvents()
    assert gui.frozen
    QtTest.QTest.keyClick(gui.window,QtCore.Qt.Key_Space);gui.app.processEvents()
    assert not gui.frozen
    gui.viewport.set_angles(50,123)
    QtTest.QTest.keyClick(gui.window,QtCore.Qt.Key_R);gui.app.processEvents()
    camera=gui.viewport.camera;expected=np.array(camera.GetPosition());gui.viewport.set_angles(24,-55)
    assert np.allclose(camera.GetPosition(),expected)


def fake_camera_class():
    class FakeCamera:
        def __init__(self,*a):
            self.meta,self.frames,_=inputs()
            self.latest=self.frames[-1];self.error=None;self.count=5;self.maximum_gap_s=.1;self.closed=False
        def close(self):self.closed=True
    return FakeCamera


def test_capture_saves_live_frames_and_report_with_new_view(tmp_path,monkeypatch):
    from drum_planner import live_view
    class View:
        def __init__(self,*a):self.keys=iter([ord('s'),ord('q')]);self.closed=False
        def poll(self,camera):return next(self.keys)
        def saved(self):pass
        def close(self):self.closed=True
        def report(self):return {'type':'test_live_3d'}
    captured=[]
    monkeypatch.setattr(depth,'DepthCamera',fake_camera_class())
    monkeypatch.setattr(live_view,'LiveDepthView',View)
    monkeypatch.setattr(depth,'snapshot',lambda folder,meta,frames,cal:captured.append(frames[-1][0]['sequence']))
    folder=tmp_path/'capture';depth.capture(folder)
    report=json.loads((folder/'capture_report.json').read_text())
    assert captured==[5,5] and report['passed'] and report['display_mode']=='3d'
    assert report['visualization']['type']=='test_live_3d'


def test_headless_capture_does_not_construct_gui(tmp_path,monkeypatch):
    from drum_planner import live_view
    def fail(*a):raise AssertionError('No GUI allowed')
    monkeypatch.setattr(depth,'DepthCamera',fake_camera_class())
    monkeypatch.setattr(live_view,'LiveDepthView',fail)
    monkeypatch.setattr(depth,'snapshot',lambda *a:None)
    folder=tmp_path/'headless';depth.capture(folder,seconds=.001,headless=True)
    report=json.loads((folder/'capture_report.json').read_text())
    assert report['passed'] and report['display_mode']=='headless' and report['visualization'] is None


def test_gui_errors_cleanup_camera_and_report_failure(tmp_path,monkeypatch):
    from drum_planner import live_view
    cameras=[];views=[]
    class Camera(fake_camera_class()):
        def __init__(self,*a):super().__init__(*a);cameras.append(self)
    class View:
        def __init__(self,*a):self.closed=False;views.append(self)
        def poll(self,camera):raise RuntimeError('render failed')
        def close(self):self.closed=True
        def report(self):return {}
    monkeypatch.setattr(depth,'DepthCamera',Camera);monkeypatch.setattr(live_view,'LiveDepthView',View)
    folder=tmp_path/'bad'
    with pytest.raises(RuntimeError,match='render failed'):depth.capture(folder)
    report=json.loads((folder/'capture_report.json').read_text())
    assert not report['passed'] and cameras[0].closed and views[0].closed


def test_camera_stale_check_still_runs_with_frozen_gui(tmp_path,monkeypatch):
    from drum_planner import live_view
    cameras=[];views=[]
    class Camera(fake_camera_class()):
        def __init__(self,*a):
            super().__init__(*a);cameras.append(self)
            self.latest[0]['received_monotonic']=time.monotonic()-2
    class View:
        def __init__(self,*a):self.closed=False;self.frozen=True;views.append(self)
        def poll(self,camera):raise AssertionError('Staleness must be checked before the GUI')
        def close(self):self.closed=True
        def report(self):return {}
    monkeypatch.setattr(depth,'DepthCamera',Camera);monkeypatch.setattr(live_view,'LiveDepthView',View)
    folder=tmp_path/'stale'
    with pytest.raises(RuntimeError,match='stale'):depth.capture(folder)
    report=json.loads((folder/'capture_report.json').read_text())
    assert not report['passed'] and cameras[0].closed and views[0].closed
