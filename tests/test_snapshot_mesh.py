"""Offline-only mesh math, one-shot lifecycle and static trajectory viewer."""
import io
import json
import socket
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from drum_planner import common,depth,snapshot_preview as snapshot
from drum_planner.surface_mesh import build_surface,depth_metres
from drum_planner.trajectory_view import PlanDisplay


DEMO=common.ROOT/'planner_runs/software_demo/plan.json'
DepthCamera=depth.DepthCamera


@pytest.fixture(autouse=True)
def no_devices(monkeypatch):
    real=socket.socket
    def guarded(family=socket.AF_INET,*args,**kwargs):
        if family==socket.AF_CAN:raise AssertionError('No CAN in preview')
        return real(family,*args,**kwargs)
    def denied(*a,**kw):raise AssertionError('Offline tests cannot access the camera')
    monkeypatch.setattr(socket,'socket',guarded)
    monkeypatch.setattr(depth,'DepthCamera',denied);monkeypatch.setattr(snapshot,'DepthCamera',denied)


def meta():return {'depth':{'K':[[500,0,20],[0,500,16],[0,0,1]]}}


def frame():
    return dict(sequence=1,received_monotonic=time.monotonic(),value_scale_m=.001),np.full((40,48),1000,np.uint16)


def make(raw,**kwargs):return build_surface(raw,.001,meta(),np.eye(4),**kwargs)


def test_plane_native_grid_units_fov_winding_and_immutable():
    raw=np.full((34,42),1000,np.uint16);mesh=make(raw)
    assert mesh.faces.shape==(2*5*6,3)
    assert set(mesh.source_pixels[:,0])=={0,8,16,24,32,40,41}
    assert set(mesh.source_pixels[:,1])=={0,8,16,24,32,33}
    assert np.allclose(mesh.vertices[:,2],1)
    assert np.allclose(mesh.vertices[:,:2],(mesh.source_pixels-[20,16])/500)
    tri=mesh.vertices[mesh.faces]
    assert np.all(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])[:,2]>0)
    with pytest.raises(ValueError):mesh.vertices[0,0]=7
    with pytest.raises(ValueError):mesh.faces[0,0]=7
    assert not mesh.stats['hole_filling'] and mesh.stats['preview_only']


def test_calibrated_transform_applied():
    raw=np.full((17,17),1000,np.uint16);T=np.eye(4)
    T[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];T[:3,3]=[.3,-.2,1]
    mesh=build_surface(raw,.001,meta(),T)
    native=make(raw)
    assert np.allclose(mesh.vertices,native.vertices@T[:3,:3].T+T[:3,3])


@pytest.mark.parametrize('bad',[0,65535,100,6000])
def test_invalid_unsampled_pixel_never_gets_bridged(bad):
    raw=np.full((17,17),1000,np.uint16);raw[3,4]=bad
    mesh=make(raw);assert len(mesh.faces)==6
    pixels=mesh.source_pixels[mesh.faces]
    assert not np.any(np.all((pixels[:,:,0]<=8)&(pixels[:,:,1]<=8),axis=1))


def test_depth_boundaries_even_between_sampled_corners():
    raw=np.full((17,17),1000,np.uint16);raw[4,4]=1600
    assert len(make(raw).faces)==6
    raw=np.full((17,17),1000,np.uint16);raw[:,9:]=1800
    mesh=make(raw);assert len(mesh.faces)==4
    assert np.all(np.ptp(mesh.camera_depth_m[mesh.faces],axis=1)<.001)


def test_long_edges_empty_input_and_bad_parameters():
    raw=np.full((17,17),1000,np.uint16)
    assert len(make(raw,max_edge_m=.001).faces)==0
    empty=make(np.zeros((17,17),np.uint16))
    assert empty.vertices.shape==(0,3) and empty.faces.shape==(0,3)
    with pytest.raises(ValueError):make(raw,step=0)
    with pytest.raises(ValueError):make(raw,max_edge_m=float('nan'))
    with pytest.raises(ValueError):depth_metres(raw,0)
    raw=raw.astype(float);raw[3,4]=float('nan')
    assert np.isfinite(depth_metres(raw,.001)).all()
    assert len(make(raw).faces)==6


def test_prepare_rejects_empty_or_discontinuous_frame():
    h,raw=frame()
    with pytest.raises(ValueError,match='Too few'):snapshot.prepare_scene(h,raw*0,meta(),np.eye(4))
    raw[:,::2]=2000
    with pytest.raises(ValueError,match='No continuous'):snapshot.prepare_scene(h,raw,meta(),np.eye(4))


def test_saved_mesh_is_not_a_planner_scene(tmp_path):
    h,raw=frame();cal=common.calibration()
    packet,mesh=snapshot.prepare_scene(h,raw,meta(),np.eye(4))
    snapshot.save_scene(tmp_path,h,raw,meta(),cal,common.DEFAULT_CALIBRATION,mesh)
    saved=json.loads((tmp_path/'scene.json').read_text())
    assert saved['schema']=='x5-depth-surface-preview-v1' and saved['frames_used']==1
    assert not saved['collision_scene'] and not saved['motion_authorized']
    assert 'free_depth_file' not in saved
    data=np.load(tmp_path/'surface_mesh.npz');assert np.array_equal(data['faces'],mesh.faces)
    assert np.array_equal(np.load(tmp_path/'raw_depth.npz')['depth'],raw)
    with pytest.raises(ValueError):packet.raw_points[0,0]=3
    from drum_planner.planning import scene as load_scene
    # A display mesh is not silently upgraded into planner free-space evidence.
    with pytest.raises(ValueError):load_scene(tmp_path/'scene.json',common.DEFAULT_CALIBRATION)


@pytest.fixture
def gui(tmp_path,monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')
    v=snapshot.SnapshotPlaybackView(common.calibration(),tmp_path,PlanDisplay(DEMO),autoplay=True,loop=True)
    h,raw=frame();packet,mesh=snapshot.prepare_scene(h,raw,meta(),v.transform)
    v.set_scene(packet,mesh)
    try:yield v
    finally:v.close()


def test_snapshot_gui_play_layers_save_and_no_reacquisition(gui):
    assert gui.clock.playing and not gui.freeze_button.isEnabled()
    assert 'CAMERA OFF' in gui.viewport.title
    vertices=gui.mesh.vertices.copy();faces=gui.mesh.faces.copy();packet=gui.packet
    for mode in (1,2,0):
        gui.mode.setCurrentIndex(mode);gui.poll()
        assert gui.cloud.get_visible()==(mode!=0)
        assert gui.surface.get_visible()==(mode!=1)
    gui.color.setCurrentIndex(1);gui.reset_view();gui.focus_arm()
    gui.seek(2);assert gui.clock.position==2 and not gui.clock.playing
    gui.toggle_freeze();assert not gui.frozen
    with pytest.raises(RuntimeError):gui.accept(packet)
    with pytest.raises(RuntimeError):gui.set_scene(packet,gui.mesh)
    gui.request_save();assert gui.poll()==ord('s');gui.save_view()
    assert (gui.folder/'preview_view.png').stat().st_size>1000
    assert np.array_equal(gui.mesh.vertices,vertices) and np.array_equal(gui.mesh.faces,faces)
    assert gui.packet is packet
    report=gui.report()
    assert report['worker_submissions']==0 and report['rendered_updates']==1 and report['mesh_builds']==1
    assert report['first_displayed_sequence']==report['last_displayed_sequence']==1
    assert not report['camera_updates'] and not report['live_collision_revalidation']
    assert not report['motion_authorized'] and not report['hardware_execution_available']
    assert set(report['modes_rendered'])=={'surface mesh','depth points','mesh + points'}


def test_snapshot_idle_does_not_redraw_or_queue_jobs(gui):
    gui.clock.pause();count=gui.animation_draws;draws=len(gui.render_draw_seconds)
    for _ in range(10):gui.poll()
    assert gui.animation_draws==count and len(gui.render_draw_seconds)==draws
    assert gui.worker.future is None and gui.worker.submissions==0


def test_one_shot_reader_stops_at_first_frame(tmp_path,monkeypatch):
    h,raw=frame();raw=np.full((800,1280),1000,dtype='<u2')
    h.update(n=raw.nbytes,device_timestamp_us=100)
    data=json.dumps({'mock':True}).encode()+b'\n'+json.dumps(h).encode()+b'\n'+raw.tobytes()
    # Any attempt to read a second header is a regression.
    stream=io.BytesIO(data+b'NOT A SECOND FRAME\n')
    class Process:
        def __init__(self,command,**kw):
            assert command==['/mock','mock','--single-frame']
            self.stdout=stream;self.stopped=False
        def poll(self):return 0 if self.stopped else None
        def terminate(self):self.stopped=True
        def wait(self,**kw):return 0
    monkeypatch.setattr(depth,'build_depth',lambda:Path('/mock'))
    monkeypatch.setattr(depth,'check_metadata',lambda *a:None)
    monkeypatch.setattr(depth.subprocess,'Popen',Process)
    camera=DepthCamera(tmp_path,{'camera':{'serial':'mock'}},single_frame=True)
    camera.thread.join(3);camera.close()
    assert camera.error is None and camera.count==1 and camera.latest[0]['sequence']==1
    assert not camera.thread.is_alive() and camera.process.poll()==0
    assert stream.read()==b'NOT A SECOND FRAME\n'


@pytest.mark.parametrize('failure',[None,'camera','empty','release','exit'])
def test_runner_releases_before_mesh_and_no_camera_use_afterward(tmp_path,monkeypatch,failure):
    events=[];h,raw=frame()
    class Camera:
        def __init__(self,folder,cal,*,single_frame):
            assert single_frame;events.append('capture')
            self.latest=(h,raw if failure!='empty' else raw*0);self.count=1
            self.meta=meta();self.error='camera failure' if failure=='camera' else None
            self.closed=False;self.process=SimpleNamespace(pid=123,poll=lambda:(-9 if failure=='exit' else 0) if self.closed else None)
            self.thread=SimpleNamespace(is_alive=lambda:failure=='release')
        def close(self):self.closed=True;events.append('release')
    class Viewer:
        def __init__(self,*a):self.packet=None;self.closed=False
        def poll(self):
            assert 'release' in events
            events.append('poll');return ord('q')
        def set_scene(self,packet,mesh):
            assert 'release' in events
            events.append('mesh');self.packet=packet
        def close(self):self.closed=True
        def report(self):return {}
    monkeypatch.setattr(snapshot,'DepthCamera',Camera)
    folder=tmp_path/'run'
    if failure:
        with pytest.raises((RuntimeError,ValueError)):snapshot.run(folder,None,viewer_factory=Viewer)
    else:
        assert snapshot.run(folder,None,viewer_factory=Viewer)==folder/'scene.json'
        assert events.index('release')<events.index('mesh')<events.index('poll')
    report=json.loads((folder/'capture_report.json').read_text())
    assert report['passed']==(failure is None) and report['frames']==1
    assert not report['can_access'] and report['motor_commands']==0
    assert 'release' in events
