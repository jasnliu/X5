import hashlib
import json
from pathlib import Path
import socket
import sys
import time

ROOT=Path('/home/jason/Proyectos3/X5');sys.path.insert(0,str(ROOT))
from drum_planner import snapshot_preview as preview
from drum_planner.common import atomic_json
from drum_planner.trajectory_view import PlanDisplay
from PyQt5 import QtCore,QtTest

folder=Path(__file__).parent/'live_controls'
checks={};attempts=[];real_socket=socket.socket

def guarded(family=socket.AF_INET,*args,**kwargs):
    if family==socket.AF_CAN:
        attempts.append(str(args));raise AssertionError('No CAN access allowed')
    return real_socket(family,*args,**kwargs)
socket.socket=guarded

real_camera=preview.DepthCamera;camera_holder=[]
def camera(*a,**kw):
    c=real_camera(*a,**kw);camera_holder.append(c);return c
preview.DepthCamera=camera

def fingerprint(view):
    digest=hashlib.sha256()
    for array in (view.packet.raw_points,view.packet.raw_image,view.mesh.vertices,view.mesh.faces):
        digest.update(array.tobytes())
    return digest.hexdigest()

class View(preview.SnapshotPlaybackView):
    def set_scene(self,packet,mesh):
        c=camera_holder[0]
        assert c.count==1 and not c.thread.is_alive() and c.process.poll()==0
        checks['camera_cleanly_exited_before_display']=True
        super().set_scene(packet,mesh)
        self.started=time.monotonic();self.steps=set();self.original=fingerprint(self)
        self.clock.pause();self.idle_draws=self.animation_draws
        self.initial_files={name:hashlib.sha256((folder/name).read_bytes()).hexdigest()
                            for name in ('scene.json','surface_mesh.npz','raw_depth.npz')}
        self.window.grab().save(str(folder/'initial_surface.png'))
        self.window.activateWindow();self.window.setFocus();self.app.processEvents()
    def do(self,after,name,callback):
        if time.monotonic()-self.started>=after and name not in self.steps:
            callback();self.steps.add(name)
    def poll(self,camera=None):
        if self.packet is not None:
            c=camera_holder[0]
            assert c.count==1 and not c.thread.is_alive() and c.process.poll()==0
            assert self.worker.submissions==0 and self.rendered==1 and self.mesh_builds==1
            self.do(5,'idle',self.idle_check)
            self.do(13,'points',lambda:self.mode.setCurrentIndex(1))
            self.do(14,'both',self.both)
            self.do(15,'surface',lambda:self.mode.setCurrentIndex(0))
            self.do(16,'height',lambda:self.color.setCurrentIndex(1))
            self.do(17,'full_view',self.full_view)
            self.do(19,'focus',self.focus_arm)
            self.do(21,'rotate_zoom',self.rotate_zoom)
            self.do(24,'seek',self.scrub)
            self.do(25,'resume',self.toggle_play)
            self.do(26,'speed',lambda:self.speed.setCurrentIndex(2))
            self.do(30,'save',self.request_save)
            self.do(35,'space',self.space)
            self.do(60,'frozen',self.frozen_check)
        return super().poll()
    def idle_check(self):
        assert self.animation_draws==self.idle_draws
        checks['five_seconds_idle_no_animation_redraws']=True
        QtTest.QTest.keyClick(self.window,QtCore.Qt.Key_P);self.app.processEvents()
        assert self.clock.playing;checks['play_keyboard']=True
    def both(self):
        self.mode.setCurrentIndex(2)
        self.window.grab().save(str(folder/'mesh_and_points.png'))
    def full_view(self):
        self.reset_view();self.window.grab().save(str(folder/'full_scene_height.png'))
    def rotate_zoom(self):
        from types import SimpleNamespace
        self.ax.view_init(elev=20,azim=-40)
        self.zoom(SimpleNamespace(inaxes=self.ax,button='up'))
        assert self.ax.elev==20 and self.ax.azim==-40
        checks['rotate_zoom']=True
    def scrub(self):
        self.timeline.setValue(2000)
        assert not self.clock.playing and self.clock.position==2.
        checks['timeline_pauses_and_seeks']=True
    def space(self):
        QtTest.QTest.keyClick(self.window,QtCore.Qt.Key_Space);self.app.processEvents()
        assert not self.frozen and not self.freeze_button.isEnabled()
        checks['space_cannot_resume_capture']=True
    def frozen_check(self):
        assert fingerprint(self)==self.original
        assert self.initial_files=={name:hashlib.sha256((folder/name).read_bytes()).hexdigest()
                                   for name in self.initial_files}
        assert (folder/'preview_view.png').stat().st_size>1000
        checks['scene_arrays_and_saved_files_unchanged_after_60_seconds']=True
        assert self.clock.completed_cycles>=2
        checks['multiple_trajectory_cycles']=True
        self.window.grab().save(str(folder/'final_view.png'))

try:
    preview.run(folder,PlanDisplay(ROOT/'planner_runs/software_demo/plan.json'),
                seconds=65,autoplay=True,loop=True,viewer_factory=View)
    assert len(checks)==8 and all(checks.values()) and not attempts
    atomic_json(folder/'control_checks.json',dict(passed=True,checks=checks,can_attempts=attempts))
except BaseException as exc:
    folder.mkdir(exist_ok=True)
    atomic_json(folder/'control_checks.json',dict(passed=False,error=str(exc),checks=checks,can_attempts=attempts))
    raise
