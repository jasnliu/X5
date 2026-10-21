"""Exercise production capture/Qt controls with the actual camera; forbid CAN."""
import json, socket, sys, time
from pathlib import Path
from PyQt5 import QtCore, QtTest
from matplotlib.backend_bases import MouseEvent
from drum_planner import live_view, cli

out=Path(sys.argv[1]);real_socket=socket.socket;can_attempts=[]
def guarded(family=socket.AF_INET,*args,**kw):
    if family==socket.AF_CAN:
        can_attempts.append(time.monotonic());raise AssertionError('CAN forbidden in camera-only UI test')
    return real_socket(family,*args,**kw)
socket.socket=guarded
Original=live_view.LiveDepthView
class ExercisedView(Original):
    def __init__(self,*args):
        super().__init__(*args);self.events=[];self.step=0;self.base_time=None;self.held=None
    def evidence(self,name,**kw):
        self.events.append(dict(name=name,elapsed=time.monotonic()-self.base_time,**kw))
    def drag(self,button,dx,dy):
        box=self.ax.bbox;x,y=box.x0+box.width*.5,box.y0+box.height*.5
        before=(self.ax.elev,self.ax.azim,*self.ax.get_xlim3d(),*self.ax.get_ylim3d())
        for event,xx,yy in [('button_press_event',x,y),('motion_notify_event',x+dx,y+dy),('button_release_event',x+dx,y+dy)]:
            self.canvas.callbacks.process(event,MouseEvent(event,self.canvas,xx,yy,button=button))
        self.canvas.draw()
        after=(self.ax.elev,self.ax.azim,*self.ax.get_xlim3d(),*self.ax.get_ylim3d())
        assert before!=after,(button,before,after)
    def poll(self,camera):
        key=super().poll(camera)
        if self.packet is None:return key
        if self.base_time is None:self.base_time=time.monotonic()
        elapsed=time.monotonic()-self.base_time
        if self.step==0 and elapsed>=5:
            self.window.grab().save(str(out/'raw_live.png'))
            self.mode.setCurrentIndex(1);self.evidence('voxel_layer');self.step=1
        elif self.step==1 and elapsed>=10:
            QtTest.QTest.mouseClick(self.freeze_button,QtCore.Qt.LeftButton)
            assert self.frozen
            self.held=(self.packet.sequence,camera.count,self.rendered)
            self.evidence('freeze',sequence=self.packet.sequence,acquired=camera.count);self.step=2
        elif self.step==2 and elapsed>=14:
            assert self.packet.sequence==self.held[0] and self.rendered==self.held[2]
            assert camera.count>=self.held[1]+20
            self.color.setCurrentIndex(1);self.drag(1,45,30);self.drag(2,30,-20)
            before=self.ax.get_xlim3d()
            x=self.ax.bbox.x0+self.ax.bbox.width*.5;y=self.ax.bbox.y0+self.ax.bbox.height*.5
            event=MouseEvent('scroll_event',self.canvas,x,y,button='up',step=1)
            self.canvas.callbacks.process('scroll_event',event);self.canvas.draw()
            assert self.ax.get_xlim3d()!=before
            self.window.grab().save(str(out/'frozen_voxels_height.png'))
            self.evidence('freeze_kept_view_acquisition_continued',acquired=camera.count,sequence=self.packet.sequence)
            self.evidence('rotate_pan_scroll_and_height');self.step=3
        elif self.step==3 and elapsed>=17:
            self.request_save();self.evidence('save_live_while_frozen');self.step=4
        elif self.step==4 and elapsed>=20:
            assert (out/'scene.json').exists()
            sc=json.loads((out/'scene.json').read_text());assert sc['frames'][-1]['sequence']>self.held[0]
            self.evidence('saved_newer_live_frames',saved=sc['frames'][-1]['sequence'],frozen=self.held[0])
            QtTest.QTest.mouseClick(self.freeze_button,QtCore.Qt.LeftButton);assert not self.frozen
            self.evidence('resume');self.step=5
        elif self.step==5 and elapsed>=25:
            assert self.packet.sequence>self.held[0]+50
            assert camera.latest[0]['sequence']-self.packet.sequence<20
            self.evidence('resume_caught_up',displayed=self.packet.sequence,latest=camera.latest[0]['sequence'])
            self.reset_view();self.mode.setCurrentIndex(0);self.color.setCurrentIndex(0);self.step=6
        elif self.step==6 and elapsed>=35:
            self.mode.setCurrentIndex(1);self.evidence('live_voxels_again');self.step=7
        elif self.step==7 and elapsed>=45:
            self.window.grab().save(str(out/'live_voxels.png'));self.mode.setCurrentIndex(0)
            self.evidence('raw_again');self.step=8
        return key
    def close(self):
        if not self.closed:
            (out/'interaction_report.json').write_text(json.dumps(dict(completed_all_steps=self.step==8,events=self.events,can_attempts=can_attempts),indent=2))
        super().close()
live_view.LiveDepthView=ExercisedView
raise SystemExit(cli.main(['camera','--seconds','65','--output',str(out)]))
