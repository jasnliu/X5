"""Actual-camera preview integration check: only simulated robot and GUI actions."""
import json,socket,sys,time
from pathlib import Path
from PyQt5 import QtCore,QtTest
from matplotlib.backend_bases import MouseEvent
import numpy as np
from drum_planner import cli,live_playback

out=Path(sys.argv[1]);original_socket=socket.socket;can_attempts=[]
def guarded(family=socket.AF_INET,*a,**kw):
    if family==socket.AF_CAN:
        can_attempts.append(time.monotonic());raise AssertionError('CAN prohibited during preview testing')
    return original_socket(family,*a,**kw)
socket.socket=guarded
Original=live_playback.LivePlaybackView
class Exercised(Original):
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw);self.step=0;self.base=None;self.events=[]
    def record(self,name,**kw):
        self.events.append(dict(name=name,elapsed=time.monotonic()-self.base,**kw))
    def click(self,button):QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton)
    def poll(self,camera):
        key=super().poll(camera)
        if self.packet is None:return key
        if self.base is None:self.base=time.monotonic()
        t=time.monotonic()-self.base
        if self.step==0 and t>=4:
            assert not self.clock.playing and self.clock.position==0
            self.window.grab().save(str(out/'paused_start_live_depth.png'))
            self.click(self.play_button);assert self.clock.playing
            self.record('clicked_play');self.step=1
        elif self.step==1 and t>=8:
            self.click(self.freeze_button);assert self.frozen and self.clock.playing
            self.held=(self.packet.sequence,camera.count,self.clock.position)
            self.record('freeze_depth_only',depth_sequence=self.packet.sequence,animation_time=self.clock.position);self.step=2
        elif self.step==2 and t>=12:
            assert self.packet.sequence==self.held[0] and camera.count>self.held[1]+20
            assert abs(self.clock.position-self.held[2])>2
            self.record('animation_continued_while_depth_frozen',depth_sequence=self.packet.sequence,acquired=camera.count,animation_time=self.clock.position)
            self.click(self.play_button);assert not self.clock.playing
            self.click(self.freeze_button);assert not self.frozen
            self.paused_at=self.clock.position;self.step=3
        elif self.step==3 and t>=17:
            assert self.clock.position==self.paused_at and self.packet.sequence>self.held[0]+30
            self.record('depth_continued_while_animation_paused',depth_sequence=self.packet.sequence,animation_time=self.clock.position)
            goal=next(b for b in self.window.findChildren(self.QtWidgets.QPushButton) if b.text()=='Go to goal')
            self.click(goal);assert np.array_equal(self.robot.last_positions,self.plan.req['goal'])
            self.record('exact_goal_scrub');self.step=4
        elif self.step==4 and t>=21:
            self.mode.setCurrentIndex(1);self.color.setCurrentIndex(1)
            before=(self.ax.elev,self.ax.azim)
            box=self.ax.bbox;x,y=box.x0+box.width*.5,box.y0+box.height*.5
            for event,xx,yy in [('button_press_event',x,y),('motion_notify_event',x+25,y+15),('button_release_event',x+25,y+15)]:
                self.canvas.callbacks.process(event,MouseEvent(event,self.canvas,xx,yy,button=1))
            assert (self.ax.elev,self.ax.azim)!=before
            self.window.grab().save(str(out/'goal_live_voxels_height.png'))
            self.record('voxel_height_and_rotate');self.step=5
        elif self.step==5 and t>=25:
            self.request_save();self.record('save_live_scene');self.step=6
        elif self.step==6 and t>=28:
            assert (out/'scene.json').exists()
            self.focus_arm();self.color.setCurrentIndex(0);self.mode.setCurrentIndex(0)
            self.speed.setCurrentIndex(2);assert self.clock.rate==2
            self.loop_box.setChecked(True);self.seek(0);self.click(self.play_button)
            self.record('loop_and_2x_animation');self.step=7
        elif self.step==7 and t>=40:
            assert self.clock.completed_cycles>=1
            self.speed.setCurrentIndex(1);self.mode.setCurrentIndex(1)
            self.window.grab().save(str(out/'looped_animation_live_voxels.png'))
            self.record('completed_animation_cycle',cycles=self.clock.completed_cycles);self.step=8
        elif self.step==8 and t>=55:
            assert self.rendered>100 and self.animation_draws>100
            self.record('continuous_live_updates',depth_updates=self.rendered,animation_draws=self.animation_draws)
            self.step=9
        return key
    def close(self):
        if not self.closed:
            (out/'interaction_report.json').write_text(json.dumps(dict(completed_all_steps=self.step==9,events=self.events,can_attempts=can_attempts),indent=2))
        super().close()
live_playback.LivePlaybackView=Exercised
raise SystemExit(cli.main(['preview','planner_runs/software_demo/plan.json','--seconds','65','--output',str(out)]))
