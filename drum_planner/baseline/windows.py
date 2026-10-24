"""Three small, separate windows. Rendering/preview code has no device API."""
import copy,time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PyQt5 import QtCore,QtGui,QtWidgets
from safe_zone.geometry import Model
from ..common import JOINTS,LEFT_JOINTS,arm_side,arm_joints
from .arms import other_side
from ..model import generate
from ..trajectory_view import PlaybackClock
from ..vtk_view import VtkViewport,RobotActors
from .paths import speed


class DisplayModel:
    """Calibrated visuals before Calculate; no fabricated trajectory file."""
    def __init__(self,folder,cal_path,*,moving_side='right'):
        self.side=arm_side(moving_side);self.other_side=other_side(self.side)
        self.joints=arm_joints(self.side);self.stick_link=self.side+'_drumstick'
        manifest=generate(folder,cal_path,moving_side=self.side)
        self.model=Model(Path(folder)/'robot.urdf');self.root=ET.parse(Path(folder)/'robot.urdf').getroot()
        self.obj={'model':manifest};self.duration=0.
    def positions(self,t):return np.zeros(7)
    def frames(self,t):return self.model.transforms({})


class ArmWindow(QtWidgets.QWidget):
    def __init__(self,title,model):
        super().__init__();self.setWindowTitle(title);self.resize(880,680)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose,False)
        self.layout_=QtWidgets.QVBoxLayout(self);self.layout_.setContentsMargins(12,12,12,12)
        self.heading=QtWidgets.QLabel(title);self.heading.setStyleSheet('font-size:17px;font-weight:600')
        self.layout_.addWidget(self.heading)
        self.viewport=VtkViewport(self);self.layout_.addWidget(self.viewport,1)
        self.robot=RobotActors(self.viewport,model);self.robot.set_visible(False)
        self.viewport.fit(self.robot.focus_bounds(),elev=25,azim=-55)
        self.caption=QtWidgets.QLabel('Waiting for camera snapshot and encoders…');self.caption.setWordWrap(True)
        self.caption.setMaximumHeight(85)
        self.layout_.addWidget(self.caption)
        row=QtWidgets.QHBoxLayout();row.addWidget(QtWidgets.QLabel('Drag: rotate · middle drag: pan · wheel: zoom'),1)
        self.fit=QtWidgets.QPushButton('Fit view',self);self.fit.clicked.connect(self.fit_view);row.addWidget(self.fit)
        self.layout_.addLayout(row)
        self.pose=None;self.dirty=True;self.last_draw=0.;self.scene=None
    def fit_view(self):
        self.viewport.fit(self.robot.focus_bounds(),elev=25,azim=-55);self.dirty=True
    def set_scene(self,packet,mesh):
        self.scene=(packet,mesh)
        self.viewport.set_color_range(.15,5.,'Camera depth (m)')
        self.viewport.cloud.set_visible(False)
        self.viewport.set_surface(mesh,mesh.camera_depth_m,False,visible=True);self.dirty=True
    def set_pose(self,pose):
        self.pose=pose;self.robot.draw_joints(pose['joints']);self.robot.set_visible(True);self.dirty=True
    def draw(self,now=None):
        now=time.monotonic() if now is None else now
        if self.isVisible() and self.dirty and now-self.last_draw>=.05:
            self.viewport.render();self.last_draw=now;self.dirty=False
    def shutdown(self):self.viewport.shutdown();self.hide();self.deleteLater()


class PlanWindow(ArmWindow):
    def __init__(self,model,rate,on_speed=lambda *_:None):
        super().__init__(f'X5 · {model.side.capitalize()} planned trajectory — disconnected preview',model)
        self.plan=None;self.source_outward=None;self.clock=PlaybackClock(0.);self.clock.loop=False
        self.notice=lambda message:None;self.on_speed=on_speed
        self.caption.setText('No trajectory yet. Click Calculate trajectory in the control panel.')
        self.viewport.set_title('PLANNED PREVIEW — does not follow the physical arm')
        controls=QtWidgets.QHBoxLayout()
        self.play=QtWidgets.QPushButton('Play preview',self);self.play.clicked.connect(self.play_preview)
        self.pause=QtWidgets.QPushButton('Pause',self);self.pause.clicked.connect(self.pause_preview)
        controls.addWidget(self.play);controls.addWidget(self.pause)
        controls.addWidget(QtWidgets.QLabel('Speed (preview / next real run):'))
        self.rate=QtWidgets.QDoubleSpinBox(self);self.rate.setRange(.25,1.);self.rate.setSingleStep(.05)
        self.rate.setValue(speed(rate));self.rate.setSuffix(' ×');self.rate.valueChanged.connect(self.rate_changed)
        controls.addWidget(self.rate);controls.addStretch(1);self.layout_.addLayout(controls)
        self.timeline=QtWidgets.QSlider(QtCore.Qt.Horizontal,self);self.timeline.setRange(0,1000)
        self.timeline.valueChanged.connect(lambda n:self.seek(n/1000));self.layout_.addWidget(self.timeline)
        self.time_label=QtWidgets.QLabel('No calculated trajectory');self.layout_.addWidget(self.time_label)
    def install_plan(self,plan):
        # Separate object/clock/model state from live feedback and owner input.
        self.plan=copy.copy(plan);self.source_outward=copy.deepcopy(plan.obj['outward'])
        self.robot.plan=self.plan;self.rate_changed(self.rate.value())
    def rate_changed(self,value):
        if self.plan is not None and self.plan.obj.get('checkpoint_sequence'):
            self.plan.set_preview_rate(value)
            self.clock.pause();self.clock.duration=self.plan.duration;self.clock.seek(0)
            self.timeline.setRange(0,max(1,round(self.plan.duration*1000)));self.draw_plan()
        elif self.plan is not None:
            from ..planning import cycle
            outward=copy.deepcopy(self.source_outward)
            for p in outward:
                p['time_s']/=value
                for key,power in [('velocities_rad_s',1),('accelerations_rad_s2',2)]:
                    if key in p:p[key]=(np.asarray(p[key])*value**power).tolist()
            self.plan.obj=dict(self.plan.obj,outward=outward,cycle=cycle(outward))
            # Scale the analytic clock too; changing only preview samples would
            # leave a retimed curve playing at 1x and mis-time its goal/return.
            self.plan.preview_rate=value
            self.plan.obj['duration_outward_s']=outward[-1]['time_s']
            self.plan.curves=self.plan.obj['cycle'];self.plan.times=np.array([p['time_s'] for p in self.plan.curves])
            self.plan.values=np.array([p['positions_rad'] for p in self.plan.curves]);self.plan.duration=float(self.plan.times[-1])
            self.clock.pause();self.clock.duration=self.plan.duration;self.clock.seek(0)
            self.timeline.setRange(0,max(1,round(self.plan.duration*1000)));self.draw_plan()
        self.on_speed(value)
    def play_preview(self):
        if self.plan is None:self.notice('No calculated trajectory. Click Calculate trajectory first.');return
        self.clock.play();self.draw_plan()
    def pause_preview(self):self.clock.pause();self.draw_plan()
    def seek(self,t):
        if self.plan is None:self.notice('No calculated trajectory. Click Calculate trajectory first.');return
        self.clock.pause();self.clock.seek(t);self.draw_plan()
    def draw_plan(self):
        if self.plan is None:return
        t=self.clock.sample();self.robot.draw(t);self.robot.set_visible(True)
        self.viewport.set_title('DISCONNECTED PREVIEW — '+self.plan.phase(t))
        self.time_label.setText(f'{t:.2f} / {self.plan.duration:.2f} seconds')
        self.timeline.blockSignals(True);self.timeline.setValue(round(t*1000));self.timeline.blockSignals(False)
        self.dirty=True
    def tick(self,now):
        if self.clock.playing and now-self.last_draw>=.05:self.draw_plan()
        self.draw(now)


class ControlWindow(QtWidgets.QWidget):
    def __init__(self,owner):
        super().__init__();self.setWindowTitle(f'X5 · Hardware · Workspace curve — {owner.side.upper()} arm controls');self.resize(510,440)
        self.owner=owner;layout=QtWidgets.QVBoxLayout(self);layout.setContentsMargins(18,18,18,18);layout.setSpacing(6)
        self.menu=QtWidgets.QMenuBar(self);windows=self.menu.addMenu('Windows')
        for name,window in [('Planned preview',owner.preview),('Live arm',owner.live)]:
            action=windows.addAction(name);action.triggered.connect(lambda _,w=window:(w.show(),w.raise_()))
        layout.setMenuBar(self.menu)
        title=QtWidgets.QLabel(f'Workspace curve · {owner.side.upper()}');title.setStyleSheet('font-size:22px;font-weight:600');layout.addWidget(title)
        self.status=QtWidgets.QLabel('Click Calculate trajectory to begin.');self.status.setWordWrap(True)
        self.status.setMinimumHeight(40);self.status.setMaximumHeight(75);layout.addWidget(self.status)
        from .destinations import Selector
        self.destinations=Selector(owner);layout.addWidget(self.destinations)
        self.buttons={}
        for name,label,callback in [('calculate','1 · Calculate trajectory',owner.calculate),
            ('start',f'2 · START trajectory on {owner.side} arm',owner.start),('hold','EMERGENCY HOLD  [Space]',owner.hold),
            ('reverse','Reverse to rest + relax',owner.reverse),('cancel','Cancel calculation / pending start',owner.cancel_calculation)]:
            button=QtWidgets.QPushButton(label,self);button.setMinimumHeight(40)
            button.clicked.connect(lambda _,f=callback:owner.action(f));layout.addWidget(button);self.buttons[name]=button
        self.buttons['hold'].setStyleSheet('background:#b22222;color:white;font-weight:bold')
        self.run_status=QtWidgets.QLabel('Arm: not commanded by this program');self.run_status.setWordWrap(True)
        self.run_status.setMaximumHeight(85);layout.addWidget(self.run_status)
        note=QtWidgets.QLabel(f'Calculate is read-only. Keep the {owner.side} arm still afterward.\nPassive {owner.other} sway is allowed. Keep obstacles as captured.\nHold keeps torque; Reverse returns to start before relaxing.')
        note.setWordWrap(True);layout.addWidget(note)
        self.log=QtWidgets.QPlainTextEdit(self);self.log.setReadOnly(True);self.log.setMaximumBlockCount(120)
        self.log.setMaximumHeight(100);layout.addWidget(self.log)
    def closeEvent(self,event):
        self.owner.request_close();event.ignore()
