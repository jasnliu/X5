"""Saved trajectory animation over live Gemini depth; never robot execution."""
import time

import numpy as np

from .live_view import LiveDepthView
from .trajectory_view import PlaybackClock, RobotArtists


class LivePlaybackView(LiveDepthView):
    def __init__(self,cal,folder,plan,autoplay=False,loop=False):
        self.plan=plan;self.clock=PlaybackClock(plan.duration);self.clock.loop=loop
        self.robot=None;self.autoplay_pending=autoplay;self.animation_draws=0
        self.last_animation_draw=0.;self.play_clicks=0;self.seeks=0;self.phases_seen=set()
        super().__init__(cal,folder)
        Qt=self.QtWidgets
        self.window.setWindowTitle('X5 trajectory + live depth — SIMULATION ONLY / no motors')
        self.window.resize(1440,900)
        self.banner.setText('SIMULATED TRAJECTORY + LIVE GEMINI DEPTH  •  NO CAN / NO MOTOR COMMANDS')
        self.info.setText('Black = no depth data. 2D colors = depth (0.15–5 m).\n'
                          'Solid robot + tan stick = animation, NOT actual joint feedback.\n'
                          'Freeze depth and Pause animation are independent.\n'
                          'Save always saves fresh live depth, never modifies the plan.')
        row=Qt.QHBoxLayout()
        self.play_button=Qt.QPushButton('Play animation [P]');self.play_button.setEnabled(False)
        self.play_button.clicked.connect(self.toggle_play);row.addWidget(self.play_button)
        restart=Qt.QPushButton('Go to start');restart.clicked.connect(lambda:self.seek(0));row.addWidget(restart)
        goal=Qt.QPushButton('Go to goal');goal.setEnabled(plan.playable)
        goal.clicked.connect(lambda:self.seek(float(plan.obj['outward'][-1]['time_s'])));row.addWidget(goal)
        self.timeline=Qt.QSlider(self.QtCore.Qt.Horizontal)
        self.timeline.setRange(0,max(1,int(round(plan.duration*1000))))
        self.timeline.setEnabled(plan.playable);self.timeline.valueChanged.connect(lambda value:self.seek(value/1000))
        row.addWidget(self.timeline,1)
        self.time_label=Qt.QLabel();self.time_label.setMinimumWidth(185);row.addWidget(self.time_label)
        row.addWidget(Qt.QLabel('Speed:'));self.speed=Qt.QComboBox();self.speed.addItems(['0.5×','1×','2×'])
        self.speed.setCurrentIndex(1);self.speed.currentIndexChanged.connect(self.change_speed);row.addWidget(self.speed)
        self.loop_box=Qt.QCheckBox('Loop');self.loop_box.setChecked(loop)
        self.loop_box.toggled.connect(lambda checked:setattr(self.clock,'loop',checked));row.addWidget(self.loop_box)
        focus=Qt.QPushButton('Fit arm + kit');focus.clicked.connect(self.focus_arm);row.addWidget(focus)
        self.window.layout().insertLayout(2,row)
        self.plan_label=Qt.QLabel(plan.warning+' • LIVE DEPTH DOES NOT REVALIDATE THIS PATH')
        self.plan_label.setStyleSheet('color:#9b2922; font-weight:bold;');self.plan_label.setWordWrap(True)
        self.window.layout().insertWidget(3,self.plan_label)
        shortcut=Qt.QShortcut(self.QtGui.QKeySequence('P'),self.window)
        shortcut.activated.connect(self.toggle_play);self.shortcuts.append(shortcut)
        self.note.setText('DISPLAY ONLY — saved trajectory + current depth, not online collision checking. '
                          'Missing/hidden space is unknown; tool mounting and depth alignment remain provisional. '
                          'View clipping/sampling does not alter the planner data.')
        self.robot=RobotArtists(self.ax,plan)
        self.focus_box=self.robot.focus_bounds()
        self.focus_arm()
        self.freeze_button.setText('Freeze depth [Space]')
        self.canvas.mpl_connect('button_release_event',lambda event:self.refresh())
        self.app.processEvents()

    def display_points(self,points):
        # Matplotlib 3.6 does not clip 3D points to axis bounds automatically.
        # This ONLY clips the displayed point sample, never camera/scene data.
        if self.fit_needed:return points
        limits=np.array([self.ax.get_xlim3d(),self.ax.get_ylim3d(),self.ax.get_zlim3d()])
        return points[np.all((points>=limits[:,0])&(points<=limits[:,1]),axis=1)]

    def focus_arm(self):
        if self.robot is None:return
        self.fit_needed=False
        for i,setter in enumerate((self.ax.set_xlim3d,self.ax.set_ylim3d,self.ax.set_zlim3d)):
            setter(*self.focus_box[:,i])
        self.ax.set_box_aspect(self.focus_box[1]-self.focus_box[0]);self.ax.view_init(elev=25,azim=-55)
        if self.packet is not None:self.refresh()
        else:self.draw_canvas()

    def reset_view(self):
        self.ax.set_box_aspect((1,1,1))
        super().reset_view()

    def zoom(self,event):
        super().zoom(event)
        if event.inaxes is self.ax:self.refresh()

    def toggle_freeze(self):
        super().toggle_freeze()
        self.freeze_button.setText('Resume depth [Space]' if self.frozen else 'Freeze depth [Space]')

    def toggle_play(self):
        if not self.plan.playable or self.packet is None:return
        self.play_clicks+=1
        if self.clock.playing:self.clock.pause()
        else:self.clock.play()
        self.draw_canvas()

    def seek(self,t):
        self.clock.pause();self.clock.seek(t);self.seeks+=1;self.draw_canvas()

    def change_speed(self,index):
        self.clock.set_rate([.5,1.,2.][index])

    def accept(self,packet):
        accepted=super().accept(packet)
        if accepted:
            self.play_button.setEnabled(self.plan.playable)
            if self.autoplay_pending:
                self.autoplay_pending=False
                if self.plan.playable:self.clock.play()
        return accepted

    def draw_canvas(self):
        if self.robot is not None:
            t=self.clock.sample();self.robot.draw(t);phase=self.plan.phase(t)
            self.phases_seen.add(phase);self.animation_draws+=1
            self.time_label.setText(f'{t:.2f} / {self.plan.duration:.2f} s • {phase}')
            self.timeline.blockSignals(True);self.timeline.setValue(int(round(t*1000)));self.timeline.blockSignals(False)
            self.play_button.setText('Pause animation [P]' if self.clock.playing else 'Play animation [P]')
            source=self.depth_label()
            layer=self.layer_label()
            self.ax.set_title(f'SIMULATED ARM — {phase} — t={t:.2f}s\n{source} / {layer}',fontsize=11)
            self.detail.setText(self.scene_detail())
        super().draw_canvas();self.last_animation_draw=time.monotonic()

    def depth_label(self):
        if self.packet is None:return 'waiting for camera'
        return ('FROZEN' if self.frozen else 'LIVE')+f' depth frame {self.packet.sequence:,}'

    def layer_label(self):
        return '10 mm voxel centers' if self.mode.currentIndex()==1 else 'raw points'

    def scene_detail(self):
        return ('Current camera surfaces + saved-plan robot animation. '
                'No measured joint pose, reconstructed hidden surfaces, or live collision verdict.')

    def poll(self,camera):
        key=super().poll(camera)
        if self.clock.playing and not self.close_requested and time.monotonic()-self.last_animation_draw>=.05:
            self.draw_canvas()
        return key

    def report(self):
        report=super().report()
        report.update(type='live_depth_trajectory_preview',robot_overlay=True,robot_overlay_source='saved plan simulation, not encoders',
                      plan_file=str(self.plan.path),plan_warning=self.plan.warning,animation_draws=self.animation_draws,
                      animation_time_s=self.clock.position,animation_completed_cycles=self.clock.completed_cycles,
                      animation_play_clicks=self.play_clicks,animation_seeks=self.seeks,animation_phases_seen=sorted(self.phases_seen),
                      live_collision_revalidation=False,motion_authorized=False,hardware_execution_available=False)
        return report

    def close(self):
        if not self.closed:self.clock.pause()
        super().close()
