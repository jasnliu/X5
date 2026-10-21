"""Legacy single-window diagnostics (not the baseline CLI).

The user-facing baseline entry point is now workspace.py's three-window UI.
This module is retained for old diagnostic scripts and backend regression tests.

Planning and hardware ownership run outside the GUI/editor process. A renderer
exception, lost UI heartbeat, close, or Ctrl-C never authorizes motor relaxation.
"""
import copy,json,signal,time,uuid
from pathlib import Path
import numpy as np
from ..common import DEFAULT_CALIBRATION,atomic_json,calibration
from ..experiment.runtime import Job
from ..planned_view import load
from ..snapshot_preview import SnapshotPlaybackView
from ..trajectory_view import PlanDisplay
from .paths import speed
from . import service


def job(action,folder,**kwargs):
    cmd=['/usr/bin/python3','-m','drum_planner.baseline.tasks',action,'--folder',str(folder)]
    cmd.extend(v for k,x in kwargs.items() if x is not None for v in ('--'+k,str(x)))
    return Job(cmd,folder,timeout=300 if action in ('plan','prepare') else 180,allow_can=action=='capture')


def failure(j):
    path=j.folder/'error.json'
    return json.loads(path.read_text())['error'] if path.exists() else 'Worker failed; see '+str(j.log_path)


def wait(j):
    try:
        while j.poll() is None:time.sleep(.05)
        if j.process.returncode:raise ValueError(failure(j))
    finally:j.close()


def initial(folder,cal_path):
    from ..bus_ownership import check_available
    check_available()
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    print('BASELINE HARDWARE MODE: startup only reads disabled encoders and the camera. It NEVER enables/moves the arm.',flush=True)
    capture=folder/'capture';wait(job('capture',capture,calibration=cal_path))
    plan=folder/'plan';wait(job('plan',plan,capture=capture,calibration=cal_path))
    return plan/'result/plan.json'


class View(SnapshotPlaybackView):
    def __init__(self,plan_path,folder,cal_path=DEFAULT_CALIBRATION,*,rate=1.,simulated=False,resume=None):
        self.session=Path(folder);self.cal_path=Path(cal_path);self.simulated=simulated
        self.client=None;self.client_info=resume;self.hardware_status={};self.job=None;self.stage=None
        self.prepared=None;self.pending_start=False;self.start_sent=False;self.prepare_folder=None;self.start_when_ready=False
        self.locked=False;self.last_error='';self.reverse_error='';self.connection_failed=False;self.requested_rate=speed(rate)
        self.telemetry=None;self.display_pose=None;self.display_error='';self.next_telemetry=0.;self.feedback_draws=0
        self.live_display=not simulated;self.live_box=None
        plan=PlanDisplay(plan_path);plan.check_live_calibration(cal_path)
        self.source_outward=copy.deepcopy(plan.obj.get('outward',[]))
        super().__init__(calibration(cal_path),self.session,plan,autoplay=False,loop=False)
        Qt=self.QtWidgets;Core=self.QtCore
        self.window.setWindowTitle('X5 BASELINE — '+('SIMULATED hardware controls / NO CAN' if simulated else 'supervised RIGHT ARM'))
        self.banner.setText('BASELINE ONLY • '+('MOCK MOTORS — NO CAN / NO PHYSICAL MOTION' if simulated else 'START REAL ARM authorizes motion after automatic checks'))
        self.speed.hide();self.loop_box.hide();self.clock.loop=False
        for label in self.window.findChildren(Qt.QLabel):
            if label.text()=='Speed:':label.hide()
        controls=Qt.QHBoxLayout()
        self.rate_box=Qt.QDoubleSpinBox();self.rate_box.setRange(.25,1.);self.rate_box.setSingleStep(.05);self.rate_box.setDecimals(2)
        self.rate_box.setValue(rate);self.rate_box.setSuffix(' ×');self.rate_box.valueChanged.connect(self.rate_changed)
        self.rate_box.setFixedWidth(85)
        controls.addWidget(Qt.QLabel('Movement speed:'));controls.addWidget(self.rate_box)
        # Never show a parentless control: setVisible(True) creates a native
        # top-level window before the layout can adopt it (and leaves ghost
        # button-sized windows/overpainting on X11).
        self.prepare_button=Qt.QPushButton('Check readiness (no motion)',self.window);self.prepare_button.clicked.connect(self.prepare_clicked)
        controls.addWidget(self.prepare_button)
        self.execute_button=Qt.QPushButton('Start SIMULATED cycle' if simulated else 'START REAL ARM')
        self.execute_button.setEnabled(plan.playable);self.execute_button.clicked.connect(self.execute_clicked)
        controls.addWidget(self.execute_button)
        self.hold_button=Qt.QPushButton('EMERGENCY HOLD [Space]');self.hold_button.setStyleSheet('background:#b22222;color:white;font-weight:bold;padding:10px;')
        self.hold_button.clicked.connect(self.emergency_hold);controls.addWidget(self.hold_button)
        self.reverse_button=Qt.QPushButton('Reverse to rest + relax')
        self.reverse_button.setToolTip('Always available to request recovery. Reconnect/recheck the held arm, validate the traveled return, then relax only at rest. A drive fault is reported, never bypassed.')
        self.reverse_button.clicked.connect(self.reverse_clicked)
        self.window.layout().insertLayout(2,controls)
        self.prepare_button.setVisible(not simulated)
        self.hardware_label=Qt.QLabel('IDLE — no motors enabled by this window.');self.hardware_label.setWordWrap(True)
        self.hardware_label.setStyleSheet('font-weight:bold;');self.window.layout().insertWidget(3,self.hardware_label)
        row=Qt.QHBoxLayout()
        row.addWidget(self.reverse_button)
        self.retry_hold_button=Qt.QPushButton('Recheck hold');self.retry_hold_button.setEnabled(False)
        self.retry_hold_button.clicked.connect(lambda:self.send('retry_hold'));row.addWidget(self.retry_hold_button)
        self.relax_button=Qt.QPushButton('Relax now (torque OFF)');self.relax_button.clicked.connect(self.relax_clicked)
        self.relax_button.setToolTip('Immediately releases right arm and gripper torque. No return first. An unsupported arm can fall.')
        self.relax_button.setEnabled(False);row.addWidget(self.relax_button)
        self.support_button=Qt.QPushButton('Exit controller');self.support_button.clicked.connect(self.supported_clicked);row.addWidget(self.support_button)
        self.support_button.setToolTip('Stops the controller without disabling motors. Does NOT relax the arm.')
        self.support_button.setEnabled(False)
        self.reconnect_button=Qt.QPushButton('Reconnect controller');self.reconnect_button.clicked.connect(self.reconnect);self.reconnect_button.setEnabled(bool(resume));row.addWidget(self.reconnect_button)
        self.window.layout().insertLayout(4,row)
        feedback_row=Qt.QHBoxLayout()
        self.live_box=Qt.QCheckBox('Show live arm position');self.live_box.setChecked(self.live_display)
        self.live_box.setToolTip('Measured encoders, not commanded targets. Play/seek switches to planned preview; check this to return to live feedback.')
        self.live_box.toggled.connect(self.set_live_display);feedback_row.addWidget(self.live_box)
        self.feedback_label=Qt.QLabel('Waiting for encoder feedback');self.feedback_label.setWordWrap(True)
        feedback_row.addWidget(self.feedback_label,1);self.window.layout().insertLayout(5,feedback_row)
        if simulated:
            mock_row=Qt.QHBoxLayout()
            for label,kind in [('Mock contact','contact'),('Mock CAN fault','can'),('Clear mock fault','clear')]:
                b=Qt.QPushButton(label);b.clicked.connect(lambda _,k=kind:self.send('inject_fault',kind=k));mock_row.addWidget(b)
            self.window.layout().insertLayout(6,mock_row)
        self.info.setText('Live arm follows measured encoders; P/Play/seek switches to planned preview. Space = emergency hold.\n'
            'Real motion: start → record3 endpoint → hold 2 seconds → reverse → settled original start → relax.\n'
            'Start only from a physically supported resting pose; encoders cannot verify physical support.\n'
            'Left arm stays relaxed; right gripper stays custom-closed until final release.\n'
            'Speed is locked once preparing/armed. 1× respects existing validated motion limits.')
        self.note.setText('SOFTWARE HOLD IS NOT A HARDWARE E-STOP. Power/CAN/drive failure may prevent holding torque. '
            'No fault auto-relaxes or auto-returns. Relax now immediately releases torque: support the arm first. '
            'Reverse assumes the traveled corridor is clear and unchanged. '
            'Frozen observed scene only; unseen/moved obstacles are not covered. zone3: original right gripper point only.')
        self.close_button.setText('Close window [Q]')
        self.close_button.setToolTip('Closes this window. An active controller keeps holding; it does not relax. Use --resume to reconnect.')
        self.owner=self
        owner=self
        class CloseGuard(Core.QObject):
            def eventFilter(self,obj,event):
                if event.type()==Core.QEvent.Close and not owner.closed:
                    owner.request_close();event.ignore();return True
                return False
        self.guard=CloseGuard(self.window);self.window.installEventFilter(self.guard)
        packet,mesh=load(plan,cal_path);self.set_scene(packet,mesh);self.frame_count=5;self.rate_changed(rate)
        self.status.setText('FIVE-FRAME PLANNING SNAPSHOT — camera released — mesh remains frozen')
        self.plan_label.setText('BASELINE • zone3 right-gripper point • full arm/stick collisions • live feedback and planned preview are separate')
        self.freeze_button.setText('Frozen planning scene');self.save_status.setText('Session: '+str(self.session))
        if resume:self.reconnect()

    def set_live_display(self,enabled):
        self.live_display=bool(enabled)
        if enabled:self.clock.pause()
        if self.live_box is not None and self.live_box.isChecked()!=bool(enabled):
            self.live_box.blockSignals(True);self.live_box.setChecked(bool(enabled));self.live_box.blockSignals(False)
        self.draw_canvas()

    def toggle_play(self):
        self.set_live_display(False);super().toggle_play()

    def seek(self,t):
        self.set_live_display(False);super().seek(t)

    def draw_canvas(self):
        if not getattr(self,'live_display',False):
            if self.robot is not None:self.robot.set_visible(True)
            return super().draw_canvas()
        from .feedback import STALE_SECONDS
        from ..live_view import LiveDepthView
        now=time.monotonic();pose=self.display_pose
        label='NO ENCODER FEEDBACK — robot hidden' if pose is None else (
            ('SIMULATED ENCODERS' if self.simulated else 'LIVE MEASURED ARM') if 0<=now-pose['stamp']<=STALE_SECONDS
            else 'STALE ENCODERS — LAST KNOWN POSE, NOT CURRENT')
        if self.robot is not None:
            self.robot.set_visible(pose is not None)
            if pose is not None:self.robot.draw_joints(pose['joints'])
            self.viewport.set_title(label+'\n'+self.depth_label()+' / '+self.layer_label())
            self.detail.setText(self.scene_detail())
            self.time_label.setText('Live pose / no preview time')
            self.play_button.setText('Play animation [P]')
        LiveDepthView.draw_canvas(self);self.last_animation_draw=now;self.feedback_draws+=1

    def stop_telemetry(self):
        reader=getattr(self,'telemetry',None)
        if reader:
            reader.close()  # join + release exclusive locks BEFORE the next owner
            self.telemetry=None

    def poll_feedback(self):
        from .feedback import DisplayTelemetry,normalized_pose,STALE_SECONDS
        now=time.monotonic();s=self.hardware_status
        owner_present=self.client_info is not None and not s.get('bus_released',False)
        if owner_present or self.job or self.pending_start:
            self.stop_telemetry()
            self.display_error='Reading feedback from the controller' if owner_present else 'Read-only acquisition/check in progress'
            pose=s.get('measured_pose') if owner_present else None
            if pose:
                try:self.display_pose=normalized_pose(pose['joints'],pose['stamp'])
                except (KeyError,ValueError,TypeError) as exc:self.display_error='Invalid owner feedback: '+str(exc)
        elif not self.simulated:
            if self.telemetry is None and now>=self.next_telemetry:
                self.telemetry=DisplayTelemetry(self.session/'live-encoders');self.next_telemetry=now+5.
            if self.telemetry:
                pose=self.telemetry.snapshot()
                if pose:self.display_pose=pose
                self.display_error=self.telemetry.error
                if self.telemetry.error:self.stop_telemetry();self.next_telemetry=now+5.
        else:self.display_error='Mock feedback starts with the simulated controller; no CAN access'
        fresh=self.display_pose is not None and 0<=now-self.display_pose['stamp']<=STALE_SECONDS
        source=('SIMULATED' if self.simulated else 'MEASURED')+' encoders' if fresh else 'Encoder feedback unavailable/stale'
        self.feedback_label.setText(source+(' • '+self.display_error if self.display_error else '')+
            (' • PLANNED PREVIEW shown' if not self.live_display else ''))
        if self.live_display and now-self.last_animation_draw>=.05:self.draw_canvas()

    def adopt_replanned(self,packet):
        replacement=PlanDisplay(packet['plan']);replacement.check_live_calibration(self.cal_path)
        if replacement.obj['model']['calibrated_urdf_sha256']!=self.plan.obj['model']['calibrated_urdf_sha256']:
            raise ValueError('Replacement model changed; restart the viewer before execution')
        self.plan=replacement;self.robot.plan=replacement
        self.source_outward=copy.deepcopy(replacement.obj['outward'])
        self.locked=False;self.rate_changed(self.requested_rate);self.rate_box.setEnabled(True)
        self.play_button.setEnabled(True);self.timeline.setEnabled(True)
        self.set_live_display(False)  # expose the newly generated path, never auto-play real motors
        print('Replacement Baseline route loaded. Preview it; a NEW Start click is required. No motion authorized.',flush=True)

    @property
    def active(self):
        # An unknown/lost controller is NOT equivalent to a relaxed arm.
        return self.pending_start or (self.start_sent and self.hardware_status.get('phase') not in ('RELAXED','DISABLED_EXTERNALLY','REJECTED','SUPPORTED_EXIT','CONTROLLER_EXIT')) or (self.client_info is not None and not self.hardware_status.get('safe_to_close',False))

    def rate_changed(self,value):
        if self.locked:return
        self.requested_rate=speed(value)
        if not self.source_outward:return
        from ..planning import cycle
        outward=copy.deepcopy(self.source_outward)
        for p in outward:
            p['time_s']/=self.requested_rate
            for key,power in [('velocities_rad_s',1),('accelerations_rad_s2',2)]:
                if key in p:p[key]=(np.array(p[key])*self.requested_rate**power).tolist()
        self.plan.obj=dict(self.plan.obj,outward=outward,cycle=cycle(outward))
        self.plan.curves=self.plan.obj['cycle'];self.plan.times=np.array([p['time_s'] for p in self.plan.curves])
        self.plan.values=np.array([p['positions_rad'] for p in self.plan.curves]);self.plan.duration=float(self.plan.times[-1])
        self.clock.pause();self.clock.duration=self.plan.duration;self.clock.set_rate(1.);self.clock.seek(0)
        self.timeline.setRange(0,int(round(self.plan.duration*1000)));self.draw_canvas()

    def toggle_freeze(self):self.emergency_hold()

    def error(self,message):
        self.last_error=str(message);self.hardware_label.setText('ERROR / NO AUTOMATIC RELAX: '+str(message));print(message,flush=True)

    def send(self,op,**kwargs):
        if self.client is None:
            if op!='hold':self.error('No controller connection')
            return False
        try:self.client.send(op,**kwargs);return True
        except Exception as e:self.error(e);return False

    def prepare_clicked(self):
        self.begin_prepare(start_after=False)

    def preparation_failed(self,message):
        self.start_when_ready=False;self.prepared=None;self.stage=None;self.error(message)
        if not self.active:
            self.locked=False;self.rate_box.setEnabled(True);self.prepare_button.setEnabled(True)
            self.execute_button.setEnabled(self.plan.playable and not self.close_requested)

    def begin_prepare(self,*,start_after):
        if self.simulated or self.active or self.job or self.close_requested or not self.plan.playable:return
        self.start_when_ready=start_after;self.last_error=''
        self.clock.pause();self.locked=True;self.rate_box.setEnabled(False);self.execute_button.setEnabled(False)
        self.prepare_button.setEnabled(False);self.prepared=None
        try:
            self.stop_telemetry()
            self.prepare_folder=self.session/('prepare-'+uuid.uuid4().hex[:8]);self.prepare_folder.mkdir()
            self.stage='capture';self.job=job('capture',self.prepare_folder/'capture',calibration=self.cal_path)
            prefix='START REQUESTED — automatic checks, then motion. HOLD cancels. ' if start_after else 'READ ONLY — no motion authorized. '
            self.hardware_label.setText(prefix+'Acquiring current encoders and fresh depth.')
        except Exception as e:self.preparation_failed(e)

    def execute_clicked(self):
        # This explicit button click authorizes ONE cycle, not future retries.
        if self.active or self.job or self.close_requested or not self.plan.playable:return
        try:
            if not self.simulated:
                if not self.prepared or not 0<=time.monotonic()-json.loads(self.prepared.read_text())['scene_stamp']<30:
                    self.begin_prepare(start_after=True);return
            self.start_controller()
        except Exception as e:self.preparation_failed(e)

    def start_controller(self):
        """Called only after a Start click and completed automatic preparation."""
        if self.active or self.job or self.close_requested:raise ValueError('Controller already active or window closing')
        if not self.simulated and self.prepared is None:raise ValueError('Prepare and recheck before arming')
        self.stop_telemetry()
        if getattr(self,'live_box',None) is not None:self.set_live_display(True)
        self.start_when_ready=False
        folder=self.session/('controller-'+uuid.uuid4().hex[:8]);folder.mkdir()
        self.client_info=service.launch(folder,prepared=self.prepared,
            simulate_plan=self.plan.path if self.simulated else None,rate=self.requested_rate)
        self.hardware_status={};self.pending_start=True;self.locked=True;self.rate_box.setEnabled(False)
        self.execute_button.setEnabled(False);self.prepare_button.setEnabled(False);self.clock.pause()
        self.connect_deadline=time.monotonic()+10
        print('Recovery/reconnect: ./plan_drums.sh baseline --resume '+str(folder),flush=True)

    def reconnect(self):
        if not self.client_info:return
        try:
            if self.client:self.client.close()
            self.client=service.Client(self.client_info);self.connection_failed=False
        except Exception as e:self.client=None;self.error(e)

    def emergency_hold(self):
        self.start_when_ready=False
        if self.job:
            # This job is acquisition/prepare ONLY. It owns no powered motors.
            self.job.cancel();self.stage='cancelled'
        if self.pending_start:
            # Prevent a deferred connection from starting after a pre-arm Hold.
            self.pending_start=False
        self.send('hold');self.clock.pause()
        if self.client_info:self.hardware_label.setText('HOLD REQUESTED — wait for confirmed feedback. NO relaxation sent.')

    def reverse_clicked(self):
        self.reverse_error=''
        # A return request must never leave a deferred Start armed behind it.
        self.start_when_ready=False;self.pending_start=False;self.clock.pause()
        if self.job:self.job.cancel();self.stage='cancelled'
        if self.client is None and self.client_info:self.reconnect()
        if self.client is None:
            self.error('Reverse not started: no connected active trajectory. No motor state changed.');return
        # Older still-running owners require an IPC acknowledgement field.
        # Populate it from the click, never by reopening a confirmation dialog.
        if self.send('reverse',**({} if 'can_relax' in self.hardware_status else dict(scene_confirmed=True))):
            self.hardware_label.setText('REVERSE REQUESTED — checking current feedback and the return path; no relaxation before rest.')

    def relax_clicked(self):
        self.start_when_ready=False;self.pending_start=False;self.clock.pause()
        if self.job:self.job.cancel();self.stage='cancelled'
        if self.send('relax'):
            self.hardware_label.setText('RELAX REQUESTED — right arm/gripper torque off; awaiting motor feedback.')

    def supported_clicked(self):
        self.send('supported_exit',**({} if 'can_relax' in self.hardware_status else dict(confirmation='ARM PHYSICALLY SUPPORTED')))

    def request_close(self):
        self.start_when_ready=False
        if self.active:
            self.emergency_hold()
            print('Window closing; independent controller keeps HOLD, not relaxation. Reconnect with ./plan_drums.sh baseline --resume '+
                  str((self.client_info or {}).get('folder',self.session)),flush=True)
        if self.job:self.job.cancel();self.stage='cancelled'
        # The final close() owns the one close message. Sending it here as
        # well races the next GUI poll against the owner's expected exit.
        self.close_requested=True

    def poll_hardware(self):
        if self.pending_start and self.client is None:
            if Path(self.client_info['socket']).exists():self.reconnect()
            elif time.monotonic()>self.connect_deadline:
                self.pending_start=False;self.connection_failed=True;self.error('Controller startup not confirmed. Reconnect/check controller.log; no assumed relaxation.')
        if self.client:
            try:
                self.hardware_status=self.client.poll()
                if self.pending_start and self.hardware_status.get('phase')=='READY':
                    self.start_sent=True;self.client.send('start',confirmation='MOVE RIGHT ARM');self.pending_start=False
                s=self.hardware_status
                # Availability is not a claim that movement is safe/possible.
                # Let the owner recheck every click, even with stale status or
                # an unconfirmed hold, and explain any concrete blocker.
                self.support_button.setEnabled(s.get('phase') in ('HELD','FAULT_HOLD','RELAX_UNCONFIRMED'))
                self.relax_button.setEnabled(bool(s.get('can_relax')))
                self.retry_hold_button.setEnabled(bool(s.get('can_retry_hold')))
                self.reconnect_button.setEnabled(True)
                if s:
                    if s['phase'] in ('RECOVERY','RELAXED'):self.reverse_error=''
                    self.hardware_label.setText(('MOCK ' if self.simulated else 'REAL ARM: ')+s['phase']+
                        (' • HOLD CONFIRMED' if s.get('hold_confirmed') else '')+
                        (' • '+s['error'] if s.get('error') else '')+(' • '+s['feedback_error'] if s.get('feedback_error') else '')+
                        (' • Older controller: Relax now requires a new session' if s.get('started') and 'can_relax' not in s else '')+
                        (' • '+getattr(self,'reverse_error','') if getattr(self,'reverse_error','') else ''))
                for msg in self.client.messages:
                    if msg.get('error'):
                        if msg.get('reply')=='reverse':self.reverse_error='Reverse not started: '+msg['error']
                        self.error(msg['error'])
                self.client.messages.clear()
                if s.get('phase') in ('CONTROLLER_EXIT','SUPPORTED_EXIT'):
                    self.client.close();self.client=None;self.reconnect_button.setEnabled(False)
                    self.hardware_label.setText('CONTROLLER EXITED — no motor state was changed by Exit.')
            except Exception as e:
                self.connection_failed=True;self.error(str(e)+' — reconnect to inspect motor state; connection loss never authorizes relaxation.')
                self.client.close();self.client=None
        if self.job:
            code=self.job.poll()
            if code is not None:
                current=self.job;self.job=None
                try:
                    if self.stage=='cancelled':raise ValueError('Preparation cancelled; no enable sent')
                    if code:raise ValueError(failure(current))
                    if self.stage=='capture':
                        self.stage='prepare';self.job=job('prepare',self.prepare_folder/'checked',plan=self.plan.path,
                            capture=self.prepare_folder/'capture',calibration=self.cal_path)
                        prefix='START REQUESTED — HOLD cancels. ' if self.start_when_ready else 'READ ONLY — no motion authorized. '
                        self.hardware_label.setText(prefix+'Checking this exact path against fresh depth with Baseline collision margins.')
                    else:
                        self.prepared=current.folder/'prepared.json';packet=json.loads(self.prepared.read_text())
                        replaced=bool(packet.get('replanned'))
                        if replaced:
                            self.start_when_ready=False;self.pending_start=False
                            self.adopt_replanned(packet)
                        shadow=copy.copy(self.plan);shadow.obj=copy.deepcopy(self.plan.obj);shadow.req=copy.deepcopy(self.plan.req)
                        shadow.obj['provenance']['scene']=packet['fresh_scene'];shadow.req['scene']=packet['request']['scene']
                        display,mesh=load(shadow,self.cal_path);self.packet=None;self.set_scene(display,mesh)
                        self.frame_count=5;self.status.setText('FRESH FIVE-FRAME EXECUTION CHECK — camera released — mesh remains frozen')
                        expired=not 0<=time.monotonic()-packet['scene_stamp']<30
                        if expired and not replaced:raise ValueError('Readiness check expired; click Start again. No enable sent.')
                        self.stage=None
                        if self.start_when_ready and not self.close_requested:self.start_controller()
                        else:
                            self.execute_button.setEnabled(not self.close_requested);self.prepare_button.setEnabled(True)
                            if replaced:
                                if expired:self.prepared=None
                                self.hardware_label.setText('Old route blocked by fresh depth; REPLACEMENT BASELINE loaded. Preview, then click START REAL ARM. '+
                                    ('Start will refresh the expired scene check. ' if expired else '')+'No motion authorized.')
                            else:self.hardware_label.setText('Ready — no motion authorized. Click START REAL ARM to run; expired checks refresh automatically.')
                except Exception as e:self.preparation_failed(e)
                finally:current.close()
        if self.prepared and not self.client_info and self.job is None:
            stamp=json.loads(self.prepared.read_text())['scene_stamp'];remaining=30-(time.monotonic()-stamp)
            if remaining<=0:
                self.start_when_ready=False;self.prepared=None;self.locked=False;self.rate_box.setEnabled(True)
                self.execute_button.setEnabled(self.plan.playable and not self.close_requested)
                self.hardware_label.setText('Readiness expired. START REAL ARM will automatically check fresh depth before moving.')

    def poll(self,camera=None):
        key=super().poll(camera)
        if key!=ord('q'):
            self.poll_hardware();self.poll_feedback()
        return key

    def close(self):
        self.start_when_ready=False
        self.stop_telemetry()
        if self.job:self.job.close();self.job=None
        if self.client:
            if self.active:self.send('hold')
            else:self.send('close')
            self.client.close();self.client=None
        # Independent controller keeps ownership/torque on an active GUI crash.
        super().close()

    def report(self):
        report=super().report()
        report.update(type='baseline_supervised_view',
                      robot_overlay_source=('mock measured encoders' if self.simulated else 'live measured encoders') if self.live_display else 'planned preview',
                      feedback_draws=self.feedback_draws,latest_feedback_stamp=None if self.display_pose is None else self.display_pose['stamp'],
                      display_feedback_error=self.display_error,hardware_execution_available=True,motion_authorized=self.start_sent)
        return report


def show(path,folder,cal_path=DEFAULT_CALIBRATION,*,rate=1.,simulated=False,resume=None,seconds=0.):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    view=View(path,folder,cal_path,rate=rate,simulated=simulated,resume=resume)
    def interrupt(*_):view.request_close()
    old={s:signal.signal(s,interrupt) for s in (signal.SIGINT,signal.SIGTERM)}
    start=time.monotonic();error=None
    try:
        while True:
            key=view.poll()
            if key==ord('q'):break
            if key==ord('s'):view.save_view()
            if seconds and time.monotonic()-start>=seconds and not view.active:view.request_close()
            time.sleep(.008)
    except BaseException as e:
        error=str(e)
        if view.client:view.send('hold')
        raise
    finally:
        atomic_json(folder/'baseline_view_report.json',dict(error=error,simulated=simulated,hardware_status=view.hardware_status,
            controller=view.client_info,rendering=view.report(),last_error=view.last_error))
        view.close()
        for s,h in old.items():signal.signal(s,h)
