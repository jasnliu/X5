"""Baseline-only preview + explicitly armed right-arm controls.

Planning and hardware ownership run outside the GUI/editor process. A renderer
exception, lost UI heartbeat, close, or Ctrl-C never authorizes motor relaxation.
"""
import copy,json,signal,time,uuid
from pathlib import Path
import numpy as np
from ..common import ROOT,DEFAULT_CALIBRATION,DEFAULT_RECORDING,atomic_json,calibration,digest
from ..experiment.runtime import Job
from ..planned_view import load
from ..snapshot_preview import SnapshotPlaybackView
from ..trajectory_view import PlanDisplay
from .paths import speed
from . import service
from .tasks import FLAGS


def job(action,folder,**kwargs):
    cmd=['/usr/bin/python3','-m','drum_planner.baseline.tasks',action,'--folder',str(folder)]
    cmd.extend(v for k,x in kwargs.items() if x is not None for v in ('--'+k,str(x)))
    return Job(cmd,folder,timeout=300 if action=='plan' else 180,allow_can=action=='capture')


def failure(j):
    path=j.folder/'error.json'
    return json.loads(path.read_text())['error'] if path.exists() else 'Worker failed; see '+str(j.log_path)


def wait(j):
    try:
        while j.poll() is None:time.sleep(.05)
        if j.process.returncode:raise ValueError(failure(j))
    finally:j.close()


def initial(folder,cal_path):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    print('BASELINE HARDWARE MODE: startup only reads disabled encoders and the camera. It NEVER enables/moves the arm.',flush=True)
    capture=folder/'capture';wait(job('capture',capture,calibration=cal_path))
    plan=folder/'plan';wait(job('plan',plan,capture=capture,calibration=cal_path))
    return plan/'result/plan.json'


class View(SnapshotPlaybackView):
    def __init__(self,plan_path,folder,cal_path=DEFAULT_CALIBRATION,*,rate=1.,simulated=False,resume=None):
        self.session=Path(folder);self.cal_path=Path(cal_path);self.simulated=simulated
        self.client=None;self.client_info=resume;self.hardware_status={};self.job=None;self.stage=None
        self.prepared=None;self.pending_start=False;self.start_sent=False;self.prepare_folder=None;self.review=None
        self.locked=False;self.last_error='';self.connection_failed=False;self.requested_rate=speed(rate)
        plan=PlanDisplay(plan_path);plan.check_live_calibration(cal_path)
        self.source_outward=copy.deepcopy(plan.obj.get('outward',[]))
        super().__init__(calibration(cal_path),self.session,plan,autoplay=False,loop=False)
        Qt=self.QtWidgets;Core=self.QtCore
        self.window.setWindowTitle('X5 BASELINE — '+('SIMULATED hardware controls / NO CAN' if simulated else 'supervised RIGHT ARM'))
        self.banner.setText('BASELINE ONLY • '+('MOCK MOTORS — NO CAN / NO PHYSICAL MOTION' if simulated else 'REAL RIGHT ARM ONLY AFTER EXPLICIT CONFIRMATION'))
        self.speed.hide();self.loop_box.hide();self.clock.loop=False
        for label in self.window.findChildren(Qt.QLabel):
            if label.text()=='Speed:':label.hide()
        controls=Qt.QHBoxLayout()
        self.rate_box=Qt.QDoubleSpinBox();self.rate_box.setRange(.25,1.);self.rate_box.setSingleStep(.05);self.rate_box.setDecimals(2)
        self.rate_box.setValue(rate);self.rate_box.setSuffix(' ×');self.rate_box.valueChanged.connect(self.rate_changed)
        self.rate_box.setFixedWidth(85)
        controls.addWidget(Qt.QLabel('Total movement speed (25–100%):'));controls.addWidget(self.rate_box)
        self.prepare_button=Qt.QPushButton('Prepare real execution (read-only)');self.prepare_button.clicked.connect(self.prepare_clicked)
        self.prepare_button.setVisible(not simulated);controls.addWidget(self.prepare_button)
        self.execute_button=Qt.QPushButton('Start SIMULATED cycle' if simulated else 'PLAY ON REAL ARM…')
        self.execute_button.setEnabled(simulated and plan.playable);self.execute_button.clicked.connect(self.execute_clicked)
        controls.addWidget(self.execute_button)
        self.hold_button=Qt.QPushButton('EMERGENCY HOLD [Space]');self.hold_button.setStyleSheet('background:#b22222;color:white;font-weight:bold;padding:10px;')
        self.hold_button.clicked.connect(self.emergency_hold);controls.addWidget(self.hold_button)
        self.reverse_button=Qt.QPushButton('Reverse traveled path → rest → relax…');self.reverse_button.setEnabled(False)
        self.reverse_button.clicked.connect(self.reverse_clicked);controls.addWidget(self.reverse_button)
        self.window.layout().insertLayout(2,controls)
        self.hardware_label=Qt.QLabel('PREVIEW ONLY — no motors enabled.');self.hardware_label.setWordWrap(True)
        self.hardware_label.setStyleSheet('font-weight:bold;');self.window.layout().insertWidget(3,self.hardware_label)
        row=Qt.QHBoxLayout()
        self.retry_hold_button=Qt.QPushButton('Recheck hold after fault');self.retry_hold_button.setEnabled(False)
        self.retry_hold_button.clicked.connect(lambda:self.send('retry_hold'));row.addWidget(self.retry_hold_button)
        self.support_button=Qt.QPushButton('Physically supported exit…');self.support_button.clicked.connect(self.supported_clicked);row.addWidget(self.support_button)
        self.support_button.setEnabled(False)
        self.reconnect_button=Qt.QPushButton('Reconnect controller');self.reconnect_button.clicked.connect(self.reconnect);self.reconnect_button.setEnabled(bool(resume));row.addWidget(self.reconnect_button)
        if simulated:
            for label,kind in [('Mock contact','contact'),('Mock CAN fault','can'),('Clear mock fault','clear')]:
                b=Qt.QPushButton(label);b.clicked.connect(lambda _,k=kind:self.send('inject_fault',kind=k));row.addWidget(b)
        self.window.layout().insertLayout(4,row)
        self.info.setText('Preview animates the model ONLY. P = preview, Space = emergency hold.\n'
            'Real motion: start → record3 endpoint → hold 2 seconds → reverse → verified supported start → relax.\n'
            'Left arm stays relaxed; right gripper stays custom-closed until final release.\n'
            'Speed is locked once preparing/armed. 1× respects existing validated motion limits.')
        self.note.setText('SOFTWARE HOLD IS NOT A HARDWARE E-STOP. Power/CAN/drive failure may prevent holding torque. '
            'No fault auto-relaxes or auto-returns. Reverse requires healthy feedback and your confirmation that the return corridor is clear. '
            'Frozen observed scene only; unseen/moved obstacles are not covered. zone3: original right gripper point only.')
        self.owner=self
        owner=self
        class CloseGuard(Core.QObject):
            def eventFilter(self,obj,event):
                if event.type()==Core.QEvent.Close and owner.active:
                    owner.request_close();event.ignore();return True
                return False
        self.guard=CloseGuard(self.window);self.window.installEventFilter(self.guard)
        packet,mesh=load(plan,cal_path);self.set_scene(packet,mesh);self.frame_count=5;self.rate_changed(rate)
        self.status.setText('FIVE-FRAME PLANNING SNAPSHOT — camera released — mesh remains frozen')
        self.plan_label.setText('BASELINE • original zone3 right-gripper point • full arm/stick collisions • PREVIEW MODEL, NOT LIVE FEEDBACK')
        self.freeze_button.setText('Frozen planning scene');self.save_status.setText('Session: '+str(self.session))
        if resume:self.reconnect()

    @property
    def active(self):
        # An unknown/lost controller is NOT equivalent to a relaxed arm.
        return self.pending_start or (self.start_sent and self.hardware_status.get('phase') not in ('RELAXED','REJECTED','SUPPORTED_EXIT')) or (self.client_info is not None and not self.hardware_status.get('safe_to_close',False))

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

    def physical_review(self):
        Qt=self.QtWidgets;dialog=Qt.QDialog(self.window);dialog.setWindowTitle('Physical setup review — no motion yet')
        layout=Qt.QVBoxLayout(dialog)
        layout.addWidget(Qt.QLabel('These require YOUR physical verification. Software cannot infer support from stationary encoders.'))
        labels=['Camera/robot depth alignment has been physically verified.',
                'Custom right gripper/stick mount matches the model; gripper is CLOSED.',
                'Relaxed left arm AND its stick are clear of the whole planned motion.',
                'The ORIGINAL relaxed starting pose is physically supported; releasing there will not drop the arm.',
                'Kit/camera/support are fixed and the motion area is clear of people and new obstacles.',
                'Hardware emergency stop is accessible; I understand software hold cannot survive power/drive failure.']
        checks=[]
        for label in labels:
            b=Qt.QCheckBox(label);layout.addWidget(b);checks.append(b)
        notes=Qt.QPlainTextEdit();notes.setPlaceholderText('Describe the physical support, tool mount and alignment checks (at least 20 characters).');layout.addWidget(notes)
        buttons=Qt.QDialogButtonBox(Qt.QDialogButtonBox.Ok|Qt.QDialogButtonBox.Cancel);buttons.accepted.connect(dialog.accept);buttons.rejected.connect(dialog.reject);layout.addWidget(buttons)
        if dialog.exec_()!=Qt.QDialog.Accepted:return None
        if not all(x.isChecked() for x in checks) or len(notes.toPlainText().strip())<20:
            self.error('All physical checks and meaningful notes are required. No enable sent.');return None
        o=self.plan.obj;r=self.plan.req
        return dict(schema='x5-drum-planner-commissioning-v1',calibration_sha256=digest(self.cal_path),
            urdf_sha256=digest(r['urdf']),srdf_sha256=digest(r['srdf']),recording_sha256=digest(DEFAULT_RECORDING),
            supported_right_rad=r['start'],verification_notes=notes.toPlainText().strip(),**{k:True for k in FLAGS})

    def prepare_clicked(self):
        if self.simulated or self.active or self.job:return
        review=self.physical_review()
        if review is None:return
        self.clock.pause();self.review=review;self.locked=True;self.rate_box.setEnabled(False);self.execute_button.setEnabled(False)
        self.prepare_button.setEnabled(False);self.prepared=None
        self.prepare_folder=self.session/('prepare-'+uuid.uuid4().hex[:8]);self.prepare_folder.mkdir()
        atomic_json(self.prepare_folder/'physical_review.json',review)
        self.stage='capture';self.job=job('capture',self.prepare_folder/'capture',calibration=self.cal_path)
        self.hardware_label.setText('READ ONLY: acquiring current encoders and a fresh camera scene. No enable commands.')

    def execute_clicked(self):
        if self.active or self.job or not self.plan.playable:return
        if not self.simulated and not self.prepared:return
        Qt=self.QtWidgets
        message=('SIMULATION ONLY: type MOVE RIGHT ARM to test the same controls with fake motors.' if self.simulated else
                 'PHYSICAL RIGHT-ARM MOTION. Confirm supported start and clear unchanged scene.\n'
                 'Emergency Hold freezes; Reverse returns only after checks. Type MOVE RIGHT ARM:')
        answer,ok=Qt.QInputDialog.getText(self.window,'Authorize '+('simulation' if self.simulated else 'physical motion'),message)
        if not ok or answer.strip()!='MOVE RIGHT ARM':return
        self.start_controller()

    def start_controller(self):
        """Called ONLY after explicit typed UI authorization (or mock GUI tests)."""
        if self.active or self.job:raise ValueError('Controller already active')
        if not self.simulated and self.prepared is None:raise ValueError('Prepare and recheck before arming')
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
        if self.job:
            # This job is acquisition/prepare ONLY. It owns no powered motors.
            self.job.cancel();self.stage='cancelled'
        if self.pending_start:
            # Prevent a deferred connection from starting after a pre-arm Hold.
            self.pending_start=False
        self.send('hold');self.clock.pause()
        if self.client_info:self.hardware_label.setText('HOLD REQUESTED — wait for confirmed feedback. NO relaxation sent.')

    def reverse_clicked(self):
        if not self.hardware_status.get('can_return'):return
        Qt=self.QtWidgets
        answer=Qt.QMessageBox.question(self.window,'Reverse traveled path — not a blind retreat',
            'Have you removed the contact/obstruction, checked the traveled return corridor, and confirmed the original support is unchanged?\n'
            'The camera remains frozen. A new moving obstacle cannot be detected here.\n'
            'Return is allowed only with healthy, stationary feedback and a checked path.',Qt.QMessageBox.Yes|Qt.QMessageBox.No,Qt.QMessageBox.No)
        if answer==Qt.QMessageBox.Yes:self.send('reverse',scene_confirmed=True)

    def supported_clicked(self):
        Qt=self.QtWidgets
        answer,ok=Qt.QInputDialog.getText(self.window,'LAST RESORT — support physically first',
            'Use only after physically supporting the arm. This closes ownership WITHOUT sending relax.\n'
            'It does not repair a motor/CAN fault. Type ARM PHYSICALLY SUPPORTED:')
        if ok and answer.strip()=='ARM PHYSICALLY SUPPORTED':self.send('supported_exit',confirmation=answer.strip())

    def request_close(self):
        if self.active:
            self.emergency_hold();self.error('Cannot close an unsupported active arm. Reverse to rest, or physically support it before Supported exit.');return
        if self.job:self.job.cancel();self.stage='cancelled'
        if self.client:self.send('close')
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
                self.reverse_button.setEnabled(bool(s.get('can_return')))
                self.support_button.setEnabled(s.get('phase') in ('HELD','FAULT_HOLD'))
                self.retry_hold_button.setEnabled(bool(s.get('can_retry_hold')))
                self.reconnect_button.setEnabled(True)
                if s:
                    self.hardware_label.setText(('MOCK ' if self.simulated else 'REAL ARM: ')+s['phase']+
                        (' • HOLD CONFIRMED' if s.get('hold_confirmed') else '')+
                        (' • '+s['error'] if s.get('error') else '')+(' • '+s['feedback_error'] if s.get('feedback_error') else ''))
                for msg in self.client.messages:
                    if msg.get('error'):self.error(msg['error'])
                self.client.messages.clear()
            except Exception as e:
                self.connection_failed=True;self.error(str(e)+' — controller watchdog requests HOLD; reconnect, do not power off blindly.')
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
                            capture=self.prepare_folder/'capture',review=self.prepare_folder/'physical_review.json',calibration=self.cal_path)
                        self.hardware_label.setText('Checking this exact baseline path against fresh depth with Baseline collision margins; no motion.')
                    else:
                        self.prepared=current.folder/'prepared.json';packet=json.loads(self.prepared.read_text())
                        shadow=copy.copy(self.plan);shadow.obj=copy.deepcopy(self.plan.obj);shadow.req=copy.deepcopy(self.plan.req)
                        shadow.obj['provenance']['scene']=packet['fresh_scene'];shadow.req['scene']=packet['request']['scene']
                        display,mesh=load(shadow,self.cal_path);self.packet=None;self.set_scene(display,mesh)
                        self.frame_count=5;self.status.setText('FRESH FIVE-FRAME EXECUTION CHECK — camera released — mesh remains frozen')
                        self.execute_button.setEnabled(True);self.prepare_button.setEnabled(True)
                        self.hardware_label.setText('Fresh scene checked. PLAY ON REAL ARM still requires typed confirmation within 30 seconds of capture.')
                except Exception as e:
                    self.prepared=None;self.error(e);self.locked=False;self.rate_box.setEnabled(True);self.prepare_button.setEnabled(True)
                finally:current.close()
        if self.prepared and not self.client_info and self.job is None:
            stamp=json.loads(self.prepared.read_text())['scene_stamp'];remaining=30-(time.monotonic()-stamp)
            if remaining<=0:
                self.execute_button.setEnabled(False);self.prepared=None;self.locked=False;self.rate_box.setEnabled(True)
                self.hardware_label.setText('Preparation expired; preview is still available. Prepare again for fresh depth before arming.')

    def poll(self,camera=None):
        key=super().poll(camera);self.poll_hardware();return key

    def close(self):
        if self.job:self.job.close();self.job=None
        if self.client:
            if self.active:self.send('hold')
            else:self.send('close')
            self.client.close();self.client=None
        # Independent controller keeps ownership/torque on an active GUI crash.
        super().close()


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
