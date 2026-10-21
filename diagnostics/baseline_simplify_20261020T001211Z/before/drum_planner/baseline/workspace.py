"""Baseline's three-window UI. Explicit Calculate; recoverable, always-on actions.

Read-only workers and the independent motor owner are separate lifecycles.
An unavailable action explains its prerequisite; widgets are never disabled.
No UI error, worker cancellation, reconnect or window close authorizes relax.
"""
import copy,json,signal,time,uuid
from pathlib import Path

from PyQt5 import QtCore,QtGui,QtWidgets

from ..common import DEFAULT_CALIBRATION,JOINTS,LEFT_JOINTS,atomic_json
from ..experiment.runtime import Job
from ..planned_view import load,load_scene
from ..trajectory_view import PlanDisplay
from . import service,tasks
from .feedback import DisplayTelemetry,normalized_pose,movement_error,STALE_SECONDS
from .windows import DisplayModel,ArmWindow,PlanWindow,ControlWindow


class Workspace:
    def __init__(self,folder,cal_path=DEFAULT_CALIBRATION,*,rate=1.,simulate_plan=None,resume=None,autoscene=True):
        self.session=Path(folder).resolve();self.session.mkdir(parents=True,exist_ok=True);self.cal_path=Path(cal_path)
        self.simulated=simulate_plan is not None or bool(resume and resume['simulated'])
        self.simulate_plan=Path(simulate_plan or resume['plan']) if self.simulated else None
        self.plan=None;self.plan_valid=False;self.anchor=None;self.prepared=None
        self.pose=None;self.initial_preview_pose=False;self.reader=None;self.reader_retry=0.;self.reader_error=''
        self.job=None;self.job_kind=None;self.work_scene=None;self.want_start=False;self.start_rate=rate
        self.owner_info=resume;self.last_owner_info=resume;self.client=None;self.owner_status={};self.owner_retry=0.
        self.start_sent=False;self.reverse_intent=None;self.owner_connect_deadline=0.
        self.closed=False;self.close_requested=False;self.last_error='';self.messages=[];self.scene_path=None
        self.initial_scene_pending=autoscene and not self.simulated and not resume
        self.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([]);self.app.setQuitOnLastWindowClosed(False)
        model=PlanDisplay(self.simulate_plan or resume['plan']) if self.simulated or resume else DisplayModel(self.session/'display_model',self.cal_path)
        self.preview=PlanWindow(model,rate,self.speed_changed)
        self.live=ArmWindow('X5 · Live arm — measured status',model)
        self.live.viewport.set_title('LIVE ARM — encoder feedback only')
        self.control=ControlWindow(self);self.preview.notice=self.message
        self.shortcuts=[]
        for window in self.windows:
            shortcut=QtWidgets.QShortcut(QtGui.QKeySequence('Space'),window)
            shortcut.activated.connect(lambda:self.action(self.hold));self.shortcuts.append(shortcut)
        if self.simulated:
            self.sim_joints=dict(zip(JOINTS,model.req['start']))|dict(zip(LEFT_JOINTS,model.req['left']))
            self.sim_joints['openarmx_left_finger_joint1']=model.req['left_opening_m']
            self.control.setWindowTitle('X5 · Baseline controls — OFFLINE simulated motors')
            self.control.buttons['start'].setText('2 · Start SIMULATED trajectory')
            try:self.set_scene(model.obj['provenance']['scene'])
            except Exception as exc:self.message('Offline scene unavailable: '+str(exc),error=True)
        if resume:
            self.install_plan(model);self.plan_valid=False
            self.set_scene(model.obj['provenance']['scene']);self.connect_owner()
        self.arrange()
        for window in self.windows:window.show()
        self.message('Click Calculate trajectory to begin. Opening this program does not calculate or move the arm.')

    @property
    def windows(self):return (self.control,self.preview,self.live)

    @property
    def active(self):
        # A sent Start is active even until the next status packet replaces
        # the older READY status. Never detach an owner in that IPC gap.
        return bool(self.owner_info and (self.want_start or self.start_sent or not self.owner_status.get('safe_to_close',False)))

    def arrange(self):
        r=self.app.primaryScreen().availableGeometry();cw=min(430,max(350,r.width()//4));pw=(r.width()-cw)//2
        for window,x,w in [(self.control,r.x(),cw),(self.preview,r.x()+cw,pw),(self.live,r.x()+cw+pw,pw)]:
            window.resize(w,min(740 if window is self.control else 900,r.height()));window.move(x,r.y())

    def message(self,text,*,error=False):
        text=str(text)
        if error:self.last_error=text
        self.control.status.setText(text);self.control.status.setStyleSheet('color:#a82020' if error else 'color:#173f56')
        self.control.status.setToolTip(text)  # long errors stay in the scrollable log, not over the buttons
        if self.messages and self.messages[-1]==text:return
        self.control.log.appendPlainText(text);self.messages=(self.messages+[text])[-120:];print(text,flush=True)

    def action(self,callback):
        try:callback()
        except Exception as exc:
            self.want_start=False;self.reverse_intent=None
            self.message(str(exc)+' No automatic relaxation. You can retry using the same buttons.',error=True)

    def speed_changed(self,value):
        if hasattr(self,'control'):
            note=f'Preview / next Start speed: {value:.2f}×.'
            if self.want_start or self.active:note+=f' The requested/current arm run keeps {self.start_rate:.2f}×.'
            self.message(note)

    def set_scene(self,path):
        path=Path(path).resolve()
        if path==self.scene_path:return
        packet,mesh=load_scene(path,self.cal_path)
        self.preview.set_scene(packet,mesh);self.live.set_scene(packet,mesh);self.scene_path=path

    def set_pose(self,pose):
        pose=normalized_pose(pose['joints'],pose['stamp']);previous=self.pose;self.pose=pose
        # Polling IPC/Qt is faster than feedback. Do FK/update actors only when
        # measured angles change, not repeatedly for the same cached batch.
        if previous is None or pose['joints']!=previous['joints']:self.live.set_pose(pose)
        else:self.live.pose=pose
        # One initial disconnected pose, never following subsequent live changes.
        if not self.initial_preview_pose and self.preview.plan is None:
            self.preview.set_pose(copy.deepcopy(pose));self.initial_preview_pose=True
        if self.anchor is not None and not self.active:
            error=movement_error(self.anchor,pose)
            if error:self.invalidate(error)

    def fresh_pose(self):
        if self.reader:
            pose=self.reader.snapshot()
            if pose:self.set_pose(pose)
        if self.pose is None or not 0<=time.monotonic()-self.pose['stamp']<=STALE_SECONDS:
            raise ValueError('No fresh arm feedback yet. '+self.reader_error+' Check power/connection, then click Calculate again; the live window reconnects automatically.')
        return copy.deepcopy(self.pose)

    def stop_reader(self):
        if self.reader:
            reader=self.reader;reader.close();error=reader.movement_error();self.reader=None
            if error and self.anchor is not None:
                self.invalidate(error);raise ValueError(error)

    def invalidate(self,message):
        self.plan_valid=False;self.anchor=None;self.prepared=None;self.want_start=False
        if self.reader:self.reader.set_guard(None)
        if self.job and self.job_kind!='scene':self.cancel_job()
        self.preview.caption.setText('Preview only — previous calculation invalid. Click Calculate trajectory again.')
        self.message(message,error=True)

    def cancel_job(self):
        self.want_start=False
        if self.job:
            current=self.job;self.job=None;self.job_kind=None
            current.close()  # disposable query-only work; NEVER the motor owner

    def launch_job(self,kind,anchor=None):
        folder=self.session/(kind+'-'+uuid.uuid4().hex[:8]);folder.mkdir()
        cmd=['/usr/bin/python3','-m','drum_planner.baseline.work',kind,'--folder',str(folder),'--calibration',str(self.cal_path)]
        if anchor is not None:
            inputs=dict(anchor=anchor)
            if kind=='check':inputs['plan']=str(self.plan.path)
            atomic_json(folder/'input.json',inputs);cmd+=['--input',str(folder/'input.json')]
        self.job=Job(cmd,folder,timeout=300 if kind=='calculate' else 180,allow_can=kind!='scene')
        self.job_kind=kind;self.work_scene=None

    def install_plan(self,plan):
        if not plan.playable:raise ValueError('No valid Baseline trajectory. Click Calculate again after correcting the reported obstacle/pose.')
        self.plan=plan;self.preview.install_plan(plan)
        # The live robot remains at encoder positions, NOT the new path's time.
        self.live.robot.plan=plan
        if self.pose:self.live.set_pose(self.pose)
        self.preview.caption.setText('Calculated Baseline trajectory. Play/Pause/scrub here affect only this preview.')

    def calculate(self):
        self.want_start=False;self.reverse_intent=None
        if self.active:
            self.message('Arm is running or held. Use Emergency Hold / Reverse to rest + relax, then Calculate. No plan or motor state changed.');return
        self.detach_finished_owner()
        self.cancel_job();self.initial_scene_pending=False;self.anchor=None;self.plan_valid=False;self.prepared=None
        if self.reader:self.reader.set_guard(None)
        anchor=self.fresh_pose();self.anchor=anchor
        if self.reader:self.reader.set_guard(anchor)
        if self.simulated:
            self.install_plan(PlanDisplay(self.simulate_plan));self.plan_valid=True
            self.message('OFFLINE: saved test route loaded by Calculate. No camera/CAN. Preview, then Start simulated cycle.');return
        self.stop_reader();self.launch_job('calculate',anchor);self.last_error=''
        self.message('CALCULATING — leave both arms still. Progress is in the terminal. Calculate again restarts; Hold cancels. No motors enabled.')

    def start(self):
        if self.active:
            self.message('A real-arm cycle/hold already exists. Use Hold or Reverse to rest; Start will not create a second controller.');return
        if self.job:
            self.message('A calculation/check is still running. Start is not queued; click it when ready. Hold cancels, Calculate restarts.');return
        self.detach_finished_owner()
        if self.plan is None or not self.plan_valid:
            self.message('No current usable trajectory. Click Calculate trajectory first.',error=True);return
        self.fresh_pose()
        if not self.plan_valid:return
        self.start_rate=self.preview.rate.value();self.want_start=True;self.reverse_intent=None
        if self.simulated:self.launch_owner();return
        self.stop_reader();self.launch_job('check',self.anchor)
        self.message(f'START REQUESTED ({self.start_rate:.2f}×) — checking the SAME previewed route against fresh depth. Hold cancels. No replanning in Start.')

    def hold(self):
        had_work=self.job is not None
        self.want_start=False;self.reverse_intent=None;self.cancel_job()
        if had_work:self.plan_valid=False;self.anchor=None
        if self.owner_info:
            self.ensure_owner();self.client.send('hold')
            self.message('HOLD requested. No relaxation. Reverse to rest remains available.')
        else:self.message('Calculation/start cancelled.' if had_work else 'No active trajectory to hold. Calculate and Start remain available.')

    def cancel_calculation(self):
        if self.job and self.job_kind!='scene':
            self.cancel_job();self.plan_valid=False;self.anchor=None;self.prepared=None
            self.message('Calculation/start check cancelled. Click Calculate to try again. No motor commands sent.')
        elif self.want_start and not self.start_sent:
            self.want_start=False;self.message('Pending Start cancelled before enable. Calculate and Start remain available.')
        else:self.message('No calculation/start check is running. Use Emergency Hold to stop actual arm motion.')

    def discover_owner(self):
        """Explicit recovery action can reconnect in-place; never steal locks."""
        if self.owner_info:return
        from ..bus_ownership import owners
        import shlex
        found={x['resume'] for x in owners() if x.get('resume')}
        if len(found)==1:
            folder=Path(shlex.split(next(iter(found)))[-1])
            info=json.loads((folder/'control.json').read_text())
            if bool(info['simulated'])!=self.simulated:raise ValueError('Controller mode mismatch')
            self.owner_info=info;self.last_owner_info=info
    def ensure_owner(self):
        if not self.owner_info:self.discover_owner()
        if not self.owner_info:raise ValueError('No active controller/trajectory to recover. Click Calculate for a new cycle.')
        if not self.client:self.connect_owner()
        if not self.client:raise ValueError('Controller connection unavailable. Restore CAN/controller connection and click Reverse again. No release or new controller was sent.')

    def reverse(self):
        self.want_start=False;self.cancel_job();self.ensure_owner()
        status=self.client.poll();self.owner_status=status
        phase=status.get('phase')
        if status.get('safe_to_close'):
            self.detach_finished_owner();self.message('No active traveled trajectory. Calculate is ready for a new cycle.');return
        if phase in ('RECOVERY','CHECKING_RETURN','SETTLING_REST'):
            self.message('Return is already in progress. Emergency Hold remains available.');return
        if phase in ('HELD','FAULT_HOLD'):
            self.client.send('retry_hold')
        else:self.client.send('hold')
        self.reverse_intent=dict(after=time.monotonic()+.7,deadline=time.monotonic()+3.)
        self.message('REVERSE requested — establishing a stationary hold, checking the traveled return, then returning to rest before relaxing.')

    def launch_owner(self):
        self.stop_reader()
        folder=self.session/('controller-'+uuid.uuid4().hex[:8]);folder.mkdir()
        try:info=service.launch(folder,prepared=self.prepared,simulate_plan=self.plan.path if self.simulated else None,rate=self.start_rate)
        except Exception:
            # systemd may have accepted a launch before its client timed out.
            # Retain the address for reconnect/cleanup, but cancel Start intent.
            self.want_start=False
            if (folder/'control.json').exists():
                self.owner_info=json.loads((folder/'control.json').read_text());self.last_owner_info=self.owner_info
                self.owner_status={};self.owner_retry=0.;self.start_sent=False
            raise
        self.owner_info=info;self.last_owner_info=info;self.owner_status={};self.client=None;self.start_sent=False
        self.owner_connect_deadline=time.monotonic()+10;self.owner_retry=0.
        print('Recovery: ./plan_drums.sh baseline --resume '+str(folder),flush=True)

    def connect_owner(self):
        if not self.owner_info:return
        try:
            self.client=service.Client(self.owner_info);self.owner_retry=0.
        except Exception as exc:
            self.client=None;self.owner_retry=time.monotonic()+1.
            self.control.run_status.setText('Controller disconnected; reconnecting. '+str(exc))

    def detach_finished_owner(self):
        if not self.owner_info:return
        if not self.owner_status.get('safe_to_close'):
            raise ValueError('Controller state is unknown/active. Use Hold or Reverse; do not start another owner.')
        if self.client:
            try:self.client.send('close')
            except Exception:pass  # verified-terminal owner may already have exited
            self.client.close()
        self.client=None;self.owner_info=None;self.owner_status={};self.want_start=False;self.start_sent=False
        self.reverse_intent=None

    def poll_owner(self,now):
        if not self.owner_info:return
        if not self.client and now>=self.owner_retry:self.connect_owner()
        if not self.client:
            if self.want_start and self.owner_connect_deadline and now>self.owner_connect_deadline:
                self.want_start=False;self.message('Start connection timed out. Reconnecting; Start was cancelled. You can retry after connection recovers.',error=True)
            return
        try:
            s=self.client.poll();self.owner_status=s
            if not s:return
            if s.get('measured_pose'):self.set_pose(s['measured_pose'])
            if self.want_start and s.get('phase')=='READY' and not self.start_sent:
                self.start_sent=True;self.want_start=False;self.anchor=None
                self.client.send('start',confirmation='MOVE RIGHT ARM')
            self.control.run_status.setText(('SIMULATED ' if self.simulated else 'ARM: ')+s['phase']+
                (' · '+s['error'] if s.get('error') else '')+(' · '+s['feedback_error'] if s.get('feedback_error') else ''))
            for msg in self.client.messages:
                if msg.get('error'):
                    self.want_start=False;self.reverse_intent=None
                    self.message(str(msg['error'])+' Correct the cause and retry the button; no automatic release.',error=True)
            self.client.messages.clear()
            if self.reverse_intent:
                if now>self.reverse_intent['deadline']:
                    self.reverse_intent=None;self.message('Hold not confirmed. Restore feedback / resolve the drive fault, then click Reverse again. Torque is not released.',error=True)
                elif now>=self.reverse_intent['after'] and s.get('phase') in ('HELD','FAULT_HOLD') and s.get('hold_confirmed'):
                    self.reverse_intent=None;self.client.send('reverse')
            if s.get('phase') in ('RELAXED','DISABLED_EXTERNALLY','REJECTED','CONTROLLER_EXIT','SUPPORTED_EXIT') and s.get('safe_to_close'):
                error=s.get('error');phase=s['phase'];self.plan_valid=False;self.anchor=None;self.prepared=None
                self.detach_finished_owner()
                self.message((phase+': '+error+'. ' if error else phase+'. ')+
                    'Controller session finished. Click Calculate for another attempt/cycle; no restart needed.',error=bool(error))
        except Exception as exc:
            self.want_start=False;self.reverse_intent=None
            if self.client:self.client.close();self.client=None
            self.owner_retry=now+1.
            self.message('Controller connection lost; automatic reconnect is active. No relaxation. '+str(exc),error=True)

    def poll_job(self):
        if self.job is None:return
        current=self.job;kind=self.job_kind
        feedback=current.folder/'feedback.json'
        if feedback.exists():
            item=json.loads(feedback.read_text())
            if item.get('pose'):self.set_pose(item['pose'])
            if self.job is not current:return
            if item.get('error'):self.invalidate(item['error']);return
        progress=current.folder/'progress.json'
        if progress.exists():
            scene=json.loads(progress.read_text()).get('scene')
            if scene and self.work_scene!=scene:self.set_scene(scene);self.work_scene=scene
        code=current.poll()
        if code is None:return
        self.job=None;self.job_kind=None
        try:
            if code:
                path=current.folder/'error.json'
                raise ValueError(json.loads(path.read_text())['error'] if path.exists() else 'Worker stopped. See '+str(current.log_path))
            if kind=='scene':self.set_scene(current.folder/'scene.json');return
            result=json.loads((current.folder/'result.json').read_text());self.set_scene(result['scene'])
            if kind=='calculate':
                plan=PlanDisplay(result['plan']);plan.check_live_calibration(self.cal_path)
                self.install_plan(plan);self.plan_valid=True
                self.message('READY — calculated trajectory available. Preview/play/scrub in the disconnected window, then click Start. Leave arms still.')
            elif self.want_start:
                self.prepared=Path(result['prepared']);tasks.check_packet(self.prepared)
                self.launch_owner()
        except Exception as exc:
            if kind=='scene':self.message('Initial camera snapshot failed: '+str(exc)+' Calculate will retry it.',error=True)
            else:self.invalidate(str(exc)+' Click Calculate to retry. All controls remain available.')
        finally:current.close()

    def poll_feedback(self,now):
        if self.owner_info:return
        if self.job and self.job_kind!='scene':return  # worker publishes its exclusive encoder stream
        if self.simulated:
            self.set_pose(normalized_pose(self.sim_joints,now));self.reader_error='';return
        if self.reader is None and now>=self.reader_retry:
            self.reader=DisplayTelemetry(self.session/'live-encoders');self.reader.set_guard(self.anchor)
        if self.reader:
            pose=self.reader.snapshot()
            if pose:self.set_pose(pose)
            error=self.reader.movement_error()
            if error and self.anchor is not None:self.invalidate(error)
            self.reader_error=self.reader.error
            if self.reader.error:
                # A missing/faulted stream also invalidates the stationary
                # guarantee; reconnecting never resurrects a discarded plan.
                if self.anchor is not None:self.invalidate('Encoder monitoring interrupted. Reconnect and click Calculate again. '+self.reader.error)
                self.reader.close();self.reader=None;self.reader_retry=now+1.

    def poll(self):
        self.app.processEvents()
        if self.close_requested:return False
        now=time.monotonic()
        # Keep the independent owner heartbeat first; errors in a display or
        # query worker must not prevent the next poll/recovery button click.
        for operation in (lambda:self.poll_owner(now),self.poll_job,lambda:self.poll_feedback(now)):
            try:operation()
            except Exception as exc:
                self.want_start=False;self.reverse_intent=None
                self.message(str(exc)+' Retry Calculate or Reverse as appropriate; no automatic relaxation.',error=True)
        if self.initial_scene_pending and self.job is None:
            self.initial_scene_pending=False;self.action(lambda:self.launch_job('scene'))
        fresh=self.pose is not None and 0<=now-self.pose['stamp']<=STALE_SECONDS
        self.live.caption.setText(('SIMULATED feedback' if self.simulated else 'LIVE measured encoders') if fresh else
            'Feedback unavailable / STALE — last known pose, not current. '+self.reader_error)
        self.live.viewport.set_title('LIVE ARM — '+('measured position' if fresh else 'STALE / waiting for feedback'))
        for operation in (lambda:self.preview.tick(now),lambda:self.live.draw(now)):
            try:operation()
            except Exception as exc:self.message('Display error: '+str(exc)+' The control panel remains available.',error=True)
        return True

    def request_close(self):
        self.want_start=False;self.reverse_intent=None;self.close_requested=True

    def report(self):
        return dict(type='baseline_three_windows',plan=None if self.plan is None else str(self.plan.path),
            plan_valid=self.plan_valid,simulated=self.simulated,owner=self.last_owner_info,owner_status=self.owner_status,
            last_error=self.last_error,button_states={k:b.isEnabled() for k,b in self.control.buttons.items()},
            preview_renderer=self.preview.viewport.report(),live_renderer=self.live.viewport.report())

    def close(self):
        if self.closed:return
        self.closed=True;self.want_start=False;self.reverse_intent=None
        self.cancel_job();self.anchor=None;self.stop_reader()
        if self.client:
            try:self.client.send('hold' if self.active else 'close')
            except Exception:pass  # connection loss never authorizes another command/relaxation
            self.client.close();self.client=None
        self.preview.shutdown();self.live.shutdown();self.control.hide();self.control.deleteLater();self.app.processEvents()


def show(folder,cal_path=DEFAULT_CALIBRATION,*,rate=1.,simulate_plan=None,resume=None,seconds=0.):
    view=Workspace(folder,cal_path,rate=rate,simulate_plan=simulate_plan,resume=resume)
    old={s:signal.signal(s,lambda *_:view.request_close()) for s in (signal.SIGINT,signal.SIGTERM)}
    began=time.monotonic()
    try:
        while view.poll():
            if seconds and time.monotonic()-began>=seconds and not view.active:view.request_close()
            time.sleep(.008)
    finally:
        # Logging failure must not prevent the held owner from receiving Hold.
        try:atomic_json(view.session/'three_window_report.json',view.report())
        finally:
            view.close()
            for s,h in old.items():signal.signal(s,h)
