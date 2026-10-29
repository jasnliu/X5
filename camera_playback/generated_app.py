"""Normal start_beat hardware: frozen generated paths, no centering/alignment.

Legacy App still owns the proven ride/hi-hat/snare calibration and beat logic.
This subclass replaces ONLY acquisition, approach, Hold and rest recovery.
"""
import json
import signal
import time
import tkinter as tk
from pathlib import Path
from types import SimpleNamespace
import uuid
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from .app import App
from . import snare_workflow, hybrid_workflow
from .generated_motion import Motion
from .generated_transport import GeneratedMotors
from .hihat_calibration_runtime import HiHatCalibrationRuntime
from .hihat_sync_runtime import HiHatSyncRuntime
from .left_hold import LEFT_GRIPPER_TARGET
from centering.motors import Motors, RIGHT_GRIPPER_CLOSED
from drum_planner.common import ROOT, DEFAULT_CALIBRATION, atomic_json
from drum_planner.experiment.runtime import Job, PYTHON
from drum_planner.baseline.curve import Curve
from safe_zone.geometry import Model, Zone, RIGHT_TCP, LEFT_TCP
from smooth_playback.trajectory import Geometry


class GeneratedDual:
    """Only the interface consumed by the unchanged snare/tempo workflow."""
    def __init__(self,app,curve,geometry):
        self.app,self.g=app,geometry
        self.recording=SimpleNamespace(last_joints=curve.at(curve.duration),first_joints=curve.at(0))
        self.returning=False;self.snare_arming_at=None;self.snare_template=None
        self.snare_tuning=None;self.ready=True
    def positions(self):return self.app.bus.positions_for('left')
    def tick(self,now):
        if self.app.beat_started:snare_workflow.tick(self,now)
        return False
    def refresh(self):pass
    def stop(self):
        s=snare_workflow.session(self.app)
        if s is not None:s.stop()


class GeneratedApp(App):
    generated_mode=True
    def __init__(self,*args,simulate_mission=None,auto_preview=True,**kwargs):
        self.auto_preview=auto_preview
        self.initialized=False;self.job=None;self.job_kind=None;self.motion=None
        self.mission=None;self.beat_started=False;self.snare_preparing=False
        self.simulate_mission=Path(simulate_mission).resolve() if simulate_mission else None
        self.return_requested=False;self.last_display=0.
        self.stage='INITIALIZING';self.closed=False;self.last_error=None
        self.session=ROOT/'planner_runs'/('generated-beat-'+time.strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8])
        self.session.mkdir(parents=True)
        # UI and audio protocol only. Do NOT acquire CAN until the query-only
        # acquisition worker has CLOSED its exclusive ownership of both buses.
        args=list(args)
        if args:args[0]=False
        else:kwargs['hardware']=False
        if len(args)>3:args[3]=None;kwargs.pop('recording_path',None)
        else:kwargs['recording_path']=None
        kwargs['left_recording_path']=None
        super().__init__(*args,**kwargs)
        self.hardware=not bool(self.simulate_mission)
        self.single_run_mode=True;self.hybrid_enabled=True
        self.hybrid_tuning=hybrid_workflow.load_tuning()
        self.hihat_sync=HiHatSyncRuntime(self)
        self.recording_button.pack_forget();self.start_button.pack_forget()
        self.root.title('Generated beat — LEFT saved goal → RIGHT DT1'+(' — SOFTWARE ONLY' if self.simulate_mission else ' — HARDWARE'))
        self.header.config(text='FROZEN CAMERA + REST → LEFT PATH → RIGHT DT1 PATH\nRide depth → hi-hat timing → hybrid swing; no centering or pink-zone alignment')
        self.continue_button.config(text='START: LEFT → RIGHT → CALIBRATE → BEAT',command=self.start)
        self.emergency_button.config(text='EMERGENCY HOLD',bg='#8a5a00',command=self.emergency_hold)
        self.emergency_relax_button=tk.Button(self.root,text='EMERGENCY RELAX — TORQUE OFF NOW',
            bg='#bd1e1e',fg='white',font=('sans',14,'bold'),command=self.emergency_relax)
        self.emergency_relax_button.pack(fill='x',padx=20,pady=4,before=self.emergency_button.master)
        self.stop_button=tk.Button(self.root,text='STOP + CLOSE — RELAX BOTH ARMS',command=self.request_safe_close)
        self.stop_button.pack(fill='x',padx=20,pady=4,before=self.recording_label)
        self.center_relax_button.config(text='RETURN: RIGHT → LEFT + RELAX',command=self.return_to_rest,state='normal')
        self.recalculate_button=tk.Button(self.root,text='CAPTURE + CALCULATE AGAIN',command=self.recalculate)
        self.recalculate_button.pack(fill='x',padx=20,pady=4,before=self.recording_label)
        self.preview_button=tk.Button(self.root,text='PREVIEW GENERATED PATHS',command=self.preview,state='disabled')
        self.preview_button.pack(fill='x',padx=20,pady=4,before=self.recording_label)
        self.tempo_header_template=self.header.cget('text')
        self.tempo_continue_template=self.continue_button.cget('text')
        self._camera_loading_text=None;self._recording_loading_text=None;self._refresh_loading_label()
        self.root.bind('<Escape>',lambda _:self.emergency_relax())
        self.root.protocol('WM_DELETE_WINDOW',self.request_safe_close)
        self.root.report_callback_exception=lambda kind,value,tb:self.fail('GUI callback: '+str(value))
        signal.signal(signal.SIGINT,lambda *_:self.root.after(0,self.request_safe_close))
        signal.signal(signal.SIGTERM,lambda *_:self.root.after(0,self.request_safe_close))
        self.initialized=True
        if self.simulate_mission:
            self._load_mission(self.simulate_mission)
            self.hihat_status.set('SOFTWARE ONLY — no CAN, camera, microphone or ESP32')
        else:
            if not self.hihat.connect():self.hihat_fault_detail=self.hihat.detail
            else:self.hihat.start_service()
            self.hihat_sound_monitor.control_required=True
            self.hihat_calibration=HiHatCalibrationRuntime(self)
            self.recalculate()

    def _spawn(self,kind,command,folder,**options):
        if self.job is not None:raise RuntimeError('A bounded worker is already running')
        self.job=Job(command,folder,**options);self.job_kind=kind;self.stage=kind
        self.status.set(kind+' — no arm motion; hi-hat calibration runs independently')

    def recalculate(self):
        if self.simulate_mission:
            if self.bus and self.bus.active:return
            self._load_mission(self.simulate_mission);return
        if self.bus and self.bus.active:
            self.status.set('Return normally or use Emergency Relax before capturing again. No motor state changed.');return
        if self.job is not None:
            self.status.set('Capture/calculation already running; Hold cancels it.');return
        if self.bus:self.bus.close();self.bus=None
        self.emergency_latched=False;self.last_error=None
        self.motion=self.mission=None;self.beat_started=False;self.snare_preparing=False
        self.capture_folder=self.session/('capture-'+uuid.uuid4().hex[:8])
        if self.hihat_calibration is not None and self.hihat_calibration.closed:
            self.hihat_calibration=HiHatCalibrationRuntime(self)
        self._spawn('CAPTURING',[str(PYTHON),'-m','camera_playback.generated_acquisition','--folder',str(self.capture_folder)],
                    self.session/('acquisition-job-'+uuid.uuid4().hex[:8]),timeout=60,allow_can=True)

    def _poll_job(self):
        if self.job is None:return
        code=self.job.poll()
        if code is None:return
        kind=self.job_kind;folder=self.job.folder;cancelled=self.job.cancelled
        self.job=None;self.job_kind=None
        if cancelled:return
        if code:
            output=self.capture_folder if kind=='CAPTURING' else self.plan_folder if kind=='PLANNING' else folder
            error=output/'error.json'
            raise RuntimeError(json.loads(error.read_text())['error'] if error.exists() else 'Worker failed; see '+str(folder/'worker.log'))
        if kind=='CAPTURING':
            state=json.loads((self.capture_folder/'measured_start.json').read_text())
            rest={s:state[s+'_rad'] for s in ('left','right')}
            self.bus=GeneratedMotors.from_owner(Motors('right',True),rest)
            self.plan_folder=self.session/('plan-'+uuid.uuid4().hex[:8])
            self._spawn('PLANNING',[str(PYTHON),'-m','camera_playback.generated_planning','prepare','--folder',str(self.plan_folder),
                        '--capture',str(self.capture_folder)],self.session/('planning-job-'+uuid.uuid4().hex[:8]),timeout=300)
        elif kind=='PLANNING':self._load_mission(self.plan_folder/'mission.json')
        elif kind=='RETURN':
            result=json.loads((folder/'return.json').read_text())
            self.motion.accept_return(result);self.stage='MOVING'
            self.status.set('Playing calculated RIGHT → LEFT return to captured rest.')
            print('RETURN READY: '+result['source']+'; full scene/zone checks passed.',flush=True)

    def _load_mission(self,path):
        from .generated_planning import load
        self.mission,self.curves=load(path);self.mission_path=Path(path)
        if 'return_plan' not in self.mission:raise ValueError('Mission predates prepared returns; capture/calculate again')
        rest={s:self.mission['original'][s+'_rad'] for s in self.curves}
        if self.simulate_mission:
            from .generated_simulation import SimulatedGeneratedMotors
            self.bus=SimulatedGeneratedMotors(rest,streaming=True)
        self.motion=Motion(self.bus,self.curves)
        self.emergency_latched=False;self.last_error=None
        self.hybrid_session=None;self.hybrid_stopping=self.hybrid_restoring=False
        self.snare_preparing=False;self.return_requested=False;self.control=self.setup=None
        # Envelope membership uses the ORIGINAL URDF FK, exactly as the native
        # planner does. Joint bounds come from each calibrated generated model.
        geometries={}
        for side in ('left','right'):
            g=Geometry.__new__(Geometry);g.model=self.model;g.side=side;g.center=np.asarray(rest[side])
            g.zone=Zone.load(ROOT/(side+'_zones')/('zone3.json' if side=='right' else 'zone1.json'),self.model.digest,
                             RIGHT_TCP if side=='right' else LEFT_TCP)
            root=ET.parse(Path(self.mission['plans'][side]).parent/'model/robot.urdf').getroot()
            bounds=[root.find(f"joint[@name='openarmx_{side}_joint{i}']/limit") for i in range(1,8)]
            g.lower=np.array([float(b.get('lower')) for b in bounds]);g.upper=np.array([float(b.get('upper')) for b in bounds])
            geometries[side]=g
        self.geometry=geometries;g=geometries['right']
        self.bus.stream_geometry=geometries;self.bus.stream_directory=self.session
        self.zone=g.zone;self.lower=g.lower;self.upper=g.upper
        self.hill_ik.zone=self.zone
        self.dual=GeneratedDual(self,self.curves['left'],geometries['left'])
        snare_workflow.preflight(self.dual);snare_workflow.check_tempo(self)
        from .strike import build_strike_plan
        build_strike_plan(self.model,self.zone,self.lower,self.upper,self.curves['right'].at(self.curves['right'].duration))
        self.stage='READY';self.phase='READY';self.beat_started=False
        self.recording_status.set('GENERATED: left saved destination '+f"{self.curves['left'].duration:.3f}s → right DT1 {self.curves['right'].duration:.3f}s. No recorded paths.")
        self.status.set('Outbound AND right→left return checked and prepared. Preview, then Start. Stop/Relax is always available.')
        self.preview_button.config(state='normal')
        self.result_status.set('Exact paths: isolated 200 Hz playback; unchanged timing and motor settings.')
        self.zone_status.set('Zones: original right zone3 / left zone1; full arm and stick collision checks retained')
        for widget in self.root.winfo_children():
            if isinstance(widget,tk.Label) and str(widget.cget('text')).startswith('RViz:'):
                widget.config(text='3D views: drag to rotate · middle drag to pan · wheel to zoom')
        if self.auto_preview:self.preview()

    def _ready(self):
        calibration=self.hihat_calibration
        calibration_ok=bool(calibration and not calibration.closed and
                            not getattr(getattr(calibration,'engine',None),'failure',None))
        return bool(not getattr(self,'emergency_latched',False) and self.motion and self.motion.stage=='READY' and self.job is None and self.bus.fresh()
                    and (self.simulate_mission or (calibration_ok
                         and self._sound_ready_for_start() and self._hihat_ready_for_start())))

    def _refresh_buttons(self):
        if not getattr(self,'initialized',False):return
        self._refresh_tempo_controls()
        self.continue_button.config(state='normal' if self._ready() else 'disabled')
        self.emergency_button.config(state='normal')
        self.center_relax_button.config(state='normal',text='RETURN: RIGHT → LEFT + RELAX')
        self.emergency_relax_button.config(state='normal');self.stop_button.config(state='normal')
        self.recalculate_button.config(state='normal' if not (self.bus and self.bus.active) and self.job is None else 'disabled')
        self.preview_button.config(state='normal' if self.mission else 'disabled')

    def start(self):
        try:
            if not self._ready():raise RuntimeError('Wait for generated paths, fresh encoders, sound and a healthy hi-hat controller')
            from .generated_planning import verify
            verify(self.mission_path)  # Exact law was validated when installed; only identities on Start.
            self.bus.poll()
            self._apply_selected_tempo();self._apply_selected_bass_drum();snare_workflow.check_tempo(self)
            self.motion.start();self.stage='MOVING';self.gripper_closed_latched=True
        except Exception as exc:self.status.set(str(exc))

    def _begin_generated_beat(self):
        if self.simulate_mission:
            self.status.set('SOFTWARE: both generated goals reached. Hardware ride/snare calibration NOT run.');return
        self._goal_feedback()
        if not self.hihat_calibration.ready:
            self.phase='GENERATED GOALS — WAITING FOR HI-HAT CALIBRATION'
            self.status.set('Both generated goals reached; holding while startup hi-hat calibration finishes. No pink-zone alignment.');return
        if not self.snare_preparing:
            self.bus.handoff_goal=self.curves['left'].at(self.curves['left'].duration)
            self.dual.snare_arming_at=time.monotonic();self.snare_preparing=True
            self.phase='LEFT SNARE ARMING AT GENERATED GOAL'
        if not snare_workflow.arm(self.dual):return
        self.beat_started=True;self.stage='BEAT'
        self._begin_prime_strikes(self.arm().copy(),'GENERATED PRIME REACHED — starting existing ride J7 acoustic depth calibration; no pink-zone adjustment')

    def _goal_feedback(self):
        # Only the worker doing its one confirmed CSP -> MIT setup may briefly
        # report disabled. All other arm/gripper drives remain supervised.
        configuring=[]
        left=getattr(self.bus,'left_joint6_session',None)
        if self.snare_preparing and left is not None and not left.status.armed:
            configuring.append(('left',6))
        right=getattr(self.bus,'right_joint7_session',None)
        if right is not None and right.status.phase in ('starting','arming'):
            configuring.append(('right',7))
        return self.motion.fresh(configuring=configuring)

    def tick(self):
        if not getattr(self,'initialized',False):
            self.root.after(20,self.tick);return
        try:
            self._poll_job()
            if self.bus:
                self.bus.poll()
                if self.bus.fresh():self.q=self.bus.positions()
                if self.bus.fresh() and time.monotonic()-self.last_display>=.1:
                    self.last_display=time.monotonic()
                    atomic_json(self.session/'display.json',dict(joints=self.q,stamp=self.last_display,
                        label=('SIMULATED' if self.simulate_mission else 'MEASURED')+' arm poses — '+self.stage))
                if self.motion and not getattr(self,'emergency_latched',False):
                    # Retain the planner's bounds and original envelope checks.
                    if self.bus.active:
                        for side,g in self.geometry.items():
                            q=self.bus.positions_for(side)
                            lower=g.lower.copy()
                            if side=='right':lower[1]=np.radians(-1.)  # feedback floor, NOT a planned target
                            if np.any(q<lower-.001151) or np.any(q>g.upper+.001151):raise RuntimeError(side+' joint outside feedback limits')
                            if not g.zone.contains(g.tcp(q),.005):raise RuntimeError(side+' TCP outside its original zone')
                    if self.motion.stage=='WAIT PRIME':
                        self._finish_for_return()
                    elif self.return_requested and self.motion.stage=='HOLD':
                        self._service_return()
                    elif self.beat_started:
                        self._goal_feedback()
                        App.extra_control(self,time.monotonic())
                    else:
                        self.motion.step()
                        if self.motion.stage=='ARRIVED':self._begin_generated_beat()
                        elif self.motion.stage=='DONE':self.stage='DONE';self.status.set('Both arms returned to captured rest; disabled feedback confirmed.')
                        elif self.motion.stage not in ('READY','HOLD','LOADING PREPARED RETURN','RELAXED'):
                            self.status.set(self.motion.stage+' — generated paths, no centering')
                    # Read worker statuses every tick to keep owner heartbeats
                    # alive during Hold/return, without invoking strike logic.
                    if not self.beat_started:
                        for side,motor in (('right',7),('left',6)):
                            worker=getattr(self.bus,f'{side}_joint{motor}_session',None)
                            if worker is not None:
                                status=worker.status
                                if status.error and not (self.return_requested and self.motion.stage=='HOLD'):
                                    raise RuntimeError(side+' worker: '+status.error)
                self.publish();self.angles.set(self.encoder_text())
            self._refresh_buttons()
            rclpy.spin_once(self.node,timeout_sec=0)
        except Exception as exc:self.fail(str(exc))
        finally:
            if self.bus and not self.beat_started:
                for side,motor in (('right',7),('left',6)):
                    worker=getattr(self.bus,f'{side}_joint{motor}_session',None)
                    if worker is not None:
                        try:worker.status  # Keep healthy held owners alive even after a separate GUI/pose fault.
                        except Exception:pass
        if not self.closed:self.root.after(20,self.tick)

    def emergency_hold(self,message='Operator Emergency Hold'):
        if not getattr(self,'initialized',False):return
        if getattr(self,'emergency_latched',False):return  # Explicit torque-off must never be undone by a late callback.
        errors=[]
        if self.job:
            try:self.job.cancel()
            except Exception as exc:errors.append('calculation cancellation: '+str(exc))
        self.return_requested=False
        self.beat_started=False;self.snare_preparing=False
        self.control=self.setup=None;self.strike_active=False;self.continuous_strike_active=False
        self.strike_sound_deadline=None;self.alignment_active=False
        if self.motion:
            try:self.motion.stop_stream()
            except Exception as exc:errors.append('path writer stop: '+str(exc))
        if self.bus:
            # Priority cancellation does not depend on a fresh GUI pose. Each
            # MIT owner captures its own current feedback on its next tick.
            for side,motor in (('right',7),('left',6)):
                worker=getattr(self.bus,f'{side}_joint{motor}_session',None)
                if worker is not None:
                    try:worker.emergency_hold()
                    except Exception as exc:errors.append(side+' worker hold request failed: '+str(exc))
        # Peripheral/cleanup faults cannot veto either arm's Hold/Return.
        errors.extend(self._stop_peripherals())
        if self.motion:message=self.motion.hold(message)
        self.stage='HOLD';self.phase='GENERATED HOLD'
        self.status.set(str(message)+('; '+ '; '.join(errors) if errors else '')+' — no automatic return or arm relaxation')
        print(self.status.get(),flush=True)
        self._refresh_buttons()

    def return_to_rest(self):
        try:
            if getattr(self,'emergency_latched',False):
                self.status.set('Emergency Relax already requested. Capture again for another movement.');return
            if not self.motion or not self.bus.active:
                self.status.set('No powered generated movement to return.');return
            if self.job is not None:
                if self.job_kind=='RETURN':return  # Already calculating this explicit request.
                self.job.cancel()
            if self.motion.stage in ('RETURN','SETTLE RETURN','SETTLE REST','RELAXING'):return
            if self.motion.stage=='WAIT PRIME':return
            self.bus.return_recovery_attempted=set()
            if self.beat_started:
                # A normal Return finishes the existing strokes at their prime
                # anchors, rather than freezing mid-contact and replanning.
                self.beat_started=False;self.strike_active=self.continuous_strike_active=False
                self.strike_sound_deadline=None;self.control=self.setup=None
                self._stop_peripherals()
                for side,motor in (('right',7),('left',6)):
                    worker=getattr(self.bus,f'{side}_joint{motor}_session',None)
                    if worker is not None:
                        if side=='right':worker.request('finish')
                        else:worker.finish()
                self.motion.emit('WAIT PRIME');self.stage='RETURNING TO PRIME'
                self.return_requested=True
                self.status.set('Finishing the current strokes, then using the prepared RIGHT → LEFT return. No planning.');return
            if self.motion.stage not in ('HOLD',):
                self.emergency_hold('Return requested: using the return prepared before Start')
            self.return_requested=True
            self.motion.pending_worker_hold=True
            self.status.set('Return requested — recovering feedback/holding, then RIGHT → LEFT to captured rest. Hold cancels.')
        except Exception as exc:
            if self.bus and self.bus.active:self.emergency_hold('Return not started: '+str(exc))
            self.return_requested=bool(self.motion and self.bus and self.bus.active)
            self.status.set(str(exc)+'; recovering Hold for the requested calculated return. Hold cancels.')

    def _service_return(self):
        """One click survives transient feedback loss; never bypass live evidence."""
        if self.job is not None:return  # Cancellation is collected without another button press.
        recover=getattr(self.bus,'recover_return_workers',None)
        if recover is not None:
            try:ready=recover()
            except Exception:
                self.return_requested=False;raise  # Genuine failed owner: keep Hold, permit a new explicit retry.
            if not ready:
                self.status.set('RETURN: restoring powered strike-joint Hold; no path motion yet. Hold cancels.');return
        self.motion.step()
        if self.motion.pending_worker_hold or not self.bus.settled(self.motion.targets):
            self.status.set('RETURN: waiting for fresh stationary Hold, then calculated RIGHT → LEFT. Hold cancels.');return
        self._launch_return()

    def _finish_for_return(self):
        self._goal_feedback()
        goals={s:c.at(c.duration) for s,c in self.curves.items()}
        for side,motor in (('right',7),('left',6)):
            worker=getattr(self.bus,f'{side}_joint{motor}_session',None)
            if worker is not None:
                status=worker.status
                if status.error:raise RuntimeError(side+' strike return: '+status.error)
                if not status.ready:break
        else:
            if self.bus.settled(goals):
                self.emergency_hold('Prime poses reached; loading prepared RIGHT → LEFT return')
                self.return_requested=True;return
        if time.monotonic()-self.motion.at>8.:raise RuntimeError('Strike did not settle at prime; Hold or Emergency Relax remains available')

    def _launch_return(self):
        from .generated_planning import cached_return
        data=self.motion.return_input()
        try:result=cached_return(self.mission,self.curves,**data)
        except (ValueError,KeyError) as exc:
            # No guessed center and no Emergency Relax dead end. Same planner,
            # held encoder start, original rest and frozen scene, in a child.
            folder=self.session/('return-'+uuid.uuid4().hex[:8]);folder.mkdir()
            atomic_json(folder/'input.json',dict(mission=str(self.mission_path),**data))
            print('RETURN: cached route unavailable ('+str(exc)+'); calculating and checking from measured Hold.',flush=True)
            self._spawn('RETURN',[str(PYTHON),'-m','camera_playback.generated_planning','return',
                '--folder',str(folder),'--input',str(folder/'input.json')],folder,timeout=300)
            self.status.set('Calculating checked return from measured Hold to captured rest; both arms remain powered. Hold cancels.')
            self.return_requested=False;return
        self.motion.accept_return(result);self.stage='MOVING';self.return_requested=False
        print('USING PREPARED RETURN: RIGHT → LEFT; no new path search or collision worker.',flush=True)

    def center_relax(self):self.return_to_rest()
    def control_description(self):return 'GENERATED LEFT → RIGHT · NO CENTERING'
    def _refresh_tempo_controls(self):
        result=super()._refresh_tempo_controls()
        if getattr(self,'initialized',False) and getattr(self,'tempo_hint',None):
            self.tempo_hint.config(text=self.tempo_hint.cget('text').replace('Center + Relax','Return to rest'))
        return result
    def relax(self,message=None):self.emergency_hold(message or 'Hold requested')
    def _stop_peripherals(self):
        errors=[]
        calibration=getattr(self,'hihat_calibration',None)
        if calibration is not None:
            try:calibration.close()
            except Exception as exc:errors.append('calibration cleanup: '+str(exc))
        for name in ('hihat_sync','snare_sync'):
            sync=getattr(self,name,None)
            if sync is not None:
                try:
                    if sync.after_id is not None:self.root.after_cancel(sync.after_id);sync.after_id=None
                    sync.bass_drum=None
                    if sync.engine is not None:sync.engine.active=False
                    if sync.stream is not None:sync.stream.close();sync.stream=None
                except Exception as exc:errors.append(name+': '+str(exc))
        if self.hihat:
            try:self.hihat.release_all()
            except Exception as exc:errors.append('ESP32 release: '+str(exc))
        return errors

    def emergency_relax(self,message='Operator Emergency Relax'):
        """Explicit user stop: no pose, freshness, plan or peripheral gate."""
        if not getattr(self,'initialized',False):return
        self.emergency_latched=True;self.return_requested=False;self.beat_started=False;self.snare_preparing=False
        self.control=self.setup=None;self.strike_active=self.continuous_strike_active=False
        self.strike_sound_deadline=None;self.alignment_active=False
        self.hybrid_stopping=self.hybrid_restoring=False
        errors=[]
        if self.motion:
            if self.motion.setup is not None:
                try:self.motion.setup.close()
                except Exception as exc:errors.append(str(exc))
                self.motion.setup=None
            self.motion.emit('RELAXED')
        # Disable independently of peripheral errors, missing feedback or pose.
        if self.bus:
            try:self.bus.emergency_release()
            except Exception as exc:errors.append(str(exc))
        if self.job:
            try:self.job.cancel()
            except Exception as exc:errors.append('calculation cancellation: '+str(exc))
        errors.extend(self._stop_peripherals())
        self.stage=self.phase='EMERGENCY RELAX'
        detail=('Disable/release unconfirmed: '+'; '.join(errors)+'. Physical power cutoff may be needed.' if errors
                else 'Disable sent to BOTH arms/grippers and ESP32. No position check. Capture again for another run.')
        self.status.set(str(message)+' — '+detail);print(self.status.get(),flush=True)
        self._refresh_buttons()
    def _program_failure(self,message):self.fail(message)
    def fail(self,message):
        if getattr(self,'emergency_latched',False):
            self.status.set('Emergency Relax requested; feedback: '+str(message)+'. Stop/Close remains available.');return
        pending=bool(getattr(self,'return_requested',False))
        # Deduplicate a *held* fault, not a later failure during a retried
        # movement. The same encoder message must stop each new writer too.
        if str(message)!=self.last_error or (self.motion is not None and self.motion.stage!='HOLD'):
            if getattr(self,'session',None):
                try:
                    now=time.monotonic()
                    atomic_json(self.session/'last_fault.json',dict(error=str(message),at=now,
                        stage=getattr(self.motion,'stage',None),return_requested=pending,
                        feedback={f'{s}_J{i}':dict(position=v[0],state=v[1],age_s=now-v[2])
                                  for (s,i),v in getattr(self.bus,'states',{}).items()}))
                except Exception:pass  # Diagnostics must not veto Hold/Return.
            self.last_error=str(message);self.emergency_hold('FAULT: '+str(message))
        else:self.status.set('FAULT: '+str(message)+'; Hold/Return remain available. No automatic relaxation.')
        if pending:
            self.return_requested=True
            self.status.set('RETURN pending: '+str(message)+'; holding/reacquiring feedback, no guessed motion. Hold cancels.')
    def safety(self):pass  # tick performs mode-specific checks, never legacy auto-centering.
    def camera_tick(self):
        if getattr(self,'initialized',False):self.camera_status.set('Camera: one frozen paired snapshot; no live pink-zone tracking')
        if not getattr(self,'closed',False):self.root.after(100,self.camera_tick)
    def sound_tick(self):
        if getattr(self,'simulate_mission',None):
            self.sound_status.set('SOFTWARE ONLY — microphone/audio hardware is OFF')
            if not self.closed:self.root.after(100,self.sound_tick)
        else:super().sound_tick()
    def hihat_tick(self):
        if getattr(self,'simulate_mission',None):
            self.hihat_status.set('SOFTWARE ONLY — ESP32 is OFF; no hi-hat calibration')
            if not self.closed:self.root.after(100,self.hihat_tick)
        else:super().hihat_tick()
    def _camera_ready_for_start(self,now=None):return self.mission is not None
    def _hihat_required_now(self):return self.beat_started
    def _sound_required_now(self):return self.beat_started
    def _request_continuous_stop(self,detail='Beat stop requested'):self.emergency_hold(detail)
    def request_safe_close(self):
        try:self.emergency_relax('Program stopped by operator')
        finally:self.closed=True;self.root.quit()  # Never trap the operator behind a pose or feedback error.
    def preview(self):
        if not self.mission:return
        from .generated_preview import open_preview
        open_preview(self.mission_path,self.session)
    def run(self):
        try:
            while not self.closed:
                try:self.root.mainloop()
                except Exception as exc:self.fail('GUI loop: '+str(exc))
                if not self.closed:self.request_safe_close()
        finally:
            if self.job:self.job.close()
            if self.bus:
                try:self.bus.close()
                except Exception as exc:print('Owner cleanup: '+str(exc),flush=True)
            from .generated_preview import close_previews
            close_previews()
            self._cancel_planning();self.planner_executor.shutdown(wait=False,cancel_futures=True)
            self.audio_receiver.close();self.receiver.close()
            if self.snare_audio_receiver:self.snare_audio_receiver.close()
            self.hihat_sound_monitor.close()
            if self.hihat_calibration:self.hihat_calibration.close()
            if self.hihat:self.hihat.release_all();self.hihat.close()
            self.root.destroy();self.node.destroy_node()
