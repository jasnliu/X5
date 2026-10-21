"""Single-session experiment controller and native frozen-scene playback UI."""
import json,time,uuid
from collections import deque
from pathlib import Path
from ..common import ROOT,DEFAULT_CALIBRATION,atomic_json,digest
from .config import METHODS,DESCRIPTIONS,BOUND_FIELDS,PRESETS,settings,budget,no_can,saved_inputs,DEFAULT_METHOD
from .runtime import Job,PYTHON


def command(action,folder,**values):
    args=[str(PYTHON),'-m','drum_planner.experiment.worker',action,'--folder',str(folder)]
    for key,value in values.items():
        if value is not None:args.extend(['--'+key.replace('_','-'),str(value)])
    return args


def wait(job):
    try:
        while job.poll() is None:time.sleep(.05)
        if job.cancelled:raise RuntimeError('Job cancelled')
        if job.process.returncode:raise RuntimeError('Contained worker stopped; see '+str(job.log_path))
    finally:job.close()


def prepare(folder,state=None,scene=None,calibration=DEFAULT_CALIBRATION):
    no_can();folder=Path(folder).resolve();folder.mkdir(parents=True,exist_ok=False)
    if not PYTHON.exists():raise ValueError('Missing experiment dependencies. Run ./setup_planner_experiment.sh')
    if state is None or scene is None:
        saved_state,saved_scene=saved_inputs()
        if state is None:state=saved_state
        if scene is None:scene=saved_scene
    state=Path(state).resolve()
    print('SOFTWARE EXPERIMENT — saved arm state, NO CAN / NO MOTOR COMMANDS\nSaved pose: '+str(state),flush=True)
    print('The physical pose is NOT measured. All modes use one frozen scene and record3 FINAL pose only.',flush=True)
    wait(Job(command('build',folder),folder/'setup',timeout=240))
    wait(Job(command('initialize',folder,state=state,scene=Path(scene).resolve(),calibration=calibration),folder/'initialize_job',timeout=40))
    return json.loads((folder/'context.json').read_text())


def start_plan(folder,method,params,seconds):
    target=Path(folder)/'plans'/(method+'-'+uuid.uuid4().hex[:8]);target.mkdir(parents=True)
    atomic_json(target/'settings.json',settings(params))
    job=Job(command('plan',target,context=Path(folder)/'context.json',method=method,
                    settings=target/'settings.json',budget=budget(seconds)),target,timeout=seconds+60)
    return target,job


def headless(folder,methods,params,seconds):
    outcomes={}
    for method in methods:
        target,job=start_plan(folder,method,params,seconds);wait(job)
        obj=json.loads((target/'plan.json').read_text());outcomes[method]=dict(status=obj['status'],plan=str(target/'plan.json'))
    atomic_json(Path(folder)/'results.json',outcomes);return outcomes


def make_view_class():
    # Delay Qt/VTK import until GUI mode: headless workers never load/render meshes.
    from ..snapshot_preview import SnapshotPlaybackView
    from ..trajectory_view import PlanDisplay,PlaybackClock
    from ..planned_view import load
    from ..common import calibration

    class ExperimentView(SnapshotPlaybackView):
        def __init__(self,folder,params=None,seconds=120.,method=DEFAULT_METHOD,autoplay=False,precompute=True):
            self.session=Path(folder);self.context=json.loads((self.session/'context.json').read_text())
            self.params=settings(params);self.planning_seconds=budget(seconds);self.job=None;self.job_kind=None
            self.job_target=None;self.cache={};self.autoplay_result=autoplay;self.events=deque(maxlen=256);self.generation_count=0
            self.active_method=method;self.displayed_method=None;self.scene_generation=0
            self.pending_methods=deque();self.method_states={key:'not queued' for key in METHODS}
            self.job_method=None;self.job_signature=None;self.closing=False
            plan=PlanDisplay(self.session/'initial/plan.json')
            super().__init__(calibration(self.context['calibration']),self.session,plan)
            packet,mesh=load(plan,self.context['calibration']);self.set_scene(packet,mesh)
            Qt=self.QtWidgets;row=Qt.QHBoxLayout()
            self.method_box=Qt.QComboBox()
            self.method_box.setMinimumContentsLength(24)
            self.method_box.setSizeAdjustPolicy(Qt.QComboBox.AdjustToMinimumContentsLengthWithIcon)
            for key,title in METHODS.items():self.method_box.addItem(title,key)
            self.method_box.setCurrentIndex(list(METHODS).index(method));row.addWidget(self.method_box,1)
            self.generate_button=Qt.QPushButton('Generate');row.addWidget(self.generate_button)
            self.cache_all_button=Qt.QPushButton('Cache all');row.addWidget(self.cache_all_button)
            self.cancel_button=Qt.QPushButton('Cancel');self.cancel_button.setEnabled(False);row.addWidget(self.cancel_button)
            row.addWidget(Qt.QLabel('Pool budget:'));self.budget_spin=Qt.QSpinBox();self.budget_spin.setRange(5,600)
            self.budget_spin.setValue(int(seconds));self.budget_spin.setSuffix(' s');row.addWidget(self.budget_spin)
            self.budget_spin.setKeyboardTracking(False)
            self.settings_button=Qt.QPushButton('Gesture settings…');row.addWidget(self.settings_button)
            self.recapture_button=Qt.QPushButton('Recapture scene');row.addWidget(self.recapture_button)
            self.window.layout().insertLayout(2,row)
            self.method_description=Qt.QLabel(DESCRIPTIONS[method]);self.method_description.setWordWrap(True)
            self.window.layout().insertWidget(3,self.method_description)
            self.queue_label=Qt.QLabel();self.queue_label.setWordWrap(True);self.window.layout().insertWidget(4,self.queue_label)
            self.generate_button.clicked.connect(self.generate)
            self.cache_all_button.clicked.connect(self.cache_all)
            self.cancel_button.clicked.connect(self.cancel_job)
            self.method_box.currentIndexChanged.connect(self.select_method)
            self.budget_spin.valueChanged.connect(self.budget_changed)
            self.settings_button.clicked.connect(self.edit_settings)
            self.recapture_button.clicked.connect(self.recapture)
            self.window.setWindowTitle('X5 gesture experiments — frozen Gemini scene / SIMULATION ONLY')
            self.banner.setText('GESTURE EXPERIMENTS  •  SAVED ARM POSES  •  FROZEN CAMERA SCENE  •  NO CAN / NO MOTORS')
            self.info.setText('Both arms use saved, NOT live measurements.\nOnly record3’s FINAL pose supplies the target.\n'
                'Camera is OFF except explicit Recapture scene.\nSurviving gesture methods cache automatically from a shared proposal pool.\nSelect a ready method → Play animation.')
            self.note.setText('SOFTWARE EXPERIMENTS ONLY. Endpoint-only gesture search; shared proposals, method-specific rankings and identical checks. No recording imitation. '
                              'Original full-arm + stick collision checks remain. Unobserved space is not certified free.')
            if self.context.get('right_zone'):
                self.note.setText(self.note.text()+' ZONE3: original right gripper TCP only, existing 5 mm membership tolerance; no left/stick zone restriction.')
            self.status.setText('READY — automatic cache uses the frozen scene; camera and CAN are closed')
            # Long cache/error paths must wrap, not widen the whole window off
            # the screen and make controls appear inaccessible.
            self.status.setWordWrap(True)
            self.plan_label.setText('No generated trajectory yet. Saved resting pose shown.')
            self.frame_count=5
            self.save_status.setText('Frozen planning scene: '+self.context['scene'])
            for button in self.window.findChildren(Qt.QPushButton):
                if button.text()=='Go to goal':
                    button.clicked.disconnect()
                    button.clicked.connect(lambda:self.seek(float(self.plan.obj['outward'][-1]['time_s'])) if self.plan.playable else None)
            self.update_queue_status()
            if precompute:self.cache_all()

        def budget_changed(self,value):
            if float(value)==self.planning_seconds:return
            self.planning_seconds=float(value)
            self.restart_cache('budget_changed')

        def signature(self):return (self.context['scene_sha256'],self.context['state_sha256'],json.dumps(self.params,sort_keys=True),
                                   self.context.get('right_zone',{}).get('constraint_sha256'),self.planning_seconds)

        def locked(self,active):
            # Planning runs separately from the displayed result. Keep cached
            # playback/selection usable while another method is calculating.
            blocked=active and self.job_kind!='plan'
            self.method_box.setEnabled(not blocked)
            for widget in (self.generate_button,self.cache_all_button,self.budget_spin,self.settings_button,self.recapture_button):widget.setEnabled(not active)
            self.cancel_button.setEnabled(active or bool(self.pending_methods))
            self.play_button.setEnabled(not blocked and self.plan.playable)
            self.timeline.setEnabled(not blocked and self.plan.playable)
            for button in self.window.findChildren(self.QtWidgets.QPushButton):
                if button.text()=='Go to goal':button.setEnabled(not blocked and self.plan.playable)
            if blocked:self.clock.pause()

        def toggle_play(self):
            if self.job and self.job_kind!='plan':return
            super().toggle_play()

        def cached(self,method):
            saved=self.cache.get(method)
            return saved if saved and saved[0]==self.signature() else None

        def update_queue_status(self):
            count=sum(bool(self.cached(method)) for method in METHODS)
            running=self.job_method if self.job and self.job_kind=='plan' else None
            failed=sum(state=='failed' for state in self.method_states.values())
            text=f'Cached {count}/{len(METHODS)} methods (includes no-feasible-route results)'
            if running:text+=' • calculating '+METHODS[running]
            text+=f' • {len(self.pending_methods)} queued'
            if failed:text+=f' • {failed} failed — select to retry'
            self.queue_label.setText(text)
            self.method_box.blockSignals(True)
            for index,method in enumerate(METHODS):
                self.method_box.setItemText(index,METHODS[method]+' ['+self.method_states[method]+']')
            self.method_box.blockSignals(False)

        def prioritize(self,method):
            if method in self.pending_methods:self.pending_methods.remove(method)
            self.pending_methods.appendleft(method);self.method_states[method]='queued'

        def cache_all(self):
            if self.closing:return
            if self.job and self.job_kind!='plan':return
            order=[self.active_method,*[m for m in METHODS if m!=self.active_method]]
            self.pending_methods.clear()
            for method in order:
                if self.cached(method) or (self.job and self.job_method==method):continue
                self.pending_methods.append(method);self.method_states[method]='queued'
            print('[experiment cache] Automatically queued: '+(', '.join(self.pending_methods) or 'all results already cached')+
                  '. One memory-limited worker at a time.',flush=True)
            self.events.append(dict(event='cache_all',methods=list(self.pending_methods)))
            self.start_next();self.update_queue_status()

        def start_next(self):
            if self.closing or self.job:return
            while self.pending_methods:
                method=self.pending_methods.popleft()
                if self.cached(method):continue
                signature=self.signature()
                try:target,job=start_plan(self.session,method,self.params,self.planning_seconds)
                except Exception as exc:
                    self.method_states[method]='failed'
                    print(f'[experiment cache] Could not start {method}: {exc}',flush=True)
                    self.events.append(dict(event='worker_start_failed',method=method,error=str(exc)))
                    continue
                self.job_target=target;self.job=job;self.job_kind='plan'
                # Never infer result ownership from the currently selected UI
                # method; selection can change while this worker runs.
                self.job_method=method;self.job_signature=signature
                self.method_states[method]='calculating';self.generation_count+=1
                self.locked(True)
                self.status.setText('CALCULATING '+METHODS[method]+' — cached methods remain playable; progress is in the terminal')
                print(f'[experiment cache] Starting {method}; {len(self.pending_methods)} other methods queued.',flush=True)
                self.events.append(dict(event='generate',method=method))
                break
            self.locked(bool(self.job));self.update_queue_status()

        def restart_cache(self,reason):
            # The corresponding controls are disabled until the current job
            # finishes or Cancel stops it. No stale result crosses a signature.
            self.cache.clear();self.pending_methods.clear()
            self.method_states={key:'not queued' for key in METHODS}
            self.install_plan(self.session/'initial/plan.json',None)
            self.events.append(dict(event=reason));self.cache_all()

        def select_method(self,*_):
            if self.job and self.job_kind!='plan':return
            self.active_method=self.method_box.currentData();self.method_description.setText(DESCRIPTIONS[self.active_method])
            saved=self.cached(self.active_method)
            if saved:
                print('[experiment] Loading cached result for '+self.active_method+' (same scene, state and settings).',flush=True)
                self.install_plan(saved[1],self.active_method)
            else:
                self.install_plan(self.session/'initial/plan.json',None)
                running=self.job and self.job_method==self.active_method and not self.job.cancelled
                if not running:self.prioritize(self.active_method)
                self.plan_label.setText(METHODS[self.active_method]+(' — calculating…' if running else ' — queued next; no selection is required to calculate other methods'))
                self.start_next()
            self.update_queue_status()

        def install_plan(self,path,method):
            plan=PlanDisplay(path);plan.check_live_calibration(self.context['calibration'])
            if plan.obj['provenance']['scene']!=self.context['scene']:
                raise ValueError('Cannot play a plan against a different scene')
            if self.context.get('right_zone') and plan.obj.get('right_zone_constraint')!=self.context['right_zone']:
                raise ValueError('Cannot play a cached plan without the matching right zone3 constraint')
            self.clock.pause();self.plan=plan;self.robot.plan=plan;self.clock=PlaybackClock(plan.duration)
            self.clock.set_rate([.5,1.,2.][self.speed.currentIndex()]);self.clock.loop=self.loop_box.isChecked()
            self.timeline.blockSignals(True);self.timeline.setRange(0,max(1,int(plan.duration*1000)))
            self.timeline.setValue(0);self.timeline.setEnabled(plan.playable);self.timeline.blockSignals(False)
            self.play_button.setEnabled(plan.playable)
            for b in self.window.findChildren(self.QtWidgets.QPushButton):
                if b.text()=='Go to goal':b.setEnabled(plan.playable)
            self.displayed_method=method
            self.focus_box=self.robot.focus_bounds()
            if method:
                self.active_method=method;self.method_box.blockSignals(True)
                self.method_box.setCurrentIndex(list(METHODS).index(method));self.method_box.blockSignals(False)
                self.method_description.setText(DESCRIPTIONS[method])
            self.plan_label.setText((METHODS[method]+' • ' if method else '')+
                ('VALIDATED SOFTWARE TRAJECTORY — Play animates only the model' if plan.playable else
                 'NO TRAJECTORY — '+str(plan.obj.get('details',{}).get('reason',plan.obj['status']))))
            report=plan.obj.get('experiment',{}).get('report',{})
            selected=next((c for c in report.get('candidates',[]) if c['index']==report.get('selected')),None)
            if selected:
                m=selected['gesture']
                extra=(f" • candidate {selected['index']} • score {selected['score']:.2f} (lower is better)"
                       f" • extra joint travel {m['joint_excess_deg']:.1f}° • wrist detour {m['wrist']['detour_ratio']:.2f}×"
                       f" • source {selected['source']} • outward {selected['duration_s']:.2f}s")
                self.plan_label.setText(self.plan_label.text()+extra)
            self.draw_canvas()
            self.locked(bool(self.job))
            if plan.playable and self.autoplay_result:self.toggle_play()
            self.events.append(dict(event='installed',method=method,status=plan.obj['status']))

        def generate(self):
            if self.job:return
            self.active_method=self.method_box.currentData();self.cache.pop(self.active_method,None)
            self.install_plan(self.session/'initial/plan.json',None)
            self.prioritize(self.active_method);self.start_next()

        def cancel_job(self):
            for method in self.pending_methods:self.method_states[method]='cancelled'
            self.pending_methods.clear()
            if self.job:
                if self.job_method:self.method_states[self.job_method]='cancelling'
                self.job.cancel();self.status.setText('Cancelling the contained worker; no motors are connected…')
                self.events.append(dict(event='cancel',kind=self.job_kind))
            self.update_queue_status()

        def recapture(self):
            if self.job:return
            self.pending_methods.clear();self.clock.pause()
            self.job_target=self.session/('capture_'+uuid.uuid4().hex[:8])
            self.job=Job(command('capture',self.job_target,calibration=self.context['calibration']),
                         self.job_target/'job',timeout=35)
            self.job_kind='capture';self.job_method=None;self.locked(True)
            self.status.setText('CAPTURING NEW SCENE — saved arm pose remains unchanged; no CAN')
            self.events.append(dict(event='recapture'))

        def edit_settings(self):
            Qt=self.QtWidgets;dialog=Qt.QDialog(self.window);dialog.setWindowTitle('Gesture preferences and shared candidate pool')
            layout=Qt.QVBoxLayout(dialog);note=Qt.QLabel('Methods share generated candidates, with their own refinement families.\n'
                'These are geometric preference scales, not physical safety margins.\n'
                'No recorded intermediate poses are used. Changing settings rebuilds the pool.')
            note.setWordWrap(True);layout.addWidget(note);preset=Qt.QComboBox();preset.addItems(['Custom',*PRESETS]);layout.addWidget(preset)
            form=Qt.QFormLayout();fields={}
            labels=dict(backtrack_weight='Extra joint-travel weight',overshoot_weight='Endpoint overshoot weight',
                        clearance_weight='Clearance-blend proximity weight',corridor_deg='Base endpoint corridor expansion (degrees)')
            for key,(lo,hi) in BOUND_FIELDS.items():
                spin=Qt.QDoubleSpinBox();spin.setRange(lo,hi);spin.setDecimals(3);spin.setValue(self.params[key]);fields[key]=spin
                spin.setObjectName(key);form.addRow(labels[key],spin)
            for key,lo,hi,label in [('candidates',2,24,'Compact corridor searches'),('seed',1,2**31-1,'Random seed')]:
                spin=Qt.QSpinBox();spin.setRange(lo,hi);spin.setValue(self.params[key]);fields[key]=spin
                spin.setObjectName(key);form.addRow(label,spin)
            def apply_preset(index):
                if index:
                    for key,value in PRESETS[preset.currentText()].items():fields[key].setValue(value)
            preset.currentIndexChanged.connect(apply_preset);layout.addLayout(form)
            buttons=Qt.QDialogButtonBox(Qt.QDialogButtonBox.Ok|Qt.QDialogButtonBox.Cancel);layout.addWidget(buttons)
            buttons.accepted.connect(dialog.accept);buttons.rejected.connect(dialog.reject)
            if dialog.exec_()==Qt.QDialog.Accepted:
                updated=settings({key:field.value() for key,field in fields.items()})
                if updated==self.params:return
                self.params=updated
                atomic_json(self.session/'gesture_settings.json',self.params)
                self.restart_cache('settings_changed')

        def poll(self,camera=None):
            key=super().poll()
            if self.job and self.job.poll() is not None:
                job=self.job;kind=self.job_kind;target=self.job_target
                method=self.job_method;signature=self.job_signature
                self.job=None;self.job_kind=None;self.job_method=None;self.job_signature=None;job.close()
                self.locked(False)
                if job.cancelled:
                    if method:self.method_states[method]='cancelled'
                    self.status.setText('CANCELLED — automatic queue stopped; cached results kept. Cache all resumes missing methods.')
                elif job.process.returncode:
                    if method:self.method_states[method]='failed'
                    self.status.setText('Worker stopped (timeout/memory/error). See '+str(job.log_path)+'. You can retry.')
                    self.events.append(dict(event='worker_stopped',kind=kind,method=method,log=str(job.log_path)))
                elif kind=='plan':
                    path=target/'plan.json'
                    try:
                        obj=json.loads(path.read_text())
                        if signature!=self.signature():raise ValueError('Input signature changed during generation; stale result discarded')
                        if obj['status'] not in ('planned','no_feasible_route'):raise ValueError('Worker returned '+obj['status'])
                        self.cache[method]=(signature,path)
                        self.method_states[method]='ready' if obj['status']=='planned' else 'no feasible route'
                        if method==self.active_method:self.install_plan(path,method)
                        print(f'[experiment cache] Cached {method}: {obj["status"]}.',flush=True)
                        self.events.append(dict(event='cached',method=method,status=obj['status'],plan=str(path)))
                        self.status.setText('CACHED '+METHODS[method]+' — '+str(path))
                    except Exception as exc:
                        self.cache.pop(method,None);self.method_states[method]='failed'
                        print(f'[experiment cache] Rejected result for {method}: {exc}',flush=True)
                        self.status.setText('Result rejected: '+str(exc))
                        self.events.append(dict(event='result_rejected',method=method,error=str(exc)))
                elif kind=='capture':
                    self.job_target=target
                    self.job=Job(command('initialize',target,state=self.context['state'],scene=target/'camera/scene.json',calibration=self.context['calibration']),target/'init_job',timeout=40)
                    self.job_kind='initialize';self.locked(True)
                elif kind=='initialize':
                    context=json.loads((target/'context.json').read_text())
                    plan=PlanDisplay(target/'initial/plan.json');packet,mesh=load(plan,context['calibration'])
                    # Replace only on successful complete capture. Drop old CPU
                    # packet, mesh, result cache and GPU surface before upload.
                    self.context=context;atomic_json(self.session/'context.json',context)
                    import shutil
                    # Initial file/model paths remain absolute and hash verified.
                    for name in ('plan.json','backend_request.json'):
                        shutil.copy2(target/'initial'/name,self.session/'initial'/name)
                    self.packet=None;self.mesh=None;self.mesh_builds=0;self.fit_needed=False
                    self.set_scene(packet,mesh);self.cache.clear();self.scene_generation+=1
                    self.save_status.setText('Frozen planning scene: '+self.context['scene'])
                    self.events.append(dict(event='scene_installed',generation=self.scene_generation))
                    self.restart_cache('scene_cache_restarted')
                self.start_next();self.update_queue_status()
            return key

        def report(self):
            report=super().report()
            report.update(experiment=True,saved_state_not_live=True,can_access=False,motor_commands=0,
                          context=self.context,events=list(self.events),generations=self.generation_count,
                          scene_generations=self.scene_generation,displayed_method=self.displayed_method,
                          automatic_precompute=True,queued_methods=list(self.pending_methods),calculating_method=self.job_method,
                          method_states=dict(self.method_states),cached_results={key:str(value[1]) for key,value in self.cache.items() if self.cached(key)})
            return report

        def close(self):
            self.closing=True;self.pending_methods.clear()
            if self.job:self.job.close();self.job=None
            self.job_kind=None;self.job_method=None
            super().close()
    return ExperimentView


def show(folder,params=None,budget_s=120,method=DEFAULT_METHOD,autoplay=False,seconds=0.):
    view=make_view_class()(folder,params,budget_s,method,autoplay);began=time.monotonic()
    try:
        while not seconds or time.monotonic()-began<seconds:
            key=view.poll()
            if key==ord('q'):break
            if key==ord('s'):view.save_view()
            time.sleep(.01)
        return view.report()
    finally:
        view.close();atomic_json(Path(folder)/'experiment_view_report.json',view.report())
