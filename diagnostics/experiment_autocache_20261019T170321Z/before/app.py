"""Single-session experiment controller and native frozen-scene playback UI."""
import json,time,uuid
from collections import deque
from pathlib import Path
from ..common import ROOT,DEFAULT_CALIBRATION,atomic_json,digest
from .config import METHODS,DESCRIPTIONS,BOUND_FIELDS,PRESETS,settings,budget,latest_state,no_can
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
    state=Path(state).resolve() if state else latest_state()
    print('SOFTWARE EXPERIMENT — saved arm state, NO CAN / NO MOTOR COMMANDS\nSaved pose: '+str(state),flush=True)
    print('The physical pose is NOT measured. All modes use one frozen scene and record3 FINAL pose only.',flush=True)
    wait(Job(command('build',folder),folder/'setup',timeout=240))
    if scene is None:
        capture=folder/'capture_000';wait(Job(command('capture',capture,calibration=calibration),folder/'capture_job',timeout=35))
        scene=capture/'camera/scene.json'
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
        def __init__(self,folder,params=None,seconds=60.,method='clearance',autoplay=False):
            self.session=Path(folder);self.context=json.loads((self.session/'context.json').read_text())
            self.params=settings(params);self.planning_seconds=budget(seconds);self.job=None;self.job_kind=None
            self.job_target=None;self.cache={};self.autoplay_result=autoplay;self.events=deque(maxlen=256);self.generation_count=0
            self.active_method=method;self.displayed_method=None;self.scene_generation=0
            plan=PlanDisplay(self.session/'initial/plan.json')
            super().__init__(calibration(self.context['calibration']),self.session,plan)
            packet,mesh=load(plan,self.context['calibration']);self.set_scene(packet,mesh)
            Qt=self.QtWidgets;row=Qt.QHBoxLayout()
            self.method_box=Qt.QComboBox()
            for key,title in METHODS.items():self.method_box.addItem(title,key)
            self.method_box.setCurrentIndex(list(METHODS).index(method));row.addWidget(self.method_box,1)
            self.generate_button=Qt.QPushButton('Generate');row.addWidget(self.generate_button)
            self.cancel_button=Qt.QPushButton('Cancel');self.cancel_button.setEnabled(False);row.addWidget(self.cancel_button)
            row.addWidget(Qt.QLabel('Search budget:'));self.budget_spin=Qt.QSpinBox();self.budget_spin.setRange(5,600)
            self.budget_spin.setValue(int(seconds));self.budget_spin.setSuffix(' s');row.addWidget(self.budget_spin)
            self.settings_button=Qt.QPushButton('Uncertainty assumptions…');row.addWidget(self.settings_button)
            self.recapture_button=Qt.QPushButton('Recapture scene');row.addWidget(self.recapture_button)
            self.window.layout().insertLayout(2,row)
            self.method_description=Qt.QLabel(DESCRIPTIONS[method]);self.method_description.setWordWrap(True)
            self.window.layout().insertWidget(3,self.method_description)
            self.generate_button.clicked.connect(self.generate)
            self.cancel_button.clicked.connect(self.cancel_job)
            self.method_box.currentIndexChanged.connect(self.select_method)
            self.budget_spin.valueChanged.connect(self.budget_changed)
            self.settings_button.clicked.connect(self.edit_settings)
            self.recapture_button.clicked.connect(self.recapture)
            self.window.setWindowTitle('X5 planner experiments — frozen Gemini scene / SIMULATION ONLY')
            self.banner.setText('PLANNER EXPERIMENTS  •  SAVED ARM POSES  •  FROZEN CAMERA SCENE  •  NO CAN / NO MOTORS')
            self.info.setText('Both arms use saved, NOT live measurements.\nOnly record3’s FINAL pose supplies the target.\n'
                'Camera is OFF except explicit Recapture scene.\nSelect a method → Generate → Play animation.')
            self.note.setText('SOFTWARE EXPERIMENTS ONLY. Uncertainties are editable assumptions, not measured safety probabilities. '
                              'Original full-arm + stick collision checks remain. Unobserved space is not certified free.')
            self.status.setText('READY — choose a method; scene is frozen, camera and CAN are closed')
            self.plan_label.setText('No generated trajectory yet. Saved resting pose shown.')
            self.frame_count=5
            self.save_status.setText('Frozen planning scene: '+self.context['scene'])
            for button in self.window.findChildren(Qt.QPushButton):
                if button.text()=='Go to goal':
                    button.clicked.disconnect()
                    button.clicked.connect(lambda:self.seek(float(self.plan.obj['outward'][-1]['time_s'])) if self.plan.playable else None)

        def budget_changed(self,value):
            self.planning_seconds=float(value)

        def signature(self):return (self.context['scene_sha256'],self.context['state_sha256'],json.dumps(self.params,sort_keys=True),self.planning_seconds)

        def locked(self,active):
            for widget in (self.method_box,self.generate_button,self.budget_spin,self.settings_button,self.recapture_button):widget.setEnabled(not active)
            self.cancel_button.setEnabled(active)
            self.play_button.setEnabled(not active and self.plan.playable)
            self.timeline.setEnabled(not active and self.plan.playable)
            for button in self.window.findChildren(self.QtWidgets.QPushButton):
                if button.text()=='Go to goal':button.setEnabled(not active and self.plan.playable)
            if active:self.clock.pause()

        def toggle_play(self):
            if self.job:return
            super().toggle_play()

        def select_method(self,*_):
            if self.job:return
            self.active_method=self.method_box.currentData();self.method_description.setText(DESCRIPTIONS[self.active_method])
            saved=self.cache.get(self.active_method)
            if saved and saved[0]==self.signature():
                print('[experiment] Loading cached result for '+self.active_method+' (same scene, state and settings).',flush=True)
                self.install_plan(saved[1],self.active_method)
            else:self.generate()

        def install_plan(self,path,method):
            plan=PlanDisplay(path);plan.check_live_calibration(self.context['calibration'])
            if plan.obj['provenance']['scene']!=self.context['scene']:
                raise ValueError('Cannot play a plan against a different scene')
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
                m=selected['metrics'];extra=f" • transit clearance ≈{m['transit_min_clearance_m']*1000:.0f} mm • outward {selected['duration_s']:.2f}s"
                if 'risk' in selected:
                    risk=selected['risk'];extra+=f" • assumed-scenario failures {risk['failed_scenarios']}/{risk['scenarios']} • CVaR {risk['cvar']:.3f}"
                self.plan_label.setText(self.plan_label.text()+extra)
            self.draw_canvas()
            if plan.playable and self.autoplay_result:self.toggle_play()
            self.events.append(dict(event='installed',method=method,status=plan.obj['status']))

        def generate(self):
            if self.job:return
            self.active_method=self.method_box.currentData();self.generation_count+=1
            self.install_plan(self.session/'initial/plan.json',None)
            self.locked(True);self.status.setText('GENERATING '+METHODS[self.active_method]+' — progress is in the terminal')
            self.job_target,self.job=start_plan(self.session,self.active_method,self.params,self.planning_seconds)
            self.job_kind='plan';self.events.append(dict(event='generate',method=self.active_method))

        def cancel_job(self):
            if self.job:
                self.job.cancel();self.status.setText('Cancelling the contained worker; no motors are connected…')
                self.events.append(dict(event='cancel',kind=self.job_kind))

        def recapture(self):
            if self.job:return
            self.clock.pause();self.locked(True)
            self.job_target=self.session/('capture_'+uuid.uuid4().hex[:8])
            self.job=Job(command('capture',self.job_target,calibration=self.context['calibration']),
                         self.job_target/'job',timeout=35)
            self.job_kind='capture';self.status.setText('CAPTURING NEW SCENE — saved arm pose remains unchanged; no CAN')
            self.events.append(dict(event='recapture'))

        def edit_settings(self):
            Qt=self.QtWidgets;dialog=Qt.QDialog(self.window);dialog.setWindowTitle('Experimental uncertainty assumptions — NOT measured')
            layout=Qt.QVBoxLayout(dialog);note=Qt.QLabel('Bounds are additional to the existing collision buffer.\n'
                'Robust uses worst-case envelopes; scenarios use bounded uniform perturbations.\n'
                'These values are NOT fitted hardware error rates. Unseen obstacles remain unmodeled.')
            note.setWordWrap(True);layout.addWidget(note);preset=Qt.QComboBox();preset.addItems(['Custom',*PRESETS]);layout.addWidget(preset)
            form=Qt.QFormLayout();fields={}
            for key,(lo,hi) in BOUND_FIELDS.items():
                spin=Qt.QDoubleSpinBox();spin.setRange(lo,hi);spin.setDecimals(3);spin.setValue(self.params[key]);fields[key]=spin
                form.addRow(key.replace('_',' '),spin)
            scenarios=Qt.QSpinBox();scenarios.setRange(16,512);scenarios.setValue(self.params['scenarios']);fields['scenarios']=scenarios;form.addRow('Monte Carlo scenarios',scenarios)
            for key in ('chance_limit','cvar_alpha'):
                spin=Qt.QDoubleSpinBox();spin.setDecimals(3);spin.setRange(.001 if key=='chance_limit' else .5,.5 if key=='chance_limit' else .99)
                spin.setValue(self.params[key]);fields[key]=spin;form.addRow(key.replace('_',' '),spin)
            seed=Qt.QSpinBox();seed.setRange(1,2**31-1);seed.setValue(self.params['seed']);fields['seed']=seed;form.addRow('Random seed',seed)
            def apply_preset(index):
                if index:
                    for key,value in PRESETS[preset.currentText()].items():fields[key].setValue(value)
            preset.currentIndexChanged.connect(apply_preset);layout.addLayout(form)
            buttons=Qt.QDialogButtonBox(Qt.QDialogButtonBox.Ok|Qt.QDialogButtonBox.Cancel);layout.addWidget(buttons)
            buttons.accepted.connect(dialog.accept);buttons.rejected.connect(dialog.reject)
            if dialog.exec_()==Qt.QDialog.Accepted:
                self.params=settings({key:field.value() for key,field in fields.items()})
                atomic_json(self.session/'uncertainty_settings.json',self.params)
                self.install_plan(self.session/'initial/plan.json',None)
                self.status.setText('Assumptions saved. Generate to apply them; old results are not silently reused.')
                self.events.append(dict(event='settings_changed'))

        def poll(self,camera=None):
            key=super().poll()
            if self.job and self.job.poll() is not None:
                job=self.job;kind=self.job_kind;target=self.job_target;self.job=None;job.close()
                self.locked(False)
                if job.cancelled:self.status.setText('CANCELLED — camera/CAN closed; no partial result installed')
                elif job.process.returncode:
                    self.status.setText('Worker stopped (timeout/memory/error). See '+str(job.log_path)+'. You can retry.')
                    self.events.append(dict(event='worker_stopped',kind=kind,log=str(job.log_path)))
                elif kind=='plan':
                    path=target/'plan.json';self.cache[self.active_method]=(self.signature(),path)
                    self.install_plan(path,self.active_method);self.status.setText('FINISHED — '+str(path))
                elif kind=='capture':
                    self.locked(True)
                    self.job_target=target
                    self.job=Job(command('initialize',target,state=self.context['state'],scene=target/'camera/scene.json',calibration=self.context['calibration']),target/'init_job',timeout=40)
                    self.job_kind='initialize'
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
                    self.install_plan(self.session/'initial/plan.json',None)
                    self.status.setText('NEW SCENE FROZEN — previous trajectories invalidated; select Generate')
                    self.save_status.setText('Frozen planning scene: '+self.context['scene'])
                    self.events.append(dict(event='scene_installed',generation=self.scene_generation))
            return key

        def report(self):
            report=super().report()
            report.update(experiment=True,saved_state_not_live=True,can_access=False,motor_commands=0,
                          context=self.context,events=list(self.events),generations=self.generation_count,
                          scene_generations=self.scene_generation,displayed_method=self.displayed_method)
            return report

        def close(self):
            if self.job:self.job.close();self.job=None
            super().close()
    return ExperimentView


def show(folder,params=None,budget_s=60,method='clearance',autoplay=False,seconds=0.):
    view=make_view_class()(folder,params,budget_s,method,autoplay);began=time.monotonic()
    try:
        view.generate()
        while not seconds or time.monotonic()-began<seconds:
            key=view.poll()
            if key==ord('q'):break
            if key==ord('s'):view.save_view()
            time.sleep(.01)
        return view.report()
    finally:
        view.close();atomic_json(Path(folder)/'experiment_view_report.json',view.report())
