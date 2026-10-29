"""Standalone --fast mode and offline preview. Existing experiment UI is untouched."""
import argparse
from collections import deque
import json
import math
import time
import uuid
from pathlib import Path
from ..common import ROOT,DEFAULT_CALIBRATION,atomic_json,endpoint
from ..experiment import destinations
from ..experiment.runtime import Job,PYTHON


def resolve_destination(value,side):
    path=Path(value)
    if path.is_file():return path.resolve()
    choices=destinations.files(side)
    matches=[p for p in choices if value in (p.name,p.stem,p.stem.removeprefix('destination'))]
    if len(matches)!=1:raise ValueError(f'Unknown {side} destination: {value}')
    return matches[0]


def start_plan(session,paths,kind,compare,seconds):
    session=Path(session);context=json.loads((session/'context.json').read_text())
    if not 1<=len(paths)<=3:raise ValueError('Choose one to three destinations')
    if any(Path(a).resolve()==Path(b).resolve() for a,b in zip(paths,paths[1:])):
        raise ValueError('Consecutive destinations must differ')
    target=session/'plans'/uuid.uuid4().hex[:10];target.mkdir(parents=True)
    bindings=[destinations.freeze(p,context['moving_side'],target) for p in paths]
    config=dict(context=context,bindings=bindings,task_constraint=kind,compare=bool(compare),
                budget_s=float(seconds),output=str(target.resolve()),mode='fast',motion_authorized=False)
    atomic_json(target/'request.json',config)
    job=Job([PYTHON,'-m','drum_planner.fast_mode.worker','--request',target/'request.json'],target/'job',
            timeout=min(900,max(120,int(seconds*len(paths)*2+120))))
    return target,job


def make_view_class():
    from ..snapshot_preview import SnapshotPlaybackView
    from ..trajectory_view import PlaybackClock
    from ..experiment.checkpoints import ExperimentPlanDisplay
    from ..planned_view import load
    from ..common import calibration

    class FastView(SnapshotPlaybackView):
        def __init__(self,session,*,selected=(),kind='full-pose',compare=False,budget_s=45.,autoplay=False):
            self.session=Path(session);self.context=json.loads((self.session/'context.json').read_text())
            self.side=self.context['moving_side'];self.job=None;self.target=None;self.discard=False
            self.budget_s=budget_s;self.autoplay_result=autoplay;self.events=deque(maxlen=256);self.generation=0
            plan=ExperimentPlanDisplay(self.session/'initial/plan.json')
            super().__init__(calibration(self.context['calibration']),self.session,plan)
            packet,mesh=load(plan,self.context['calibration']);self.set_scene(packet,mesh)
            Qt=self.QtWidgets;row=Qt.QHBoxLayout();self.boxes=[]
            choices=destinations.files(self.side)
            for index in range(3):
                row.addWidget(Qt.QLabel('First target:' if index==0 else f'Then {index+1} (optional):'))
                box=Qt.QComboBox();box.setObjectName(f'fast_destination_{index+1}')
                box.addItem('Choose destination' if index==0 else 'None',None)
                for p in choices:box.addItem(p.stem,str(p))
                if index<len(selected):box.setCurrentIndex(box.findData(str(Path(selected[index]).resolve())))
                box.currentIndexChanged.connect(self.invalidate);row.addWidget(box);self.boxes.append(box)
            self.window.layout().insertLayout(3,row)
            row=Qt.QHBoxLayout();self.task=Qt.QComboBox();self.task.setObjectName('fast_task_constraint')
            self.task.addItem('Preserve full stick pose','full-pose')
            self.task.addItem('Preserve tip + stick direction; roll may change','tip-axis')
            self.task.setCurrentIndex(self.task.findData(kind));self.task.currentIndexChanged.connect(self.invalidate)
            row.addWidget(self.task,1)
            self.compare_box=Qt.QCheckBox('Compare original Workspace curve');self.compare_box.setChecked(compare)
            self.compare_box.toggled.connect(self.invalidate);row.addWidget(self.compare_box)
            self.calculate_button=Qt.QPushButton('Calculate fresh');self.calculate_button.setObjectName('fast_calculate')
            self.calculate_button.clicked.connect(self.calculate);row.addWidget(self.calculate_button)
            self.cancel_button=Qt.QPushButton('Cancel calculation');self.cancel_button.setEnabled(False)
            self.cancel_button.clicked.connect(self.cancel);row.addWidget(self.cancel_button)
            self.window.layout().insertLayout(4,row)
            self.window.setWindowTitle(f'X5 --fast {self.side} — wrist-biased planning / SIMULATION ONLY')
            origin='FRESH STARTUP SNAPSHOT' if self.context.get('input_origin')=='fresh_startup' else 'OFFLINE SAVED INPUTS'
            self.banner.setText(f'--FAST {self.side.upper()} • {origin} • TASK-POSTURE PLANNER • NO MOTOR EXECUTION')
            self.info.setText('Choose one target, or a sequence of up to three. Calculate searches alternative joint postures.\n'
                'The first approach begins at the captured pose; transfers use the optimized playing postures.\n'
                'Full stick pose is preserved by default. The optional roll-free rule still preserves tip position and stick direction.')
            self.note.setText('Original motion limits, joint limits, tools, zones and collision checks. '
                'Prefers less J1–J4 travel within 5% of the fastest checked transfer. '
                'No recorded interior motion. Two-second preview holds are retained; this is not a strike/beat controller. '
                'The opposite arm stays fixed. Frozen camera scene; unseen space is not certified clear.')
            self.status.setWordWrap(True);self.time_label.setMaximumWidth(310);self.time_label.setWordWrap(True)
            self.status.setText('Choose targets, then Calculate fresh. Play only animates the model.')
            self.plan_label.setText('No calculation at startup. Captured start shown.')
            self.save_status.setText('Frozen planning scene: '+self.context['scene'])
            self.frame_count=5

        def selected(self):
            values=[box.currentData() for box in self.boxes]
            if values[0] is None:raise ValueError('Choose the first destination')
            if values[1] is None and values[2] is not None:raise ValueError('Choose the second target before the third')
            return [p for p in values if p is not None]

        def invalidate(self,*_):
            if self.job:self.discard=True;self.job.cancel()
            self.install(self.session/'initial/plan.json')
            self.status.setText('Selection changed. Calculate fresh when ready; no pending automatic calculation.')

        def calculate(self):
            if self.job:return
            try:
                paths=self.selected();self.install(self.session/'initial/plan.json')
                self.target,self.job=start_plan(self.session,paths,self.task.currentData(),self.compare_box.isChecked(),self.budget_s)
                self.generation+=1;self.discard=False;self.calculate_button.setEnabled(False);self.cancel_button.setEnabled(True)
                self.status.setText('Calculating in one contained worker. Progress appears in the terminal.')
                self.events.append(dict(event='calculate',targets=paths))
            except Exception as exc:self.status.setText('Cannot calculate: '+str(exc))

        def cancel(self):
            if self.job:self.job.cancel();self.status.setText('Cancelling calculation; no motors are connected.')

        def install(self,path):
            plan=ExperimentPlanDisplay(path);plan.check_live_calibration(self.context['calibration'])
            if plan.side!=self.side or plan.req['scene']['source_scene_sha256']!=self.plan.req['scene']['source_scene_sha256']:
                raise ValueError('Preview arm/scene mismatch')
            self.clock.pause();self.plan=plan;self.robot.plan=plan;self.clock=PlaybackClock(plan.duration)
            self.clock.set_rate([.5,1.,2.][self.speed.currentIndex()]);self.clock.loop=self.loop_box.isChecked()
            self.timeline.blockSignals(True);self.timeline.setRange(0,max(1,int(round(plan.duration*1000))))
            self.timeline.setValue(0);self.timeline.blockSignals(False);self.timeline.setEnabled(plan.playable)
            self.play_button.setEnabled(plan.playable)
            for b in self.window.findChildren(self.QtWidgets.QPushButton):
                if b.text()=='Go to goal':b.setEnabled(plan.playable)
            self.focus_box=self.robot.focus_bounds()
            summary=plan.obj.get('fast_mode',{})
            label=('Between-target movement (excludes approach/holds)' if len(summary.get('optimized_poses',[]))>1
                   else 'Start → target movement')
            self.plan_label.setText(f'{label}: {summary["transfer_movement_s"]:.3f} s • original limits • '
                +summary['task_constraint']+' • software only' if plan.playable else 'No playable trajectory for this selection')
            self.draw_canvas()
            if plan.playable and self.autoplay_result:self.toggle_play()

        def poll(self,camera=None):
            key=super().poll()
            if self.job and self.job.poll() is not None:
                job=self.job;self.job=None;job.close()
                self.calculate_button.setEnabled(True);self.cancel_button.setEnabled(False)
                if self.discard or job.cancelled:
                    self.status.setText('Cancelled/stale result discarded. Calculate fresh to retry.');self.discard=False
                else:
                    try:
                        path=self.target/'plan.json'
                        if job.process.returncode:
                            detail=json.loads(path.read_text()).get('details',{}) if path.exists() else {'reason':str(job.log_path)}
                            raise ValueError(detail.get('reason',str(detail)))
                        self.install(path);self.status.setText('Ready — '+str(path))
                        self.events.append(dict(event='installed',plan=str(path)))
                    except Exception as exc:self.status.setText('Calculation failed: '+str(exc)+'. Change targets or retry.')
            return key

        def report(self):
            r=super().report();r.update(fast_mode=True,generations=self.generation,events=list(self.events),
                selected_arm=self.side,scene_immutable=True,can_access=False,motor_commands=0)
            return r

        def close(self):
            if self.job:self.job.close();self.job=None
            super().close()
    return FastView


def main(argv=None):
    parser=argparse.ArgumentParser(prog='./plan_drums.sh --fast',description=__doc__)
    arms=parser.add_mutually_exclusive_group();arms.add_argument('--right',dest='side',action='store_const',const='right')
    arms.add_argument('--left',dest='side',action='store_const',const='left');parser.set_defaults(side='right')
    parser.add_argument('--destinations',nargs='+',help='One to three saved target names, e.g. R1 R2 or L1')
    parser.add_argument('--task',choices=['full-pose','tip-axis'],default='full-pose')
    parser.add_argument('--compare',action='store_true',help='Also calculate captured-posture Workspace curve references')
    parser.add_argument('--offline',action='store_true',help='No acquisition; requires explicit --state and --scene')
    parser.add_argument('--state');parser.add_argument('--scene');parser.add_argument('--zone')
    parser.add_argument('--calibration',default=str(DEFAULT_CALIBRATION));parser.add_argument('--output')
    parser.add_argument('--headless',action='store_true');parser.add_argument('--autoplay',action='store_true')
    parser.add_argument('--seconds',type=float,default=0.,help='Preview auto-close timer only')
    parser.add_argument('--budget',type=float,default=45.,help='Bounded posture/path search allowance; 5–120 s')
    args=parser.parse_args(argv)
    try:
        if not math.isfinite(args.budget) or not 5<=args.budget<=120:raise ValueError('--budget must be 5–120 seconds')
        if not math.isfinite(args.seconds) or args.seconds<0:raise ValueError('Invalid --seconds')
        if args.headless and (args.autoplay or args.seconds):raise ValueError('Playback options need the GUI')
        if args.headless and not args.destinations:raise ValueError('--headless requires --destinations')
        if args.destinations and not 1<=len(args.destinations)<=3:raise ValueError('Choose one to three destinations')
        paths=[resolve_destination(v,args.side) for v in (args.destinations or [])]
        for path in paths:endpoint(path,side=args.side)
        if any(a==b for a,b in zip(paths,paths[1:])):raise ValueError('Consecutive destinations must differ')
        if bool(args.state)!=bool(args.scene) or (args.offline and not(args.state and args.scene)):
            raise ValueError('Offline use requires both --state and --scene; no stale default scene is substituted')
        from ..cli import output_folder
        from ..experiment.app import prepare,wait
        session=Path(args.output or output_folder('fast-'+args.side)).resolve()
        print('SEPARATE --FAST MODE: task-preserving wrist-biased planning; preview only; no motor execution.',flush=True)
        prepare(session,args.state,args.scene,args.calibration,side=args.side,zone=args.zone,
                fresh=not(args.state and args.scene))
        if args.headless:
            target,job=start_plan(session,paths,args.task,args.compare,args.budget)
            try:wait(job)
            except RuntimeError:
                if (target/'plan.json').exists():print(json.dumps(json.loads((target/'plan.json').read_text()).get('details'),indent=2))
                raise
            atomic_json(session/'result.json',dict(plan=str(target/'plan.json'),report=str(target/'fast_report.json')))
            print('FAST RESULT: '+str(target/'plan.json'));return 0
        view=make_view_class()(session,selected=paths,kind=args.task,compare=args.compare,
                               budget_s=args.budget,autoplay=args.autoplay);began=time.monotonic()
        try:
            while not args.seconds or time.monotonic()-began<args.seconds:
                key=view.poll()
                if key==ord('q'):break
                if key==ord('s'):view.save_view()
                time.sleep(.01)
        finally:
            view.close();atomic_json(session/'fast_view_report.json',view.report())
        return 0
    except Exception as exc:
        print('--fast stopped (no motor commands): '+str(exc));return 1
