"""Native hardware UI, real Workspace planning, fake owner/motors only."""
import argparse,json,time,sys
from pathlib import Path
from unittest.mock import patch
import numpy as np
from drum_planner.common import atomic_json,DEFAULT_CALIBRATION,arm_joints
from drum_planner.experiment import config,worker,destinations
from drum_planner.baseline.workspace import Workspace
from drum_planner.baseline.curve import Curve
from drum_planner.baseline import tasks


def main():
    p=argparse.ArgumentParser();p.add_argument('--side',required=True,choices=['right','left']);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--reuse-generated',type=Path)
    p.add_argument('--timing-backend',choices=['adaptive','sparse'],default='adaptive')
    a=p.parse_args();config.no_can();a.output.mkdir(parents=True,exist_ok=False)
    state,scene=config.saved_inputs(a.side)
    worker.initialize(a.output/'input',state,scene,DEFAULT_CALIBRATION,a.side)
    files=destinations.files(a.side)
    if len(files)<3:
        for i in range(3-len(files)):
            obj=json.loads(files[0].read_text());obj['samples'][-1]['positions_rad'][0]+=.015*(i+1)
            f=a.output/f'test-only-variation-{i+1}.json';atomic_json(f,obj);files.append(f.resolve())
    errors=[];old=sys.excepthook;sys.excepthook=lambda t,e,b:errors.append(str(e))
    rows=[];cached={}
    if a.reuse_generated:
        for f in a.reuse_generated.glob('ui/calculate-*/plan/plan.json'):
            obj=json.loads(f.read_text())
            if obj['status']=='planned':cached[len(obj['destination_bindings'])]=f.resolve()
    with patch.object(destinations,'files',return_value=files):
        view=Workspace(a.output/'ui',simulate_plan=a.output/'input/initial/plan.json',moving_side=a.side,autoscene=False,
                       **({'timing_backend':a.timing_backend} if a.timing_backend!='adaptive' else {}))
        def pump(seconds=.1):
            until=time.monotonic()+seconds
            while time.monotonic()<until:view.poll();time.sleep(.008)
            assert not errors,errors
        def wait_for(predicate,limit):
            until=time.monotonic()+limit
            while not predicate() and time.monotonic()<until:pump()
            assert predicate(),(view.last_error,view.owner_status)
        def calculate(count):
            if count in cached:
                view.simulate_plan=cached[count];view.simulation_recalculate=False
            view.control.buttons['calculate'].click();assert view.job or view.plan_valid
            wait_for(lambda:view.job is None,3000)
            assert view.plan_valid,(view.last_error,view.control.status.text())
            plan=view.plan;curve=Curve(plan.obj)
            assert plan.obj['route_planner']['method']=='workspace_curve'
            if a.timing_backend=='sparse':
                assert plan.obj['hardware_variant']=='sparse_ipopt'
                laws=([s['timing'] for s in plan.obj['checkpoint_sequence']['segments']]+
                      [plan.obj['checkpoint_sequence']['return_leg']['timing']]
                      if 'checkpoint_sequence' in plan.obj else [plan.obj['timing']])
                assert all(law['method']=='sparse_ipopt' for law in laws)
            assert plan.obj['experiment_only'] and not plan.obj['actual_current_robot_state_verified']
            assert len(plan.obj['destination_bindings'])==count
            # Independently check the exact controller law against native MoveIt.
            # No hardware preparation flags are fabricated for this offline test.
            q=curve.at_many(np.r_[np.arange(0,curve.duration,.002),curve.duration])
            proof=tasks.hard_check(plan.req,q,a.output/f'curve-check-{count}')
            assert proof['all_valid']
            rows.append(dict(checkpoints=count,plan=str(plan.path),native_proof=proof))
            for rate in (.25,.6,1.):
                view.preview.rate.setValue(rate);d=view.preview.plan
                for t in np.linspace(0,d.duration,20):
                    view.preview.seek(t);pump(.01)
                    assert np.allclose(view.preview.robot.last_positions,d.positions(t),atol=1e-10)
            view.preview.seek(0);view.preview.play.click();pump(.15);view.preview.pause.click()
            paused=view.preview.clock.position;pump();assert view.preview.clock.position==paused and paused>0
            view.preview.rate.setValue(1.)
        def cycle():
            view.control.buttons['start'].click()
            wait_for(lambda:view.owner_info is not None,3)
            assert view.owner_info['simulated']
            wait_for(lambda:view.owner_info is None,120)
            assert view.owner_status=={} and 'RELAXED' in view.control.status.text(),view.control.status.text()
        try:
            pump(.3);assert len([w for w in view.windows if w.isVisible()])==3
            assert view.job is None and view.plan is None
            calculate(1);cycle()
            # Simulated feedback follows the completed owner to its original rest.
            # No selection change triggers a search; the next Calculate owns it.
            selector=view.control.destinations;selector.add.click();pump()
            assert view.job is None and not view.plan_valid
            selector.boxes[1].setCurrentIndex(selector.boxes[1].findData(str(files[1])))
            selector.add.click();selector.boxes[2].setCurrentIndex(selector.boxes[2].findData(str(files[2])))
            assert view.job is None and len(selector.boxes)==3
            assert selector.boxes[2].findData(str(files[1]))<0
            calculate(3)
            mesh=view.preview.viewport.surface.geometry_uploads
            # Complete normal mission: independently planned final -> original.
            cycle();assert view.preview.viewport.surface.geometry_uploads==mesh
            # Reuse the checked offline result for a held-prefix recovery UI test.
            view.simulate_plan=view.plan.path;view.simulation_recalculate=False
            view.control.buttons['calculate'].click();assert view.plan_valid
            view.control.buttons['start'].click()
            wait_for(lambda:view.owner_status.get('phase')=='OUTWARD',15)
            pump(.35);view.control.buttons['hold'].click()
            wait_for(lambda:view.owner_status.get('phase')=='HELD',3)
            assert all(b.isEnabled() for b in view.control.buttons.values())
            assert not view.owner_status['disabled']
            view.control.buttons['reverse'].click()
            wait_for(lambda:view.owner_info is None,120)
            assert 'RELAXED' in view.control.status.text(),view.control.status.text()
            # Selection edits don't secretly regenerate; Cancel then Calculate recovers.
            selector.remove.click();assert len(selector.boxes)==2 and not view.plan_valid and view.job is None
            view.control.buttons['calculate'].click();assert view.job
            view.control.buttons['cancel'].click();pump();assert view.job is None
            assert all(b.isEnabled() for b in view.control.buttons.values())
            assert view.control.grab().save(str(a.output/'controls.png'))
            view.preview.viewport.save_window(view.preview,a.output/'preview.png')
            view.live.viewport.save_window(view.live,a.output/'live.png')
            assert not view.last_error,view.last_error
            assert not view.preview.viewport.errors and not view.live.viewport.errors
            report=view.report()
            if report.get('owner'):
                report['owner']=dict(report['owner']);report['owner'].pop('token',None);report['owner'].pop('socket',None)
            atomic_json(a.output/'verification.json',dict(side=a.side,errors=errors,rows=rows,motor_commands=0,camera_access=False,
                simulated_owner_cycles=3,reused_generated_plans=bool(cached),controls='select/add/remove/calculate/cancel/preview/pause/scrub/speed/start/hold/reverse/relax',
                report=report))
            print('HARDWARE WORKSPACE UI PASS',a.side,flush=True)
        finally:
            if view.client and view.owner_info and view.owner_info['simulated']:
                # Dispose only our explicitly fake owner, even after a failed test.
                try:
                    view.client.send('hold');pump(.1);view.client.send('supported_exit');pump(.1)
                except Exception:pass
            view.close();sys.excepthook=old


if __name__=='__main__':main()
