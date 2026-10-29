"""Native experiment UI integration check; motion is always forbidden.

Uses a previously captured actual scene. --recapture explicitly permits ONE
camera-only Recapture button test (still no CAN). Not an automatic unit test.
"""
import argparse,json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from PyQt5 import QtWidgets,QtCore,QtTest
from drum_planner.common import atomic_json,DEFAULT_CALIBRATION,read_state
from drum_planner.experiment import app,dt1_goal,scenario
from drum_planner.experiment.config import no_can


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--acquisition',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--recapture',action='store_true')
    args=parser.parse_args();no_can();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    qt=QtWidgets.QApplication([]);qt.setQuitOnLastWindowClosed(False)
    report=dict(motor_commands=0,can_access=False,camera_recapture_requested=args.recapture,checks=[])
    acquisition=args.acquisition.resolve();state=acquisition/'measured_start.json';scene=acquisition/'camera/scene.json'
    def pump(view,duration=.2):
        end=time.monotonic()+duration
        while time.monotonic()<end:qt.processEvents();view.poll();time.sleep(.01)
    def wait(view,timeout=200):
        end=time.monotonic()+timeout
        while view.job or view.pending_methods or view.pending_dt1:
            if time.monotonic()>end:raise TimeoutError(view.status.text())
            pump(view,.1)
        pump(view)
    view=None
    try:
        app.prepare(root/'right',state,scene,DEFAULT_CALIBRATION,side='right')
        view=app.make_view_class()(root/'right',seconds=60.);pump(view)
        assert not view.job and not view.plan.playable and view.destination_box.currentData() is None
        index=view.destination_box.findData('dt1');assert index>0
        assert view.destination_box.itemText(index)=='dt1'
        assert view.start_box.findData('dt1')==-1
        before_scene=view.context['scene_sha256'];before_state=view.context['acquired_state_sha256']
        view.destination_box.setCurrentIndex(index);pump(view,.1)
        assert view.job_kind=='dt1' and view.destination_path is None
        QtTest.QTest.mouseClick(view.cancel_button,QtCore.Qt.LeftButton);wait(view)
        assert not view.plan.playable and view.generate_button.isEnabled() and not view.context.get('destination_sha256')
        report['checks'].append('Right dt1 selection is lazy; Cancel reaps the contained goal job without stale playback')
        QtTest.QTest.mouseClick(view.generate_button,QtCore.Qt.LeftButton);wait(view)
        assert view.plan.playable,view.status.text()
        assert view.destination_box.currentData()=='dt1' and view.displayed_method=='workspace_curve'
        proof=view.context['dt1_generation'];geometry=proof['goals'][0]['geometry']
        assert abs(geometry['inward_yaw_deg'])<=20.+1e-6 and geometry['tip_error_m']<=1e-5
        assert view.context['scene_sha256']==before_scene and view.context['acquired_state_sha256']==before_state
        plan=view.plan.obj;request=view.plan.req
        np.testing.assert_allclose(plan['outward'][-1]['positions_rad'],request['goal'],rtol=0,atol=1e-10)
        assert plan['final_curve_check']['samples_checked']>0
        view.seek(plan['duration_outward_s']);pump(view)
        view.viewport.save_window(view.window,root/'right_dt1_goal.png')
        QtTest.QTest.mouseClick(view.play_button,QtCore.Qt.LeftButton);pump(view,.5)
        view.clock.pause();view.seek(0.);pump(view)
        report['checks'].append('Actual frozen-camera dt1 generates a playable balanced-timing route; seek/play work')
        report['first_plan']=str(view.plan.path);report['first_geometry']=geometry
        if args.recapture:
            before=view.context['scene_sha256'];old_destination=view.context['destination_sha256']
            QtTest.QTest.mouseClick(view.recapture_button,QtCore.Qt.LeftButton);wait(view)
            assert view.scene_generation==1 and view.context['scene_sha256']!=before
            assert view.context['acquired_state_sha256']==before_state
            assert view.plan.playable,view.status.text()
            assert view.context['dt1_generation']['identity']['scene_sha256']==view.context['scene_sha256']
            assert view.context['destination_sha256']!=old_destination
            assert view.plan.req['scene_robot_start']==read_state(state)['right_rad']
            report['checks'].append('Camera-only Recapture installs new scene and regenerates dt1; original measured self-mask retained')
            report['recapture_plan']=str(view.plan.path)
        assert not view.viewport.report()['render_errors']
        report['right']=view.report();view.close();view=None;qt.processEvents()
        # The paired source also allows both renderers to be compared with one
        # frozen input; fresh left acquisition itself remains depth-only.
        app.prepare(root/'left',state,scene,DEFAULT_CALIBRATION,side='left')
        view=app.make_view_class()(root/'left',seconds=60.);pump(view)
        assert view.destination_box.findData('dt1')==-1 and not view.job
        assert view.destination_box.count()>1
        view.viewport.save_window(view.window,root/'left_saved_only.png')
        assert not view.viewport.report()['render_errors']
        report['checks'].append('Left native window still offers saved destinations only and starts unselected')
        report['left']=view.report();report['passed']=True
    finally:
        if view:view.close()
        atomic_json(root/'ui_report.json',report)
    print(json.dumps(dict(passed=report['passed'],checks=report['checks']),indent=2))


if __name__=='__main__':main()
