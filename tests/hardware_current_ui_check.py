"""Native three-window offline check with actual saved camera scene; fake motors only."""
import argparse,json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtWidgets,QtCore,QtTest
import numpy as np
from drum_planner.common import ROOT,DEFAULT_CALIBRATION,atomic_json
from drum_planner.experiment.config import no_can
from drum_planner.baseline.workspace import Workspace


def main():
    p=argparse.ArgumentParser();p.add_argument('--folder',type=Path,required=True)
    p.add_argument('--plan',type=Path,default=ROOT/'diagnostics/hardware_current_20261026/native_dt1_v2/hardware/plan.json')
    a=p.parse_args()
    no_can();root=a.folder.resolve();root.mkdir(parents=True,exist_ok=False)
    qt=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    report=dict(software_only=True,physical_motor_commands=0,camera_opened=False,checks=[])
    view=None
    def pump(seconds=.15):
        until=time.monotonic()+seconds
        while time.monotonic()<until:view.poll();time.sleep(.01)
    def wait(test,seconds=180):
        until=time.monotonic()+seconds
        while not test():
            if time.monotonic()>until:raise TimeoutError(view.control.status.text()+' '+str(view.owner_status))
            pump(.05)
    def click(button):QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton);pump()
    try:
        plan=a.plan
        view=Workspace(root/'right',simulate_plan=plan,autoscene=False);pump()
        assert view.simulated and len([w for w in view.windows if w.isVisible()])==3
        assert view.job is None and not view.plan_valid and view.owner_info is None
        box=view.control.destinations.boxes[0];box.setCurrentIndex(box.findData('dt1'));pump()
        # Change effort then restore to exercise invalidate/no auto-calculation.
        controls=view.control.planning;controls.quality.setCurrentIndex(controls.quality.findData('fast'));pump()
        controls.quality.setCurrentIndex(controls.quality.findData('balanced'));pump()
        assert view.job is None and not view.plan_valid
        report['checks'].append('Three windows; dt1 / profile edits never calculate or move')
        click(view.control.buttons['calculate']);assert view.job is not None
        click(view.control.buttons['cancel']);assert view.job is None and not view.plan_valid
        assert all(b.isEnabled() for b in view.control.buttons.values())
        click(view.control.buttons['calculate']);wait(lambda:view.job is None)
        assert view.plan_valid,view.control.status.text()
        assert view.prepared is None and view.plan.obj['experiment_only'] is True
        assert view.plan.obj['hardware_planning_pipeline']=='workspace-current-v1'
        assert view.plan.obj['dt1_generation']['status']=='ready'
        from cymbal_destination import goal_limits
        for b in view.plan.obj['destination_bindings']:
            if b.get('destination_kind')=='dt1':
                dest=json.loads(Path(b['goal_recording']).read_text())
                goal_limits.verify(dest['samples'][0]['positions_rad'],dest['goal_joint_reserve'])
        report['checks'].append('Cancel / retry actual offline Calculate regenerates dt1 then plans; no hardware eligibility')
        initial=view.live.robot.last_positions.copy()
        view.preview.seek(view.plan.obj['duration_outward_s']);pump()
        assert np.array_equal(view.live.robot.last_positions,initial)
        view.preview.viewport.save_window(view.preview,root/'right_prime.png')
        assert view.control.grab().save(str(root/'controls.png'))
        click(view.preview.play);pump(.3);click(view.preview.pause)
        report['checks'].append('Exact-law preview, seek, play/pause leave measured/fake live pose independent')
        click(view.control.buttons['start']);wait(lambda:view.owner_status.get('phase')=='OUTWARD',20)
        old=view.motion_profile.copy();controls.quality.setCurrentIndex(controls.quality.findData('fast'));pump()
        assert view.motion_profile==old
        click(view.control.buttons['hold']);wait(lambda:view.owner_status.get('phase')=='HELD',5)
        assert not view.owner_status.get('disabled')
        click(view.control.buttons['reverse']);wait(lambda:view.owner_info is None,40)
        # status.json is a throttled periodic convenience snapshot; the final
        # IPC reply can precede its next write. The shutdown receipt is final.
        receipt=Path(view.last_owner_info['folder'])/'final_status.json'
        wait(lambda:receipt.exists(),5)
        terminal=json.loads(receipt.read_text())
        assert terminal['phase']=='RELAXED',terminal
        report['fake_controller_terminal']=terminal
        report['checks'].append('Fake-motor Start, active edit refusal, Hold and checked-prefix Reverse return/relax')
        assert not view.preview.viewport.report()['render_errors']
        assert not view.live.viewport.report()['render_errors']
        report['right']=view.report();view.close();view=None;qt.processEvents()
        left=ROOT/'diagnostics/hardware_current_20261026/left_l1/hardware/plan.json'
        view=Workspace(root/'left',simulate_plan=left,autoscene=False,moving_side='left');pump()
        assert view.control.destinations.boxes[0].findData('dt1')==-1
        click(view.control.buttons['calculate']);assert view.plan_valid
        view.preview.seek(view.plan.obj['duration_outward_s']);pump()
        view.preview.viewport.save_window(view.preview,root/'left_goal.png')
        assert not view.preview.viewport.report()['render_errors']
        report['checks'].append('Left native preview loads current route; no dt1 target')
        report['left']=view.report();report['passed']=True
    finally:
        if view:view.close()
        atomic_json(root/'ui_report.json',report)
    print(json.dumps(report['checks'],indent=2))

if __name__=='__main__':main()
