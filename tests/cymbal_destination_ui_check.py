"""Explicit OFFLINE native-UI check; no camera/CAN, no target override.

Success uses the actual corrected run. Failure controls are exercised with a
clearly labeled synthetic failure status on the same immutable scene.
"""
import argparse,copy,json,sys,tempfile,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from PyQt5 import QtWidgets,QtCore,QtTest
from drum_planner.common import atomic_json,digest
from drum_planner.experiment.config import no_can
from cymbal_destination import view,output
from cymbal_destination.config import MAX_INWARD_YAW_DEG


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--report-dir',type=Path,required=True)
    a=p.parse_args();no_can();folder=a.run.resolve();base=a.report_dir.resolve();base.mkdir(parents=True,exist_ok=True)
    app=QtWidgets.QApplication([]);app.setQuitOnLastWindowClosed(False)
    report=dict(hardware_access=False,motor_commands=0,actual_dt1_target_success=True,checks=[])
    def pump():
        for _ in range(6):app.processEvents();time.sleep(.06)
    window=view.Window(folder);pump();assert window.ready and window.export_button.isEnabled()
    assert window.pose_box.count()==2 and window.pose_box.currentData()=='goal'
    assert window.viewport.surface.get_visible() and window.viewport.surface.triangle_count>1000
    assert not window.viewport.colorbar.GetVisibility()
    assert 'shaft-above elevation' in window.status.text()
    assert 'inward aim' in window.status.text() and f'(±{MAX_INWARD_YAW_DEG:g}°)' in window.status.text()
    assert f'within ±{MAX_INWARD_YAW_DEG:g}°' in window.info.text()
    assert 'Other bounds: 3° endpoint reserve.' in window.info.text()
    assert 'J2 operating ≥ 0°; goal ≥ +1° (encoder degrees).' in window.info.text()
    assert abs(window.result['geometry']['inward_yaw_deg'])<=MAX_INWARD_YAW_DEG+1e-6
    assert 'no hard elevation cap' in window.info.text() and '8°' not in window.info.text()
    def check_markers():
        assert len(window.markers)==3  # Existing surface/green targets plus visual disc center.
        expected=[window.target['target']['world_m'],window.target['above_target']['world_m'],
                  window.target['flat_disc']['center_world_m']]
        for (source,mapper,actor),point,color in zip(window.markers,expected,[(1.,.05,.05),(.05,.9,.1),(1.,.05,.05)]):
            np.testing.assert_array_equal(source.GetCenter(),point)
            np.testing.assert_allclose(actor.GetProperty().GetColor(),color,atol=1e-12)
            assert actor.GetVisibility()
        assert window.markers[2][0].GetRadius()==.005
    check_markers()
    window.viewport.save_window(window,base/'ui_actual_target_success.png')
    # Diagnostic view perpendicular to the tilted disc, not world-vertical.
    c=np.asarray(window.target['flat_disc']['center_world_m']);n=np.asarray(window.target['flat_disc']['normal_world'])
    u=np.asarray(window.result['geometry']['inward_reference_world'])
    camera=window.viewport.camera;camera.SetPosition(*(c+1.5*n));camera.SetFocalPoint(*c)
    camera.SetViewUp(*u);camera.ParallelProjectionOn();camera.SetParallelScale(.5)
    window.viewport.renderer.ResetCameraClippingRange();window.viewport.render();pump()
    window.viewport.save_window(window,base/'ui_disc_normal_view.png')
    camera.ParallelProjectionOff();window.fit();pump()
    request=window.req;r=window.result
    window.pose_box.setCurrentIndex(1);pump();np.testing.assert_array_equal(window.pose.q,request['start'])
    window.pose_box.setCurrentIndex(0);pump();np.testing.assert_array_equal(window.pose.q,r['goal_rad'])
    check_markers()  # The center stays frozen when the displayed arm changes.
    assert window.viewport.surface.geometry_uploads==1
    report['checks'].append('Actual unshifted green target: static destination/start switch, no animation, retained scene geometry')
    for button in (window.camera_button,window.fit_button):QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton);pump()
    for box in (window.disc_box,window.axis_box):
        box.setChecked(False);pump();box.setChecked(True);pump()
    check_markers()
    report['checks'].append('Red dot at exact fitted-disc center; original red/green targets unchanged; inward aim labeled')
    window.save_button.click();pump();assert (folder/'destination_preview.png').exists()
    assert not window.viewport.report()['render_errors'];report['success_renderer']=window.viewport.report()
    with tempfile.TemporaryDirectory(prefix='x5-cymbal-UI-test-') as name:
        temp=Path(name);dest=temp/'export.json';window.export_to(dest)
        assert dest.read_bytes()==(folder/'destination.json').read_bytes()
        try:window.export_to(dest);raise AssertionError('Overwrite allowed')
        except FileExistsError:pass
        altered=temp/'altered.json';altered.write_text('{}')
        try:output.export(altered,temp/'tampered.json',r['destination_sha256']);raise AssertionError('Tampered export allowed')
        except ValueError:pass
        report['checks'].append('Export matches actual goal; existing file and tampered source rejected')
        window.close();pump()
        # Separate failure fixture: no destination, no modification of actual run.
        (temp/'acquisition').symlink_to(folder/'acquisition',target_is_directory=True)
        atomic_json(temp/'backend_request.json',request)
        failed=copy.deepcopy(r);failed.update(status='no_feasible_destination',error='SYNTHETIC FAILURE: UI-control test only',
            request_sha256=digest(temp/'backend_request.json'))
        for key in ('goal_rad','destination_sha256','geometry','approach'):failed.pop(key,None)
        atomic_json(temp/'result.json',failed)
        window=view.Window(temp);pump()
        assert not window.ready and not window.export_button.isEnabled() and window.pose_box.count()==1
        assert window.viewport.surface.get_visible()
        check_markers()
        window.viewport.save_window(window,base/'ui_synthetic_failure.png')
        assert not window.viewport.report()['render_errors'];window.close();pump()
        report['checks'].append('Synthetic failure: captured start only; no destination/export or stale goal')
        failed['version']='cymbal-prime-destination-v1';atomic_json(temp/'result.json',failed)
        try:output.verified(temp);raise AssertionError('Obsolete convention accepted')
        except ValueError as exc:assert 'destination constraints are obsolete' in str(exc)
        report['checks'].append('Obsolete v1 angle results rejected without relabeling the saved data')
    report['passed']=True;atomic_json(base/'ui_checks.json',report);print(json.dumps(report,indent=2))


if __name__=='__main__':main()
