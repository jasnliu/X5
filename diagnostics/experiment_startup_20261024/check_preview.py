"""Native preview/calculation check using the just-acquired startup snapshot.
No device access: this harness does not refresh or move hardware.
"""
import json,sys,time
from pathlib import Path
from drum_planner.common import atomic_json,digest
from drum_planner.experiment import app,destinations
from drum_planner.experiment.config import no_can
no_can()
folder=Path(sys.argv[1]).resolve()
view=app.make_view_class()(folder)
errors=[];sys.excepthook=lambda t,v,b:errors.append(str(v))
def pump(seconds=.1):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        view.poll();time.sleep(.01)
    assert not errors,errors
try:
    pump(1)
    assert view.context['input_origin']=='fresh_startup'
    assert view.context['startup_acquisition']['passed']
    assert view.destination_box.currentData() is None and view.job is None
    assert view.generation_count==0 and not view.plan.playable
    assert 'FRESH STARTUP SNAPSHOT' in view.banner.text()
    baseline={k:(view.context[k],digest(view.context[k])) for k in ('state','scene')}
    mesh=view.mesh;packet=view.packet
    view.viewport.save_window(view.window,folder/'startup_ui.png')
    primary=destinations.files(view.side)[0]
    view.destination_box.setCurrentIndex(view.destination_box.findData(str(primary)))
    end=time.monotonic()+150
    while (view.job or view.pending_methods) and time.monotonic()<end:pump()
    assert view.job is None and not view.pending_methods,view.status.text()
    assert view.plan.obj['status'] in ('planned','no_feasible_route'),view.status.text()
    assert not any(e['event'] in ('worker_stopped','result_rejected','worker_start_failed') for e in view.events),list(view.events)
    assert baseline=={k:(view.context[k],digest(view.context[k])) for k in ('state','scene')}
    assert view.mesh is mesh and view.packet is packet
    if view.plan.playable:
        view.play_button.click();pump(.2);assert view.clock.position>0
        view.play_button.click();pump()
    view.viewport.save_window(view.window,folder/'calculated_ui.png')
    atomic_json(folder/'startup_ui_check.json',dict(passed=True,side=view.side,
        status=view.plan.obj['status'],destination=str(primary),frozen_inputs=baseline,
        fresh_startup_label=True,unselected_at_start=True,no_startup_calculation=True,
        no_camera_or_can_during_preview=True,motor_commands=0,view_report=view.report()))
finally:view.close()
