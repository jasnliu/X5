"""Offline native calculation parity study. Saved actual camera data; NO devices."""
import argparse,json,time,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from drum_planner.common import ROOT,DEFAULT_CALIBRATION,atomic_json
from drum_planner.experiment.config import no_can,settings as gesture
from drum_planner.experiment import fast,checkpoints
from drum_planner.baseline import planner,targets,tasks,settings
from drum_planner.baseline.curve import Curve
from drum_planner.baseline.paths import validate_timing


def run(root,side,selected,capture=None,independent_timing=False):
    no_can();root=Path(root).resolve();root.mkdir(parents=True,exist_ok=True)
    capture=Path(capture).resolve() if capture else ROOT/'diagnostics/experiment_dt1_20261026/fresh_right/acquisition'
    binding=targets.freeze(selected,side,root)
    started=time.monotonic()
    obj=planner.plan(root/'hardware',capture/'measured_start.json',capture/'camera/scene.json',DEFAULT_CALIBRATION,
                     moving_side=side,destinations=binding,simulation=True)
    elapsed=time.monotonic()-started
    assert obj['status']=='planned',obj.get('details')
    curve=Curve(obj);validate_timing(curve)
    # Validate current DT1 identity AND full native hardware checker on the exact
    # law. This does not mark a saved capture as fresh hardware eligibility.
    from drum_planner.experiment.rgbd import scene
    world=scene(capture/'camera/scene.json',DEFAULT_CALIBRATION);targets.validate(obj,world)
    reserves=[]
    for binding in obj.get('destination_bindings',[]):
        if binding.get('destination_kind')=='dt1':
            from cymbal_destination import goal_limits
            dest=json.loads(Path(binding['goal_recording']).read_text())
            reserves.append(goal_limits.verify(dest['samples'][0]['positions_rad'],dest['goal_joint_reserve']))
    req=json.loads((root/'hardware/backend_request.json').read_text())
    proof=tasks.hard_check(req,curve.at_many(np.r_[np.arange(0,curve.duration,.002),curve.duration]),root/'native_proof')
    try:tasks.load_checked(root/'hardware/plan.json',DEFAULT_CALIBRATION)
    except ValueError as exc:assert 'newly acquired' in str(exc)
    else:raise AssertionError('Offline calculation must not become hardware eligible')
    context=json.loads((root/'hardware/calculation_context.json').read_text())
    began=time.monotonic();reference=fast.plan(context,root/'experiment','workspace_curve',gesture(),120.)
    reference_s=time.monotonic()-began
    assert reference['status']=='planned'
    exact=(reference['duration_outward_s']==obj['duration_outward_s'] and reference['outward']==obj['outward']
           and (not obj.get('checkpoint_sequence') or reference['cycle']==obj['cycle']))
    if not independent_timing:assert exact
    # Bounded wall-time optimization can end at different valid iterates on a
    # difficult checkpoint leg. Explicit opt-in compares IDENTICAL geometry and
    # independently proves BOTH clocks instead of calling them byte-identical.
    if obj.get('checkpoint_sequence'):
        a=obj['checkpoint_sequence'];b=reference['checkpoint_sequence']
        for sa,sb in zip(a['segments']+[a['return_leg']],b['segments']+[b['return_leg']]):
            assert sa['timing']['kind']==sb['timing']['kind']=='time_density'
            assert sa['timing']['path']==sb['timing']['path']
    else:assert reference.get('source_timing')==obj.get('source_timing')
    refcurve=Curve(reference);validate_timing(refcurve)
    from drum_planner.rest_paths import verify_curve
    verify_curve(curve,req);verify_curve(refcurve,req)
    if independent_timing:
        tasks.hard_check(req,refcurve.at_many(np.r_[np.arange(0,refcurve.duration,.002),refcurve.duration]),root/'reference_native_proof')
    report=dict(software_only=True,actual_camera_saved_input=str(capture),side=side,selected=selected,
        motor_commands=0,camera_opened=False,hardware_wall_s=elapsed,experiment_wall_s=reference_s,
        outward_s=obj['duration_outward_s'],full_cycle_s=obj['cycle'][-1]['time_s'],goal_joint_reserves=reserves,
        exact_sample_parity=exact,exact_geometry_parity=True,independent_timing=independent_timing,
        reference_outward_s=reference['duration_outward_s'],profile=obj['hardware_motion_profile'],native_proof=proof,
        offline_rejected_for_hardware=True)
    atomic_json(root/'report.json',report);print(json.dumps(report),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--folder',required=True);p.add_argument('--side',default='right');p.add_argument('destinations',nargs='+')
    p.add_argument('--capture',type=Path,help='Explicit saved acquisition; no camera or encoder access')
    p.add_argument('--independent-timing',action='store_true',help='For wall-time-bounded checkpoint clocks: prove both clocks and exact geometry, report timing differences')
    a=p.parse_args();run(a.folder,a.side,[str(ROOT/'destinations'/a.side/(x+'.json')) if x!='dt1' else x for x in a.destinations],a.capture,a.independent_timing)
