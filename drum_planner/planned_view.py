"""Frozen mesh playback of the EXACT scene used for a saved MoveIt plan."""
from pathlib import Path
import time
import numpy as np

from .common import DEFAULT_CALIBRATION, calibration, digest, atomic_json
from .planning import scene
from .trajectory_view import PlanDisplay
from .snapshot_preview import SnapshotPlaybackView, prepare_scene


def load(plan,cal_path):
    path=Path(plan.obj['provenance']['scene']).resolve()
    world=scene(path,cal_path)
    if world['source_scene_sha256']!=plan.req['scene']['source_scene_sha256']:
        raise ValueError('Display scene is not the scene used by this plan')
    if digest(plan.obj['provenance']['state'])!=plan.obj['provenance']['state_sha256']:
        raise ValueError('Starting state changed since planning')
    return load_scene(path,cal_path)


def load_scene(path,cal_path):
    """A hash-verified frozen camera scene, also usable before any plan exists."""
    path=Path(path).resolve();world=scene(path,cal_path)
    if len(world['frames'])!=5 or 'depth_m.bin' not in world['files_sha256']:
        raise ValueError('Invalid five-frame planning capture')
    # Display the HASH-VERIFIED surface actually used by this plan. Never
    # re-fuse old raw frames with today's algorithm: historical nearest-union
    # plans and new confirmed-depth plans must both retain their exact scene.
    minimum=np.fromfile(path.parent/'depth_m.bin',dtype='<f4')
    if minimum.size!=800*1280 or not np.isfinite(minimum).all() or np.any((minimum<0)|(minimum>5.)):
        raise ValueError('Invalid saved planning depth image')
    minimum=minimum.reshape(800,1280)
    # prepare_scene accepts floating depth with a metre scale as well as Y16.
    # Quantize toward the camera to avoid displaying farther surfaces.
    rendered=np.floor(minimum*10000).astype('<u2')
    h=dict(world['frames'][-1],value_scale_m=.0001)
    return prepare_scene(h,rendered,world['camera'],world['T_base_depth'])


def show(path,*,seconds=0.,cal_path=DEFAULT_CALIBRATION,autoplay=False):
    if not np.isfinite(seconds) or seconds<0:raise ValueError('Invalid display duration')
    plan=PlanDisplay(path);plan.check_live_calibration(cal_path)
    packet,mesh=load(plan,cal_path)
    viewer=SnapshotPlaybackView(calibration(cal_path),plan.path.parent,plan,autoplay=autoplay)
    start=time.monotonic()
    try:
        viewer.set_scene(packet,mesh)
        viewer.window.setWindowTitle('X5 current-pose plan — matching frozen scene / SIMULATION')
        viewer.banner.setText('MEASURED START → RECORD3 → 2 SECOND HOLD → RETURN  •  SIMULATION / NO MOTOR COMMANDS')
        viewer.info.setText('This is the SAME five-frame depth scene used by MoveIt.\n'
            'Camera and CAN are closed. No continuous refresh.\n'
            'Play animates the model only. Missing depth is unobserved, not an obstacle.')
        viewer.plan_label.setText(plan.warning if not plan.playable else
            ('SYNTHETIC TEST • ' if plan.obj.get('synthetic') else '')+
            'SAME PLANNING SNAPSHOT • '+plan.obj['status']+' • '+plan.warning)
        viewer.status.setText('MATCHING FROZEN PLANNING SCENE — no live camera or CAN')
        viewer.save_status.setText('Planning scene: '+plan.obj['provenance']['scene']+
                                  ' • mesh generated once for display')
        viewer.frame_count=5
        while not seconds or time.monotonic()-start<seconds:
            key=viewer.poll()
            if key==ord('q'):break
            if key==ord('s'):viewer.save_view()
            time.sleep(.01)
        report=viewer.report();report.update(same_planning_scene=True,can_access=False,camera_access=False,
            planning_frames_used=5,historical_snapshot=True,max_age_at_display_s=None)
        atomic_json(plan.path.parent/'planned_view_report.json',report)
        return report
    finally:viewer.close()
