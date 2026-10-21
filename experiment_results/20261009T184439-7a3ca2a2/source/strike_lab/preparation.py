"""Record3 preflight shared by real and simulated preparation; no device I/O."""
from camera_playback.trajectory import load_playback_trajectory
from centering.motors import SPEED, RIGHT_PLAYBACK_SPEED
from safe_zone.geometry import Model, Zone, RIGHT_TCP
from .config import ROOT, RECORDING, center_pose, joint_limits


def load_record3(stop, publish, path=RECORDING):
    def progress(detail):
        if stop.is_set():raise InterruptedError('Stopped during record3 preflight')
        publish(dict(phase='CHECKING RECORD3 — '+detail))
    progress('gripper stays closed; recorded finger motion is ignored')
    model=Model(ROOT/'model/openarmx.urdf')
    zone=Zone.load(ROOT/'right_zones/zone1.json',model.digest,RIGHT_TCP)
    lower,upper=zip(*joint_limits())
    return load_playback_trajectory(path,model,zone,lower,upper,center_pose(),SPEED,
                                    playback_speed=RIGHT_PLAYBACK_SPEED,progress=progress)
