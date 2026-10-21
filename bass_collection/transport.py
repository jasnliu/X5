"""Existing dual-center guards, with a separately audited hardware-emergency path."""
import json
import re
import time
from camera_playback.playback_transport import PlaybackMotors
from hihat_collection.transport import DataMotors
from camera_playback.mit_center_return import MitCenterReturn
from camera_playback.snare_beat import PATH_BIAS_GAIN, PATH_KP, PATH_KD


class DriveVelocityHold(MitCenterReturn):
    """Reuse main beat's corrected J6 idle/path hold, not its strike scheduler."""
    def __init__(self, previous, channel):
        super().__init__(previous.position, previous.goal, previous.bias, previous.at,
                         bias_gain=PATH_BIAS_GAIN, kp=PATH_KP, kd=PATH_KD)
        self.actual=previous.actual
        self.velocity=previous.velocity
        self.channel=channel

    def update(self, actual, now):
        return super().update(actual, now, measured_velocity=self.channel.sample.velocity)



def damaging_fault(message):
    return bool(re.search(r'(?:STALL: (?:left|right) motor \d+|(?:left|right) motor \d+: (?:too hot|fault))', str(message)))


class BassDataMotors(DataMotors):
    def _tick_left(self):
        # The older collector differentiates quantized encoder positions by
        # host loop time and uses an older friction-sensitive load estimator.
        # Main beat already corrected this. Preserve all strike code, bounds,
        # torque limits and center/relax gates; adapt only the path/idle hold.
        if self.left_mit is not None and not isinstance(self.left_mit, DriveVelocityHold):
            self.left_mit=DriveVelocityHold(self.left_mit,self.left_channel)
        return super()._tick_left()

    def guard_left_disable(self, frame):
        if getattr(self, 'bass_emergency', False): return
        return super().guard_left_disable(frame)

    def emergency_stop(self, reason):
        if not damaging_fault(reason):
            raise RuntimeError('Off-center disable requires an observed damaging hardware fault')
        if self.right_joint7_session is not None:
            raise RuntimeError('Stop strike writer before emergency disable')
        self.bass_emergency = True
        self.emergency_relax_reason = str(reason)
        (self.directory/'hardware_emergency.json').write_text(json.dumps(
            dict(reason=str(reason), t=time.monotonic(), center_exception=True), indent=2)+'\n')
        PlaybackMotors.relax(self)
        self.left_mit = None
