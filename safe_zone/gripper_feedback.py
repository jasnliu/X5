"""Unmodified motor-8 feedback, kept separate from RViz finger geometry."""
import math


def motor8_feedback(count):
    """The exact wire count and its protocol-defined angle; no zero/sign/clamp."""
    if isinstance(count, bool) or not isinstance(count, int) or not 0 <= count <= 65535:
        raise ValueError('Motor 8 encoder count must be an integer from 0 to 65535')
    return {'encoder_count': count, 'raw_rad': count / 65535 * 25.14 - 12.57}


def gripper_feedback_text(feedback):
    if feedback is None:
        return 'Gripper (motor 8): raw feedback unavailable'
    # repr retains the full decoded float; the integer count is the exact value
    # on the wire. Degrees are only a unit conversion, never a width estimate.
    value = motor8_feedback(feedback['encoder_count'])
    return (f"Gripper (motor 8): raw count {value['encoder_count']} / 65535\n"
            f"Raw angle: {value['raw_rad']!r} rad ({math.degrees(value['raw_rad']):+.6f}°)")


class EncoderState(dict):
    """Joint coordinates plus raw telemetry that is NOT a ROS joint name."""
    def __init__(self, *args, motor8_feedback=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.motor8_feedback = {} if motor8_feedback is None else motor8_feedback

    def copy(self):
        return EncoderState(self, motor8_feedback=self.motor8_feedback.copy())
