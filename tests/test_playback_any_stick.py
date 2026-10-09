"""Offline multi-stick regressions; no camera, CAN, or arm movement."""
from itertools import permutations
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock

from camera_playback.app import App
from camera_playback.camera import (
    PlaybackDetectionSender,
    pink_zone_membership,
    select_pink_zone_stick,
)
from camera_playback.playback_transport import PlaybackMotors
from camera_search.protocol import CAMERA_FRESH_SECONDS, DetectionReceiver, DetectionSender
from camera_search.vision import observation_from_message


CYMBAL = {"class_id": 0, "confidence": .9, "box": (0, 0, 300, 300)}


def stick(tip, confidence=.9, **changes):
    return {
        "class_id": 1, "confidence": confidence, "box": (10, 10, 250, 250),
        "tip": tip, "tip_source": "yolo_pose", "tip_status": "observed",
        **changes,
    }


class AnyStickTests(unittest.TestCase):
    def transmit(self, detections, age=0., sender_type=PlaybackDetectionSender):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.sock"
            receiver = DetectionReceiver(path)
            sender = sender_type(path)
            try:
                sender.frame(7, time.monotonic() - age, detections)
                messages = receiver.poll()
                self.assertEqual(len(messages), 1)
                return messages[0]
            finally:
                sender.close()
                receiver.close()

    def test_inside_ride_wins_over_higher_confidence_outside_snare_in_any_order(self):
        ride = stick((150, 150), confidence=.01)
        snare = stick((280, 280), confidence=.999)
        for detections in permutations([CYMBAL, ride, snare]):
            with self.subTest(order=detections):
                self.assertIs(select_pink_zone_stick(detections), ride)
                self.assertIs(pink_zone_membership(detections), True)
                message = self.transmit(detections)
                self.assertEqual(message["tip_point"], (150., 150.))
                self.assertTrue(message["required"])

    def test_any_inside_tip_wins_even_when_outside_tip_is_closer_to_center(self):
        # The inside corner is farther from the center than the outside edge.
        inside = stick((100, 100), confidence=0.)
        outside = stick((99, 150), confidence=1.)
        self.assertIs(select_pink_zone_stick([CYMBAL, outside, inside]), inside)
        self.assertTrue(pink_zone_membership([CYMBAL, outside, inside]))

    def test_each_pink_boundary_is_inside(self):
        outside = stick((0, 0), confidence=1.)
        for tip in ((100, 100), (200, 200), (100, 150), (150, 200), (200, 150), (150, 100)):
            with self.subTest(tip=tip):
                self.assertTrue(pink_zone_membership([CYMBAL, outside, stick(tip, 0.)]))

    def test_stick_confidence_is_never_read(self):
        class UnreadableConfidence(dict):
            def get(self, key, *args):
                if key == "confidence":
                    raise AssertionError("Stick confidence must not choose the tip")
                return super().get(key, *args)

        inside = UnreadableConfidence(stick((150, 150)))
        outside = UnreadableConfidence(stick((250, 250)))
        self.assertIs(select_pink_zone_stick([CYMBAL, outside, inside]), inside)
        self.assertIs(select_pink_zone_stick([CYMBAL, outside]), outside)
        self.assertTrue(self.transmit([CYMBAL, outside, inside])["required"])

    def test_all_outside_uses_nearest_tip_for_correction_without_confidence(self):
        near = stick((201, 150), confidence=.01)
        far = stick((300, 300), confidence=1.)
        for detections in ([CYMBAL, far, near], [CYMBAL, near, far]):
            self.assertIs(select_pink_zone_stick(detections), near)
            self.assertIs(pink_zone_membership(detections), False)
            self.assertEqual(self.transmit(detections)["tip_point"], (201., 150.))

    def test_invalid_predicted_or_wrong_class_inside_tip_cannot_accept(self):
        outside = stick((250, 250))
        changes = (
            {"tip_status": "tracked"}, {"tip_source": "predicted"},
            {"tip": None}, {"tip": (float("nan"), 150)},
            {"box": None}, {"box": (10, 10, 0, 0)}, {"class_id": 2},
        )
        for change in changes:
            with self.subTest(change=change):
                detections = [CYMBAL, outside, {**stick((150, 150)), **change}]
                self.assertFalse(pink_zone_membership(detections))
                self.assertIs(select_pink_zone_stick(detections), outside)

    def test_missing_valid_tip_is_unknown_and_not_required(self):
        for detections in ([], [CYMBAL], [CYMBAL, stick((150, 150), tip_status="tracked")]):
            self.assertIsNone(pink_zone_membership(detections))
            self.assertFalse(self.transmit(detections)["required"])

    def test_missing_cymbal_retains_readiness_but_cannot_accept_alignment(self):
        detections = [stick((150, 150))]
        self.assertIsNone(pink_zone_membership(detections))
        message = self.transmit(detections)
        self.assertTrue(message["required"])
        self.assertIsNone(observation_from_message(message))

    def test_stale_inside_tip_cannot_accept_alignment(self):
        message = self.transmit([CYMBAL, stick((150, 150))], CAMERA_FRESH_SECONDS + 1.)
        self.assertFalse(message["required"])
        self.assertIsNone(observation_from_message(message))

    def test_datagram_accepts_any_inside_tip_in_both_alignment_phases(self):
        message = self.transmit([CYMBAL, stick((280, 280), 1.), stick((100, 100), .01)])
        for phase in ("CHECKING PLAYBACK END", "HILL MEASURING ANCHOR", "HILL MEASURING CANDIDATE"):
            with self.subTest(phase=phase):
                app = App.__new__(App)
                app.phase = phase
                app.measurement_gate_frame_id = 6
                app.measurement_frame_ids = set()
                app._alignment_success = Mock()
                app._plan_next_candidate = Mock()
                app._process_alignment_frame(message, time.monotonic())
                app._alignment_success.assert_called_once_with(7)
                app._plan_next_candidate.assert_not_called()

    def test_existing_frame_gate_still_rejects_old_inside_frame(self):
        message = self.transmit([CYMBAL, stick((150, 150))])
        app = App.__new__(App)
        app.phase = "CHECKING PLAYBACK END"
        app.measurement_gate_frame_id = 7
        app._alignment_success = Mock()
        app._process_alignment_frame(message, time.monotonic())
        app._alignment_success.assert_not_called()

    def test_standalone_search_selection_is_unchanged(self):
        detections = [CYMBAL, stick((280, 280), 1.), stick((150, 150), .01)]
        self.assertEqual(self.transmit(detections, sender_type=DetectionSender)["tip_point"],
                         (280., 280.))
        self.assertEqual(self.transmit(detections)["tip_point"], (150., 150.))

    def test_right_alignment_position_commands_never_route_to_left(self):
        # Exercise the real inherited set_positions/send_control path, replacing
        # only the final CAN write with a mock. The camera never chooses an arm.
        bus = PlaybackMotors.__new__(PlaybackMotors)
        bus.control_side = "right"
        bus.active = True
        bus._send = Mock()
        bus.set_positions([0.] * 7)
        self.assertEqual(bus._send.call_count, 7)
        self.assertTrue(all(call.args[0] == "right" for call in bus._send.call_args_list))


if __name__ == "__main__":
    unittest.main()
