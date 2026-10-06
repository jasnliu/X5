"""Offline tests for recorded-path camera alignment; no CAN or camera required."""
import json
import math
from pathlib import Path
import socket
from types import SimpleNamespace
import tempfile
import time
import unittest
from unittest.mock import Mock, call, patch

import numpy as np

from camera_playback.app import (
    App,
    CENTER_RELAX_PHASE,
    CONTINUOUS_MIN_REBOUND_RAD,
    CONTINUOUS_OUT_PHASE,
    CONTINUOUS_RETURN_PHASE,
    CONTINUOUS_STRIKE_LEAD_SECONDS,
    CONTINUOUS_WAIT_PHASE,
    DEFAULT_RECORDING,
    HARDWARE_TEST_HOLD_FAULT_PHASE,
    HARDWARE_TEST_HOLD_PHASE,
    HARDWARE_TEST_OUT_PHASE,
    HARDWARE_TEST_READY_PHASE,
    HARDWARE_TEST_RESTORE_PHASE,
    HARDWARE_TEST_RETURN_PHASE,
    HARDWARE_TEST_WORKFLOW_FAULT_PHASE,
    PLAYBACK_COMMAND_INTERVAL,
    STRIKE_BPM,
    STRIKE_J7_FEEDBACK_HZ,
    STRIKE_PERIOD_SECONDS,
    STRIKE_SOUND_WAIT_PHASE,
    STRIKE_SOUND_WAIT_SECONDS,
    SWING_EVENTS,
    SWING_PICKUP_INDEX,
    SWING_TRIPLET_SECONDS,
    TEST_STRIKE_DEGREES,
)
from camera_playback.mit_strike import Sample, Status, StrikeSettings
from camera_playback.audio import AudioReceiver, AudioSender
from camera_playback.audio_bridge import (
    DEFAULT_TONOR_SOURCE,
    capture_sources,
    choose_tonor_source,
    detector_command,
)
from camera_playback.camera import (
    MIN_TOTAL_DIMENSION_CHANGE_PX,
    CymbalBoxChangeMonitor,
    PlaybackDetectionSender,
    draw_box_change_status,
    draw_pink_zone_status,
    pink_zone_membership,
    pink_zone_status,
    status_lines,
    total_dimension_change,
)
from camera_playback.hill import GuidedHillClimber
from camera_playback.planner import solve_playback_coordinate
from camera_playback.simulation import SimulatedMotors
from camera_playback.strike import (
    MANUAL_STRIKE_MIN_DEGREES,
    STRIKE_INCREMENT_DEGREES,
    STRIKE_INCREMENT_RAD,
    STRIKE_START_DEGREES,
    StrikeControl,
    StrikePlan,
    build_manual_strike_target,
    build_strike_plan,
)
from camera_playback.trajectory import (
    PLAYBACK_SPEED_FRACTION,
    PlaybackFollower,
    _curve_dynamic_maxima,
    load_playback_trajectory,
)
from camera_playback.visual import (
    DEFAULT_DIRECTION_ORDER,
    PlaybackVisualSample,
    infer_direction_order,
    inside_outer_goal,
    median_outer_result,
    normalized_center_distance,
)
from camera_search.planner import INITIAL_OFFSET, solve_search_coordinate
from camera_search.vision import VisualObservation
from cartesian_goal.ik import CartesianIK
from centering.motors import (
    RIGHT_PLAYBACK_SPEED,
    RIGHT_GRIPPER_CLOSED,
    RIGHT_GRIPPER_OPEN,
    RIGHT_STRIKE_DOWN_SPEED,
    RIGHT_STRIKE_RETURN_SPEED,
    SPEED,
)
from motion_recording.recording import MotionRecording
from safe_zone.geometry import Model, RIGHT_TCP, Zone


ROOT = Path(__file__).resolve().parents[1]


class CameraPlaybackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = Model(ROOT / "model/openarmx.urdf")
        cls.zone = Zone.load(ROOT / "right_zones/zone1.json", cls.model.digest, RIGHT_TCP)
        joints = {joint.get("name"): joint for joint in cls.model.joints}
        limits = [joints[f"openarmx_right_joint{i}"].find("limit") for i in range(1, 8)]
        cls.lower = np.array([float(limit.get("lower")) for limit in limits])
        cls.upper = np.array([float(limit.get("upper")) for limit in limits])
        cls.center = np.zeros(7)
        cls.center[6] = cls.upper[6]
        cls.ik = CartesianIK(
            cls.model, cls.zone, cls.lower, cls.upper, SPEED,
            "right", RIGHT_TCP, cls.center, np.zeros(7),
        )
        cls.anchor = solve_search_coordinate(cls.ik, INITIAL_OFFSET, cls.center).joints
        cls.candidate = solve_search_coordinate(
            cls.ik, INITIAL_OFFSET + np.array([.01, 0, 0]), cls.anchor
        ).joints

    def save_recording(self, directory, joints, stamps):
        recording = MotionRecording(self.model.digest)
        recording.start("2026-09-18T12:00:00.000Z")
        for stamp, values in zip(stamps, joints):
            recording.add(
                stamp,
                {**{f"openarmx_right_joint{i + 1}": float(values[i]) for i in range(7)},
                 "openarmx_right_finger_joint1": .02},
                self.ik.position(values),
            )
        recording.stop()
        path = Path(directory) / "path.json"
        recording.save(path)
        return path

    def load(self, path, progress=None):
        return load_playback_trajectory(
            path, self.model, self.zone, self.lower, self.upper,
            self.center, SPEED, playback_speed=RIGHT_PLAYBACK_SPEED,
            progress=progress,
        )

    def test_recording_load_preflights_and_interpolates_safe_path(self):
        progress = []
        with tempfile.TemporaryDirectory() as directory:
            path = self.save_recording(directory, [self.anchor, self.candidate], [10., 10.5])
            trajectory = self.load(path, progress.append)
        self.assertEqual(trajectory.sample_count, 2)
        self.assertGreaterEqual(trajectory.time_scale, 1.0)
        np.testing.assert_allclose(trajectory.joints_at(0.0)[0], self.anchor)
        midpoint, finished = trajectory.joints_at(trajectory.duration_s / 2.0)
        np.testing.assert_allclose(midpoint, (self.anchor + self.candidate) / 2.0)
        self.assertFalse(finished)
        np.testing.assert_allclose(trajectory.joints_at(trajectory.duration_s + 1)[0], self.candidate)
        self.assertTrue(trajectory.joints_at(trajectory.duration_s + 1)[1])
        self.assertIn("Reading JSON and checking the recording format", progress[0])
        self.assertTrue(any("recorded TCP positions" in item for item in progress))
        self.assertTrue(any("recovery-to-center paths" in item for item in progress))
        self.assertEqual(progress[-1], "Calculating velocity-only playback timing")

    def test_timing_uses_only_peak_velocity_with_a_small_motor_margin(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.save_recording(directory, [self.anchor, self.candidate], [5., 5.001])
            trajectory = self.load(path)
        self.assertAlmostEqual(trajectory.original_duration_s, .001)
        maximum_velocity, maximum_acceleration = _curve_dynamic_maxima(
            trajectory.source_times, trajectory.curve_coefficients
        )
        expected = max(
            1.0, maximum_velocity / (RIGHT_PLAYBACK_SPEED * PLAYBACK_SPEED_FRACTION)
        )
        self.assertAlmostEqual(trajectory.time_scale, expected)
        self.assertGreater(maximum_acceleration, 0.0)
        self.assertLessEqual(
            maximum_velocity / trajectory.time_scale,
            RIGHT_PLAYBACK_SPEED * PLAYBACK_SPEED_FRACTION + 1e-9,
        )

    def test_multi_sample_recording_becomes_a_smooth_best_fit_curve(self):
        stamps = np.linspace(4.0, 4.4, 9)
        samples = []
        for index, fraction in enumerate(np.linspace(0.0, 1.0, len(stamps))):
            point = self.anchor + fraction * (self.candidate - self.anchor)
            if 0 < index < len(stamps) - 1:
                point = point.copy()
                point[5] += (.002 if index % 2 else -.002)
            samples.append(point)
        with tempfile.TemporaryDirectory() as directory:
            trajectory = self.load(self.save_recording(directory, samples, stamps))

        self.assertEqual(PLAYBACK_COMMAND_INTERVAL, .02)
        np.testing.assert_array_equal(trajectory.fitted_joints[0], samples[0])
        np.testing.assert_array_equal(trajectory.fitted_joints[-1], samples[-1])
        self.assertGreater(
            np.max(np.abs(trajectory.fitted_joints[1:-1] - trajectory.joints[1:-1])),
            1e-6,
        )

        coefficients = trajectory.curve_coefficients
        intervals = np.diff(trajectory.source_times)
        np.testing.assert_allclose(coefficients[0, 1], 0.0, atol=1e-10)
        last = coefficients[-1]
        last_interval = intervals[-1]
        np.testing.assert_allclose(
            last[1] + 2.0 * last[2] * last_interval
            + 3.0 * last[3] * last_interval ** 2,
            0.0,
            atol=1e-10,
        )
        for index in range(1, len(coefficients)):
            interval = intervals[index - 1]
            previous = coefficients[index - 1]
            velocity_before = (
                previous[1] + 2.0 * previous[2] * interval
                + 3.0 * previous[3] * interval ** 2
            )
            acceleration_before = previous[2] * 2.0 + previous[3] * 6.0 * interval
            np.testing.assert_allclose(velocity_before, coefficients[index, 1], atol=1e-9)
            np.testing.assert_allclose(
                acceleration_before, coefficients[index, 2] * 2.0, atol=1e-8
            )

        maximum_velocity, _maximum_acceleration = _curve_dynamic_maxima(
            trajectory.source_times, trajectory.curve_coefficients
        )
        self.assertAlmostEqual(
            trajectory.time_scale,
            max(1.0, maximum_velocity / (RIGHT_PLAYBACK_SPEED * PLAYBACK_SPEED_FRACTION)),
        )

    def test_wrong_model_and_outside_zone_recordings_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.save_recording(directory, [self.anchor], [1.])
            payload = json.loads(path.read_text())
            payload["model_sha256"] = "0" * 64
            path.write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError, "model hash"):
                self.load(path)

            path = self.save_recording(directory, [np.zeros(7)], [1.])
            with self.assertRaisesRegex(ValueError, "leaves right zone1"):
                self.load(path)

    def test_playback_candidate_keeps_recorded_joint2(self):
        offset = INITIAL_OFFSET + np.array([.005, 0, 0])
        result = solve_playback_coordinate(self.ik, offset, self.anchor, self.anchor[1])
        self.assertAlmostEqual(result.joints[1], self.anchor[1])
        self.assertLessEqual(result.error_m, .002)

    def test_visible_pink_rectangle_is_the_complete_playback_goal(self):
        edge = VisualObservation(
            (110, 150), (0, 0, 300, 300), (100, 100, 200, 200)
        )
        center = VisualObservation(
            (150, 150), (0, 0, 300, 300), (100, 100, 200, 200)
        )
        self.assertTrue(inside_outer_goal(edge))
        self.assertGreater(normalized_center_distance(edge), 0.0)
        self.assertEqual(normalized_center_distance(center), 0.0)
        _median, _score, acquired = median_outer_result([edge])
        self.assertTrue(acquired)

    def test_camera_preview_reports_only_dimension_changes_above_minimum(self):
        monitor = CymbalBoxChangeMonitor(MIN_TOTAL_DIMENSION_CHANGE_PX)

        def cymbal(width, height):
            return [{
                "class_id": 0, "confidence": .9,
                "box": (10, 20, 10 + width, 20 + height),
            }]

        baseline = monitor.update(cymbal(100, 80))
        self.assertTrue(baseline.baseline)
        self.assertFalse(baseline.changing)

        ignored = monitor.update(cymbal(
            100 + MIN_TOTAL_DIMENSION_CHANGE_PX // 2,
            80 - MIN_TOTAL_DIMENSION_CHANGE_PX // 2,
        ))
        self.assertFalse(ignored.changing)
        self.assertIn("STABLE", status_lines(ignored)[0])
        self.assertEqual(ignored.total_change_px, MIN_TOTAL_DIMENSION_CHANGE_PX)

        changed = monitor.update(cymbal(110, 80))
        self.assertTrue(changed.changing)
        self.assertEqual(
            changed.total_change_px,
            abs(changed.delta_width_px) + abs(changed.delta_height_px),
        )
        self.assertGreater(changed.total_change_px, MIN_TOTAL_DIMENSION_CHANGE_PX)
        self.assertIn("VISUAL CYMBAL MOVEMENT", status_lines(changed)[0])
        self.assertIn("NOT HIT TRIGGER", status_lines(changed)[0])
        self.assertIn(
            f"TOTAL CHANGE={changed.total_change_px}px", status_lines(changed)[0]
        )
        self.assertEqual(total_dimension_change(-6, 4), 10)

        missing = monitor.update([])
        self.assertFalse(missing.visible)
        self.assertIn("NOT DETECTED", status_lines(missing)[0])
        self.assertTrue(monitor.update(cymbal(100, 80)).baseline)

        class FakeCV2:
            FONT_HERSHEY_SIMPLEX = 1
            LINE_AA = 2

            def __init__(self):
                self.calls = []

            def putText(self, *args):
                self.calls.append(args)

        fake_cv2 = FakeCV2()
        preview = object()
        self.assertIs(draw_box_change_status(preview, changed, fake_cv2), preview)
        self.assertEqual(len(fake_cv2.calls), 4)  # dark outline + color for two lines
        self.assertTrue(any("TOTAL CHANGE=" in call[1] for call in fake_cv2.calls))

    def test_camera_preview_monitor_does_not_change_detection_datagram(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "d.sock")
            receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            receiver.bind(path)
            receiver.settimeout(1.0)
            sender = PlaybackDetectionSender(path)
            try:
                sender.frame(7, time.monotonic(), [{
                    "class_id": 0,
                    "confidence": .9,
                    "box": (10, 20, 110, 100),
                }])
                payload = json.loads(receiver.recv(8192))
            finally:
                sender.close()
                receiver.close()
        self.assertEqual(payload["frame_id"], 7)
        self.assertEqual(payload["cymbal_box"], [10, 20, 110, 100])
        self.assertNotIn("box_change", payload)
        self.assertNotIn("changing", payload)

    def test_camera_preview_reports_visible_pink_zone_membership(self):
        cymbal_detection = {
            "class_id": 0, "confidence": .9, "box": (0, 0, 300, 300),
        }

        def stick(tip, status="observed"):
            return {
                "class_id": 1, "confidence": .9, "box": (100, 50, 200, 250),
                "tip": tip, "tip_source": "yolo_pose", "tip_status": status,
            }

        inside = [cymbal_detection, stick((100, 200))]
        outside = [cymbal_detection, stick((99, 200))]
        unavailable = [cymbal_detection, stick((150, 150), status="tracked")]

        self.assertIs(pink_zone_membership(inside), True)
        self.assertIs(pink_zone_membership(outside), False)
        self.assertIsNone(pink_zone_membership(unavailable))
        self.assertIn("YES", pink_zone_status(inside)[0])
        self.assertIn("NO", pink_zone_status(outside)[0])
        self.assertIn("UNKNOWN", pink_zone_status(unavailable)[0])

        class FakeCV2:
            FONT_HERSHEY_SIMPLEX = 1
            LINE_AA = 2

            def __init__(self):
                self.calls = []

            def putText(self, *args):
                self.calls.append(args)

        fake_cv2 = FakeCV2()
        preview = object()
        self.assertIs(draw_pink_zone_status(preview, inside, fake_cv2), preview)
        self.assertEqual(len(fake_cv2.calls), 2)  # dark outline + colored text
        self.assertTrue(all("YES" in call[1] for call in fake_cv2.calls))

    def test_playback_evidence_prefers_reverse_of_worsening_y_motion(self):
        evidence = [
            PlaybackVisualSample(float(index), np.array([0., index * .01, 0.]), .1 + index * .05)
            for index in range(5)
        ]
        order, detail = infer_direction_order(evidence)
        self.assertEqual(order[0], "-Y")
        self.assertEqual(order[-1], "+Y")
        self.assertIn("playback-guided", detail)
        fallback, _ = infer_direction_order(evidence[:2])
        self.assertEqual(fallback, DEFAULT_DIRECTION_ORDER)

    def test_guided_hill_climber_uses_prior_but_still_tests_all_directions(self):
        order = ("-Y", "+X", "-X", "+Z", "-Z", "+Y")
        hill = GuidedHillClimber(np.zeros(3), np.zeros(7), order, anchor_score=1.0)
        visited = []
        for _ in range(6):
            visited.append(hill.direction_name)
            hill.reject_or_skip()
        self.assertEqual(tuple(visited), order)
        self.assertEqual(hill.step, .005)

    def test_playback_follower_preserves_path_and_detects_tracking_failure(self):
        follower = PlaybackFollower()
        desired = np.full(7, .1)
        np.testing.assert_array_equal(follower.update(np.zeros(7), desired, 0.0), desired)
        follower = PlaybackFollower()
        far = np.full(7, math.radians(20))
        follower.update(np.zeros(7), far, 0.0)
        with self.assertRaisesRegex(RuntimeError, "tracking error"):
            follower.update(np.zeros(7), far, 1.1)

    def test_strike_plan_uses_safe_one_degree_steps_starting_at_five(self):
        class LinearJ7Model:
            def transforms(self, values):
                transform = np.eye(4)
                transform[0, 3] = values["openarmx_right_joint7"]
                return {RIGHT_TCP: transform}

        class MinimumJ7Zone:
            def contains(self, point, _buffer):
                return point[0] >= math.radians(-9.5)

        lower = np.full(7, -math.pi)
        upper = np.full(7, math.pi)
        anchor = np.zeros(7)
        plan = build_strike_plan(
            LinearJ7Model(), MinimumJ7Zone(), lower, upper, anchor
        )
        self.assertEqual(STRIKE_START_DEGREES, 5)
        self.assertEqual(STRIKE_INCREMENT_DEGREES, 1)
        self.assertEqual(STRIKE_INCREMENT_RAD, math.radians(1))
        self.assertEqual(len(plan.targets), 5)
        np.testing.assert_allclose(
            [target[6] for target in plan.targets],
            [-math.radians(amount) for amount in (5, 6, 7, 8, 9)],
        )
        self.assertTrue(all(np.array_equal(target[:6], anchor[:6])
                            for target in plan.targets))

    def test_manual_strike_target_accepts_exact_decimal_and_rejects_unsafe_amount(self):
        class LinearJ7Model:
            def transforms(self, values):
                transform = np.eye(4)
                transform[0, 3] = values["openarmx_right_joint7"]
                return {RIGHT_TCP: transform}

        class MinimumJ7Zone:
            def contains(self, point, _buffer):
                return point[0] >= math.radians(-9.5)

        lower = np.full(7, -math.pi)
        upper = np.full(7, math.pi)
        anchor = np.zeros(7)
        target = build_manual_strike_target(
            LinearJ7Model(), MinimumJ7Zone(), lower, upper, anchor, 6.25
        )
        self.assertAlmostEqual(target[6], -math.radians(6.25))
        np.testing.assert_array_equal(target[:6], anchor[:6])

        with self.assertRaisesRegex(ValueError, "leaves right zone1"):
            build_manual_strike_target(
                LinearJ7Model(), MinimumJ7Zone(), lower, upper, anchor, 10.0
            )
        with self.assertRaisesRegex(ValueError, "at least"):
            build_manual_strike_target(
                LinearJ7Model(), MinimumJ7Zone(), lower, upper, anchor,
                MANUAL_STRIKE_MIN_DEGREES / 2.0,
            )

    def test_hardware_test_drop_is_relative_to_current_encoder_and_path_checked(self):
        class LinearJ7Model:
            def transforms(self, values):
                transform = np.eye(4)
                transform[0, 3] = values["openarmx_right_joint7"]
                return {RIGHT_TCP: transform}

        class MinimumJ7Zone:
            def contains(self, point, _buffer):
                return point[0] >= math.radians(50.0)

        lower = np.full(7, -math.pi)
        upper = np.full(7, math.pi)
        anchor = np.zeros(7)
        anchor[6] = math.radians(60.0)
        target = build_manual_strike_target(
            LinearJ7Model(), MinimumJ7Zone(), lower, upper, anchor, 6.25
        )
        self.assertAlmostEqual(math.degrees(target[6]), 53.75)
        np.testing.assert_array_equal(target[:6], anchor[:6])

        with self.assertRaisesRegex(ValueError, "leaves right zone1"):
            build_manual_strike_target(
                LinearJ7Model(), MinimumJ7Zone(), lower, upper, anchor, 11.0
            )

    def test_search_degrees_are_five_six_seven_without_a_tuning_mode(self):
        app = App.__new__(App)
        app.strike_plan = StrikePlan(np.zeros(7), tuple(np.zeros(7) for _ in range(3)))
        self.assertEqual(
            [self._strike_degrees_at(app, index) for index in range(3)],
            [5, 6, 7],
        )

    @staticmethod
    def _strike_degrees_at(app, index):
        app.strike_index = index
        return app._current_strike_degrees()

    def test_test_mode_uses_safe_ten_degree_target_without_discovery(self):
        anchor = np.array([0.1, 0.0, -0.1, 0.4, 0.0, 0.0, 1.0])
        targets = []
        for degrees in range(STRIKE_START_DEGREES, TEST_STRIKE_DEGREES + 1):
            target = anchor.copy()
            target[6] -= math.radians(degrees)
            targets.append(target)
        plan = StrikePlan(anchor.copy(), tuple(targets))
        app = App.__new__(App)
        app.test_mode = True
        app.alignment_active = True
        app.arm = Mock(return_value=anchor)
        app.lower = np.full(7, -2.0)
        app.upper = np.full(7, 2.0)
        app.hill_ik = SimpleNamespace(model=object(), zone=object())
        app._cancel_planning = Mock()
        app._cancel_continuous_striking = Mock()
        app._begin_continuous_striking = Mock()
        app._begin_strike_attempt = Mock()
        app._program_failure = Mock()

        with patch("camera_playback.app.build_strike_plan", return_value=plan) as build:
            app._begin_test_striking()

        build.assert_called_once()
        np.testing.assert_array_equal(build.call_args.args[-1], anchor)
        app._begin_continuous_striking.assert_called_once()
        self.assertEqual(
            app._begin_continuous_striking.call_args.args[0], TEST_STRIKE_DEGREES
        )
        np.testing.assert_array_equal(
            app._begin_continuous_striking.call_args.args[1], targets[-1]
        )
        app._begin_strike_attempt.assert_not_called()
        app._program_failure.assert_not_called()
        self.assertFalse(app.strike_active)

    def test_test_mode_rejects_endpoint_without_safe_ten_degree_target(self):
        anchor = np.zeros(7)
        five_degree_target = anchor.copy()
        five_degree_target[6] -= math.radians(STRIKE_START_DEGREES)
        plan = StrikePlan(anchor.copy(), (five_degree_target,))
        app = App.__new__(App)
        app.test_mode = True
        app.alignment_active = True
        app.arm = Mock(return_value=anchor)
        app.lower = np.full(7, -2.0)
        app.upper = np.full(7, 2.0)
        app.hill_ik = SimpleNamespace(model=object(), zone=object())
        app._cancel_planning = Mock()
        app._cancel_continuous_striking = Mock()
        app._begin_continuous_striking = Mock()
        app._program_failure = Mock()

        with patch("camera_playback.app.build_strike_plan", return_value=plan):
            app._begin_test_striking()

        app._begin_continuous_striking.assert_not_called()
        self.assertIn(
            "10-degree J7 target is not safe",
            app._program_failure.call_args.args[0],
        )

    def test_test_mode_settled_recording_skips_camera_alignment(self):
        app = App.__new__(App)
        app.test_mode = True
        app.phase = "SETTLING RECORDING END"
        app._restore_playback_speed = Mock(return_value=True)
        app._begin_test_striking = Mock()
        app._begin_playback_alignment = Mock()

        app.complete_stage(4.0)

        app._restore_playback_speed.assert_called_once_with("Recording endpoint reached")
        app._begin_test_striking.assert_called_once_with()
        app._begin_playback_alignment.assert_not_called()

    def test_test_mode_readiness_ignores_camera_sound_and_hihat(self):
        app = App.__new__(App)
        app.test_mode = True
        app.hardware = True
        app.receiver = SimpleNamespace(fresh=Mock(return_value=False), cymbal=False)
        app.audio_receiver = SimpleNamespace(ready=Mock(return_value=False))
        app.support_fault_detail = "camera unavailable"
        app.sound_fault_detail = "microphone unavailable"
        app.hihat_fault_detail = "serial unavailable"

        self.assertTrue(app._camera_ready_for_start())
        self.assertTrue(app._sound_ready_for_start())
        self.assertTrue(app._hihat_ready_for_start())
        app.receiver.fresh.assert_not_called()
        app.audio_receiver.ready.assert_not_called()

    def test_simulated_motors_animate_without_socketcan(self):
        initial = np.zeros(7)
        bus = SimulatedMotors(initial)
        self.assertFalse(bus.active)
        self.assertFalse(hasattr(bus, "sockets"))
        setup = bus.center(initial, math.radians(-3))
        next(setup)
        with self.assertRaises(StopIteration):
            next(setup)
        self.assertTrue(bus.active)

        target = initial.copy()
        target[6] = -math.radians(5)
        bus.set_right_joint7_speed(RIGHT_STRIKE_DOWN_SPEED)
        bus.set_right_joint7_feedback_rate(STRIKE_J7_FEEDBACK_HZ)
        bus.set_positions(target)
        bus._last_poll -= 0.1
        bus.poll()
        self.assertAlmostEqual(bus.positions()["openarmx_right_joint7"], target[6])
        bus.set_right_joint7_position(0.0)
        bus._last_poll -= 0.1
        bus.poll()
        self.assertAlmostEqual(bus.positions()["openarmx_right_joint7"], 0.0)
        self.assertEqual(bus.right_joint7_feedback_hz, STRIKE_J7_FEEDBACK_HZ)

        bus.relax()
        self.assertFalse(bus.active)
        self.assertTrue(all(bus.states["right", i][1] == 0 for i in range(1, 9)))
        bus.close()

    def test_first_detected_depth_immediately_becomes_100_bpm_target(self):
        anchor = np.zeros(7)
        targets = []
        for amount in (5, 6, 7):
            target = anchor.copy()
            target[6] = -math.radians(amount)
            targets.append(target)
        app = App.__new__(App)
        app.strike_plan = StrikePlan(anchor, tuple(targets))
        app.strike_index = 2
        app.strike_hit_pending = {"message": "hit", "degrees": 7}
        app.strike_sound_deadline = 5.0
        app.strike_active = True
        app.control = object()
        app.result_status = Mock()
        app.result_label = Mock()
        app.status = Mock()
        app._refresh_buttons = Mock()
        app._program_failure = Mock()

        with patch("camera_playback.app.time.monotonic", return_value=12.0):
            app._finish_hit_strike_attempt()

        self.assertTrue(app.continuous_strike_active)
        self.assertEqual(app.continuous_strike_degrees, 7)
        self.assertEqual(app.continuous_next_beat_at, 12.0)
        self.assertEqual(app.phase, CONTINUOUS_WAIT_PHASE)
        np.testing.assert_array_equal(app.continuous_strike_target, targets[2])
        self.assertIn("FIRST DETECTED STRIKE", app.result_status.set.call_args.args[0])
        app._program_failure.assert_not_called()

    def test_strike_control_commands_exact_goal_and_returns_immediately_at_tolerance(self):
        desired = np.zeros(7)
        desired[6] = -math.radians(10)
        control = StrikeControl(desired, np.full(7, -2.0), np.full(7, 2.0), 0.0)
        command, _error, reached = control.update(np.zeros(7), .01)
        np.testing.assert_array_equal(command, desired)
        self.assertFalse(reached)
        actual = desired.copy()
        actual[6] += math.radians(1.9)
        command, _error, reached = control.update(actual, .02)
        np.testing.assert_array_equal(command, desired)
        self.assertTrue(reached)

    def test_sound_hit_triggers_only_for_the_current_moving_attempt(self):
        app = App.__new__(App)
        app.phase = "STRIKE MOVING OUT"
        app.strike_active = True
        app.strike_index = 0
        app.strike_plan = StrikePlan(np.zeros(7), (np.zeros(7),))
        app.strike_attempt_started_at = 10.0
        app.strike_attempt_returned_at = None
        app.strike_attempt_motion_seen = False
        app.strike_hit_pending = None
        app.arm = Mock(return_value=np.zeros(7))
        app.result_status = Mock()
        app.result_label = Mock()
        app.status = Mock()
        app._finish_hit_strike_attempt = Mock()

        app._process_sound_hit({
            "event_at": 9.0, "detected_at": 10.5, "score": .95,
            "normality_score": 72.0,
        })
        self.assertIsNone(app.strike_hit_pending)

        moved = np.zeros(7)
        moved[6] = -math.radians(5)
        app.arm.return_value = moved
        app._process_sound_hit({
            "event_at": 10.1, "detected_at": 10.7, "score": .97,
            "normality_score": 72.0,
        })
        self.assertIn("ST7 CYMBAL HIT DETECTED", app.strike_hit_pending["message"])
        self.assertEqual(app.strike_hit_pending["degrees"], 5)
        self.assertNotIn("normality_score", app.strike_hit_pending)
        app._finish_hit_strike_attempt.assert_not_called()

    def test_finished_strike_out_immediately_commands_anchor_return(self):
        app = App.__new__(App)
        app.phase = "STRIKE MOVING OUT"
        anchor = np.arange(7, dtype=float) / 10.0
        app.strike_plan = StrikePlan(anchor, (anchor.copy(),))
        app._begin_strike_stage = Mock()
        app.complete_stage(2.0)
        app._begin_strike_stage.assert_called_once()
        self.assertEqual(
            app._begin_strike_stage.call_args.args[0],
            "STRIKE RETURNING TO CYMBAL POSE",
        )
        np.testing.assert_array_equal(app._begin_strike_stage.call_args.args[1], anchor)

    def test_single_hill_measurement_pink_frame_starts_first_strike_immediately(self):
        app = App.__new__(App)
        anchor = np.array([0.1, 0.0, -0.1, 0.4, 0.0, 0.0, 1.0])
        plan = StrikePlan(np.zeros(7), (np.zeros(7),))
        app.alignment_active = True
        app.phase = "HILL MEASURING CANDIDATE"
        app.arm = Mock(return_value=anchor)
        app.lower = np.full(7, -2.0)
        app.upper = np.full(7, 2.0)
        app.center_goal = np.zeros(7)
        app.hill_ik = SimpleNamespace(model=object(), zone=object())
        app.strike_speed_fast = False
        app.bus = Mock()
        app.result_status = Mock()
        app.result_label = Mock()
        app.status = Mock()
        app._cancel_planning = Mock()
        app._begin_strike_attempt = Mock()
        app._program_failure = Mock()
        with patch("camera_playback.app.build_strike_plan", return_value=plan) as build:
            app._alignment_success(21)
        build.assert_called_once()
        np.testing.assert_array_equal(build.call_args.args[-1], anchor)
        app.bus.set_right_joint7_speed.assert_not_called()
        self.assertFalse(app.strike_speed_fast)
        self.assertIs(app.strike_plan, plan)
        app._begin_strike_attempt.assert_called_once_with()
        app._program_failure.assert_not_called()
        self.assertFalse(app.alignment_active)
        self.assertTrue(app.strike_active)
        self.assertIsNone(app.strike_hit_pending)

    @staticmethod
    def mit_sample():
        return Sample(0.0, 0.0, 0.1, 2, time.monotonic())

    def mit_app(self):
        app = App.__new__(App)
        app.phase = HARDWARE_TEST_READY_PHASE
        app.hardware_test_anchor = np.zeros(7)
        app.hardware_test_completed = 0
        app.hardware_test_session = Mock()
        app.hardware_test_session.status = Status("hold", True, sample=self.mit_sample())
        app.hardware_test_session.controller.settings = StrikeSettings()
        app.arm = Mock(return_value=np.zeros(7))
        app.bus = Mock(active=True)
        app.status = Mock()
        app.result_status = Mock()
        app.result_label = Mock()
        app._refresh_hardware_test_controls = Mock()
        app._refresh_buttons = Mock()
        app.fail = Mock()
        return app

    def test_hardware_test_pink_alignment_arms_mit_before_enabling_strikes(self):
        app = self.mit_app()
        anchor = np.zeros(7)
        app.hardware_test_mode = True
        app.alignment_active = True
        app.control = object()
        app.ik = SimpleNamespace(origin_tcp=np.zeros(3))
        app.hill_ik = SimpleNamespace(position=Mock(return_value=np.zeros(3)))
        app.hardware_test_degrees = Mock()
        app._cancel_planning = Mock()
        app._cancel_continuous_striking = Mock()
        app.lower, app.upper = np.full(7, -2.), np.full(7, 2.)
        with patch("camera_playback.app.Joint7Session") as session:
            app._prepare_hardware_test_manual_strikes(44, anchor)
        self.assertEqual(app.phase, HARDWARE_TEST_HOLD_PHASE)
        self.assertIsNone(app.control)
        np.testing.assert_array_equal(app.hardware_test_anchor, anchor)
        self.assertIn("arming MIT hold", app.status.set.call_args.args[0])
        session.assert_called_once_with(app.bus, 0.0, -2.0, 2.0, StrikeSettings())
        app.hardware_test_degrees.set.assert_called_once_with("5")

    def test_hardware_test_single_pink_frame_routes_to_manual_mit_ready_state(self):
        anchor = np.zeros(7)
        app = App.__new__(App)
        app.hardware_test_mode = True
        app.alignment_active = True
        app.phase = "CHECKING PLAYBACK END"
        app.arm = Mock(return_value=anchor)
        app._prepare_hardware_test_manual_strikes = Mock()

        app._alignment_success(73)

        app._prepare_hardware_test_manual_strikes.assert_called_once()
        frame_id, captured = app._prepare_hardware_test_manual_strikes.call_args.args
        self.assertEqual(frame_id, 73)
        np.testing.assert_array_equal(captured, anchor)

    def test_hardware_test_initial_gripper_goal_is_open(self):
        app = App.__new__(App)
        app.hardware_test_mode = True
        self.assertEqual(app.initial_gripper_goal(), RIGHT_GRIPPER_OPEN)

    def test_hardware_test_start_preflights_recording_camera_and_open_load_workflow(self):
        app = App.__new__(App)
        app.hardware = True
        app.hardware_test_mode = True
        app.test_mode = False
        app.phase = "READY"
        app.bus = Mock(active=False)
        app.bus.fresh.return_value = True
        app.playback_trajectory = SimpleNamespace(
            first_joints=np.zeros(7), tcp_positions=np.zeros((1, 3))
        )
        app.center_goal = np.zeros(7)
        app.zone = Mock()
        app.zone.contains.return_value = True
        app.planned_tcp = Mock(return_value=np.zeros(3))
        app.tcp = Mock(return_value=np.zeros(3))
        app.ik = SimpleNamespace(origin_tcp=np.zeros(3))
        app._clear_alignment_state = Mock()
        app.receiver = Mock(fresh=Mock(return_value=True), cymbal=True)
        app._sound_ready_for_start = Mock(return_value=True)
        app.support_fault_detail = None
        app.sound_fault_detail = "informational audio fault"
        app.hihat_fault_detail = None
        app.status = Mock()
        app.result_status = Mock()
        app.result_label = Mock()
        app.root = Mock()

        app.start()

        self.assertEqual(app.phase, "PLAYBACK PREFLIGHTED")
        app.root.after.assert_called_once_with(20, app._start_preflighted_recording)
        self.assertFalse(app.gripper_closed_latched)
        self.assertIsNone(app.gripper_close_until)

    def test_hardware_test_accepts_recording_loader(self):
        app = App.__new__(App)
        app.hardware_test_mode = True
        app.test_mode = False
        app.bus = None
        app.status = Mock()
        app.model = object()
        app.zone = object()
        app.lower = np.zeros(7)
        app.upper = np.ones(7)
        app.center_goal = np.zeros(7)
        app.ik = SimpleNamespace(speed=0.4, origin_tcp=np.zeros(3))
        app._show_recording_loading = Mock()
        app._hide_recording_loading = Mock()
        app.recording_status = Mock()
        app.recording_label = Mock()
        app.result_status = Mock()
        app.result_label = Mock()
        app._refresh_buttons = Mock()
        trajectory = SimpleNamespace(
            first_joints=np.zeros(7), tcp_positions=np.zeros((1, 3)),
            time_scale=1.0, source=Path("manual.json"), sample_count=2,
            original_duration_s=1.0, duration_s=1.0,
        )

        with patch(
                "camera_playback.app.load_playback_trajectory",
                return_value=trajectory,
        ) as loader:
            self.assertTrue(app.load_recording("manual.json"))

        loader.assert_called_once()
        self.assertIs(app.playback_trajectory, trajectory)

    def test_hardware_test_button_keeps_fixed_reference_for_decimal_drop(self):
        app = self.mit_app()
        app.hardware_test_mode = app.hardware = True
        app.phase = HARDWARE_TEST_READY_PHASE
        app.setup = None
        app.hardware_test_degrees = SimpleNamespace(get=lambda: "7.25")
        app.hardware_test_strike_count = 0
        app.arm.return_value = np.full(7, 0.1)  # Never recapture this as the anchor.
        app.hill_ik = SimpleNamespace(model=object(), zone=object())
        app.lower, app.upper = np.full(7, -2.), np.full(7, 2.)
        target = np.zeros(7); target[6] = -math.radians(7.25)
        with patch("camera_playback.app.build_manual_strike_target", return_value=target) as build:
            app.hardware_test_strike_once()
        np.testing.assert_array_equal(build.call_args.args[-2], np.zeros(7))
        self.assertEqual(build.call_args.args[-1], 7.25)
        np.testing.assert_array_equal(app.hardware_test_anchor, np.zeros(7))
        app.hardware_test_session.strike.assert_called_once_with(math.radians(7.25))
        self.assertEqual(app.phase, HARDWARE_TEST_OUT_PHASE)
        app.hardware_test_strike_once()
        self.assertEqual(app.hardware_test_session.strike.call_count, 1)

    def test_hardware_test_controls_show_entered_degree_and_disable_during_motion(self):
        app = App.__new__(App)
        app._hardware_test_session_ready = Mock(return_value=True)
        app.hardware_test_button = Mock()
        app.hardware_test_entry = Mock()
        app.hardware_test_degrees = SimpleNamespace(get=Mock(return_value="6.125"))
        app.hardware = True
        app.bus = Mock(active=True)
        app.bus.fresh.return_value = True
        app.setup = None
        app.phase = HARDWARE_TEST_READY_PHASE

        app._refresh_hardware_test_controls()

        self.assertEqual(
            app.hardware_test_button.config.call_args.kwargs,
            {"text": "FREEFALL 6.125° + REBOUND", "state": "normal"},
        )
        app.hardware_test_entry.config.assert_called_with(state="normal")

        app.hardware_test_degrees_value = 6.125
        app.phase = HARDWARE_TEST_OUT_PHASE
        app._refresh_hardware_test_controls()
        self.assertEqual(
            app.hardware_test_button.config.call_args.kwargs,
            {"text": "MIT FREEFALL 6.125° DOWN…", "state": "disabled"},
        )
        app.hardware_test_entry.config.assert_called_with(state="disabled")

    def test_hardware_test_gui_never_clocks_the_fall_or_catch(self):
        app = self.mit_app()
        app.hardware_test_session.status = Status("fall", sample=self.mit_sample())
        app._advance_hardware_test_mit(time.monotonic())
        self.assertEqual(app.phase, HARDWARE_TEST_OUT_PHASE)
        app.hardware_test_session.status = Status("catch", sample=self.mit_sample())
        app._advance_hardware_test_mit(time.monotonic())
        self.assertEqual(app.phase, HARDWARE_TEST_RETURN_PHASE)
        self.assertEqual(app.bus.method_calls, [])

    def test_hardware_test_gui_does_not_send_extra_upward_command_at_anchor(self):
        app = self.mit_app()
        app.hardware_test_degrees_value = 2.0
        app.hardware_test_session.status = Status("hold", True, 1, self.mit_sample())
        app._advance_hardware_test_mit(time.monotonic())
        self.assertEqual(app.phase, HARDWARE_TEST_READY_PHASE)
        self.assertEqual(app.bus.method_calls, [])
        self.assertIn("encoder position stable", app.status.set.call_args.args[0])

    def test_hardware_test_completion_stays_in_mit_and_logs_measured_depth(self):
        app = self.mit_app()
        app.hardware_test_degrees_value = 1.0
        app.hardware_test_session.status = Status(
            "hold", True, 1, self.mit_sample(), math.radians(1.1), math.radians(.05))
        app._advance_hardware_test_mit(time.monotonic())
        self.assertEqual(app.hardware_test_completed, 1)
        self.assertIn("measured peak drop 1.100", app.result_status.set.call_args.args[0])
        self.assertIn("MIT remains enabled", app.result_status.set.call_args.args[0])
        app.bus.restore_right_joint7_csp.assert_not_called()
        app.hardware_test_session.stop.assert_not_called()

    def test_hardware_test_session_exit_requires_fresh_powered_csp_feedback(self):
        app = self.mit_app()
        app.hardware_test_session = None
        app.phase = HARDWARE_TEST_RESTORE_PHASE
        app.hardware_test_csp_enabled_at = 10.0
        app.hardware_test_restore_deadline = 12.0
        app.hardware_test_last_enable_retry = 10.0
        app.center_goal = np.zeros(7)
        app.begin_stage = Mock()
        app.bus.right_joint7_mode_readback.return_value = 5
        app.bus.right_joint7_operating_state.return_value = (2, 9.99)
        app._advance_hardware_test_mit(10.01)
        app.begin_stage.assert_not_called()
        app.bus.right_joint7_operating_state.return_value = (0, 10.10)
        app.bus.reassert_right_joint7_csp_hold.return_value = 10.11
        app._advance_hardware_test_mit(10.11)
        app.bus.reassert_right_joint7_csp_hold.assert_called_once_with(0.0)
        app.begin_stage.assert_not_called()
        app.bus.right_joint7_operating_state.return_value = (2, 10.12)
        app._advance_hardware_test_mit(10.12)
        app.begin_stage.assert_called_once()
        self.assertEqual(app.begin_stage.call_args.args[0], CENTER_RELAX_PHASE)

    def test_hardware_test_does_not_mark_overshot_moving_arm_ready(self):
        app = self.mit_app()
        sample = Sample(math.radians(2), 1.0, 0.1, 2, time.monotonic())
        app.hardware_test_session.status = Status("hold", False, 0, sample)
        app._advance_hardware_test_mit(time.monotonic())
        self.assertEqual(app.phase, HARDWARE_TEST_HOLD_PHASE)
        self.assertFalse(app._hardware_test_session_ready())
        app.bus.restore_right_joint7_csp.assert_not_called()

    def test_hardware_test_ui_has_heartbeat_budget_not_motor_deadline(self):
        app=self.mit_app()
        with patch('camera_playback.app.time.monotonic',return_value=10.):
            app.hardware_test_session.status=Status('hold',True,sample=Sample(0,.13,.1,2,9.95))
            self.assertTrue(app._hardware_test_session_ready())
            app.hardware_test_session.status=Status('hold',True,sample=Sample(0,0,.1,2,9.89))
            self.assertFalse(app._hardware_test_session_ready())
            app.hardware_test_session.status=Status('fall',True,sample=Sample(0,0,.1,2,10.))
            self.assertFalse(app._hardware_test_session_ready())

    def test_hardware_test_ui_uses_one_snapshot_for_phase_and_button(self):
        app=self.mit_app();snapshot=Status('hold',True,sample=self.mit_sample())
        class Session:
            reads=0
            controller=SimpleNamespace(settings=StrikeSettings())
            @property
            def status(self):
                self.reads+=1
                return snapshot if self.reads==1 else Status('hold',False,sample=snapshot.sample)
        app.hardware_test_session=Session()
        app._advance_hardware_test_mit(time.monotonic())
        self.assertEqual(app.hardware_test_session.reads,1)
        self.assertEqual(app.phase,HARDWARE_TEST_READY_PHASE)
        app._refresh_hardware_test_controls.assert_called_once_with(snapshot)

    def test_hardware_test_reports_other_joint_drift_separately(self):
        app = self.mit_app()
        app.arm.return_value[0] = math.radians(4)
        app._advance_hardware_test_mit(time.monotonic())
        app.fail.assert_called_once()
        self.assertIn("J1-J6 drift exceeded", app.fail.call_args.args[0])

    def test_hardware_test_worker_error_is_forwarded_to_existing_fault_handler(self):
        app = self.mit_app()
        app.hardware_test_session.status = Status(error="J7 feedback stale")
        app._advance_hardware_test_mit(time.monotonic())
        app.fail.assert_called_once_with("J7 feedback stale")

    def test_hardware_test_exit_mode_confirmation_times_out(self):
        app = self.mit_app()
        app.hardware_test_session = None
        app.phase = HARDWARE_TEST_RESTORE_PHASE
        app.hardware_test_restore_deadline = 12.0
        app.hardware_test_csp_enabled_at = 10.0
        app.bus.right_joint7_mode_readback.return_value = None
        app.bus.right_joint7_operating_state.return_value = None
        app._advance_hardware_test_mit(12.01)
        app.fail.assert_called_once_with("J7 position-mode confirmation missing on MIT session exit")

    def test_hardware_test_mode_fault_enters_powered_hold_without_relaxing_arm(self):
        anchor = np.zeros(7)
        anchor[6] = 1.1
        app = App.__new__(App)
        app.hardware_test_mit_active = True
        app.hardware_test_anchor = anchor
        app.bus = Mock(active=True)
        app.setup = object()
        app.control = object()
        app.hold_until = 20.0
        app.hardware_test_last_mit_command = 0.0
        app.result_status = Mock()
        app.result_label = Mock()
        app.status = Mock()
        app._refresh_buttons = Mock()

        with patch("camera_playback.app.time.monotonic", return_value=10.0):
            app.fail("J7 run-mode 5 readback missing during hardware test")

        self.assertEqual(app.phase, HARDWARE_TEST_HOLD_FAULT_PHASE)
        self.assertTrue(app.hardware_test_mit_active)
        app.bus.hold_right_joint7_unknown_mode.assert_called_once_with(1.1)
        app.bus.relax.assert_not_called()
        message = app.result_status.set.call_args.args[0]
        self.assertIn("right arm remains powered", message)
        self.assertIn("strikes locked", message)

    def test_hardware_test_fault_reenables_j7_when_csp_mode_is_known(self):
        anchor = np.zeros(7)
        anchor[6] = 1.1
        app = App.__new__(App)
        app.hardware_test_mit_active = True
        app.hardware_test_anchor = anchor
        app.bus = Mock(active=True)
        app.bus.right_joint7_mode_readback.return_value = 5
        app.setup = None
        app.control = None
        app.hold_until = None
        app.hardware_test_last_mit_command = 0.0
        app.result_status = Mock()
        app.result_label = Mock()
        app.status = Mock()
        app._refresh_buttons = Mock()

        with patch("camera_playback.app.time.monotonic", return_value=10.0):
            app.fail("J7 CSP powered-hold confirmation missing after rebound")

        app.bus.reassert_right_joint7_csp_hold.assert_called_once_with(1.1)
        app.bus.hold_right_joint7_unknown_mode.assert_not_called()
        app.bus.relax.assert_not_called()

    def test_hardware_test_stall_still_requests_whole_arm_relaxation(self):
        app = App.__new__(App)
        app.hardware_test_mode = True
        app.hardware_test_mit_active = True
        app.side = "right"
        app.search_active = True
        app.pending_detection_frame_id = 3
        app.result_status = Mock()
        app.result_label = Mock()
        app._clear_alignment_state = Mock()
        app.relax = Mock()

        app.fail("STALL: motor 7")

        app.relax.assert_called_once()
        self.assertIn("motors disabled", app.relax.call_args.args[0])
        self.assertEqual(app.phase, "FAULT")

    def test_hardware_test_worker_is_joined_before_gui_takes_j7_ownership(self):
        app = self.mit_app()
        worker = app.hardware_test_session
        app._stop_hardware_test_session()
        worker.stop.assert_called_once_with()
        self.assertIsNone(app.hardware_test_session)
        app._stop_hardware_test_session()
        self.assertEqual(worker.stop.call_count, 1)

    def test_hardware_test_startup_motor_fault_isolates_only_named_drive(self):
        app = App.__new__(App)
        app.hardware_test_mode = True
        app.phase = "SETTING UP"
        app.bus = Mock(active=True)
        app.arm = Mock(return_value=np.zeros(7))
        app._cancel_planning = Mock()
        app._cancel_continuous_striking = Mock()
        app.gripper_closed_latched = False
        app.playback_speed_fast = False
        app.strike_feedback_fast = False
        app.strike_speed_fast = False
        app.result_status = Mock()
        app.result_label = Mock()
        app.status = Mock()
        app.relax = Mock()
        app.begin_stage = Mock()

        app.fail(
            "Motor 7: CSP enable not confirmed after 3 attempts "
            "(run-mode readback=5, feedback operating state=0, encoder=20.000 deg)"
        )

        app.begin_stage.assert_not_called()
        app.relax.assert_not_called()
        app.bus.isolate_control_motor.assert_called_once_with(7)
        app.bus.set_positions.assert_called_once()
        self.assertEqual(app.phase, HARDWARE_TEST_WORKFLOW_FAULT_PHASE)
        self.assertIn("motor 7 disabled in isolation", app.status.set.call_args.args[0])

    def test_hardware_test_generic_fault_holds_without_whole_arm_relaxation(self):
        hold = np.arange(7, dtype=float) * 0.1
        app = App.__new__(App)
        app.hardware_test_mode = True
        app.hardware_test_mit_active = False
        app.phase = "RECORDING PLAYBACK"
        app.bus = Mock(active=True)
        app.arm = Mock(return_value=hold)
        app._cancel_planning = Mock()
        app._cancel_continuous_striking = Mock()
        app.gripper_closed_latched = True
        app.playback_speed_fast = False
        app.strike_feedback_fast = False
        app.strike_speed_fast = False
        app.result_status = Mock()
        app.result_label = Mock()
        app.status = Mock()
        app.relax = Mock()

        app.fail("processed camera detections became stale")

        app.relax.assert_not_called()
        app.bus.isolate_control_motor.assert_not_called()
        app.bus.set_positions.assert_called_once()
        np.testing.assert_array_equal(app.bus.set_positions.call_args.args[0], hold)
        app.bus.set_gripper.assert_called_once_with(RIGHT_GRIPPER_CLOSED)
        self.assertEqual(app.phase, HARDWARE_TEST_WORKFLOW_FAULT_PHASE)

    def test_hardware_test_speed_restore_fault_never_auto_relaxes(self):
        app = App.__new__(App)
        app.hardware_test_mode = True
        app.playback_speed_fast = True
        app.bus = Mock(active=True)
        app.bus.set_right_arm_speed.side_effect = RuntimeError("CAN write failed")
        app.playback_active = True
        app.result_status = Mock()
        app.result_label = Mock()
        app.fail = Mock()
        app.relax = Mock()

        self.assertFalse(app._restore_playback_speed("playback ended"))

        app.fail.assert_called_once()
        app.relax.assert_not_called()

    def test_fault_motor_parser_never_maps_a_left_fault_to_the_right_arm(self):
        self.assertIsNone(App._fault_motor_number("left motor 3: fault"))
        self.assertEqual(App._fault_motor_number("right motor 3: fault"), 3)
        self.assertEqual(App._fault_motor_number("Motor 7: CSP enable failed"), 7)

    def test_hardware_test_powered_fault_hold_is_refreshed(self):
        anchor = np.zeros(7)
        anchor[6] = 1.2
        app = App.__new__(App)
        app.phase = HARDWARE_TEST_HOLD_FAULT_PHASE
        app.hardware_test_anchor = anchor
        app.hardware_test_last_mit_command = 9.0
        app.hardware_test_hold_delivery_error = "old failure"
        app.bus = Mock(active=True)

        app._maintain_hardware_test_fault_hold(10.0)

        app.bus.hold_right_joint7_unknown_mode.assert_called_once_with(1.2)
        self.assertEqual(app.hardware_test_last_mit_command, 10.0)
        self.assertIsNone(app.hardware_test_hold_delivery_error)

    def test_hardware_test_powered_hold_blocks_inherited_zone_recenter(self):
        app = App.__new__(App)
        app.hardware_test_mit_active = True
        app.phase = HARDWARE_TEST_HOLD_FAULT_PHASE
        app.bus = Mock(active=True)
        app.zone = Mock()
        app.zone.contains.return_value = False
        app.tcp = Mock(return_value=np.zeros(3))
        app.zone_status = Mock()
        app.fail = Mock()

        app.safety()

        app.fail.assert_not_called()
        app.zone_status.set.assert_called_once_with("Zone: OUTSIDE")
        app.bus.set_positions.assert_not_called()

    def test_hardware_test_center_exit_joins_before_restoring_csp(self):
        app=self.mit_app()
        app.hardware=True;app.hardware_test_mit_active=True;app.setup=None
        app.center_goal=np.zeros(7)
        app.zone=Mock();app.zone.contains.return_value=True
        app.planned_tcp=Mock(return_value=np.zeros(3))
        events=[]
        app.hardware_test_session.stop.side_effect=lambda:events.append('join')
        app.bus.restore_right_joint7_csp.side_effect=lambda q:events.append('csp') or 1.
        app.center_relax()
        self.assertEqual(events,['join','csp'])
        self.assertIsNone(app.hardware_test_session)
        self.assertEqual(app.phase,HARDWARE_TEST_RESTORE_PHASE)

    def test_hardware_test_center_exit_rechecks_worker_stationary_state(self):
        app=self.mit_app()
        app.hardware=True;app.hardware_test_mit_active=True;app.setup=None
        app.center_goal=np.zeros(7)
        app.zone=Mock();app.zone.contains.return_value=True
        app.planned_tcp=Mock(return_value=np.zeros(3))
        app.hardware_test_session.status=Status('hold',False,sample=self.mit_sample())
        app.center_relax()
        app.hardware_test_session.stop.assert_not_called()
        app.bus.restore_right_joint7_csp.assert_not_called()

    def test_hardware_test_logs_sound_without_using_it_as_a_trigger(self):
        app = App.__new__(App)
        app.hardware_test_mode = True
        app.bus = Mock(active=True)
        self.assertTrue(app._hardware_test_sound_monitor_active())

    def test_hardware_test_does_not_require_or_open_hihat(self):
        app = App.__new__(App)
        app.test_mode = False
        app.hardware_test_mode = True
        app.hardware = True
        app.hihat = None
        app.hihat_fault_detail = "ignored for manual arm strikes"
        app.bus = Mock(active=True)
        app.phase = HARDWARE_TEST_READY_PHASE

        self.assertTrue(app._hihat_ready_for_start())
        self.assertFalse(app._hihat_required_now())

    def test_hardware_test_recording_preflight_starts_confirmed_open_center(self):
        app = App.__new__(App)
        app.phase = "PLAYBACK PREFLIGHTED"
        app.test_mode = False
        app.hardware_test_mode = True
        app.status = Mock()
        app.receiver = Mock()
        app.audio_receiver = Mock()
        app.hihat = None
        app.bus = Mock()
        app.bus.fresh.return_value = True
        app._camera_ready_for_start = Mock(return_value=True)
        app._sound_ready_for_start = Mock(return_value=True)
        app._hihat_ready_for_start = Mock(return_value=True)
        app.zone = Mock()
        app.zone.contains.return_value = True
        app.tcp = Mock(return_value=np.zeros(3))
        app.center_goal = np.zeros(7)
        setup = object()
        app.bus.center.return_value = setup

        with patch("camera_playback.app.JointGoalApp.start") as start:
            app._start_preflighted_recording()

        app.receiver.poll.assert_called_once_with()
        app.audio_receiver.poll.assert_called_once_with()
        start.assert_not_called()
        app.bus.center.assert_called_once_with(
            app.center_goal, RIGHT_GRIPPER_OPEN, confirm_enabled=True,
        )
        self.assertIs(app.setup, setup)
        self.assertEqual(app.phase, "SETTING UP")

    def test_normal_hardware_preflight_still_ticks_hihat_controller(self):
        app = App.__new__(App)
        app.phase = "PLAYBACK PREFLIGHTED"
        app.test_mode = False
        app.hardware_test_mode = False
        app.receiver = Mock()
        app.audio_receiver = Mock()
        app.hihat = Mock()
        app.bus = Mock()
        app.bus.fresh.return_value = True
        app._camera_ready_for_start = Mock(return_value=True)
        app._sound_ready_for_start = Mock(return_value=True)
        app._hihat_ready_for_start = Mock(return_value=True)

        with patch("camera_playback.app.JointGoalApp.start") as start:
            app._start_preflighted_recording()

        app.hihat.tick.assert_called_once_with()
        start.assert_called_once_with(app)

    def test_discovery_strike_keeps_speed_and_sends_j7_return_immediately(self):
        app = App.__new__(App)
        app.lower = np.full(7, -2.0)
        app.upper = np.full(7, 2.0)
        app.ik = SimpleNamespace(origin_tcp=np.zeros(3))
        app.hill_ik = SimpleNamespace(position=Mock(return_value=np.zeros(3)))
        app.strike_plan = StrikePlan(np.zeros(7), (np.zeros(7),))
        app.strike_index = 0
        app.strike_speed_fast = False
        app.status = Mock()
        app._program_failure = Mock()
        events = []
        app.bus = Mock()
        app.bus.set_right_joint7_speed.side_effect = (
            lambda speed: events.append(("speed", speed))
        )
        app.bus.set_positions.side_effect = (
            lambda target: events.append(("target", float(target[6])))
        )
        app.bus.set_right_joint7_position.side_effect = (
            lambda target: events.append(("j7 target", float(target)))
        )

        down = np.zeros(7)
        down[6] = -math.radians(5)
        app._begin_strike_stage("STRIKE MOVING OUT", down)
        app._begin_strike_stage("STRIKE RETURNING TO CYMBAL POSE", np.zeros(7))

        self.assertEqual(events, [
            ("speed", RIGHT_STRIKE_DOWN_SPEED),
            ("target", float(down[6])),
            ("j7 target", 0.0),
        ])
        self.assertEqual(RIGHT_STRIKE_RETURN_SPEED, RIGHT_STRIKE_DOWN_SPEED)
        app.bus.set_right_joint7_feedback_rate.assert_called_once_with(
            STRIKE_J7_FEEDBACK_HZ
        )
        self.assertTrue(app.strike_speed_fast)
        app._program_failure.assert_not_called()

    def test_detected_sound_hit_finishes_return_then_starts_same_depth_at_100_bpm(self):
        app = App.__new__(App)
        anchor = np.zeros(7)
        anchor[6] = math.radians(80)
        target = anchor.copy()
        target[6] = anchor[6] - math.radians(6)
        app.phase = "STRIKE MOVING OUT"
        app.strike_active = True
        app.strike_index = 1
        app.strike_plan = StrikePlan(anchor, (anchor.copy(), target))
        app.strike_speed_fast = True
        app.strike_selected_speed = RIGHT_STRIKE_DOWN_SPEED
        app.strike_feedback_fast = True
        app.strike_hit_pending = None
        app.strike_attempt_started_at = 2.0
        app.strike_attempt_returned_at = None
        app.strike_attempt_motion_seen = True
        app.strike_sound_deadline = None
        app.arm = Mock(return_value=target)
        app.bus = Mock(active=True)
        app.center_goal = np.ones(7)
        app.result_status = Mock()
        app.result_label = Mock()
        app.status = Mock()
        app._begin_strike_attempt = Mock()
        app._begin_continuous_striking = Mock()
        app._program_failure = Mock()

        app._process_sound_hit({
            "event_at": 2.2, "detected_at": 2.8, "score": .98,
            "normality_score": 91.0,
        })
        app.bus.set_right_joint7_speed.assert_not_called()
        app._begin_strike_attempt.assert_not_called()
        self.assertTrue(app.strike_active)

        app.phase = "STRIKE RETURNING TO CYMBAL POSE"
        app.complete_stage(3.0)
        app.bus.set_right_joint7_speed.assert_not_called()
        app._begin_strike_attempt.assert_not_called()
        self.assertTrue(app.strike_active)
        self.assertTrue(app.strike_speed_fast)
        app._begin_continuous_striking.assert_called_once()
        self.assertEqual(app._begin_continuous_striking.call_args.args[0], 6)
        np.testing.assert_array_equal(
            app._begin_continuous_striking.call_args.args[1], target
        )

    def test_detected_depth_starts_continuous_100_bpm_mode(self):
        app = App.__new__(App)
        target = np.arange(7, dtype=float) / 10.0
        app.strike_active = True
        app.strike_sound_deadline = 5.0
        app.strike_hit_pending = {"message": "hit"}
        app.control = object()
        app.result_status = Mock()
        app.result_label = Mock()
        app.status = Mock()
        app._refresh_buttons = Mock()
        app.hihat = Mock()

        with patch("camera_playback.app.time.monotonic", return_value=12.0):
            app._begin_continuous_striking(7, target)

        self.assertEqual(STRIKE_BPM, 100.0)
        self.assertAlmostEqual(STRIKE_PERIOD_SECONDS, 0.6)
        self.assertAlmostEqual(SWING_TRIPLET_SECONDS, 0.2)
        self.assertFalse(app.strike_active)
        self.assertTrue(app.continuous_strike_active)
        self.assertEqual(app.continuous_strike_degrees, 7)
        self.assertEqual(app.continuous_next_beat_at, 12.0)
        self.assertEqual(app.continuous_swing_index, SWING_PICKUP_INDEX)
        self.assertTrue(app.continuous_first_swing_hit)
        self.assertEqual(app.phase, CONTINUOUS_WAIT_PHASE)
        np.testing.assert_array_equal(app.continuous_strike_target, target)
        self.assertIn("FIRST DETECTED STRIKE", app.result_status.set.call_args.args[0])
        app.hihat.start_sequence.assert_called_once_with()

    def test_continuous_strike_starts_early_enough_to_reach_swing_deadline(self):
        target = np.zeros(7)
        target[6] = -math.radians(10)
        app = App.__new__(App)
        app.continuous_strike_active = True
        app.continuous_stop_requested = False
        app.continuous_next_beat_at = 10.0
        app.continuous_strike_degrees = 10
        app.continuous_strike_target = target
        app.continuous_swing_index = 0
        app.continuous_first_swing_hit = False
        app.continuous_hihat_pending = False
        app.phase = CONTINUOUS_WAIT_PHASE
        app.arm = Mock(return_value=np.zeros(7))
        app.status = Mock()
        app._begin_continuous_stage = Mock()

        self.assertAlmostEqual(CONTINUOUS_STRIKE_LEAD_SECONDS, .04)
        app._advance_continuous_striking(9.89)
        app._begin_continuous_stage.assert_not_called()
        app._advance_continuous_striking(9.92)
        app._begin_continuous_stage.assert_called_once()
        self.assertEqual(
            app._begin_continuous_stage.call_args.args[0], CONTINUOUS_OUT_PHASE
        )
        np.testing.assert_array_equal(
            app._begin_continuous_stage.call_args.args[1], target
        )
        self.assertEqual(app._begin_continuous_stage.call_args.args[2], 9.92)

    def test_partial_return_is_interrupted_only_after_real_rebound(self):
        target = np.zeros(7)
        target[6] = -math.radians(10)
        app = App.__new__(App)
        app.continuous_strike_active = True
        app.continuous_stop_requested = False
        app.continuous_next_beat_at = 10.0
        app.continuous_strike_degrees = 10
        app.continuous_strike_target = target
        app.continuous_swing_index = 0
        app.continuous_first_swing_hit = False
        app.continuous_hihat_pending = False
        app.phase = CONTINUOUS_RETURN_PHASE
        app.status = Mock()
        app._begin_continuous_stage = Mock()

        partial = target.copy()
        partial[6] += math.radians(5)
        app.arm = Mock(return_value=partial)
        app._advance_continuous_striking(9.92)
        app._begin_continuous_stage.assert_not_called()
        app._advance_continuous_striking(9.94)
        app._begin_continuous_stage.assert_called_once_with(
            CONTINUOUS_OUT_PHASE, target, 9.94
        )

        app._begin_continuous_stage.reset_mock()
        insufficient = target.copy()
        insufficient[6] += math.radians(2)
        app.arm.return_value = insufficient
        app._advance_continuous_striking(10.1)
        app._begin_continuous_stage.assert_not_called()
        self.assertGreater(CONTINUOUS_MIN_REBOUND_RAD, math.radians(2))

    def test_swing_events_start_with_pickup_and_follow_triplet_pattern(self):
        app = App.__new__(App)
        app.continuous_swing_index = SWING_PICKUP_INDEX
        app.continuous_first_swing_hit = True
        app.continuous_current_swing_label = None
        app.continuous_current_interval_seconds = None

        events = [app._take_next_swing_event() for _ in range(7)]

        self.assertEqual([event[0] for event in events], [
            "pickup (extra before beat 1)",
            "beat 1",
            "beat 2",
            "extra after beat 2",
            "beat 3",
            "beat 4",
            "extra after beat 4",
        ])
        self.assertEqual([event[1] for event in events], [
            False, True, True, False, True, True, False,
        ])
        np.testing.assert_allclose(
            [event[2] for event in events],
            [.2, .6, .4, .2, .6, .4, .2],
        )
        self.assertEqual(SWING_EVENTS[app.continuous_swing_index][0], "beat 1")

    def test_continuous_impact_keeps_next_deadline_on_swing_grid(self):
        anchor = np.zeros(7)
        app = App.__new__(App)
        app.strike_plan = StrikePlan(anchor, (anchor.copy(),))
        app.continuous_strike_count = 3
        app.continuous_last_beat_at = 20.0
        app.continuous_current_interval_seconds = SWING_TRIPLET_SECONDS
        app.continuous_current_event_at = 20.0
        app.continuous_swing_index = 0
        app.continuous_stop_requested = False
        app.control = object()
        app.status = Mock()
        app._begin_continuous_stage = Mock()

        app.phase = CONTINUOUS_OUT_PHASE
        app.complete_stage(20.05)
        self.assertEqual(
            app._begin_continuous_stage.call_args.args[0], CONTINUOUS_RETURN_PHASE
        )
        np.testing.assert_array_equal(
            app._begin_continuous_stage.call_args.args[1], anchor
        )

        app.phase = CONTINUOUS_RETURN_PHASE
        app.complete_stage(20.1)
        self.assertEqual(app.phase, CONTINUOUS_WAIT_PHASE)
        self.assertIsNone(app.control)
        self.assertAlmostEqual(app.continuous_next_beat_at, 20.2)

    def test_opening_pickup_establishes_grid_when_target_is_reached(self):
        anchor = np.zeros(7)
        app = App.__new__(App)
        app.strike_plan = StrikePlan(anchor, (anchor.copy(),))
        app.continuous_strike_count = 1
        app.continuous_last_beat_at = None
        app.continuous_current_interval_seconds = SWING_TRIPLET_SECONDS
        app.continuous_current_event_at = None
        app.continuous_swing_index = 0
        app.continuous_stop_requested = False
        app.control = object()
        app.status = Mock()
        app._begin_continuous_stage = Mock()

        app.phase = CONTINUOUS_OUT_PHASE
        app.complete_stage(5.25)

        self.assertEqual(app.continuous_current_event_at, 5.25)
        self.assertEqual(app.continuous_last_beat_at, 5.25)
        self.assertAlmostEqual(app.continuous_next_beat_at, 5.45)
        app._begin_continuous_stage.assert_called_once()
        self.assertEqual(
            app._begin_continuous_stage.call_args.args[0], CONTINUOUS_RETURN_PHASE
        )

    def test_continuous_stages_use_fast_leg_speeds_and_exact_targets(self):
        anchor = np.zeros(7)
        target = anchor.copy()
        target[6] = -math.radians(8)
        app = App.__new__(App)
        app.lower = np.full(7, -2.0)
        app.upper = np.full(7, 2.0)
        app.ik = SimpleNamespace(origin_tcp=np.zeros(3))
        app.hill_ik = SimpleNamespace(position=Mock(return_value=np.zeros(3)))
        app.strike_speed_fast = False
        app.continuous_strike_count = 0
        app.continuous_strike_degrees = 8
        app.continuous_swing_index = 0
        app.continuous_first_swing_hit = False
        app.continuous_current_swing_label = None
        app.continuous_current_main_beat = False
        app.continuous_current_interval_seconds = None
        app.continuous_current_event_at = None
        app.continuous_next_beat_at = 1.1
        app.continuous_hihat_pending = False
        app.strike_plan = StrikePlan(anchor, (target.copy(),))
        app.arm = Mock(return_value=anchor.copy())
        app.status = Mock()
        app._program_failure = Mock()
        app.hihat = Mock()
        app.hihat.send_beat.return_value = b"C"
        events = []
        app.bus = Mock()
        app.bus.set_right_joint7_speed.side_effect = (
            lambda speed: events.append(("speed", speed))
        )
        app.bus.set_positions.side_effect = (
            lambda desired: events.append(("target", float(desired[6])))
        )
        app.bus.set_right_joint7_position.side_effect = (
            lambda target: events.append(("j7 target", float(target)))
        )

        app._begin_continuous_stage(CONTINUOUS_OUT_PHASE, target, 1.0)
        app.hihat.send_beat.assert_not_called()
        self.assertTrue(app.continuous_hihat_pending)
        self.assertTrue(app._send_pending_hihat(1.09))
        app.hihat.send_beat.assert_not_called()
        self.assertTrue(app._send_pending_hihat(1.1))
        app._begin_continuous_stage(CONTINUOUS_RETURN_PHASE, anchor, 1.2)

        self.assertEqual(events, [
            ("speed", RIGHT_STRIKE_DOWN_SPEED),
            ("target", float(target[6])),
            ("j7 target", float(anchor[6])),
        ])
        app.bus.set_right_joint7_feedback_rate.assert_called_once_with(
            STRIKE_J7_FEEDBACK_HZ
        )
        self.assertEqual(app.continuous_strike_count, 1)
        self.assertEqual(app.continuous_j7_measurement_strike_count, 1)
        app.hihat.send_beat.assert_called_once_with()
        self.assertTrue(any(
            "hi-hat motor 2 CLOSE 110°" in call.args[0]
            for call in app.status.set.call_args_list
        ))
        app._program_failure.assert_not_called()

    def test_test_mode_arm_beat_does_not_issue_a_hihat_command(self):
        target = np.zeros(7)
        target[6] = -math.radians(STRIKE_START_DEGREES)
        app = App.__new__(App)
        app.test_mode = True
        app.lower = np.full(7, -2.0)
        app.upper = np.full(7, 2.0)
        app.ik = SimpleNamespace(origin_tcp=np.zeros(3))
        app.hill_ik = SimpleNamespace(position=Mock(return_value=np.zeros(3)))
        app.strike_speed_fast = False
        app.continuous_strike_count = 0
        app.continuous_strike_degrees = STRIKE_START_DEGREES
        app.continuous_swing_index = SWING_PICKUP_INDEX
        app.continuous_first_swing_hit = True
        app.continuous_current_swing_label = None
        app.continuous_current_main_beat = False
        app.continuous_current_interval_seconds = None
        app.continuous_current_event_at = None
        app.continuous_next_beat_at = 1.0
        app.continuous_hihat_pending = False
        app.hihat = None
        app.strike_plan = StrikePlan(np.zeros(7), (target.copy(),))
        app.arm = Mock(return_value=np.zeros(7))
        app.status = Mock()
        app._program_failure = Mock()
        app.bus = Mock()

        app._begin_continuous_stage(CONTINUOUS_OUT_PHASE, target, 1.0)

        app.bus.set_positions.assert_called_once()
        self.assertEqual(app.continuous_j7_measurement_strike_count, 1)
        self.assertIn("hi-hat ignored (test mode)", app.status.set.call_args.args[0])
        app._program_failure.assert_not_called()

    def test_swing_log_reports_lowest_observed_j7_encoder_for_each_hit(self):
        anchor = np.zeros(7)
        anchor[6] = math.radians(70)
        target = anchor.copy()
        target[6] -= math.radians(10)
        actual = anchor.copy()
        app = App.__new__(App)
        app.strike_plan = StrikePlan(anchor, (target,))
        app.continuous_strike_degrees = 10
        app.continuous_strike_count = 4
        app.continuous_current_swing_label = "extra after beat 2"
        app.arm = lambda: actual.copy()

        app._start_continuous_j7_measurement()
        actual[6] = math.radians(61)
        app._sample_continuous_j7_minimum()
        actual[6] = math.radians(59)
        app._sample_continuous_j7_minimum()
        actual[6] = math.radians(63)
        app.continuous_j7_measurement_reached = True

        with patch("builtins.print") as output:
            app._log_continuous_j7_minimum()

        output.assert_called_once()
        message = output.call_args.args[0]
        self.assertIn("strike 4 [extra after beat 2]", message)
        self.assertIn("11.00 deg down from anchor", message)
        self.assertIn("lowest J7 encoder: 59.00 deg", message)
        self.assertIn("commanded: 10 deg", message)
        self.assertTrue(output.call_args.kwargs["flush"])
        self.assertIsNone(app.continuous_j7_measurement_strike_count)

    def test_swing_pickup_skips_hihat_then_beat_one_sends_first_close(self):
        target = np.zeros(7)
        target[6] = -math.radians(7)
        app = App.__new__(App)
        app.lower = np.full(7, -2.0)
        app.upper = np.full(7, 2.0)
        app.ik = SimpleNamespace(origin_tcp=np.zeros(3))
        app.hill_ik = SimpleNamespace(position=Mock(return_value=np.zeros(3)))
        app.strike_speed_fast = False
        app.continuous_strike_count = 0
        app.continuous_strike_degrees = 7
        app.continuous_swing_index = SWING_PICKUP_INDEX
        app.continuous_first_swing_hit = True
        app.continuous_current_swing_label = None
        app.continuous_current_main_beat = False
        app.continuous_current_interval_seconds = None
        app.continuous_current_event_at = None
        app.continuous_next_beat_at = 1.0
        app.continuous_hihat_pending = False
        app.status = Mock()
        app._program_failure = Mock()
        app.hihat = Mock()
        app.hihat.send_beat.return_value = b"C"
        app.bus = Mock()

        app._begin_continuous_stage(CONTINUOUS_OUT_PHASE, target, 1.0)
        app.hihat.send_beat.assert_not_called()
        self.assertIn("pickup (extra before beat 1)", app.status.set.call_args.args[0])
        self.assertIn("hi-hat unchanged", app.status.set.call_args.args[0])

        app.continuous_next_beat_at = 1.2
        app._begin_continuous_stage(CONTINUOUS_OUT_PHASE, target, 1.1)
        app.hihat.send_beat.assert_not_called()
        self.assertTrue(app._send_pending_hihat(1.19))
        app.hihat.send_beat.assert_not_called()
        self.assertTrue(app._send_pending_hihat(1.2))
        app.hihat.send_beat.assert_called_once_with()
        self.assertIn("beat 1", app.status.set.call_args.args[0])
        self.assertIn("CLOSE 110°", app.status.set.call_args.args[0])

    def test_hihat_command_failure_at_beat_requests_safe_stop(self):
        app = App.__new__(App)
        app.continuous_hihat_pending = True
        app.continuous_current_event_at = 1.0
        app.continuous_current_swing_label = "beat 1"
        app.status = Mock()
        app.hihat = Mock()
        app.hihat.send_beat.side_effect = RuntimeError("serial disconnected")
        app._request_continuous_stop = Mock()

        self.assertFalse(app._send_pending_hihat(1.0))

        app._request_continuous_stop.assert_called_once_with()
        self.assertIn("serial disconnected", app.hihat_fault_detail)

    def test_stop_finishes_active_return_then_centers_and_relaxes(self):
        anchor = np.zeros(7)
        app = App.__new__(App)
        app.phase = CONTINUOUS_OUT_PHASE
        app.strike_plan = StrikePlan(anchor, (anchor.copy(),))
        app.continuous_strike_active = True
        app.continuous_stop_requested = False
        app.continuous_strike_count = 2
        app.continuous_last_beat_at = 30.0
        app.continuous_current_event_at = 30.0
        app.continuous_current_interval_seconds = SWING_TRIPLET_SECONDS
        app.strike_speed_fast = True
        app.strike_selected_speed = RIGHT_STRIKE_DOWN_SPEED
        app.strike_feedback_fast = True
        app.bus = Mock(active=True)
        app.center_goal = np.ones(7)
        app.control = object()
        app.status = Mock()
        app.center_relax_button = Mock()
        app._begin_continuous_stage = Mock()
        app.begin_stage = Mock()
        app.hihat = Mock()

        app._request_continuous_stop()
        self.assertTrue(app.continuous_stop_requested)
        app.hihat.stop_sequence.assert_called_once_with()
        app.bus.set_right_joint7_speed.assert_not_called()
        app.begin_stage.assert_not_called()

        app.complete_stage(30.2)
        self.assertEqual(
            app._begin_continuous_stage.call_args.args[0], CONTINUOUS_RETURN_PHASE
        )
        app.phase = CONTINUOUS_RETURN_PHASE
        app.complete_stage(30.4)
        app.bus.set_right_joint7_feedback_rate.assert_called_once_with(None)
        app.bus.set_right_joint7_speed.assert_called_once_with(SPEED)
        app.begin_stage.assert_called_once()
        self.assertEqual(app.begin_stage.call_args.args[0], CENTER_RELAX_PHASE)
        np.testing.assert_array_equal(
            app.begin_stage.call_args.args[1], app.center_goal
        )
        self.assertFalse(app.continuous_strike_active)

    def test_center_relax_button_routes_continuous_mode_to_safe_stop(self):
        app = App.__new__(App)
        app.hardware = True
        app.bus = Mock(active=True)
        app.bus.fresh.return_value = True
        app.setup = None
        app.continuous_strike_active = True
        app.center_goal = np.zeros(7)
        app.zone = SimpleNamespace(contains=Mock(return_value=True))
        app.planned_tcp = Mock(return_value=np.zeros(3))
        app.status = Mock()
        app._request_continuous_stop = Mock()

        app.center_relax()

        app._request_continuous_stop.assert_called_once_with()

    def test_sound_is_not_required_after_detected_continuous_mode_starts(self):
        app = App.__new__(App)
        app.bus = Mock(active=True)
        app.continuous_strike_active = True
        app.phase = CONTINUOUS_OUT_PHASE
        self.assertFalse(app._sound_required_now())

    def test_return_waits_for_delayed_sound_decision_before_next_depth(self):
        anchor = np.zeros(7)
        second = anchor.copy()
        second[6] = -math.radians(6)
        app = App.__new__(App)
        app.phase = "STRIKE RETURNING TO CYMBAL POSE"
        app.strike_plan = StrikePlan(anchor, (anchor.copy(), second))
        app.strike_index = 0
        app.strike_hit_pending = None
        app.control = object()
        app.status = Mock()
        app._begin_strike_attempt = Mock()
        app._program_failure = Mock()

        app.complete_stage(20.0)
        self.assertEqual(app.phase, STRIKE_SOUND_WAIT_PHASE)
        self.assertIsNone(app.control)
        self.assertAlmostEqual(app.strike_sound_deadline, 20.0 + STRIKE_SOUND_WAIT_SECONDS)
        app._begin_strike_attempt.assert_not_called()

        app._finish_no_hit_attempt()
        self.assertEqual(app.strike_index, 1)
        app._begin_strike_attempt.assert_called_once_with()
        app._program_failure.assert_not_called()

    def test_all_safe_depths_without_sound_hit_fail_after_wait(self):
        app = App.__new__(App)
        app.phase = STRIKE_SOUND_WAIT_PHASE
        app.strike_plan = StrikePlan(np.zeros(7), (np.zeros(7),))
        app.strike_index = 0
        app.strike_hit_pending = None
        app.strike_sound_deadline = 1.0
        app._begin_strike_attempt = Mock()
        app._program_failure = Mock()
        app._finish_no_hit_attempt()
        app._program_failure.assert_called_once()
        self.assertIn("without an ST7 sound HIT", app._program_failure.call_args.args[0])
        app._begin_strike_attempt.assert_not_called()

    def test_audio_protocol_round_trip_and_freshness(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audio.sock"
            receiver = AudioReceiver(path)
            sender = AudioSender(path)
            try:
                sender.send({
                    "kind": "hit", "event_at": 9.0, "detected_at": 9.5,
                    "score": .99,
                })
                self.assertEqual(receiver.poll(), [])
                sender.status("ready", "ST7 ready on TONOR")
                sender.hit(10.0, 10.6, .97, 89.4)
                messages = receiver.poll()
                self.assertTrue(receiver.ready())
                self.assertEqual(receiver.detail, "ST7 ready on TONOR")
                hit = next(message for message in messages if message["kind"] == "hit")
                self.assertEqual(hit["event_at"], 10.0)
                self.assertEqual(hit["score"], .97)
                self.assertEqual(hit["normality_score"], 89.4)
            finally:
                sender.close()
                receiver.close()

    def test_audio_bridge_selects_stable_tonor_and_real_st7_assets(self):
        listing = (
            "1\talsa_output.test.monitor\tmodule\n"
            f"2\t{DEFAULT_TONOR_SOURCE}\tmodule\n"
        )
        sources = capture_sources(listing)
        self.assertEqual(sources, [DEFAULT_TONOR_SOURCE])
        self.assertEqual(
            choose_tonor_source(sources, DEFAULT_TONOR_SOURCE), DEFAULT_TONOR_SOURCE
        )
        changed_name = "alsa_input.usb-TONOR_reconnected.analog-stereo"
        self.assertEqual(
            choose_tonor_source([changed_name], DEFAULT_TONOR_SOURCE), changed_name
        )
        with self.assertRaisesRegex(RuntimeError, "no TONOR capture source"):
            choose_tonor_source(["alsa_input.pci-built-in"], DEFAULT_TONOR_SOURCE)
        st7 = ROOT.parents[1] / "st7"
        if not (st7 / "cymbal.py").is_file():
            st7 = ROOT.parent / "st7"
        command = detector_command(st7, "pipewire")
        self.assertEqual(Path(command[0]), st7 / ".venv/bin/python")
        self.assertEqual(command[-2:], ["--device", "pipewire"])

    def test_sound_process_failure_during_strike_uses_safe_program_failure(self):
        app = App.__new__(App)
        app.audio_receiver = SimpleNamespace(
            poll=Mock(return_value=[{
                "kind": "status", "state": "error", "detail": "microphone stopped",
            }]),
            state="error",
            detail="microphone stopped",
            ready=Mock(return_value=False),
        )
        app.sound_fault_detail = None
        app.bus = Mock(active=True)
        app.phase = "STRIKE MOVING OUT"
        app.sound_status = Mock()
        app.sound_label = Mock()
        app.root = Mock()
        app._process_sound_hit = Mock()
        app._refresh_buttons = Mock()
        app._program_failure = Mock()

        app.sound_tick()

        app._program_failure.assert_called_once_with(
            "FAILED: sound detector error: microphone stopped"
        )
        app.root.after.assert_called_once_with(20, app.sound_tick)

    def test_center_relax_cancels_active_work_restores_speed_and_commands_center(self):
        app = App.__new__(App)
        app.hardware = True
        app.bus = Mock(active=True)
        app.bus.fresh.return_value = True
        app.phase = "RECORDING PLAYBACK"
        app.setup = None
        app.hold_until = 10.0
        app.search_active = True
        app.pending_detection_frame_id = 22
        app.gripper_close_until = 30.0
        app.center_goal = np.arange(7, dtype=float) / 10.0
        app.zone = SimpleNamespace(contains=Mock(return_value=True))
        app.planned_tcp = Mock(return_value=np.zeros(3))
        app.status = Mock()
        app._clear_alignment_state = Mock()
        app._restore_playback_speed = Mock(return_value=True)
        app._restore_strike_speed = Mock(return_value=True)
        app.begin_stage = Mock()
        app.fail = Mock()

        app.center_relax()

        app._clear_alignment_state.assert_called_once_with()
        app._restore_playback_speed.assert_called_once_with("Center + Relax requested")
        app._restore_strike_speed.assert_called_once_with("Center + Relax requested")
        app.begin_stage.assert_called_once()
        self.assertEqual(app.begin_stage.call_args.args[0], CENTER_RELAX_PHASE)
        np.testing.assert_array_equal(app.begin_stage.call_args.args[1], app.center_goal)
        self.assertFalse(app.search_active)
        self.assertIsNone(app.pending_detection_frame_id)
        self.assertIsNone(app.gripper_close_until)
        app.fail.assert_not_called()

    def test_center_relax_completion_disables_arm_only_after_center_is_reached(self):
        app = App.__new__(App)
        app.phase = CENTER_RELAX_PHASE
        app.control = object()
        app._clear_alignment_state = Mock()
        app.relax = Mock()

        app.complete_stage(4.0)

        self.assertIsNone(app.control)
        app._clear_alignment_state.assert_called_once_with()
        app.relax.assert_called_once_with(
            "Center + Relax complete; centered; right motors disabled"
        )

    def test_sound_based_strike_no_longer_depends_on_camera_process(self):
        app = App.__new__(App)
        app.alignment_active = False
        app.playback_active = False
        app.strike_active = True
        app.support_fault_detail = None
        app.receiver = SimpleNamespace(
            poll=Mock(return_value=[{
                "kind": "status", "state": "error", "detail": "camera stopped",
            }]),
            state="error",
            detail="camera stopped",
        )
        app.bus = Mock(active=True)
        app.camera_status = Mock()
        app.camera_label = Mock()
        app.root = Mock()
        app._sync_camera_loading_label = Mock()
        app._camera_text = Mock(return_value=("Camera ERROR", "#b00020"))
        app._refresh_buttons = Mock()
        app._program_failure = Mock()
        app.camera_tick()
        app._sync_camera_loading_label.assert_called_once_with()
        app._program_failure.assert_not_called()
        self.assertTrue(app.strike_active)
        app.root.after.assert_called_once()

    def test_deeper_strikes_always_round_trip_through_same_anchor(self):
        anchor = np.zeros(7)
        anchor[6] = math.radians(80)
        targets = []
        for amount in (5, 10, 15):
            target = anchor.copy()
            target[6] -= math.radians(amount)
            targets.append(target)
        app = App.__new__(App)
        app.strike_plan = StrikePlan(anchor, tuple(targets))
        app.strike_index = 0
        app.strike_hit_pending = None
        app.strike_attempt_motion_seen = False
        app._begin_strike_stage = Mock()
        app.status = Mock()

        # Attempt 1 has already been sent to 75 degrees. Reaching it sends 80.
        app.phase = "STRIKE MOVING OUT"
        app.complete_stage(1.0)
        np.testing.assert_array_equal(
            app._begin_strike_stage.call_args.args[1], anchor
        )

        # At anchor, wait for delayed ST7 output before trying 70 degrees.
        app._begin_strike_stage.reset_mock()
        app.phase = "STRIKE RETURNING TO CYMBAL POSE"
        app.complete_stage(2.0)
        self.assertEqual(app.phase, STRIKE_SOUND_WAIT_PHASE)
        app._begin_strike_stage.assert_not_called()
        app._finish_no_hit_attempt()
        np.testing.assert_array_equal(
            app._begin_strike_stage.call_args.args[1], targets[1]
        )

        # Attempt 2 repeats the same 70 -> 80 round trip before 65 degrees.
        app._begin_strike_stage.reset_mock()
        app.phase = "STRIKE MOVING OUT"
        app.complete_stage(3.0)
        np.testing.assert_array_equal(
            app._begin_strike_stage.call_args.args[1], anchor
        )
        app._begin_strike_stage.reset_mock()
        app.phase = "STRIKE RETURNING TO CYMBAL POSE"
        app.strike_hit_pending = None
        app.complete_stage(4.0)
        self.assertEqual(app.phase, STRIKE_SOUND_WAIT_PHASE)
        app._finish_no_hit_attempt()
        np.testing.assert_array_equal(
            app._begin_strike_stage.call_args.args[1], targets[2]
        )
        self.assertEqual(app.strike_index, 2)

    def test_recording_start_and_end_phases_wrap_playback(self):
        app = App.__new__(App)
        app.phase = "MOVING TO RECORDING START"
        app._begin_playback = Mock()
        app.complete_stage(3.0)
        app._begin_playback.assert_called_once_with(3.0)

        app.phase = "SETTLING RECORDING END"
        app._begin_playback_alignment = Mock()
        app.complete_stage(4.0)
        app._begin_playback_alignment.assert_called_once_with(4.0)

    def test_recording_playback_uses_point_eight_then_restores_normal_speed(self):
        app = App.__new__(App)
        app.bus = Mock(active=True)
        app.control = object()
        app.playback_speed_fast = False
        app.result_status = Mock()
        app.result_label = Mock()
        app.status = Mock()

        app._begin_playback(3.0)

        app.bus.set_right_arm_speed.assert_called_once_with(RIGHT_PLAYBACK_SPEED)
        self.assertTrue(app.playback_speed_fast)
        self.assertEqual(app.phase, "RECORDING PLAYBACK")

        app.phase = "SETTLING RECORDING END"
        app._begin_playback_alignment = Mock()
        app.complete_stage(4.0)

        self.assertEqual(
            app.bus.set_right_arm_speed.call_args_list,
            [call(RIGHT_PLAYBACK_SPEED), call(SPEED)],
        )
        self.assertFalse(app.playback_speed_fast)
        app._begin_playback_alignment.assert_called_once_with(4.0)

    def test_endpoint_alignment_installs_playback_guided_direction_order(self):
        app = App.__new__(App)
        actual = self.anchor.copy()
        app.playback_active = True
        app.arm = Mock(return_value=actual)
        app.lower = self.lower
        app.upper = self.upper
        app.tcp = Mock(return_value=self.ik.origin_tcp + np.array([.25, .04, .35]))
        app.ik = SimpleNamespace(origin_tcp=self.ik.origin_tcp)
        app.playback_visual_samples = [
            PlaybackVisualSample(float(index), np.array([.25, index * .01, .35]), .1 + index * .05)
            for index in range(5)
        ]
        app.receiver = SimpleNamespace(frame_id=12)
        app._reset_hold_controller = Mock()
        app.result_status = Mock()
        app.result_label = Mock()
        app.status = Mock()
        app._begin_playback_alignment(10.0)
        self.assertEqual(app.hill.direction_name, "-Y")
        self.assertTrue(app.alignment_active)
        self.assertEqual(app.phase, "CHECKING PLAYBACK END")
        self.assertEqual(app.measurement_gate_frame_id, 12)
        app._reset_hold_controller.assert_called_once_with()

    def test_single_playback_end_frame_uses_outer_pink_zone(self):
        edge_message = {
            "kind": "frame", "frame_id": 13, "required": True, "cymbal": True,
            "cymbal_box": (0., 0., 300., 300.),
            "target_box": (100., 100., 200., 200.),
            "drumstick_box": (100., 100., 130., 200.),
            "tip_point": (110., 150.),
        }
        app = App.__new__(App)
        app.phase = "CHECKING PLAYBACK END"
        app.measurement_gate_frame_id = 12
        app.alignment_loss_deadline = 30.0
        app._alignment_success = Mock()
        app._plan_next_candidate = Mock()
        app._process_alignment_frame(edge_message, 1.0)
        app._alignment_success.assert_called_once_with(13)
        app._plan_next_candidate.assert_not_called()

        outside = dict(edge_message, frame_id=14, tip_point=(90., 150.))
        app.phase = "CHECKING PLAYBACK END"
        app.measurement_gate_frame_id = 13
        app.hill = GuidedHillClimber(
            np.zeros(3), np.zeros(7), DEFAULT_DIRECTION_ORDER
        )
        app.tcp = Mock(return_value=np.zeros(3))
        app.ik = SimpleNamespace(origin_tcp=np.zeros(3))
        app.arm = Mock(return_value=np.zeros(7))
        app.result_status = Mock()
        app.result_label = Mock()
        app._alignment_success.reset_mock()
        app._process_alignment_frame(outside, 1.1)
        app._alignment_success.assert_not_called()
        app._plan_next_candidate.assert_called_once_with()
        self.assertIsNotNone(app.hill.anchor_score)

    def test_one_pink_frame_during_hill_measurement_continues_immediately(self):
        app = App.__new__(App)
        app.phase = "HILL MEASURING ANCHOR"
        app.measurement_gate_frame_id = 20
        app.measurement_frame_ids = set()
        app.measurement_samples = []
        app.measurement_first_valid_at = None
        app.alignment_loss_deadline = 30.0
        app._alignment_success = Mock()
        edge_message = {
            "kind": "frame", "frame_id": 21, "required": True, "cymbal": True,
            "cymbal_box": (0., 0., 300., 300.),
            "target_box": (100., 100., 200., 200.),
            "drumstick_box": (100., 100., 130., 200.),
            "tip_point": (110., 150.),
        }
        app._process_alignment_frame(edge_message, 1.0)
        app._alignment_success.assert_called_once_with(21)
        self.assertEqual(app.measurement_samples, [])
        self.assertIsNone(app.measurement_first_valid_at)

        outside_message = dict(edge_message, frame_id=22, tip_point=(90., 150.))
        app._alignment_success.reset_mock()
        app._process_alignment_frame(outside_message, 1.1)
        app._alignment_success.assert_not_called()
        self.assertEqual(len(app.measurement_samples), 1)
        self.assertEqual(app.measurement_first_valid_at, 1.1)

    def test_new_launcher_is_separate_from_original_camera_program(self):
        new_launcher = (ROOT / "launch_right_camera_playback.py").read_text()
        playback_app = (ROOT / "camera_playback/app.py").read_text()
        new_script = (ROOT / "start_beat.sh").read_text()
        original_launcher = (ROOT / "launch_right_camera_cartesian.py").read_text()
        self.assertIn("camera_playback.app", new_launcher)
        self.assertIn("camera_playback/camera.py", new_launcher)
        self.assertIn("camera_playback.audio_bridge", new_launcher)
        self.assertIn('ST7 = ROOT.parents[1] / "st7"', new_launcher)
        self.assertIn("runs/cymbal-normality/reference.npz", new_launcher)
        self.assertIn("--audio-socket", new_launcher)
        self.assertIn("--esp-port", new_launcher)
        self.assertIn("camera_playback.hihat", new_launcher)
        self.assertIn('"--test"', new_launcher)
        self.assertIn('"--hardwaretest"', new_launcher)
        self.assertIn('app_command.append("--hardwaretest")', new_launcher)
        self.assertIn(
            "perception_enabled = not args.test",
            new_launcher,
        )
        self.assertIn("if perception_enabled:", new_launcher)
        self.assertIn('optional(app_command, "--recording", args.recording)', new_launcher)
        self.assertIn("test_mode=args.test", playback_app)
        self.assertIn("args.hardware or args.hardwaretest", playback_app)
        self.assertIn("hardware_test_mode=args.hardwaretest", playback_app)
        self.assertIn("alsa_input.usb-TONOR_TONOR_TD510_Dynamic_Mic_", new_launcher)
        self.assertNotIn("camera_playback/camera.py", original_launcher)
        self.assertIn("ROS_DOMAIN_ID=92", new_script)
        self.assertFalse((ROOT / "start_right_camera_playback.sh").exists())
        self.assertIn('default=ROOT / "recordings/record1.json"', new_launcher)
        self.assertEqual(DEFAULT_RECORDING, ROOT / "recordings/record1.json")
        self.assertIn("STRIKE_SOUND_WAIT_SECONDS", playback_app)
        self.assertIn("STRIKE_BPM = 100.0", playback_app)
        self.assertIn("STRIKE_START_DEGREES + self.strike_index", playback_app)
        self.assertNotIn("build_tuning_candidates", playback_app)
        self.assertIn("continuous_strike", playback_app)
        self.assertIn("_request_continuous_stop", playback_app)
        copied_firmware = (ROOT / "esp32_hihat/motor_beat/motor_beat.ino").read_text()
        self.assertIn("BEAT_TARGET_DEGREES   = 110.0f", copied_firmware)
        self.assertIn("c == 'C'", copied_firmware)
        self.assertIn("c == 'O'", copied_firmware)
        self.assertNotIn("camera_playback", original_launcher)


if __name__ == "__main__":
    unittest.main()
