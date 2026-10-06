from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import unittest
from unittest.mock import Mock

import numpy as np

from camera_search.app import App
from camera_search.camera import cymbal_grid_geometry
from camera_search.control import CameraGoalControl, ENCODER_LSB_RAD, SETTLE_SECONDS
from camera_search.hill_climb import HILL_DIRECTIONS, HILL_STEPS_M, HillClimber
from camera_search.planner import (
    IK_SEED_LIMIT_TOLERANCE_RAD,
    INITIAL_OFFSET,
    Y_STEP_M,
    build_search_plan,
    scan_offsets,
    solve_search_coordinate,
)
from camera_search.protocol import DetectionReceiver, DetectionSender, has_required_detection
from camera_search.vision import (
    VisualObservation,
    median_observation,
    normalized_target_distance,
    observation_from_message,
)
from cartesian_goal.ik import CartesianIK
from centering.motors import SPEED
from goal_motion.control import TOLERANCE
from safe_zone.geometry import Model, Zone, RIGHT_TCP, MEMBERSHIP_BUFFER_M


ROOT = Path(__file__).resolve().parents[1]


def direct_stick(tip=(150, 150), confidence=.9):
    return {
        "class_id": 1,
        "confidence": confidence,
        "box": [120, 100, 180, 220],
        "tip": list(tip),
        "tip_source": "yolo_pose",
        "tip_status": "observed",
    }


def cymbal(box=(0, 0, 300, 300), confidence=.9):
    return {"class_id": 0, "confidence": confidence, "box": list(box)}


def alignment_message(frame_id=1, tip=(150, 150)):
    return {
        "kind": "frame",
        "frame_id": frame_id,
        "required": True,
        "cymbal": True,
        "cymbal_box": (0.0, 0.0, 300.0, 300.0),
        "target_box": (100.0, 100.0, 200.0, 200.0),
        "drumstick_box": (120.0, 100.0, 180.0, 220.0),
        "tip_point": tuple(map(float, tip)),
    }


class CameraSearchTests(unittest.TestCase):
    def test_cymbal_target_is_center_ninth_of_highest_confidence_box(self):
        detections = [
            {"class_id": 1, "confidence": .99, "box": [0, 0, 300, 300]},
            {"class_id": 0, "confidence": .40, "box": [0, 0, 30, 30]},
            {"class_id": 0, "confidence": .90, "box": [30, 60, 330, 240]},
        ]
        geometry = cymbal_grid_geometry(detections)
        self.assertEqual(geometry["outer"], (30, 60, 330, 240))
        self.assertEqual(geometry["x_lines"], (130, 230))
        self.assertEqual(geometry["y_lines"], (120, 180))
        self.assertEqual(geometry["target"], (130, 120, 230, 180))

    def test_cymbal_target_is_absent_without_valid_cymbal_box(self):
        self.assertIsNone(cymbal_grid_geometry([]))
        self.assertIsNone(cymbal_grid_geometry([
            {"class_id": 1, "confidence": .9, "box": [0, 0, 30, 30]},
            {"class_id": 0, "confidence": .9, "box": [10, 10, 10, 40]},
        ]))

    def test_requirement_needs_box_and_directly_observed_yolo_tip(self):
        stick = {"class_id": 1, "confidence": .9, "box": [0, 0, 20, 40]}
        self.assertFalse(has_required_detection([cymbal(), stick]))
        stick.update(tip=[3, 4], tip_source="yolo_pose", tip_status="observed")
        self.assertTrue(has_required_detection([cymbal(), stick]))
        stick["tip_status"] = "tracked"
        self.assertFalse(has_required_detection([stick]))
        stick["tip_status"] = "observed"
        stick.pop("box")
        self.assertFalse(has_required_detection([stick]))

    def test_protocol_transmits_selected_visual_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "d.sock"
            receiver = DetectionReceiver(path)
            sender = DetectionSender(path)
            try:
                detections = [cymbal((30, 60, 330, 240)), direct_stick((151, 119))]
                sender.frame(7, time.monotonic(), detections)
                messages = receiver.poll()
                message = messages[-1]
                self.assertEqual(message["frame_id"], 7)
                self.assertTrue(receiver.required)
                self.assertTrue(receiver.cymbal)
                self.assertEqual(receiver.cymbal_box, (30.0, 60.0, 330.0, 240.0))
                self.assertEqual(receiver.target_box, (130.0, 120.0, 230.0, 180.0))
                self.assertEqual(receiver.tip_point, (151.0, 119.0))
                self.assertEqual(receiver.drumstick_box, (120.0, 100.0, 180.0, 220.0))
                self.assertTrue(receiver.fresh())
                self.assertFalse(receiver.fresh(receiver.received_at + .51))
            finally:
                sender.close()
                receiver.close()

    def test_protocol_reports_stick_without_fabricating_missing_cymbal(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "d.sock"
            receiver = DetectionReceiver(path)
            sender = DetectionSender(path)
            try:
                sender.frame(3, time.monotonic(), [direct_stick()])
                receiver.poll()
                self.assertTrue(receiver.required)
                self.assertFalse(receiver.cymbal)
                self.assertIsNone(receiver.cymbal_box)
                self.assertIsNone(receiver.target_box)
            finally:
                sender.close()
                receiver.close()

    def test_old_inference_cannot_trigger_required_detection(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "d.sock"
            receiver = DetectionReceiver(path)
            sender = DetectionSender(path)
            try:
                sender.frame(8, time.monotonic() - 1.0, [cymbal(), direct_stick()])
                receiver.poll()
                self.assertFalse(receiver.required)
                self.assertFalse(receiver.fresh())
            finally:
                sender.close()
                receiver.close()

    def test_visual_score_is_zero_inside_and_normalized_outside(self):
        inside = VisualObservation((150, 150), (0, 0, 300, 300), (100, 100, 200, 200))
        self.assertEqual(normalized_target_distance(inside), 0.0)
        right = VisualObservation((230, 150), (0, 0, 300, 300), (100, 100, 200, 200))
        self.assertAlmostEqual(normalized_target_distance(right), .1)
        diagonal = VisualObservation((230, 230), (0, 0, 300, 300), (100, 100, 200, 200))
        self.assertAlmostEqual(normalized_target_distance(diagonal), np.hypot(.1, .1))

    def test_component_median_rejects_one_tip_and_box_outlier(self):
        samples = [
            VisualObservation((149, 150), (0, 0, 300, 300), (100, 100, 200, 200)),
            VisualObservation((151, 152), (1, 0, 301, 300), (101, 100, 201, 200)),
            VisualObservation((900, -500), (-400, 0, 900, 300), (500, 100, 700, 200)),
        ]
        result = median_observation(samples)
        self.assertEqual(result.tip, (151.0, 150.0))
        self.assertEqual(result.cymbal_box, (0.0, 0.0, 301.0, 300.0))
        self.assertEqual(result.target_box, (101.0, 100.0, 201.0, 200.0))

    def test_observation_requires_simultaneous_direct_tip_and_cymbal_geometry(self):
        message = alignment_message()
        self.assertIsNotNone(observation_from_message(message))
        message["required"] = False
        self.assertIsNone(observation_from_message(message))
        message = alignment_message()
        message["target_box"] = None
        self.assertIsNone(observation_from_message(message))

    def test_camera_controller_settles_at_original_tolerance_plus_one_encoder_count(self):
        desired = np.zeros(7)
        lower = np.full(7, -2.0)
        upper = np.full(7, 2.0)
        controller = CameraGoalControl(desired, lower, upper, 0.0)
        actual = np.zeros(7)
        actual[1] = TOLERANCE + ENCODER_LSB_RAD * 0.5
        reached = False
        for n in range(1, 100):
            _, _, reached = controller.update(actual, n * .02)
            if reached:
                break
        self.assertTrue(reached)
        self.assertGreaterEqual(n * .02, SETTLE_SECONDS)

        controller = CameraGoalControl(desired, lower, upper, 0.0)
        actual[1] = TOLERANCE + ENCODER_LSB_RAD * 1.5
        for n in range(1, 100):
            _, _, reached = controller.update(actual, n * .02)
        self.assertFalse(reached)

    def test_camera_wrapper_defines_only_processed_window(self):
        source = (ROOT / "camera_search/camera.py").read_text()
        self.assertEqual(source.count("cv2.namedWindow("), 1)
        self.assertNotIn("RAW_WINDOW_NAME", source)

    def test_support_process_exit_notifies_controller_instead_of_forcing_shutdown(self):
        source = (ROOT / "launch_right_camera_cartesian.py").read_text()
        self.assertIn("Processed camera process exited", source)
        self.assertIn("Robot-state publisher exited", source)
        self.assertIn("RViz exited", source)
        self.assertEqual(source.count("EmitEvent(event=Shutdown"), 1)

    def real_ik(self):
        model = Model(ROOT / "model/openarmx.urdf")
        zone = Zone.load(ROOT / "right_zones/zone1.json", model.digest, RIGHT_TCP)
        joints = {joint.get("name"): joint for joint in model.joints}
        limits = [joints[f"openarmx_right_joint{i}"].find("limit") for i in range(1, 8)]
        lower = np.array([float(limit.get("lower")) for limit in limits])
        upper = np.array([float(limit.get("upper")) for limit in limits])
        center = np.zeros(7)
        center[6] = upper[6]
        return model, zone, center, CartesianIK(
            model, zone, lower, upper, SPEED, "right", RIGHT_TCP, center, np.zeros(7)
        )

    def test_real_right_zone_uses_requested_ten_mm_y_steps(self):
        model = Model(ROOT / "model/openarmx.urdf")
        zone = Zone.load(ROOT / "right_zones/zone1.json", model.digest, RIGHT_TCP)
        origin = model.transforms({})[RIGHT_TCP][:3, 3]
        ik = SimpleNamespace(origin_tcp=origin)
        offsets, outside = scan_offsets(ik, zone)
        np.testing.assert_allclose(offsets[0], INITIAL_OFFSET)
        self.assertTrue(all(abs((b[1] - a[1]) - Y_STEP_M) < 1e-12
                            for a, b in zip(offsets, offsets[1:])))
        self.assertTrue(all(zone.contains(origin + offset, MEMBERSHIP_BUFFER_M) for offset in offsets))
        self.assertFalse(zone.contains(origin + outside, MEMBERSHIP_BUFFER_M))
        self.assertAlmostEqual(offsets[-1][1], .19)
        self.assertAlmostEqual(outside[1], .20)

    def test_real_search_plan_and_dynamic_candidate_keep_joint2_centered(self):
        _model, _zone, center, ik = self.real_ik()
        plan = build_search_plan(ik, ik.zone)
        self.assertEqual(len(plan.results), 20)
        self.assertTrue(all(abs(result.joints[1] - center[1]) < 1e-12
                            for result in plan.results))
        self.assertTrue(all(result.error_m <= .002 for result in plan.results))
        candidate = solve_search_coordinate(
            ik, INITIAL_OFFSET + np.array([.01, 0, 0]), plan.results[0].joints
        )
        self.assertAlmostEqual(candidate.joints[1], center[1])
        self.assertLessEqual(candidate.error_m, .002)

    def test_encoder_sized_limit_overshoot_is_clipped_only_for_optimizer_seed(self):
        _model, _zone, _center, ik = self.real_ik()
        anchor = solve_search_coordinate(ik, INITIAL_OFFSET, ik.home).joints
        measured_anchor = anchor.copy()
        measured_anchor[6] = ik.upper[6] + 2.0 * ENCODER_LSB_RAD
        unchanged_anchor = measured_anchor.copy()

        candidate = solve_search_coordinate(
            ik, INITIAL_OFFSET + np.array([.01, 0, 0]), measured_anchor
        )

        np.testing.assert_array_equal(measured_anchor, unchanged_anchor)
        self.assertTrue(np.all(candidate.joints >= ik.lower))
        self.assertTrue(np.all(candidate.joints <= ik.upper))
        self.assertAlmostEqual(candidate.joints[1], ik.home[1])
        self.assertLessEqual(candidate.error_m, .002)

    def test_real_limit_violation_is_not_hidden_by_optimizer_seed_clipping(self):
        _model, _zone, _center, ik = self.real_ik()
        measured_anchor = solve_search_coordinate(ik, INITIAL_OFFSET, ik.home).joints
        measured_anchor[6] = ik.upper[6] + 2.0 * IK_SEED_LIMIT_TOLERANCE_RAD

        with self.assertRaisesRegex(ValueError, "outside IK limits.*encoder tolerance"):
            solve_search_coordinate(
                ik, INITIAL_OFFSET + np.array([.01, 0, 0]), measured_anchor
            )

    def test_hill_climber_uses_fixed_order_and_step_reductions(self):
        hill = HillClimber(INITIAL_OFFSET.copy(), np.zeros(7), anchor_score=1.0)
        self.assertEqual(tuple(name for name, _ in HILL_DIRECTIONS),
                         ("+X", "-X", "+Y", "-Y", "+Z", "-Z"))
        self.assertEqual(HILL_STEPS_M, (.01, .005, .0025))
        for expected in ("+X", "-X", "+Y", "-Y", "+Z", "-Z"):
            self.assertEqual(hill.direction_name, expected)
            self.assertTrue(hill.reject_or_skip())
        self.assertEqual(hill.step, .005)
        for _ in range(6):
            self.assertTrue(hill.reject_or_skip())
        self.assertEqual(hill.step, .0025)
        for _ in range(5):
            self.assertTrue(hill.reject_or_skip())
        self.assertFalse(hill.reject_or_skip())

    def test_hill_accept_keeps_direction_until_it_stops_improving(self):
        hill = HillClimber(np.zeros(3), np.zeros(7), anchor_score=1.0)
        candidate = hill.candidate_offset()
        hill.accept(candidate, np.ones(7), .8)
        self.assertEqual(hill.direction_name, "+X")
        np.testing.assert_allclose(hill.candidate_offset(), [.02, 0, 0])
        self.assertTrue(hill.reject_or_skip())
        self.assertEqual(hill.direction_name, "-X")
        for _ in range(5):
            self.assertTrue(hill.reject_or_skip())
        self.assertEqual(hill.direction_name, "+X")
        self.assertEqual(hill.step, .01)

    def test_final_planner_skip_reports_actual_infeasible_reason(self):
        app = self.search_app()
        app.alignment_active = True
        app.phase = "HILL PLANNING"
        app.hill = HillClimber(
            INITIAL_OFFSET.copy(), np.zeros(7), anchor_score=.2,
            step_index=2, direction_index=5,
        )
        app.planner_token = 7
        future = Mock()
        future.done.return_value = True
        future.result.side_effect = ValueError("test seed is infeasible")
        app.planner_future = (7, future)
        app._program_failure = Mock()
        app._plan_next_candidate = Mock()

        app._poll_hill_planner(1.0)

        app._program_failure.assert_called_once()
        failure = app._program_failure.call_args.args[0]
        self.assertIn("no feasible Cartesian direction", failure)
        self.assertIn("test seed is infeasible", failure)
        app._plan_next_candidate.assert_not_called()

    def search_app(self):
        app = App.__new__(App)
        app.side = "right"
        app.hardware = True
        app.phase = "MOVING TO SEARCH START"
        app.search_active = True
        app.search_index = 0
        app.pending_detection_frame_id = None
        app.search_plan = SimpleNamespace(
            results=(SimpleNamespace(joints=np.ones(7), target=np.ones(3)),
                     SimpleNamespace(joints=np.ones(7) * 2, target=np.ones(3) * 2)),
            first_outside_offset=INITIAL_OFFSET + np.array([0., .02, 0.]),
        )
        app.ik = SimpleNamespace(origin_tcp=np.zeros(3))
        app.target_offset = INITIAL_OFFSET.copy()
        app.tcp = Mock(return_value=INITIAL_OFFSET.copy())
        app.arm = Mock(return_value=np.zeros(7))
        app.begin_stage = Mock()
        app.status = Mock()
        app.result_status = Mock()
        app.result_label = Mock()
        app.root = Mock()
        app.center_goal = np.zeros(7)
        app.control = SimpleNamespace(desired=np.zeros(7))
        app.relax = Mock()
        app.bus = Mock(active=True)
        app.bus.fresh.return_value = True
        app.zone = Mock()
        app.zone.contains.return_value = True
        app.planned_tcp = Mock(return_value=np.zeros(3))
        app.setup = None
        app.receiver = Mock(frame_id=44, cymbal=True)
        app.receiver.fresh.return_value = True
        app.continue_button = Mock()
        app.gripper_closed_latched = False
        app.gripper_close_until = None
        app.last_gripper_command = 0.0
        app.alignment_active = False
        app.alignment_loss_deadline = None
        app.hill = None
        app.measurement_kind = None
        app.measurement_samples = []
        app.measurement_frame_ids = set()
        app.measurement_first_valid_at = None
        app.hold_inside_since = None
        app.planner_token = 0
        app.planner_future = None
        app.pending_candidate_result = None
        app.pending_candidate_offset = None
        app._camera_fault_handled = False
        app._last_fault_report = None
        app.support_fault_detail = None
        return app

    def test_no_detection_advances_only_y(self):
        app = self.search_app()
        app.complete_stage()
        self.assertEqual(app.search_index, 1)
        np.testing.assert_allclose(app.target_offset, INITIAL_OFFSET + np.array([0., Y_STEP_M, 0.]))
        app.begin_stage.assert_called_once_with("SEARCHING +Y", app.goal)

    def test_detection_is_latched_until_pose_settles_then_starts_hill(self):
        app = self.search_app()
        app.target_offset = INITIAL_OFFSET.copy()
        app._detection_success(12)
        self.assertEqual(app.pending_detection_frame_id, 12)
        self.assertTrue(app.search_active)
        app.begin_stage.assert_not_called()

        app._begin_hill_climb = Mock()
        app.complete_stage(8.0)
        app._begin_hill_climb.assert_called_once_with(8.0)

    def test_search_exhaustion_recenters_before_relax(self):
        app = self.search_app()
        app.search_index = 1
        app.complete_stage()
        app.begin_stage.assert_called_once_with("RECENTERING AFTER FAILURE", app.center_goal)
        app.relax.assert_not_called()
        self.assertIn("outside right zone1", app.result_status.set.call_args_list[0].args[0])

    def test_start_and_continue_are_blocked_without_cymbal(self):
        app = App.__new__(App)
        app.hardware = True
        app.phase = "READY"
        app.bus = Mock()
        app.bus.fresh.return_value = True
        app.receiver = Mock(cymbal=False)
        app.receiver.fresh.return_value = True
        app.support_fault_detail = None
        app.status = Mock()
        app.start()
        self.assertIn("no cymbal", app.status.set.call_args.args[0])

        app = self.search_app()
        app.phase = "WAITING FOR LOAD"
        app.receiver.cymbal = False
        app.continue_motion()
        self.assertIn("no longer visible", app.status.set.call_args.args[0])
        app.bus.set_gripper.assert_not_called()

    def test_gripper_close_requires_fresh_cymbal_then_starts_search(self):
        app = self.search_app()
        app.phase = "CLOSING GRIPPER"
        app.gripper_closed_latched = True
        app.gripper_close_until = 5.0
        app.last_gripper_command = 5.0
        app.goal = np.ones(7)
        app.extra_control(5.0)
        self.assertTrue(app.search_active)
        self.assertEqual(app.detection_gate_frame_id, 44)
        app.begin_stage.assert_called_once_with("MOVING TO SEARCH START", app.goal)

        app = self.search_app()
        app.phase = "CLOSING GRIPPER"
        app.gripper_closed_latched = True
        app.gripper_close_until = 5.0
        app.last_gripper_command = 5.0
        app.receiver.cymbal = False
        app.extra_control(5.0)
        app.begin_stage.assert_called_once_with("RECENTERING AFTER FAILURE", app.center_goal)

    def test_camera_failure_during_search_returns_to_center(self):
        app = self.search_app()
        app.setup = object()
        app._camera_failure("test camera loss")
        self.assertTrue(app._camera_fault_handled)
        self.assertFalse(app.search_active)
        self.assertIsNone(app.setup)
        app.begin_stage.assert_called_once_with("RECENTERING AFTER FAILURE", app.center_goal)
        self.assertIn("camera loss", app.result_status.set.call_args_list[0].args[0])

    def test_missing_tip_frames_do_not_cancel_measurement_or_target_hold(self):
        app = self.search_app()
        app.alignment_active = True
        app.phase = "HILL MEASURING ANCHOR"
        app.measurement_gate_frame_id = 10
        app.measurement_samples = []
        app.measurement_frame_ids = set()
        app.measurement_first_valid_at = None
        app.alignment_loss_deadline = 35.0
        missing = alignment_message(11)
        missing["required"] = False
        missing["tip_point"] = None
        app._process_alignment_frame(missing, 5.0)
        self.assertEqual(app.measurement_samples, [])
        self.assertEqual(app.alignment_loss_deadline, 35.0)
        self.assertEqual(app.phase, "HILL MEASURING ANCHOR")

        app.phase = "HOLDING CYMBAL TARGET"
        app.hold_inside_since = 1.0
        app._process_alignment_frame(missing, 4.0)
        self.assertEqual(app.hold_inside_since, 1.0)
        app.begin_stage.assert_not_called()

    def test_baseline_measurement_uses_median_then_plans_first_direction(self):
        app = self.search_app()
        app.alignment_active = True
        app.phase = "HILL MEASURING ANCHOR"
        app.measurement_kind = "anchor"
        app.measurement_first_valid_at = 1.0
        app.measurement_samples = [
            VisualObservation((250, 150), (0, 0, 300, 300), (100, 100, 200, 200)),
            VisualObservation((248, 151), (0, 0, 300, 300), (100, 100, 200, 200)),
        ]
        app.hill = HillClimber(np.array([.4, .02, .25]), np.zeros(7))
        app._plan_next_candidate = Mock()
        app._advance_measurement(1.5)
        self.assertGreater(app.hill.anchor_score, 0.0)
        self.assertEqual(app.hill.direction_name, "+X")
        app._plan_next_candidate.assert_called_once()

    def test_candidate_accepts_only_lower_score_and_reject_returns_to_anchor(self):
        app = self.search_app()
        app.alignment_active = True
        app.hill = HillClimber(
            np.array([.4, .02, .25]), np.zeros(7), anchor_score=.2
        )
        app.phase = "HILL MEASURING CANDIDATE"
        app.measurement_kind = "candidate"
        app.measurement_first_valid_at = 1.0
        app.measurement_samples = [
            VisualObservation((230, 150), (0, 0, 300, 300), (100, 100, 200, 200))
        ]
        app.pending_candidate_offset = np.array([.41, .02, .25])
        app.pending_candidate_result = SimpleNamespace(joints=np.ones(7))
        app._plan_next_candidate = Mock()
        app._advance_measurement(1.5)
        self.assertAlmostEqual(app.hill.anchor_score, .1)
        self.assertEqual(app.hill.direction_name, "+X")
        app._plan_next_candidate.assert_called_once()

        app = self.search_app()
        app.alignment_active = True
        app.hill = HillClimber(
            np.array([.4, .02, .25]), np.zeros(7), anchor_score=.05
        )
        app.phase = "HILL MEASURING CANDIDATE"
        app.measurement_kind = "candidate"
        app.measurement_first_valid_at = 1.0
        app.measurement_samples = [
            VisualObservation((230, 150), (0, 0, 300, 300), (100, 100, 200, 200))
        ]
        app.pending_candidate_offset = np.array([.41, .02, .25])
        app.pending_candidate_result = SimpleNamespace(joints=np.ones(7))
        app._advance_measurement(1.5)
        self.assertEqual(app.begin_stage.call_args.args[0], "HILL RETURNING TO ANCHOR")
        np.testing.assert_allclose(app.begin_stage.call_args.args[1], app.hill.anchor_joints)
        self.assertAlmostEqual(app.hill.anchor_score, .05)

    def test_prolonged_visual_loss_routes_to_center_recovery(self):
        app = self.search_app()
        app.alignment_active = True
        app.phase = "HILL PLANNING"
        app.alignment_loss_deadline = 5.0
        app._poll_hill_planner = Mock()
        app._advance_measurement = Mock()
        app._advance_target_hold = Mock()
        app._program_failure = Mock()
        app.extra_control(5.0)
        app._program_failure.assert_called_once_with(
            "FAILED: no simultaneous cymbal + observed drumstick tip for 30 seconds"
        )

    def test_target_requires_two_seconds_and_fresh_inside_confirmation(self):
        app = self.search_app()
        app.alignment_active = True
        app.phase = "HOLDING CYMBAL TARGET"
        app.hold_inside_since = 10.0
        app._process_alignment_frame(alignment_message(50), 11.9)
        app.begin_stage.assert_not_called()
        app._process_alignment_frame(alignment_message(51), 12.1)
        app.begin_stage.assert_called_once_with("RECENTERING AFTER SUCCESS", app.center_goal)
        self.assertFalse(app.alignment_active)
        app.relax.assert_not_called()

    def test_outside_tip_resets_hold_and_remeasures_without_moving(self):
        app = self.search_app()
        app.alignment_active = True
        app.phase = "HOLDING CYMBAL TARGET"
        app.hold_inside_since = 10.0
        app._start_measurement = Mock()
        app._process_alignment_frame(alignment_message(52, tip=(250, 150)), 12.1)
        app._start_measurement.assert_called_once_with("anchor", 12.1)
        app.begin_stage.assert_not_called()
        self.assertEqual(len(app.measurement_samples), 1)

    def test_nonstall_fault_recenters_but_stall_relaxes_immediately(self):
        app = self.search_app()
        app.zone = Mock()
        app.zone.contains.return_value = True
        app.planned_tcp = Mock(return_value=np.zeros(3))
        app.setup = object()
        app.hold_until = 1.0
        app.fail("Position timeout")
        app.begin_stage.assert_called_once_with("FAULT RECENTERING", app.center_goal)
        app.relax.assert_not_called()

        app = self.search_app()
        app.fail("STALL: motor 3")
        app.begin_stage.assert_not_called()
        app.relax.assert_called_once()
        self.assertEqual(app.phase, "FAULT")

    def test_repeated_nonstall_recovery_fault_never_relaxes_away_from_center(self):
        app = self.search_app()
        app.zone = Mock()
        app.zone.contains.return_value = True
        app.planned_tcp = Mock(return_value=np.zeros(3))
        app.phase = "FAULT RECENTERING"
        app.control = object()
        app.fail("Position timeout")
        app.begin_stage.assert_called_once_with("FAULT RECENTERING", app.center_goal)
        app.relax.assert_not_called()

        app = self.search_app()
        app.zone = Mock()
        app.zone.contains.return_value = True
        app.planned_tcp = Mock(return_value=np.zeros(3))
        app.begin_stage.side_effect = OSError("temporary CAN congestion")
        app.fail("Position timeout")
        app.relax.assert_not_called()
        self.assertEqual(app.phase, "FAULT RECENTERING")
        self.assertIn("motors remain enabled", app.result_status.set.call_args.args[0])

    def test_success_failure_and_fault_returns_relax_only_after_center_completion(self):
        for phase, expected in (
            ("RECENTERING AFTER SUCCESS", "Cymbal target held"),
            ("RECENTERING AFTER FAILURE", "Sequence failed"),
            ("FAULT RECENTERING", "Fault recovery complete"),
        ):
            with self.subTest(phase=phase):
                app = self.search_app()
                app.phase = phase
                app.complete_stage()
                self.assertEqual(app.relax.call_count, 1)
                self.assertIn(expected, app.relax.call_args.args[0])


if __name__ == "__main__":
    unittest.main()
