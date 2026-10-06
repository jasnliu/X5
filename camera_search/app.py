"""Right-arm camera search followed by deterministic Cartesian hill climbing."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import math
from pathlib import Path
import time
import tkinter as tk

import numpy as np
import rclpy

from cartesian_goal.app import (
    App as CartesianApp,
    GRIPPER_COMMAND_INTERVAL,
    GRIPPER_TOLERANCE,
)
from cartesian_goal.ik import CartesianIK
from centering.motors import RIGHT_GRIPPER_OPEN, RIGHT_GRIPPER_CLOSED
from goal_motion.app import App as JointGoalApp
from safe_zone.geometry import MEMBERSHIP_BUFFER_M, Model, RIGHT_TCP, Zone
from .control import CameraGoalControl
from .hill_climb import HillClimber
from .planner import INITIAL_OFFSET, Y_STEP_M, build_search_plan, solve_search_coordinate
from .protocol import CAMERA_FRESH_SECONDS, DetectionReceiver
from .vision import median_target_distance, normalized_target_distance, observation_from_message


ROOT = Path(__file__).resolve().parents[1]
MEASUREMENT_SECONDS = CAMERA_FRESH_SECONDS
DETECTION_LOSS_TIMEOUT_SECONDS = 30.0
TARGET_HOLD_SECONDS = 2.0


class App(CartesianApp):
    ACTIVE_SEARCH_PHASES = {"MOVING TO SEARCH START", "SEARCHING +Y"}
    RETURN_PHASES = {"RECENTERING AFTER SUCCESS", "RECENTERING AFTER FAILURE"}
    MEASUREMENT_PHASES = {"HILL MEASURING ANCHOR", "HILL MEASURING CANDIDATE"}

    def __init__(self, hardware: bool, detection_socket: str):
        # The standalone right Cartesian panel has a manual End button.  This
        # camera workflow owns its separate automatic success/failure ending.
        super().__init__(hardware, "right", manual_finish=False)
        self.receiver = DetectionReceiver(detection_socket)
        self.search_plan = None
        self.search_index = 0
        self.search_active = False
        self.detection_gate_frame_id = -1
        self.last_qualifying_frame_id = -1
        self.pending_detection_frame_id = None
        self._camera_fault_handled = False
        self._last_fault_report = None
        self.support_fault_detail = None

        # Keep SciPy planning off the Tk/CAN thread. An independent model and
        # zone also avoid sharing mutable RViz geometry with the worker.
        planner_model = Model(ROOT / "model/openarmx.urdf")
        planner_zone = Zone.load(
            ROOT / "right_zones/zone1.json", planner_model.digest, RIGHT_TCP
        )
        self.hill_ik = CartesianIK(
            planner_model, planner_zone, self.lower, self.upper, self.ik.speed,
            "right", RIGHT_TCP, self.center_goal, np.zeros(7),
        )
        self.planner_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="hill-ik")
        self.planner_future = None
        self.planner_token = 0
        self.pending_candidate_result = None
        self.pending_candidate_offset = None

        self.alignment_active = False
        self.hill = None
        self.measurement_kind = None
        self.measurement_gate_frame_id = -1
        self.measurement_frame_ids = set()
        self.measurement_samples = []
        self.measurement_first_valid_at = None
        self.alignment_loss_deadline = None
        self.hold_inside_since = None

        self.root.title("RIGHT arm: Camera Cartesian hill climber" +
                        ("" if hardware else " — OFFLINE PREVIEW"))
        self.header.config(text="RIGHT arm: fixed search + deterministic cymbal alignment")
        self._set_preview(INITIAL_OFFSET)
        self.coordinate_form.destroy()
        self.preview_label.destroy()
        self.xyz_label.destroy()
        self.start_button.config(text="START: CENTER + OPEN GRIPPER TO −3°")
        self.continue_button.config(
            text="CONTINUE: CLOSE GRIPPER TO +7° → SEARCH + ALIGN"
        )
        self.isolation_label.config(
            text=(f"Fixed start (m): X={INITIAL_OFFSET[0]:+.3f}, "
                  f"Y={INITIAL_OFFSET[1]:+.3f}, Z={INITIAL_OFFSET[2]:+.3f} "
                  f"• initial Y += {Y_STEP_M:.3f}\n"
                  "hill order +X, −X, +Y, −Y, +Z, −Z • right zone1 enforced")
        )
        self.camera_status = tk.StringVar(value="Camera: starting")
        self.camera_label = tk.Label(
            self.root, textvariable=self.camera_status, font=("Sans", 12, "bold"), fg="#8a5a00"
        )
        self.camera_label.pack(fill="x", padx=20, pady=5, before=self.start_button)
        self.result_status = tk.StringVar(value="Result: not started")
        self.result_label = tk.Label(
            self.root, textvariable=self.result_status, font=("Sans", 13, "bold"),
            wraplength=650, fg="navy",
        )
        self.result_label.pack(fill="x", padx=20, pady=5, after=self.continue_button)
        self.status.set("Waiting for a cymbal, camera frames, and live encoders" if hardware
                        else "Offline preview — no CAN; camera remains live")
        self.root.after(20, self.camera_tick)

    def _camera_text(self, now: float) -> tuple[str, str]:
        if self.support_fault_detail:
            return "Process fault — restart program: " + self.support_fault_detail, "#b00020"
        if self.receiver.state == "error":
            return "Camera ERROR: " + self.receiver.detail, "#b00020"
        if not self.receiver.fresh(now):
            return "Camera: waiting for a fresh processed detection frame", "#8a5a00"
        pieces = [
            "cymbal YES" if self.receiver.cymbal else "cymbal no",
            "drumstick YES" if self.receiver.drumstick else "drumstick no",
            "tip YES" if self.receiver.tip else "tip no",
        ]
        if self.receiver.required and self.receiver.cymbal:
            return "Camera: ALIGNMENT DATA — cymbal + drumstick + observed YOLO tip", "#087f23"
        if self.receiver.required:
            return "Camera: drumstick + observed YOLO tip; waiting for cymbal", "#8a5a00"
        return "Camera: " + " • ".join(pieces), "#444444"

    def camera_tick(self):
        try:
            messages = self.receiver.poll()
            now = time.monotonic()
            text, color = self._camera_text(now)
            self.camera_status.set(text)
            self.camera_label.config(fg=color)
            for message in messages:
                if message.get("kind") != "frame":
                    continue
                if (message.get("required") is True
                        and message["frame_id"] > self.detection_gate_frame_id
                        and message["frame_id"] > self.last_qualifying_frame_id
                        and self.search_active and self.phase in self.ACTIVE_SEARCH_PHASES):
                    self.last_qualifying_frame_id = message["frame_id"]
                    self._detection_success(message["frame_id"])
                if self.alignment_active:
                    self._process_alignment_frame(message, now)

            process_problem = next(
                (str(message.get("detail", "camera/visualization process stopped"))
                 for message in messages
                 if message.get("kind") == "status"
                 and message.get("state") in {"error", "stopped"}),
                None,
            )
            if process_problem is None and self.receiver.state in {"error", "stopped"}:
                process_problem = self.receiver.detail or "camera/visualization process stopped"

            if process_problem is not None:
                self.support_fault_detail = process_problem
                if self.search_active:
                    self._camera_failure("camera/visualization process error: " + process_problem)
                elif self.alignment_active or (self.bus and self.bus.active):
                    self._program_failure(
                        "FAILED: camera/visualization process error: " + process_problem
                    )
            elif self.search_active and self.phase in self.ACTIVE_SEARCH_PHASES:
                if not self.receiver.fresh(now):
                    self._camera_failure("processed camera detections became stale")
            self._refresh_buttons()
        except Exception as exc:
            if self.search_active or self.alignment_active:
                self._program_failure("FAILED: camera status failure: " + str(exc))
            else:
                self.camera_status.set("Camera ERROR: " + str(exc))
                self.camera_label.config(fg="#b00020")
        self.root.after(20, self.camera_tick)

    def _camera_ready_for_start(self, now: float | None = None) -> bool:
        return (not getattr(self, "support_fault_detail", None)
                and self.receiver.fresh(now) and self.receiver.cymbal)

    def _refresh_buttons(self) -> None:
        if not self.hardware or not self.bus:
            self.start_button.config(state="disabled")
            self.continue_button.config(state="disabled")
            return
        bus_fresh = self.bus.fresh()
        camera_ready = self._camera_ready_for_start()
        self.start_button.config(
            state="normal" if (self.phase in ("READY", "RELAXED")
                               and bus_fresh and camera_ready) else "disabled"
        )
        self.continue_button.config(
            state="normal" if (self.phase == "WAITING FOR LOAD"
                               and bus_fresh and camera_ready) else "disabled"
        )

    def tick(self):
        """Apply cymbal gating after the inherited tick updates its buttons."""
        try:
            super().tick()
        except Exception as exc:
            # The base loop catches CAN/control/publish failures, but ROS
            # spin_once runs just outside that catch. Route it through the same
            # center-before-relax policy and keep the recovery loop alive.
            self.fail(str(exc))
            self.root.after(20, self.tick)
        try:
            self._refresh_buttons()
        except tk.TclError:
            pass

    def _select_result(self, index: int) -> None:
        result = self.search_plan.results[index]
        self.search_index = index
        self.goal = result.joints.copy()
        self.target_point = result.target.copy()
        self.target_offset = INITIAL_OFFSET.copy()
        self.target_offset[1] = INITIAL_OFFSET[1] + index * Y_STEP_M
        self.target_inside = True

    def begin_stage(self, name, desired):
        """Use the original controller plus one encoder count of hysteresis."""
        self.hold_until = None
        self.control = CameraGoalControl(desired, self.lower, self.upper, time.monotonic())
        self.phase = name
        self.bus.set_positions(desired)
        self.last_command = 0.0
        self.status.set(name.replace("_", " "))

    def _cancel_planning(self) -> None:
        self.planner_token = getattr(self, "planner_token", 0) + 1
        if getattr(self, "planner_future", None) is not None:
            self.planner_future[1].cancel()
        self.planner_future = None
        self.pending_candidate_result = None
        self.pending_candidate_offset = None

    def _clear_alignment_state(self) -> None:
        self._cancel_planning()
        self.alignment_active = False
        self.hill = None
        self.measurement_kind = None
        self.measurement_samples = []
        self.measurement_frame_ids = set()
        self.measurement_first_valid_at = None
        self.alignment_loss_deadline = None
        self.hold_inside_since = None

    def start(self):
        if not self.hardware:
            self.status.set("Offline preview only — add --hardware to run the button sequence")
            return
        if self.phase not in ("READY", "RELAXED") or not self.bus.fresh():
            return
        if self.support_fault_detail:
            self.status.set("Cannot start: support-process fault is latched; restart the program")
            return
        if not self.receiver.fresh():
            self.status.set("Cannot start: waiting for a fresh processed camera detection frame")
            return
        if not self.receiver.cymbal:
            self.status.set("Cannot start: no cymbal is visible in the processed camera frame")
            return
        if not self.zone.contains(self.planned_tcp(self.center_goal), MEMBERSHIP_BUFFER_M):
            self.status.set("ERROR: configured right center is outside right_zones/zone1.json")
            return
        self.status.set("Preflighting the fixed +Y search — motors remain disabled")
        self.root.update_idletasks()
        try:
            self.search_plan = build_search_plan(self.ik, self.zone)
        except ValueError as exc:
            self.status.set("ERROR: camera-search preflight failed: " + str(exc))
            return
        self._select_result(0)
        self.gripper_closed_latched = False
        self.gripper_close_until = None
        self.last_gripper_command = 0.0
        self.search_active = False
        self.pending_detection_frame_id = None
        self._clear_alignment_state()
        self._camera_fault_handled = False
        self.result_status.set("Result: searching has not started")
        self.result_label.config(fg="navy")
        self.pending_result = self.search_plan.results[0]
        self.phase = "IK SOLVED"
        self.ik_refresh_deadline = time.monotonic() + 2.0
        last_safe = self.search_plan.results[-1].target - self.ik.origin_tcp
        outside = self.search_plan.first_outside_offset
        self.status.set(
            f"Preflight ready: {len(self.search_plan.results)} safe goals through Y={last_safe[1]:+.3f}; "
            f"next Y={outside[1]:+.3f} is outside — refreshing camera and encoders"
        )
        self.root.after(20, self._start_solved_goal)

    def _start_solved_goal(self):
        if self.phase != "IK SOLVED":
            self.pending_result = None
            return
        self.receiver.poll()
        if not self.bus.fresh() or not self._camera_ready_for_start():
            if time.monotonic() >= self.ik_refresh_deadline:
                self.pending_result = None
                self.phase = "READY"
                self.status.set("ERROR: fresh encoders and a visible cymbal did not return after preflight")
                return
            self.root.after(20, self._start_solved_goal)
            return
        result = self.pending_result
        self.pending_result = None
        self.phase = "READY"
        JointGoalApp.start(self)
        if self.phase == "SETTING UP":
            self.status.set(
                f"Search IK ready (initial error {result.error_m*1000:.2f} mm); "
                f"centering with J7={math.degrees(self.center_goal[6]):.1f}° and opening gripper to −3°"
            )

    def continue_motion(self):
        if not self.hardware or self.phase != "WAITING FOR LOAD":
            return
        self.receiver.poll()
        if self.support_fault_detail:
            self.status.set("Cannot continue: support-process fault is latched; restart the program")
            return
        if not self.receiver.fresh():
            self.status.set("Cannot continue: waiting for fresh processed camera detections")
            return
        if not self.receiver.cymbal:
            self.status.set("Cannot continue: the cymbal is no longer visible")
            return
        super().continue_motion()

    def extra_control(self, now):
        self.continue_button.config(
            state="normal" if (self.phase == "WAITING FOR LOAD" and self.bus.fresh()
                               and self._camera_ready_for_start(now)) else "disabled"
        )
        if not self.bus.active:
            return
        target = None
        if self.gripper_closed_latched:
            target = RIGHT_GRIPPER_CLOSED
        elif self.phase in ("CENTERING", "WAITING FOR LOAD"):
            target = RIGHT_GRIPPER_OPEN
        if target is not None and now - self.last_gripper_command >= GRIPPER_COMMAND_INTERVAL:
            self.bus.set_gripper(target)
            self.last_gripper_command = now

        if self.phase == "CLOSING GRIPPER":
            remaining = max(0.0, self.gripper_close_until - now)
            self.status.set(f"Closing gripper to +7° — camera search starts in {remaining:.1f} s")
            if remaining <= 0.0:
                self.receiver.poll()
                if not self.receiver.fresh():
                    self._program_failure(
                        "FAILED: fresh processed camera detections unavailable after gripper close"
                    )
                    return
                if not self.receiver.cymbal:
                    self._program_failure("FAILED: cymbal unavailable after gripper close")
                    return
                self.search_active = True
                self.detection_gate_frame_id = self.receiver.frame_id
                self.begin_stage("MOVING TO SEARCH START", self.goal)
                self.status.set(
                    "Gripper closed at +7° — moving to "
                    f"X={self.target_offset[0]:+.3f}, Y={self.target_offset[1]:+.3f}, "
                    f"Z={self.target_offset[2]:+.3f}"
                )

        if self.alignment_active:
            self._poll_hill_planner(now)
            self._advance_measurement(now)
            self._advance_target_hold(now)
            if (self.alignment_active and self.alignment_loss_deadline is not None
                    and now >= self.alignment_loss_deadline):
                self._program_failure(
                    "FAILED: no simultaneous cymbal + observed drumstick tip for 30 seconds"
                )

    def complete_stage(self, now=None):
        now = time.monotonic() if now is None else now
        if self.phase == "CENTERING":
            self.centered(now)
            return
        if self.phase in self.ACTIVE_SEARCH_PHASES:
            if self.pending_detection_frame_id is not None:
                self._begin_hill_climb(now)
            elif self.search_index + 1 < len(self.search_plan.results):
                self._select_result(self.search_index + 1)
                self.begin_stage("SEARCHING +Y", self.goal)
                self.status.set(
                    "No drumstick + tip detection — moving to "
                    f"X={self.target_offset[0]:+.3f}, Y={self.target_offset[1]:+.3f}, "
                    f"Z={self.target_offset[2]:+.3f}"
                )
            else:
                outside_y = self.search_plan.first_outside_offset[1]
                self._program_failure(
                    f"FAILED: no drumstick + tip detection; next Y={outside_y:+.3f} m "
                    "is outside right zone1"
                )
            return
        if self.phase == "HILL MOVING CANDIDATE":
            self._start_measurement("candidate", now)
            return
        if self.phase == "HILL RETURNING TO ANCHOR":
            if not self.hill.reject_or_skip():
                self._program_failure(
                    "FAILED: no improving Cartesian direction remained at the 0.0025 m step"
                )
            else:
                self._plan_next_candidate()
            return
        if self.phase in self.RETURN_PHASES:
            finished_phase = self.phase
            self.control = None
            self.search_active = False
            self._clear_alignment_state()
            if finished_phase == "RECENTERING AFTER SUCCESS":
                self.relax("Cymbal target held for 2 seconds; centered; right motors disabled")
            else:
                self.relax("Sequence failed; centered; right motors disabled")
            return
        if self.phase == "FAULT RECENTERING":
            self.control = None
            self.search_active = False
            self._clear_alignment_state()
            self.relax("Fault recovery complete: centered; right motors disabled")
            return
        super().complete_stage(now)

    def _reset_hold_controller(self) -> None:
        desired = (self.control.desired.copy() if self.control is not None
                   else self.arm().copy())
        self.control = CameraGoalControl(desired, self.lower, self.upper, time.monotonic())
        self.last_command = 0.0

    def _detection_success(self, frame_id: int) -> None:
        """Latch a flash; wait until the active Cartesian stage has settled."""
        if (not self.search_active or self.phase not in self.ACTIVE_SEARCH_PHASES
                or self.pending_detection_frame_id is not None):
            return
        self.pending_detection_frame_id = frame_id
        actual_offset = self.tcp() - self.ik.origin_tcp
        message = (f"DETECTED: drumstick + observed YOLO tip at actual Y={actual_offset[1]:+.3f} m "
                   f"(active goal Y={self.target_offset[1]:+.3f}, camera frame {frame_id})")
        self.result_status.set(message + " — settling before cymbal alignment")
        self.result_label.config(fg="#087f23")
        print(message, flush=True)
        self.status.set(message + " — finish settling, then start deterministic hill climbing")
        try:
            self.root.bell()
        except tk.TclError:
            pass

    def _begin_hill_climb(self, now: float) -> None:
        frame_id = self.pending_detection_frame_id
        self.pending_detection_frame_id = None
        self.search_active = False
        self.alignment_active = True
        actual_offset = self.tcp() - self.ik.origin_tcp
        self.hill = HillClimber(actual_offset, self.arm())
        self.alignment_loss_deadline = now + DETECTION_LOSS_TIMEOUT_SECONDS
        self._start_measurement("anchor", now)
        message = (f"DETECTED in camera frame {frame_id}; hill climber started at "
                   f"X={actual_offset[0]:+.3f}, Y={actual_offset[1]:+.3f}, Z={actual_offset[2]:+.3f}")
        self.result_status.set(message)
        self.result_label.config(fg="#087f23")
        self.status.set(message + " — collecting a robust baseline")

    def _start_measurement(self, kind: str, now: float) -> None:
        if kind not in {"anchor", "candidate"}:
            raise ValueError("Invalid hill-climber measurement kind")
        self.measurement_kind = kind
        self.measurement_gate_frame_id = self.receiver.frame_id
        self.measurement_frame_ids = set()
        self.measurement_samples = []
        self.measurement_first_valid_at = None
        self.hold_inside_since = None
        self.phase = "HILL MEASURING " + kind.upper()
        self._reset_hold_controller()
        label = "baseline" if kind == "anchor" else "candidate"
        self.status.set(
            f"Holding still — collecting fresh observed tips for {MEASUREMENT_SECONDS:.1f} s "
            f"to score the {label}"
        )

    def _process_alignment_frame(self, message: dict, now: float) -> None:
        observation = observation_from_message(message)
        if observation is None:
            # Missing tip/cymbal frames are deliberately ignored. They neither
            # fabricate a point nor cancel an in-progress measurement/hold.
            return
        self.alignment_loss_deadline = now + DETECTION_LOSS_TIMEOUT_SECONDS
        frame_id = message["frame_id"]
        if self.phase in self.MEASUREMENT_PHASES:
            if (frame_id <= self.measurement_gate_frame_id
                    or frame_id in self.measurement_frame_ids):
                return
            self.measurement_frame_ids.add(frame_id)
            self.measurement_samples.append(observation)
            if self.measurement_first_valid_at is None:
                self.measurement_first_valid_at = now
            return
        if self.phase == "HOLDING CYMBAL TARGET":
            score = normalized_target_distance(observation)
            if score == 0.0:
                if self.hold_inside_since is None:
                    self.hold_inside_since = now
                if now - self.hold_inside_since >= TARGET_HOLD_SECONDS:
                    self._alignment_success(frame_id)
            else:
                self.hold_inside_since = None
                self.status.set("Tip left the cymbal target — hold reset; remeasuring current pose")
                self._start_measurement("anchor", now)
                # Include this current valid outside observation in the robust window.
                self.measurement_gate_frame_id = frame_id - 1
                self.measurement_frame_ids.add(frame_id)
                self.measurement_samples.append(observation)
                self.measurement_first_valid_at = now

    def _advance_measurement(self, now: float) -> None:
        if self.phase not in self.MEASUREMENT_PHASES:
            return
        if self.measurement_first_valid_at is None:
            remaining = max(0.0, self.alignment_loss_deadline - now)
            self.status.set(
                "Holding still — waiting for simultaneous cymbal + observed tip "
                f"({remaining:.1f} s before abort)"
            )
            return
        elapsed = now - self.measurement_first_valid_at
        if elapsed < MEASUREMENT_SECONDS:
            self.status.set(
                f"Holding still — robust camera sample {len(self.measurement_samples)}; "
                f"{MEASUREMENT_SECONDS-elapsed:.1f} s remaining"
            )
            return
        kind = self.measurement_kind
        score = median_target_distance(self.measurement_samples)
        self.measurement_kind = None
        if score == 0.0:
            if kind == "candidate":
                self.hill.accept(
                    self.pending_candidate_offset,
                    self.pending_candidate_result.joints,
                    score,
                )
            else:
                self.hill.anchor_score = score
                self.hill.anchor_offset = (self.tcp() - self.ik.origin_tcp).copy()
                self.hill.anchor_joints = self.arm().copy()
            self._start_target_hold(now)
            return
        if kind == "anchor":
            self.hill.anchor_score = score
            self.hill.anchor_offset = (self.tcp() - self.ik.origin_tcp).copy()
            self.hill.anchor_joints = self.arm().copy()
            self._plan_next_candidate()
            return
        if self.hill.accepts(score):
            direction = self.hill.direction_name
            previous = self.hill.anchor_score
            self.hill.accept(
                self.pending_candidate_offset,
                self.pending_candidate_result.joints,
                score,
            )
            message = (f"IMPROVED {direction}: normalized target distance "
                       f"{previous:.5f} → {score:.5f}; continuing {direction}")
            self.result_status.set(message)
            self.result_label.config(fg="#087f23")
            self._plan_next_candidate()
        else:
            direction = self.hill.direction_name
            message = (f"No improvement in {direction}: {score:.5f} versus "
                       f"anchor {self.hill.anchor_score:.5f}; returning to anchor")
            self.result_status.set(message)
            self.result_label.config(fg="#8a5a00")
            self.goal = self.hill.anchor_joints.copy()
            self.target_offset = self.hill.anchor_offset.copy()
            self.target_point = self.ik.origin_tcp + self.target_offset
            self.target_inside = True
            self.begin_stage("HILL RETURNING TO ANCHOR", self.goal)
            self.status.set(message)

    def _start_target_hold(self, now: float) -> None:
        self.phase = "HOLDING CYMBAL TARGET"
        self.hold_inside_since = now
        self._reset_hold_controller()
        message = "TARGET REACHED: observed tip is inside the cymbal center cell"
        self.result_status.set(message + " — verifying a 2-second hold")
        self.result_label.config(fg="#087f23")
        self.status.set(message + " — hold still for 2.0 s")
        print(message, flush=True)

    def _advance_target_hold(self, now: float) -> None:
        if self.phase != "HOLDING CYMBAL TARGET":
            return
        if self.hold_inside_since is None:
            self.status.set("Target hold reset — waiting for a fresh observed tip inside the center cell")
            return
        remaining = max(0.0, TARGET_HOLD_SECONDS - (now - self.hold_inside_since))
        if remaining > 0.0:
            self.status.set(f"Tip inside cymbal target — holding still for {remaining:.1f} s")
        else:
            self.status.set("Two seconds elapsed — waiting for a fresh inside-tip confirmation")

    def _alignment_success(self, frame_id: int) -> None:
        if not self.alignment_active or self.phase != "HOLDING CYMBAL TARGET":
            return
        message = (f"SUCCESS: drumstick tip held inside cymbal center cell for 2 seconds "
                   f"(confirmed by camera frame {frame_id})")
        self.result_status.set(message)
        self.result_label.config(fg="#087f23")
        print(message, flush=True)
        try:
            self.root.bell()
        except tk.TclError:
            pass
        self.alignment_active = False
        self._cancel_planning()
        try:
            self.begin_stage("RECENTERING AFTER SUCCESS", self.center_goal)
            self.status.set(message + " — returning to customized right center, then relaxing")
        except Exception as exc:
            self.fail("success recenter command failed: " + str(exc))

    def _plan_next_candidate(self) -> None:
        if not self.alignment_active:
            return
        candidate = self.hill.candidate_offset()
        direction = self.hill.direction_name
        step = self.hill.step
        token = self.planner_token + 1
        self.planner_token = token
        future = self.planner_executor.submit(
            solve_search_coordinate,
            self.hill_ik,
            candidate.copy(),
            self.hill.anchor_joints.copy(),
        )
        self.planner_future = (token, future)
        self.pending_candidate_offset = candidate.copy()
        self.pending_candidate_result = None
        self.phase = "HILL PLANNING"
        self._reset_hold_controller()
        self.status.set(
            f"Holding anchor — planning {direction} by {step:.4f} m and validating "
            "candidate, return, and center paths"
        )

    def _poll_hill_planner(self, now: float) -> None:
        if self.phase == "HILL WAITING FOR CAMERA":
            if self.receiver.fresh(now) and self.receiver.cymbal:
                self._begin_candidate_move()
            return
        if self.phase != "HILL PLANNING" or self.planner_future is None:
            return
        token, future = self.planner_future
        if not future.done():
            return
        self.planner_future = None
        if token != self.planner_token or not self.alignment_active:
            return
        try:
            result = future.result()
        except ValueError as exc:
            direction = self.hill.direction_name
            message = f"Skipped {direction}: {exc}"
            self.result_status.set(message)
            self.result_label.config(fg="#8a5a00")
            print(message, flush=True)
            if not self.hill.reject_or_skip():
                self._program_failure(
                    "FAILED: no feasible Cartesian direction remained at the 0.0025 m step; "
                    f"last planner rejection for {direction}: {exc}"
                )
            else:
                self._plan_next_candidate()
            return
        except Exception as exc:
            self._program_failure("FAILED: hill-climber IK error: " + str(exc))
            return
        self.pending_candidate_result = result
        if self.receiver.fresh(now) and self.receiver.cymbal:
            self._begin_candidate_move()
        else:
            self.phase = "HILL WAITING FOR CAMERA"
            self.status.set("Candidate is safe — holding anchor until the cymbal camera stream is fresh")

    def _begin_candidate_move(self) -> None:
        result = self.pending_candidate_result
        if result is None or not self.alignment_active:
            return
        self.goal = result.joints.copy()
        self.target_point = result.target.copy()
        self.target_offset = self.pending_candidate_offset.copy()
        self.target_inside = True
        direction = self.hill.direction_name
        step = self.hill.step
        self.begin_stage("HILL MOVING CANDIDATE", self.goal)
        self.status.set(
            f"Testing {direction} by {step:.4f} m → "
            f"X={self.target_offset[0]:+.4f}, Y={self.target_offset[1]:+.4f}, "
            f"Z={self.target_offset[2]:+.4f}"
        )

    def _program_failure(self, message: str) -> None:
        if self.phase in self.RETURN_PHASES or self.phase == "FAULT RECENTERING":
            return
        message = str(message)
        self.result_status.set(message)
        self.result_label.config(fg="#b00020")
        print(message, flush=True)
        self.search_active = False
        self.pending_detection_frame_id = None
        self._clear_alignment_state()
        if self.bus and self.bus.active:
            self.setup = None
            if not self.zone.contains(
                    self.planned_tcp(self.center_goal), MEMBERSHIP_BUFFER_M):
                self.fail(message)
                return
            try:
                self.begin_stage("RECENTERING AFTER FAILURE", self.center_goal)
                self.status.set(message + " — returning to customized right center, then relaxing")
            except Exception as exc:
                self.fail(message + "; center command failed: " + str(exc))
        else:
            self.fail(message)

    def fail(self, message):
        """Recenter on every recoverable fault; only a detected stall relaxes immediately."""
        message = str(message)
        self.search_active = False
        self.pending_detection_frame_id = None
        self._clear_alignment_state()
        if message.startswith("STALL:"):
            result = "STALL FAULT: immediate right-arm relaxation — " + message
            if hasattr(self, "result_status"):
                self.result_status.set(result)
                self.result_label.config(fg="#b00020")
            print(result, flush=True)
            super().fail(message)
            return
        if self.bus and self.bus.active:
            center_is_valid = self.zone.contains(
                self.planned_tcp(self.center_goal), MEMBERSHIP_BUFFER_M
            )
            if not center_is_valid:
                result = ("FAULT: configured right center is invalid; motors remain enabled "
                          "because automatic relaxation before centering is prohibited — " + message)
                self.phase = "FAULT CENTER BLOCKED"
                self.control = None
                self.status.set(result)
                if hasattr(self, "result_status"):
                    self.result_status.set(result)
                    self.result_label.config(fg="#b00020")
                if result != getattr(self, "_last_fault_report", None):
                    print(result, flush=True)
                    self._last_fault_report = result
                return
            try:
                self.setup = None
                self.hold_until = None
                if (self.phase != "FAULT RECENTERING" or self.control is None
                        or message == "Position timeout"):
                    self.begin_stage("FAULT RECENTERING", self.center_goal)
                result = "FAULT: " + message + " — returning to customized right center before relaxing"
                if hasattr(self, "result_status"):
                    self.result_status.set(result)
                    self.result_label.config(fg="#b00020")
                self.status.set(result)
                if result != getattr(self, "_last_fault_report", None):
                    print(result, flush=True)
                    self._last_fault_report = result
                return
            except Exception as recovery_error:
                # The center controller is installed before its first CAN write.
                # Keep it pending; never drop a non-stalled arm at an arbitrary pose.
                self.phase = "FAULT RECENTERING"
                result = ("FAULT: center command retry pending; motors remain enabled until centered "
                          "or the user presses EMERGENCY RELAX — " + message +
                          "; " + str(recovery_error))
                self.status.set(result)
                if hasattr(self, "result_status"):
                    self.result_status.set(result)
                    self.result_label.config(fg="#b00020")
                if result != getattr(self, "_last_fault_report", None):
                    print(result, flush=True)
                    self._last_fault_report = result
                return
        result = "FAULT: " + message + "; right motors were already disabled"
        if hasattr(self, "result_status"):
            self.result_status.set(result)
            self.result_label.config(fg="#b00020")
        self.phase = "FAULT"
        self.control = None
        self.status.set(result)
        if result != getattr(self, "_last_fault_report", None):
            print(result, flush=True)
            self._last_fault_report = result

    def _camera_failure(self, message: str) -> None:
        if self._camera_fault_handled:
            return
        self._camera_fault_handled = True
        self._program_failure("FAILED: " + message)

    def centered(self, now=None):
        # Keep the original right-Cartesian loading workflow exactly.
        if abs(self.gripper_encoder() - RIGHT_GRIPPER_OPEN) > GRIPPER_TOLERANCE:
            self.status.set("Arm centered — opening gripper to −3°")
            return
        self.control = None
        self.phase = "WAITING FOR LOAD"
        self._refresh_buttons()
        self.status.set(
            "Centered with gripper open at −3° — make sure the cymbal is visible, "
            "load the drumstick, then press Continue"
        )

    def run(self):
        try:
            super().run()
        finally:
            self._cancel_planning()
            self.planner_executor.shutdown(wait=False, cancel_futures=True)
            self.receiver.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--hardware", action="store_true")
    parser.add_argument("--detection-socket", required=True)
    args = parser.parse_args()
    rclpy.init(args=[])
    try:
        App(args.hardware, args.detection_socket).run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
