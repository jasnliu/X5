"""Right-arm recorded playback followed by camera-guided Cartesian alignment."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import signal
import time
import tkinter as tk
from tkinter import filedialog, messagebox

import numpy as np
import rclpy

from camera_search.app import (
    App as CameraSearchApp,
    DETECTION_LOSS_TIMEOUT_SECONDS,
    MEASUREMENT_SECONDS,
)
from camera_search.control import CameraGoalControl, ENCODER_LSB_RAD
from camera_search.protocol import DetectionReceiver
from camera_search.vision import observation_from_message
from cartesian_goal.app import GRIPPER_COMMAND_INTERVAL, GRIPPER_TOLERANCE
from centering.motors import (
    MAX_RIGHT_JOINT7_FEEDBACK_HZ,
    MOTOR_RUNNING_STATE,
    RIGHT_GRIPPER_CLOSED,
    RIGHT_GRIPPER_OPEN,
    RIGHT_PLAYBACK_SPEED,
    RIGHT_STRIKE_DOWN_SPEED,
    RIGHT_STRIKE_RETURN_SPEED,
    SPEED,
)
from goal_motion.app import App as JointGoalApp
from goal_motion.control import STALL_ERROR, STALL_MOVEMENT, TOLERANCE
from safe_zone.geometry import MEMBERSHIP_BUFFER_M
from safe_zone.gripper_feedback import gripper_feedback_text

from .tempo import (
    MIN_BPM, MAX_BPM, ESTIMATED_PRACTICAL_MAX_BPM, parse_bpm, swing_events,
    CONTINUOUS_WAIT_PHASE, CONTINUOUS_OUT_PHASE, CONTINUOUS_RETURN_PHASE,
)
from .audio import AudioReceiver
from .sound_monitor import HiHatSoundMonitor
from .mit_strike import Joint7Session, StrikeSettings
from .hill import GuidedHillClimber
from .hihat import DEFAULT_ESP_PORT, HiHatController, MOTOR2_TARGET_DEGREES
from .hihat_sync_runtime import HiHatSyncRuntime
from .hihat_calibration_runtime import HiHatCalibrationRuntime
from .planner import solve_playback_coordinate
from .simulation import SimulatedMotors
from .strike import (
    STRIKE_INCREMENT_DEGREES,
    STRIKE_REACHED_TOLERANCE_RAD,
    STRIKE_STALL_SECONDS,
    STRIKE_TIMEOUT_SECONDS,
    STRIKE_START_DEGREES,
    StrikeControl,
    build_manual_strike_target,
    build_strike_plan,
)
from .trajectory import PlaybackFollower
from .smooth_recording import SmoothRecording
from .recording_cache import (
    load_cached_playback_trajectory as load_playback_trajectory,
    load_cached_smooth_recording as load_smooth_recording,
)
from .recording_preflight import RecordingPreflight
from . import dual_recording as dual_control
from .zone_markers import left_zone_markers
from .smooth_recording_worker import RecordingSession
from .playback_transport import PlaybackMotors
from centering.motors import Motors
from . import recording_only as recording_only_control
from . import hybrid_workflow as hybrid_control
from . import snare_workflow as snare_control
from .visual import (
    PlaybackVisualSample,
    infer_direction_order,
    inside_outer_goal,
    median_outer_result,
    normalized_center_distance,
)


ROOT = Path(__file__).resolve().parents[1]
RECORDINGS_DIR = ROOT / "recordings"
DEFAULT_RECORDING = RECORDINGS_DIR / "record3.json"
# Legacy simulation/preview playback; physical playback uses an independent
# 200 Hz paced_precise worker, leaving the rest of the GUI workflow unchanged.
PLAYBACK_COMMAND_INTERVAL = .02
SINGLE_RUN_GRIPPER_TIMEOUT = 3.0
STRIKE_SOUND_WAIT_SECONDS = 1.25
HYBRID_AUDIO_BACKLOG_LIMIT = 3.0
STRIKE_SOUND_EVENT_EARLY_TOLERANCE_SECONDS = 0.05
STRIKE_SOUND_EVENT_LATE_TOLERANCE_SECONDS = 0.10
STRIKE_SOUND_WAIT_PHASE = "STRIKE WAITING FOR SOUND"
STRIKE_BPM = 100.0
STRIKE_PERIOD_SECONDS = 60.0 / STRIKE_BPM
TEST_STRIKE_DEGREES = 10
STRIKE_J7_FEEDBACK_HZ = MAX_RIGHT_JOINT7_FEEDBACK_HZ
# The ride pattern is the usual triplet-based jazz swing: beats 1 and 3 are
# single hits, while beats 2 and 4 are followed by the third triplet partial.
# Each entry stores (display label, is a main quarter-note beat, delay to the
# next ride hit).  Starting at the final entry creates the requested pickup
# immediately before beat 1.
SWING_TRIPLET_SECONDS = STRIKE_PERIOD_SECONDS / 3.0
SWING_EVENTS = swing_events(STRIKE_BPM)  # Default only; each App owns its tempo.
SWING_PICKUP_INDEX = len(SWING_EVENTS) - 1
# Continuous swing hits are scheduled as target-arrival deadlines rather than
# as outbound-command times.  While J7 is returning, the live encoder distance
# predicts when another downstroke must begin.  Two control ticks of lead time
# cover command delivery and the fact that a short move may not instantly reach
# its configured velocity. A real rebound must span twice the strike
# controller's reached tolerance, so the following downstroke still has to
# travel through that full tolerance band instead of completing immediately.
CONTINUOUS_STRIKE_LEAD_SECONDS = 0.04
CONTINUOUS_MIN_REBOUND_RAD = 2.0 * STRIKE_REACHED_TOLERANCE_RAD
CENTER_RELAX_PHASE = "CENTER RELAX RECENTERING"
HIHAT_CALIBRATION_WAIT_PHASE = "WAITING FOR HI-HAT CALIBRATION"
HARDWARE_TEST_READY_PHASE = "HARDWARE TEST READY FOR MANUAL STRIKE"
HARDWARE_TEST_OUT_PHASE = "HARDWARE TEST J7 MIT FREEFALL"
HARDWARE_TEST_RETURN_PHASE = "HARDWARE TEST J7 MIT REBOUND"
HARDWARE_TEST_HOLD_PHASE = "HARDWARE TEST SETTLING MIT HOLD"
HARDWARE_TEST_RESTORE_PHASE = "HARDWARE TEST LEAVING MIT SESSION"
HARDWARE_TEST_HOLD_FAULT_PHASE = "HARDWARE TEST POWERED HOLD FAULT"
HARDWARE_TEST_WORKFLOW_FAULT_PHASE = "HARDWARE TEST ISOLATED FAULT"
HARDWARE_TEST_FAULT_HOLD_INTERVAL = 0.1
HARDWARE_TEST_DEFAULT_DROP_DEGREES = 5.0
HARDWARE_TEST_OTHER_JOINT_DRIFT_RAD = math.radians(3.0)
HARDWARE_TEST_ACTIVE_PHASES = {
    HARDWARE_TEST_OUT_PHASE,
    HARDWARE_TEST_RETURN_PHASE,
    HARDWARE_TEST_HOLD_PHASE,
    HARDWARE_TEST_RESTORE_PHASE,
}
HARDWARE_TEST_BLOCKED_PHASES = (
    HARDWARE_TEST_ACTIVE_PHASES | {
        HARDWARE_TEST_HOLD_FAULT_PHASE,
        HARDWARE_TEST_WORKFLOW_FAULT_PHASE,
    }
)


class HiHatCalibrationHold(CameraGoalControl):
    """Stationary wait, not a 30-second movement; retain all stall history."""

    def update(self, actual, now):
        actual = np.asarray(actual, dtype=float)
        if (actual.shape == (7,) and np.isfinite(actual).all()
                and np.max(np.abs(self.desired-actual)) <= TOLERANCE + ENCODER_LSB_RAD):
            # A reached stationary hold may outlast the normal movement
            # deadline while ST7 warms/calibrates. Never extend a failed move,
            # discard history or suppress the inherited stall/feedback checks.
            self.started = now
        return super().update(actual, now)


class App(CameraSearchApp):
    CAMERA_REQUIRED_PHASES = {
        "CLOSING GRIPPER",
        "MOVING TO RECORDING START",
        "RECORDING PLAYBACK",
        "SETTLING RECORDING END",
        "CHECKING PLAYBACK END",
        HIHAT_CALIBRATION_WAIT_PHASE,
        "HILL MEASURING ANCHOR",
        "HILL MEASURING CANDIDATE",
        "HILL PLANNING",
        "HILL WAITING FOR CAMERA",
        "HILL MOVING CANDIDATE",
        "HILL RETURNING TO ANCHOR",
    }
    RETURN_PHASES = CameraSearchApp.RETURN_PHASES | {
        CENTER_RELAX_PHASE, dual_control.CENTERING,
    }

    def __init__(self, hardware: bool, detection_socket: str, audio_socket: str,
                 recording_path: str | Path | None = None,
                 esp_port: str | Path | None = DEFAULT_ESP_PORT,
                 test_mode: bool = False,
                 hardware_test_mode: bool = False,
                 hardware_test_settings: StrikeSettings | None = None,
                 recording_only: bool = False, auto_run: bool = False,
                 hihat_audio_socket: str | None = None,
                 left_recording_path: str | Path | None = dual_control.DEFAULT_LEFT_RECORDING):
        self._strike_bpm = STRIKE_BPM
        self.dual = None
        self.recording_only = bool(recording_only)
        self.recording_only_result = None
        if auto_run and not recording_only:
            raise ValueError('Auto-run requires recording-only mode')
        if recording_only and (not hardware or test_mode):
            raise ValueError('Recording-only requires physical hardware mode')
        self.recording_only_directory = recording_only_control.evidence_directory() if recording_only else None
        self.test_mode = bool(test_mode)
        self.hardware_test_mode = bool(hardware_test_mode)
        self.hardware_test_settings = hardware_test_settings or StrikeSettings()
        if self.test_mode and self.hardware_test_mode:
            raise ValueError("Pure simulation and hardware test modes are mutually exclusive")
        # Test mode must never construct Motors/SocketCAN. Build the inherited
        # UI offline, then attach only the in-memory bus used by the shared
        # playback state machine and RViz joint-state publisher.
        real_hardware = bool(hardware and not self.test_mode)
        self.single_run_mode = bool(real_hardware and not hardware_test_mode and not recording_only)
        self.single_run_gripper_deadline = None
        self.hybrid_enabled = bool(real_hardware and not hardware_test_mode and not recording_only)
        self.hybrid_session = None
        self.hybrid_stopping = self.hybrid_restoring = False
        self.hybrid_tuning = hybrid_control.load_tuning() if self.hybrid_enabled else None
        if self.hybrid_enabled:
            print('HARDWARE: automatic hi-hat calibration 90°..115° in 5° steps; '
                  'one RUN button → center + close gripper → recording → '
                  'wait for hi-hat calibration → '
                  'experiment hybrid J7 search → first-hit depth +0.5° → '
                  '20–120 BPM (default 100) hybrid swing plus 1–2 spaced random eighth-note 11° left snares per measure; ride v2 controls search; '
                  'hi-hat v1 continuously tunes only hi-hat timing; ride timing unchanged', flush=True)
        super().__init__(real_hardware, detection_socket)
        if real_hardware and isinstance(self.bus, Motors):
            self.bus = PlaybackMotors.adopt(self.bus, self.center_goal,
                strict_center_relax=recording_only, directory=self.recording_only_directory,
                left_hold=True)
            self.bus.require_center_before_relax = self.single_run_mode
        if self.test_mode:
            self.bus = SimulatedMotors(self.center_goal, RIGHT_GRIPPER_OPEN)
            self.hardware = True
            self.q = self.bus.positions()
        self.audio_receiver = AudioReceiver(audio_socket)
        self.sound_fault_detail = None
        self.hihat = (
            None if self.test_mode or self.hardware_test_mode or self.recording_only
            else HiHatController(esp_port)
        )
        self.hihat_sync = HiHatSyncRuntime(self) if self.hybrid_enabled else None
        self.hihat_calibration = None
        self.hihat_fault_detail = None
        if real_hardware and self.hihat is not None and not self.hihat.connect():
            self.hihat_fault_detail = self.hihat.detail
        RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
        self.playback_trajectory = None
        self.recording_preflight = None
        self.smooth_playback_session = None
        self.smooth_playback_cleanup_error = None
        self.playback_active = False
        self.playback_started_at = None
        self.playback_last_command = 0.0
        self.playback_follower = None
        self.playback_visual_samples = []
        self.playback_frame_ids = set()
        self.playback_fixed_joint2 = None
        self.direction_prior_detail = "standard direction order"
        self.strike_active = False
        self.strike_plan = None
        self.strike_index = 0
        self.strike_speed_fast = False
        self.strike_selected_speed = None
        self.strike_feedback_fast = False
        self.playback_speed_fast = False
        self.strike_attempt_motion_seen = False
        self.strike_hit_pending = None
        self.strike_attempt_started_at = None
        self.strike_attempt_returned_at = None
        self.strike_sound_deadline = None
        self.hardware_test_session = None
        self.hardware_test_completed = 0
        self.hardware_test_anchor = None
        self.hardware_test_target = None
        self.hardware_test_degrees_value = None
        self.hardware_test_strike_count = 0
        self.hardware_test_sound_count = 0
        self.hardware_test_mit_active = False
        self.hardware_test_last_mit_command = 0.0
        self.hardware_test_csp_enabled_at = None
        self.hardware_test_restore_deadline = None
        self.hardware_test_last_enable_retry = 0.0
        self.hardware_test_hold_fault_message = None
        self.hardware_test_hold_delivery_error = None
        self.hardware_test_workflow_fault_message = None
        self.hardware_test_workflow_hold_joints = None
        self.hardware_test_isolated_motor = None
        self.hardware_test_last_workflow_hold = 0.0
        self.continuous_strike_active = False
        self.continuous_strike_target = None
        self.continuous_strike_degrees = None
        self.continuous_strike_count = 0
        self.continuous_last_beat_at = None
        self.continuous_next_beat_at = None
        self.continuous_swing_index = SWING_PICKUP_INDEX
        self.continuous_first_swing_hit = True
        self.continuous_current_swing_label = None
        self.continuous_current_main_beat = False
        self.continuous_current_interval_seconds = None
        self.continuous_current_event_at = None
        self.continuous_hihat_pending = False
        self.continuous_stop_requested = False
        self.continuous_j7_lowest_encoder_rad = None
        self.continuous_j7_measurement_anchor_rad = None
        self.continuous_j7_measurement_strike_count = None
        self.continuous_j7_measurement_label = None
        self.continuous_j7_measurement_reached = False

        if self.test_mode:
            self.root.title("RIGHT arm: Recorded playback — PURE SIMULATION")
            self.header.config(
                text=("TEST: recorded motion → assume endpoint is correct → "
                      f"fixed {TEST_STRIKE_DEGREES}° swing ride at 100 BPM")
            )
        elif self.hardware_test_mode:
            self.root.title("RIGHT arm: Playback + J7 MIT — HARDWARE TEST")
            self.header.config(
                text=("HARDWARE TEST: center/open → load + Continue → recording → "
                      "one pink-zone detection → editable J7 MIT freefall + rebound")
            )
        else:
            self.root.title("RIGHT arm: Recorded camera Cartesian alignment" +
                            ("" if real_hardware else " — OFFLINE PREVIEW"))
            self.header.config(
                text=(("ONE RUN: center + close → recording → " if self.single_run_mode else
                       "RIGHT arm: recorded motion → ") + "camera alignment → ST7 hit "
                      "HYBRID search (5°, 6°, 7°…) → hybrid swing + ESP32 hi-hat at 100 BPM")
            )
        self.loading_status = tk.StringVar()
        self.loading_label = tk.Label(
            self.root,
            textvariable=self.loading_status,
            wraplength=650,
            justify="center",
            fg="#8a5a00",
            font=("Sans", 12, "bold"),
        )
        self._recording_loading_text = None
        self._camera_loading_text = (None if self.test_mode else
            "LOADING CAMERA\nStarting the processed camera and YOLO pose model")
        self._refresh_loading_label()
        self.start_button.config(text="START: CENTER + OPEN GRIPPER TO −3°")
        if self.test_mode:
            continue_text = (
                f"CONTINUE: CLOSE → RECORDING → FIXED {TEST_STRIKE_DEGREES}° "
                "SWING AT 100 BPM"
            )
        elif self.hardware_test_mode:
            continue_text = (
                "CONTINUE: CLOSE → RECORDING → PINK ZONE → MANUAL MIT STRIKES"
            )
        else:
            continue_text = (
                "CONTINUE: CLOSE → RECORDING → ALIGN → HYBRID HIT → "
                "FIRST DEPTH HYBRID SWING + HI-HAT AT 100 BPM"
            )
        self.continue_button.config(text=continue_text)
        self.hardware_test_button = None
        self.hardware_test_entry = None
        self.hardware_test_sound_status = None
        if self.hardware_test_mode:
            self.hardware_test_frame = tk.Frame(self.root)
            self.hardware_test_frame.columnconfigure(1, weight=1)
            tk.Label(
                self.hardware_test_frame,
                text="J7 maximum drop goal (degrees):",
                font=("Sans", 12, "bold"),
            ).grid(row=0, column=0, sticky="w", padx=(0, 8))
            self.hardware_test_degrees = tk.StringVar(value="")
            self.hardware_test_entry = tk.Entry(
                self.hardware_test_frame,
                textvariable=self.hardware_test_degrees,
                font=("Sans", 13),
                justify="center",
                state="disabled",
            )
            self.hardware_test_entry.grid(row=0, column=1, sticky="ew")
            self.hardware_test_button = tk.Button(
                self.hardware_test_frame,
                text="MIT TEST: REACH PINK ZONE FIRST",
                font=("Sans", 14, "bold"),
                bg="#d98200",
                fg="white",
                state="disabled",
                command=self.hardware_test_strike_once,
            )
            self.hardware_test_button.grid(
                row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0), ipady=5
            )
            self.hardware_test_frame.pack(
                fill="x", padx=20, pady=8, after=self.continue_button
            )
            self.hardware_test_degrees.trace_add(
                "write", lambda *_: self._refresh_hardware_test_controls()
            )
        self.emergency_button.destroy()
        self.relax_button_frame = tk.Frame(self.root)
        self.relax_button_frame.columnconfigure(0, weight=1)
        self.relax_button_frame.columnconfigure(1, weight=1)
        self.emergency_button = tk.Button(
            self.relax_button_frame,
            text="EMERGENCY RELAX",
            font=("Sans", 14, "bold"),
            bg="#bd2020",
            fg="white",
            command=self.relax,
        )
        self.emergency_button.grid(row=0, column=0, sticky="nsew", padx=(0, 4), ipady=8)
        self.center_relax_button = tk.Button(
            self.relax_button_frame,
            text="CENTER + RELAX",
            font=("Sans", 14, "bold"),
            bg="#8a5a00",
            fg="white",
            state="disabled",
            command=self.center_relax,
        )
        self.center_relax_button.grid(row=0, column=1, sticky="nsew", padx=(4, 0), ipady=8)
        self.relax_button_frame.pack(
            fill="x", padx=20, pady=8,
            after=(getattr(self, "hardware_test_frame", None) or self.continue_button),
        )
        # The inherited panel places a large blue instruction block directly
        # below the title. The playback panel keeps that space uncluttered.
        self.isolation_label.destroy()

        self.recording_status = tk.StringVar(value="Recording: none selected")
        self.recording_label = tk.Label(
            self.root, textvariable=self.recording_status, wraplength=650,
            justify="left", fg="#8a5a00", font=("Sans", 11, "bold"),
        )
        self.recording_label.pack(fill="x", padx=20, pady=4, before=self.start_button)
        self.recording_button = tk.Button(
            self.root, text="SELECT RIGHT-ARM RECORDING…",
            command=self.choose_recording,
        )
        self.recording_button.pack(fill="x", padx=20, pady=4, before=self.start_button)
        self._create_tempo_controls()
        self.sound_status = tk.StringVar(value=(
            "Sound: ignored in test mode — ST7/TONOR not started"
            if self.test_mode else
            "Sound: starting ST7 and TONOR"
        ))
        self.sound_label = tk.Label(
            self.root, textvariable=self.sound_status, font=("Sans", 12, "bold"),
            wraplength=650, fg="#8a5a00",
        )
        self.sound_label.pack(fill="x", padx=20, pady=5, before=self.start_button)
        self.hihat_sound_monitor = HiHatSoundMonitor(
            self.root, hihat_audio_socket, before=self.start_button, disabled=self.test_mode)
        if self.hybrid_enabled:
            self.hihat_calibration = HiHatCalibrationRuntime(self)
            if self.hihat.state in ('starting', 'ready'):
                self.hihat.start_service()
            self.hihat_sound_monitor.control_required = True
        self.hihat_sound_monitor.on_messages = self._hihat_sound_messages
        if self.test_mode:
            hihat_text = "Hi-hat ESP32: ignored in test mode — serial port not opened"
        elif self.hardware_test_mode or self.recording_only:
            hihat_text = (
                "Hi-hat ESP32: not used in hardware test — serial port not opened"
            )
        elif real_hardware:
            hihat_text = "Hi-hat ESP32: " + self.hihat.detail
        else:
            hihat_text = "Hi-hat ESP32: offline preview — serial port not opened"
        self.hihat_status = tk.StringVar(value=hihat_text)
        self.hihat_label = tk.Label(
            self.root, textvariable=self.hihat_status, font=("Sans", 12, "bold"),
            wraplength=650, fg="#8a5a00",
        )
        self.hihat_label.pack(fill="x", padx=20, pady=5, before=self.start_button)
        self.result_status.set(
            "Result: select a recording, then press RUN" if self.single_run_mode else
            "Result: select a recording, then start"
        )
        if self.test_mode:
            self.status.set(
                "PURE SIMULATION — no CAN or physical hardware; select a recording "
                "(camera, sound, and hi-hat are bypassed)"
            )
        else:
            if not real_hardware:
                initial_status = "Offline preview — select a recording to preflight; no CAN"
            elif self.hardware_test_mode:
                initial_status = (
                    "HARDWARE TEST — select a recording; waiting for camera and "
                    "fresh encoders (ST7 is informational; ESP32 hi-hat is not used)"
                )
            else:
                initial_status = (
                    "Select a right-arm recording; waiting for camera, ST7/TONOR, "
                    "ESP32 hi-hat, and encoders"
                )
            self.status.set(initial_status)
        if not self.hardware_test_mode and not self.recording_only:
            self.dual = dual_control.DualRecording(self)
            self.root.title('BOTH arms: Left → Right recorded beat playback' +
                            (' — PURE SIMULATION' if self.test_mode else '' if real_hardware else ' — OFFLINE PREVIEW'))
        if recording_path is not None:
            self.load_recording(recording_path, show_dialog=False)
        if self.dual is not None and left_recording_path is not None:
            self.dual.load(left_recording_path)
        if self.hardware_test_mode:
            # The ESP32 is deliberately absent in manual hardware test mode.
            # Camera alignment and informational ST7 logging remain visible.
            self.hihat_label.pack_forget()
        if self.single_run_mode:
            # Keep the inherited widget object for base-class config() calls,
            # but remove the old Start control from the visible panel. Reuse
            # Continue's location for the one complete, user-authorized run.
            self.start_button.pack_forget()
            self.continue_button.config(
                text='RUN: CENTER BOTH → LEFT → RIGHT → HYBRID SWING',
                command=self.start,
            )
            self.emergency_button.config(text='EMERGENCY RELAX (NO CENTER)', command=self.emergency_relax)
            self.root.bind('<Escape>', lambda _: self.emergency_relax())
            self.close_requested = False
            self.root.protocol('WM_DELETE_WINDOW', self.request_safe_close)
            self.root.report_callback_exception = lambda kind, value, tb: self.fail(
                'GUI callback failed: '+str(value))
            # Queue stop handling at a callback boundary, never reenter a
            # half-executed motor handoff/setup from Python's signal handler.
            signal.signal(signal.SIGINT, lambda *_: self.root.after(0, self.request_safe_close))
            signal.signal(signal.SIGTERM, lambda *_: self.root.after(0, self.request_safe_close))
        self._refresh_buttons()
        self.root.after(20, self.sound_tick)
        if not self.hardware_test_mode:
            self.root.after(20, self.hihat_tick)
        if self.recording_only:
            self.header.config(text='RECORDING ONLY: center/open/load/close → playback → verified center → relax')
            self.continue_button.config(text='CONTINUE: CLOSE → RECORDING → CENTER → RELAX')
            self.emergency_button.config(text='STOP: CENTER THEN RELAX')
            if getattr(self, 'hardware_test_frame', None) is not None:
                self.hardware_test_frame.pack_forget()
            print('NO-STRIKE RECORDING-ONLY MODE; evidence: '+str(self.recording_only_directory), flush=True)
            if auto_run:
                self.root.after(100, lambda: recording_only_control.auto_tick(self))
        self.header.config(text=self.header.cget('text') + (
            '\nBOTH center; LEFT then RIGHT recording; 1–2 spaced random eighth-note 11° snares per 4-quarter-note measure'
            if self.dual is not None else
            '\nBOTH arms center; LEFT holds center with its closed gripper'))

        if hasattr(self, 'tempo_header_template'):
            self.tempo_header_template = self.header.cget('text')

    @property
    def strike_bpm(self):
        return getattr(self, '_strike_bpm', STRIKE_BPM)

    @property
    def swing_events(self):
        return swing_events(self.strike_bpm)

    def _create_tempo_controls(self):
        # No tempo control in one-shot MIT tests or no-strike playback.
        if self.hardware_test_mode or self.recording_only:
            return
        self.tempo_header_template = self.header.cget('text')
        self.tempo_continue_template = self.continue_button.cget('text')
        self.tempo_frame = tk.Frame(self.root)
        self.tempo_frame.columnconfigure(2, weight=1)
        tk.Label(self.tempo_frame, text='Beat BPM:', font=('Sans', 12, 'bold')).grid(
            row=0, column=0, sticky='w', padx=(0, 8))
        self.bpm_text = tk.StringVar(value=f'{self.strike_bpm:g}')
        self.bpm_entry = tk.Entry(self.tempo_frame, textvariable=self.bpm_text,
                                  width=8, justify='center', font=('Sans', 13))
        self.bpm_entry.grid(row=0, column=1, sticky='w')
        self.tempo_hint = tk.Label(self.tempo_frame, wraplength=480, justify='left')
        self.tempo_hint.grid(row=0, column=2, sticky='w', padx=(10, 0))
        self.tempo_frame.pack(fill='x', padx=20, pady=4, before=self.start_button)
        self.bpm_text.trace_add('write', lambda *_: self._refresh_buttons())

    def _tempo_editable(self):
        return (self.phase in ('READY', 'RELAXED')
                and not (self.bus and self.bus.active))

    def _apply_selected_tempo(self):
        value = getattr(self, 'bpm_text', None)
        self._strike_bpm = parse_bpm(value.get() if value is not None else self.strike_bpm)

    def _refresh_tempo_controls(self):
        if getattr(self, 'bpm_entry', None) is None:
            return True
        editable = self._tempo_editable()
        self.bpm_entry.config(state='normal' if editable else 'disabled')
        if not editable:
            self.tempo_hint.config(text=f'{self.strike_bpm:g} BPM locked for this run; '
                                   'Center + Relax before changing.', fg='#555555')
            return True
        try:
            self._apply_selected_tempo()
            snare_control.check_tempo(self)
        except ValueError as exc:
            self.tempo_hint.config(text=str(exc), fg='#b00020')
            return False
        maximum = 120. if getattr(getattr(self, 'dual', None), 'snare_template', None) is not None else MAX_BPM
        self.tempo_hint.config(
            text=f'{MIN_BPM:g}–{maximum:g} BPM; estimated practical upper '
                 f'~{ESTIMATED_PRACTICAL_MAX_BPM:g}. Set before Start/RUN.',
            fg='#8a5a00' if self.strike_bpm > ESTIMATED_PRACTICAL_MAX_BPM else '#555555')
        tempo = f'{self.strike_bpm:g} BPM'
        self.header.config(text=self.tempo_header_template.replace('100 BPM', tempo))
        if not getattr(self, 'single_run_mode', False):
            self.continue_button.config(text=self.tempo_continue_template.replace('100 BPM', tempo))
        return True

    def choose_recording(self) -> None:
        if getattr(self, 'recording_preflight', None) is not None:
            return
        calibration = getattr(self, 'hihat_calibration', None)
        if calibration is not None and calibration.busy:
            self.status.set('Wait for hi-hat calibration before changing recordings')
            return  # Keep recording changes separate from calibration motion.
        if self.bus and self.bus.active:
            self.status.set("Cannot change recordings while the right arm is powered")
            return
        selected = filedialog.askopenfilename(
            initialdir=RECORDINGS_DIR,
            filetypes=[("OpenArmX right motion JSON", "*.json")],
        )
        if selected:
            self.load_recording(selected, show_dialog=True)

    def _refresh_loading_label(self) -> None:
        text = self._recording_loading_text or self._camera_loading_text
        if text:
            self.loading_status.set(text)
            if not self.loading_label.winfo_manager():
                self.loading_label.pack(
                    fill="x", padx=20, pady=(0, 7), after=self.header
                )
        else:
            self.loading_status.set("")
            if self.loading_label.winfo_manager():
                self.loading_label.pack_forget()

    def _show_recording_loading(self, source_name: str, detail: str) -> None:
        self._recording_loading_text = (
            f"LOADING RECORDING: {source_name}\n{detail}"
        )
        self._refresh_loading_label()
        # Only the GUI thread calls this, including queued background progress.
        self.root.update_idletasks()

    def _hide_recording_loading(self) -> None:
        self._recording_loading_text = None
        self._refresh_loading_label()
        self.root.update_idletasks()

    def _sync_camera_loading_label(self) -> None:
        if self.receiver.state == "starting":
            detail = self.receiver.detail.strip() or "Starting the processed camera"
            self._camera_loading_text = "LOADING CAMERA\n" + detail
        else:
            self._camera_loading_text = None
        self._refresh_loading_label()

    def load_recording(self, path, show_dialog: bool = True) -> bool:
        if (getattr(self, 'recording_preflight', None) is not None
                or getattr(self, 'close_requested', False)):
            return False
        calibration = getattr(self, 'hihat_calibration', None)
        if calibration is not None and calibration.busy:
            # A modal picker can remain open while Tk starts calibration.
            # Recheck before accepting a recording change during motion.
            self.status.set('Hi-hat calibration is moving; load the recording after it finishes')
            return False
        if self.bus and self.bus.active:
            self.status.set("Cannot load a recording while the right arm is powered")
            return False
        source_name = Path(path).expanduser().name or str(path)
        self.status.set("Preflighting recorded path and center returns — motors remain disabled")
        self._show_recording_loading(
            source_name, "Preparing recording validation"
        )
        if getattr(self, 'hybrid_enabled', False):
            # Heavy geometry/retiming must not suspend ESP32 heartbeats, status
            # queries, audio polling, or arm encoder observation on the Tk loop.
            # Clear the old selection so RUN cannot race a replacement load.
            self.playback_trajectory = None
            self._refresh_buttons()
            try:
                self.recording_preflight = RecordingPreflight(
                    path, self.model, self.zone, self.lower.copy(), self.upper.copy(),
                    self.center_goal.copy(), self.ik.speed,
                    playback_speed=RIGHT_PLAYBACK_SPEED)
            except Exception as exc:
                self._hide_recording_loading()
                self._recording_load_failed(str(exc), show_dialog=False)
                return False
            self._refresh_buttons()
            self.root.after(20, lambda: self._poll_recording_preflight(source_name))
            return True  # Accepted for validation, not yet ready to RUN.
        try:
            loader = (load_smooth_recording
                      if getattr(self, "hardware", False) and not self.test_mode
                      else load_playback_trajectory)
            trajectory = loader(
                path, self.model, self.zone, self.lower, self.upper,
                self.center_goal, self.ik.speed,
                playback_speed=RIGHT_PLAYBACK_SPEED,
                progress=lambda detail: self._show_recording_loading(
                    source_name, detail
                ),
            )
        except ValueError as exc:
            self.playback_trajectory = None
            self.recording_status.set("Recording rejected: " + str(exc))
            self.recording_label.config(fg="#b00020")
            self.status.set("ERROR: recording preflight failed: " + str(exc))
            if show_dialog:
                messagebox.showerror("Recording rejected", str(exc))
            self._refresh_buttons()
            return False
        finally:
            self._hide_recording_loading()
        self._install_recording(trajectory)
        return True

    def _poll_recording_preflight(self, source_name):
        job = getattr(self, 'recording_preflight', None)
        if job is None:
            return
        if getattr(self, 'close_requested', False):
            self._cancel_recording_preflight()
            return
        for kind, value in job.poll():
            if kind == 'progress':
                self._show_recording_loading(source_name, value)
            else:
                self.recording_preflight = None
                self._hide_recording_loading()
                if kind == 'error':
                    # A modal error dialog must not delay motor supervision.
                    self._recording_load_failed(value, show_dialog=False)
                elif self.bus and self.bus.active:
                    self._recording_load_failed('Arm became active during validation', show_dialog=False)
                else:
                    self._install_recording(value)
                return
        self.root.after(20, lambda: self._poll_recording_preflight(source_name))

    def _cancel_recording_preflight(self):
        if getattr(self, "dual", None) is not None:
            self.dual.cancel_load()
        job = getattr(self, 'recording_preflight', None)
        if job is not None:
            job.cancel()
            self.recording_preflight = None

    def _recording_load_failed(self, detail, show_dialog=False):
        self.playback_trajectory = None
        self.recording_status.set('Recording rejected: ' + detail)
        self.recording_label.config(fg='#b00020')
        self.status.set('ERROR: recording preflight failed: ' + detail)
        print('RECORDING PREFLIGHT FAILED: ' + detail, flush=True)
        if show_dialog:
            messagebox.showerror('Recording rejected', detail)
        self._refresh_buttons()

    def _install_recording(self, trajectory):
        self.playback_trajectory = trajectory
        self.goal = trajectory.first_joints
        self.target_point = trajectory.tcp_positions[0].copy()
        self.target_offset = self.target_point - self.ik.origin_tcp
        self.target_inside = True
        scale = trajectory.time_scale
        method = ("paced_precise 200 Hz; endpoint-exact; temporary gains restored"
                  if isinstance(trajectory, SmoothRecording) else
                  f"smooth 50 Hz; 0.8 rad/s playback limit; velocity-only scale ×{scale:.3f}")
        self.recording_status.set(
            f"Recording: {trajectory.source.name} • {trajectory.sample_count} samples • "
            f"{trajectory.original_duration_s:.2f} s source → "
            f"{trajectory.duration_s:.2f} s playback "
            f"({method})"
        )
        self.recording_label.config(fg="#087f23")
        if getattr(self, 'recording_only', False):
            ready_status = 'Recording-only preflight passed; waiting for fresh camera frames and encoders; no strikes'
        elif self.test_mode:
            ready_status = "Recording preflight passed; pure simulation is ready"
        elif self.hardware_test_mode:
            ready_status = (
                "Recording preflight passed; waiting for camera and fresh encoders "
                "(ST7 is informational; ESP32 is not used)"
            )
        else:
            ready_status = (
                "Recording preflight passed; waiting for camera, ST7/TONOR, "
                "ESP32 hi-hat, and fresh encoders"
            )
        self.status.set(ready_status)
        self.result_status.set("Result: recording ready; sequence not started")
        self.result_label.config(fg="navy")
        self._refresh_buttons()

    def _refresh_buttons(self) -> None:
        if getattr(self, "dual", None) is not None:
            self.dual.refresh()
        selector = getattr(self, "recording_button", None)
        if selector is not None:
            selectable = not (self.bus and self.bus.active)
            selectable = selectable and getattr(self, 'recording_preflight', None) is None
            calibration = getattr(self, 'hihat_calibration', None)
            selectable = selectable and not (calibration is not None and calibration.busy)
            selector.config(state="normal" if selectable else "disabled")
        tempo_ready = self._refresh_tempo_controls()
        self._refresh_hardware_test_controls()
        continuous = getattr(self, "continuous_strike_active", False)
        stop_requested = getattr(self, "continuous_stop_requested", False)
        self.center_relax_button.config(
            text=(f"STOP {self.strike_bpm:g} BPM STRIKING: CENTER + RELAX"
                  if continuous else "CENTER + RELAX")
        )
        if not self.hardware or not self.bus:
            self.start_button.config(state="disabled")
            self.continue_button.config(state="disabled")
            self.center_relax_button.config(state="disabled")
            return
        bus_fresh = self.bus.fresh()
        camera_ready = self._camera_ready_for_start()
        sound_ready = self._sound_ready_for_start()
        hihat_ready = self._hihat_ready_for_start()
        recording_ready = (self.playback_trajectory is not None
                           and (getattr(self, "dual", None) is None or self.dual.ready))
        ready_to_run = (not getattr(self, 'close_requested', False)
                        and not getattr(self, 'emergency_latched', False)
                        and self.phase in ("READY", "RELAXED") and bus_fresh
                        and camera_ready and sound_ready and hihat_ready and recording_ready and tempo_ready)
        self.start_button.config(
            state="normal" if ready_to_run and not getattr(self, 'single_run_mode', False) else "disabled"
        )
        self.continue_button.config(
            state="normal" if (ready_to_run if getattr(self, 'single_run_mode', False) else (
                self.phase == "WAITING FOR LOAD" and bus_fresh
                and camera_ready and sound_ready and hihat_ready
            )) else "disabled"
        )
        if continuous:
            center_relax_available = (
                self.bus.active and bus_fresh and self.setup is None
                and not stop_requested
            )
        else:
            center_relax_available = (
                self.bus.active and bus_fresh and self.setup is None
                and self.phase not in self.RETURN_PHASES
                and self.phase not in HARDWARE_TEST_BLOCKED_PHASES
                and self.phase not in {
                    "FAULT RECENTERING", "ZONE RECENTERING", "RELAXING",
                }
            )
        self.center_relax_button.config(
            state="normal" if (center_relax_available and self.phase not in {
                hybrid_control.FINISHING, hybrid_control.RESTORING, hybrid_control.FAULT
            }) else "disabled"
        )

    def _hardware_test_entered_degrees(self) -> float | None:
        value = getattr(self, "hardware_test_degrees", None)
        if value is None:
            return None
        try:
            amount = float(value.get().strip())
        except (TypeError, ValueError, tk.TclError):
            return None
        if not math.isfinite(amount):
            return None
        return amount

    @staticmethod
    def _format_hardware_test_degrees(amount: float | None) -> str:
        return "?" if amount is None else f"{float(amount):.6g}"

    def _refresh_hardware_test_controls(self, status=None) -> None:
        """Enable the editable one-strike control only at its safe checkpoint."""
        button = getattr(self, "hardware_test_button", None)
        if button is None:
            return
        entry = self.hardware_test_entry
        phase = getattr(self, "phase", "")
        button_state = "disabled"
        entry_state = "disabled"
        if phase == HARDWARE_TEST_READY_PHASE:
            amount = self._hardware_test_entered_degrees()
            text = (
                f"FREEFALL {self._format_hardware_test_degrees(amount)}° + REBOUND"
                if amount is not None else
                "ENTER FREEFALL AMOUNT"
            )
            ready = (
                self.hardware and self.bus and self.bus.active and self.bus.fresh()
                and self.setup is None and self._hardware_test_session_ready(status)
            )
            entry_state = "normal" if ready else "disabled"
            button_state = "normal" if ready and amount is not None else "disabled"
        elif phase == HARDWARE_TEST_OUT_PHASE:
            text = (
                f"MIT FREEFALL {self._format_hardware_test_degrees(self.hardware_test_degrees_value)}° DOWN…"
            )
        elif phase == HARDWARE_TEST_RETURN_PHASE:
            text = (
                f"SMOOTH CATCH / RETURN — {self._format_hardware_test_degrees(self.hardware_test_degrees_value)}° GOAL…"
            )
        elif phase == HARDWARE_TEST_HOLD_PHASE:
            text = "MIT HOLD — WAITING FOR ENCODER POSITION TO STABILIZE…"
        elif phase == HARDWARE_TEST_RESTORE_PHASE:
            text = "LEAVING MIT SESSION — RESTORING POSITION MODE…"
        elif phase == HARDWARE_TEST_HOLD_FAULT_PHASE:
            text = "FAULT — POWERED HOLD ACTIVE; STRIKES LOCKED"
        elif phase == HARDWARE_TEST_WORKFLOW_FAULT_PHASE:
            text = (
                "FAULT — AFFECTED JOINT ISOLATED; STRIKES LOCKED"
                if getattr(self, "hardware_test_isolated_motor", None) is not None
                else "FAULT — HEALTHY DRIVES HOLDING; STRIKES LOCKED"
            )
        else:
            text = "MIT TEST: REACH PINK ZONE FIRST"
        entry.config(state=entry_state)
        button.config(text=text, state=button_state)

    def _hardware_test_session_ready(self, status=None) -> bool:
        session = getattr(self, "hardware_test_session", None)
        if session is None:
            return False
        status = session.status if status is None else status
        # UI heartbeat is not the motor's 20 ms control deadline. The worker
        # revalidates fresh feedback and encoder settling when consuming a press.
        return bool(status.phase == "hold" and status.ready and status.sample is not None and not status.error
                    and 0 <= time.monotonic() - status.sample.at
                    <= session.controller.settings.status_timeout)

    def hardware_test_strike_once(self) -> None:
        """Submit one validated drop to the worker; no GUI-timed motion."""
        if not self.hardware_test_mode or self.phase != HARDWARE_TEST_READY_PHASE:
            return
        if (not self.bus or not self.bus.active or not self.bus.fresh()
                or self.setup is not None or not self._hardware_test_session_ready()):
            self.status.set("Waiting for fresh feedback and a stationary MIT anchor")
            return
        amount = self._hardware_test_entered_degrees()
        if amount is None:
            self.status.set("Enter a finite positive J7 maximum drop goal")
            return
        # This reference stays fixed throughout the session. A correcting or
        # overshot pose must never become the next strike's new starting point.
        anchor = np.asarray(self.hardware_test_anchor, dtype=float)
        try:
            target = build_manual_strike_target(
                self.hill_ik.model, self.hill_ik.zone, self.lower, self.upper,
                anchor.copy(), amount,
            )
            self.hardware_test_session.strike(math.radians(amount))
        except (ValueError, RuntimeError) as exc:
            self.status.set("MIT strike rejected: " + str(exc))
            return
        self.hardware_test_target = target
        self.hardware_test_degrees_value = amount
        self.hardware_test_strike_count += 1
        self.phase = HARDWARE_TEST_OUT_PHASE
        self.status.set(
            f"MANUAL MIT STRIKE {self.hardware_test_strike_count}: {amount:g}° "
            "maximum-drop goal; damped freefall, predictive catch, continuous withdrawal"
        )
        self._refresh_buttons()

    def _stop_hardware_test_session(self) -> None:
        session = getattr(self, "hardware_test_session", None)
        if session is not None:
            session.stop()  # Join before ANY GUI-owned J7 write/disable.
            self.hardware_test_session = None

    def _advance_hardware_test_mit(self, now: float) -> None:
        """Display worker state only. Tk scheduling never triggers the catch."""
        session = getattr(self, "hardware_test_session", None)
        if session is None:
            if self.phase == HARDWARE_TEST_RESTORE_PHASE:
                state = self.bus.right_joint7_operating_state()
                if (self.bus.right_joint7_mode_readback() == 5 and state is not None
                        and state[0] == MOTOR_RUNNING_STATE
                        and state[1] >= self.hardware_test_csp_enabled_at):
                    self.hardware_test_mit_active = False
                    self.begin_stage(CENTER_RELAX_PHASE, self.center_goal)
                elif now > self.hardware_test_restore_deadline:
                    self.fail("J7 position-mode confirmation missing on MIT session exit")
                elif now - self.hardware_test_last_enable_retry >= 0.1:
                    self.hardware_test_csp_enabled_at = self.bus.reassert_right_joint7_csp_hold(
                        float(self.hardware_test_anchor[6]))
                    self.hardware_test_last_enable_retry = now
            return
        status = session.status
        if status.error:
            self.fail(status.error)
            return
        anchor = np.asarray(self.hardware_test_anchor, dtype=float)
        if np.max(np.abs(self.arm()[:6] - anchor[:6])) > HARDWARE_TEST_OTHER_JOINT_DRIFT_RAD:
            self.fail("J1-J6 drift exceeded the independent 3.000° MIT-test limit")
            return
        if status.rejected:
            self.status.set("Manual strike rejected: " + status.rejected)
        new_phase = {
            "arming": HARDWARE_TEST_HOLD_PHASE,
            "pending": HARDWARE_TEST_OUT_PHASE,
            "fall": HARDWARE_TEST_OUT_PHASE,
            "catch": HARDWARE_TEST_RETURN_PHASE,
            "return": HARDWARE_TEST_RETURN_PHASE,
            "hold": (HARDWARE_TEST_READY_PHASE if self._hardware_test_session_ready(status)
                     else HARDWARE_TEST_HOLD_PHASE),
        }[status.phase]
        self.phase = new_phase
        if status.completed > self.hardware_test_completed:
            self.hardware_test_completed = status.completed
            self.hardware_test_target = None
            message = (
                f"MIT strike complete: goal {self.hardware_test_degrees_value:g}°, "
                f"measured peak drop {math.degrees(status.peak_drop):.3f}°, "
                f"return overshoot {math.degrees(status.overshoot):.3f}°; "
                "encoder position stable — MIT remains enabled"
            )
            self.result_status.set(message)
            self.result_label.config(fg="#087f23")
            self.status.set(message)
            print(message, flush=True)  # UI-side reporting after motion settled.
        self._refresh_hardware_test_controls(status)

    def initial_gripper_goal(self):
        """Normal hardware retains the mounted stick; other modes still load."""
        if getattr(self, 'single_run_mode', False):
            return RIGHT_GRIPPER_CLOSED
        if getattr(self, "hardware_test_mode", False):
            return RIGHT_GRIPPER_OPEN
        return super().initial_gripper_goal()

    def start(self):
        if getattr(self, 'emergency_latched', False):
            self.status.set('Emergency stop latched; restart required')
            return
        if getattr(self, "dual", None) is not None and not self.dual.ready:
            self.status.set("Cannot RUN: select and finish preflighting the left recording")
            return
        if getattr(self, 'recording_preflight', None) is not None:
            self.status.set('Wait for recording validation before RUN')
            return
        if not self.hardware:
            self.status.set("Offline preview only — add --hardware to run the sequence")
            return
        if self.playback_trajectory is None:
            self.status.set("Cannot start: select and preflight a right-arm recording")
            return
        if self.phase not in ("READY", "RELAXED") or not self.bus.fresh():
            return
        try:
            self._apply_selected_tempo()
            snare_control.check_tempo(self)
        except ValueError as exc:
            self.status.set('Cannot start: ' + str(exc))
            return
        required_support_fault = (
            self.support_fault_detail
            or (not self.hardware_test_mode and not getattr(self, 'recording_only', False)
                and (self.sound_fault_detail or self.hihat_fault_detail))
        )
        if not getattr(self, "test_mode", False) and required_support_fault:
            self.status.set("Cannot start: support-process fault is latched; restart the program")
            return
        if (not getattr(self, "test_mode", False)
                and (not self.receiver.fresh() or
                     (not self.receiver.cymbal and not getattr(self, 'recording_only', False)))):
            self.status.set("Cannot start: waiting for a fresh visible cymbal")
            return
        if not self._sound_ready_for_start():
            self.status.set("Cannot start: waiting for ST7 to listen on the TONOR microphone")
            return
        if not self.zone.contains(self.planned_tcp(self.center_goal), MEMBERSHIP_BUFFER_M):
            self.status.set("ERROR: configured right center is outside right_zones/zone1.json")
            return

        if getattr(self, "dual", None) is not None:
            self.dual.reset()
        self.goal = self.playback_trajectory.first_joints
        self.target_point = self.playback_trajectory.tcp_positions[0].copy()
        self.target_offset = self.target_point - self.ik.origin_tcp
        self.target_inside = True
        self.gripper_closed_latched = getattr(self, 'single_run_mode', False)
        self.single_run_gripper_deadline = None
        self.gripper_close_until = None
        self.last_gripper_command = 0.0
        self.search_active = False
        self.pending_detection_frame_id = None
        self._clear_alignment_state()
        self.playback_visual_samples = []
        self.playback_frame_ids = set()
        self._camera_fault_handled = False
        self.result_status.set("Result: recording playback has not started")
        self.result_label.config(fg="navy")
        self.phase = "PLAYBACK PREFLIGHTED"
        self.hihat_start_query_at = None
        self.ik_refresh_deadline = time.monotonic() + 2.0
        if getattr(self, "test_mode", False):
            preflight_status = (
                "PURE SIMULATION: recording ready — preparing the simulated arm"
            )
        else:
            if self.hardware_test_mode:
                preflight_status = (
                    "Hardware-test recording ready — refreshing camera and encoders "
                    "before the center/open/load sequence"
                )
            else:
                preflight_status = (
                    "Recording ready — refreshing camera, TONOR sound detection, "
                    "ESP32 hi-hat, and encoders before centering"
                )
        self.status.set(preflight_status)
        if getattr(self, 'single_run_mode', False) or getattr(self, 'bpm_entry', None) is not None:
            self._refresh_buttons()
        self.root.after(20, self._start_preflighted_recording)

    def _start_preflighted_recording(self) -> None:
        if self.phase != "PLAYBACK PREFLIGHTED":
            return
        if not getattr(self, "test_mode", False):
            self.receiver.poll()
            self.audio_receiver.poll()
            if self.hihat is not None:
                self.hihat.tick()
        fresh_hihat_reply = True
        if getattr(self, 'hybrid_enabled', False):
            # A serial descriptor being open is not proof that the ESP32 is
            # still alive. Require a real response newer than this RUN request
            # before any arm enable. Normal calibration polling stays active.
            now = time.monotonic()
            requested = getattr(self, 'hihat_start_query_at', None)
            if requested is None and self.hihat.ready() and not self.hihat_fault_detail:
                self.hihat_start_query_at = now
                self.hihat.query_status()
            stamp = self.hihat.telemetry_at
            fresh_hihat_reply = (requested is not None and stamp is not None
                                and requested <= stamp and 0 <= now - stamp <= .6)
        if (not self.bus.fresh() or not self._camera_ready_for_start()
                or not self._sound_ready_for_start()
                or not self._hihat_ready_for_start() or not fresh_hihat_reply):
            if time.monotonic() >= self.ik_refresh_deadline:
                self.phase = "READY"
                if getattr(self, "test_mode", False):
                    detail = "ERROR: simulated joint state did not become ready"
                else:
                    required = (
                        "encoders and a visible cymbal"
                        if self.hardware_test_mode else
                        "encoders, visible cymbal, TONOR sound detection, and ESP32 hi-hat"
                    )
                    detail = "ERROR: " + required + " did not all become ready"
                self.status.set(detail)
                return
            self.root.after(20, self._start_preflighted_recording)
            return
        self.phase = "READY"
        if self.hardware_test_mode:
            if not self.zone.contains(self.tcp(), MEMBERSHIP_BUFFER_M):
                self.status.set(
                    "Cannot start: current right TCP is outside right_zones/zone1.json"
                )
                return
            self.setup = self.bus.center(
                self.center_goal, self.initial_gripper_goal(), confirm_enabled=True
            )
            self.phase = "SETTING UP"
        else:
            JointGoalApp.start(self)
        if self.phase == "SETTING UP":
            self.status.set(
                "Recording preflight passed; "
                + ("confirming all drives at their live poses, then "
                   if self.hardware_test_mode else "")
                + f"centering BOTH arms (left J5 −50°, others 0°, gripper raw target +14.16°; right J7={math.degrees(self.center_goal[6]):.1f}°) "
                + ("and closing gripper to +7°; recording follows automatically"
                   if getattr(self, 'single_run_mode', False) else
                   "and opening gripper to −3°")
            )

    def camera_tick(self):
        if getattr(self, "test_mode", False):
            self._camera_loading_text = None
            self._refresh_loading_label()
            self.camera_status.set("Camera: ignored in test mode — process not started")
            self.camera_label.config(fg="#087f23")
            self._refresh_buttons()
            self.root.after(20, self.camera_tick)
            return
        try:
            messages = self.receiver.poll()
            self._sync_camera_loading_label()
            now = time.monotonic()
            text, color = self._camera_text(now)
            self.camera_status.set(text)
            self.camera_label.config(fg=color)
            for message in messages:
                if message.get("kind") != "frame":
                    continue
                if self.playback_active:
                    self._record_playback_observation(message, now)
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
                if self.alignment_active or self.playback_active:
                    self._program_failure(
                        "FAILED: camera/visualization process error: " + process_problem
                    )
            elif (self.bus and self.bus.active and self.phase in self.CAMERA_REQUIRED_PHASES
                  and not self.receiver.fresh(now)):
                self._camera_failure("processed camera detections became stale")
            self._refresh_buttons()
        except Exception as exc:
            if self.alignment_active or self.playback_active:
                self._program_failure("FAILED: camera status failure: " + str(exc))
            else:
                self.camera_status.set("Camera ERROR: " + str(exc))
                self.camera_label.config(fg="#b00020")
        self.root.after(20, self.camera_tick)

    def _camera_ready_for_start(self, now: float | None = None) -> bool:
        if getattr(self, "test_mode", False):
            return True
        if getattr(self, 'recording_only', False):
            return not self.support_fault_detail and self.receiver.fresh(now)
        return super()._camera_ready_for_start(now)

    def _sound_ready_for_start(self, now: float | None = None) -> bool:
        return (getattr(self, "test_mode", False)
                or getattr(self, 'recording_only', False)
                or getattr(self, "hardware_test_mode", False)
                or (not self.sound_fault_detail and self.audio_receiver.ready(now)))

    def _hihat_ready_for_start(self) -> bool:
        return (getattr(self, "test_mode", False)
                or getattr(self, 'recording_only', False)
                or getattr(self, "hardware_test_mode", False) or not self.hardware
                or (not self.hihat_fault_detail and self.hihat.ready()))

    def _hihat_required_now(self) -> bool:
        if (getattr(self, "test_mode", False)
                or getattr(self, 'recording_only', False)
                or getattr(self, "hardware_test_mode", False)
                or not self.hardware or not self.bus or not self.bus.active):
            return False
        if self.phase in self.RETURN_PHASES:
            return False
        return self.phase not in {
            "READY", "RELAXED", "FAULT", "FAULT RECENTERING",
            "ZONE RECENTERING", "RELAXING",
        }

    def _sound_required_now(self) -> bool:
        if (getattr(self, "test_mode", False)
                or getattr(self, 'recording_only', False)
                or getattr(self, "hardware_test_mode", False)):
            return False
        if not self.bus or not self.bus.active:
            return False
        if getattr(self, "continuous_strike_active", False):
            return False
        if self.phase in self.RETURN_PHASES:
            return False
        return self.phase not in {
            "READY", "RELAXED", "FAULT", "FAULT RECENTERING",
            "ZONE RECENTERING", "RELAXING",
        }

    def sound_tick(self) -> None:
        if getattr(self, "test_mode", False):
            self.sound_status.set("Sound: ignored in test mode — ST7/TONOR not started")
            self.sound_label.config(fg="#087f23")
            self._refresh_buttons()
            self.root.after(20, self.sound_tick)
            return
        try:
            messages = self.audio_receiver.poll()
            sync = getattr(self, 'hihat_sync', None)
            if sync is not None:
                sync.feed('ride', messages)
            now = time.monotonic()
            for message in messages:
                if message.get("kind") == "hit" and message.get('instrument', 'ride') == 'ride':
                    evidence = getattr(self, 'beat_evidence', None)
                    if evidence is not None:
                        evidence.audio(message)
                    if self._hardware_test_sound_monitor_active():
                        self._log_hardware_test_sound_hit(message)
                    self._process_sound_hit(message)

            process_problem = next(
                (str(message.get("detail", "ST7/TONOR sound detector stopped"))
                 for message in messages
                 if message.get("kind") == "status"
                 and message.get("state") in {"error", "stopped"}),
                None,
            )
            if process_problem is None and self.audio_receiver.state in {"error", "stopped"}:
                process_problem = self.audio_receiver.detail or "ST7/TONOR sound detector stopped"
            if process_problem is not None:
                self.sound_fault_detail = process_problem
                if self._sound_required_now():
                    self._program_failure("FAILED: sound detector error: " + process_problem)
            elif self._sound_required_now() and not self.audio_receiver.ready(now):
                self._program_failure("FAILED: ST7/TONOR sound detector heartbeat became stale")

            if self.sound_fault_detail:
                text = "Sound ERROR: " + self.sound_fault_detail
                color = "#b00020"
            elif self.audio_receiver.ready(now):
                text = (
                    "Sound: ST7 ride v2 READY — informational hit logging only"
                    if self.hardware_test_mode else
                    "Sound: ST7 ride v2 READY — TONOR ride-only hit search"
                )
                color = "#087f23"
            else:
                text = "Sound: " + (self.audio_receiver.detail or "starting ST7 and TONOR")
                color = "#8a5a00"
            self.sound_status.set(text)
            self.sound_label.config(fg=color)
            self._refresh_buttons()
        except Exception as exc:
            if self._sound_required_now():
                self._program_failure("FAILED: sound detector status failure: " + str(exc))
            else:
                self.sound_status.set("Sound ERROR: " + str(exc))
                self.sound_label.config(fg="#b00020")
        self.root.after(20, self.sound_tick)

    def _hardware_test_sound_monitor_active(self) -> bool:
        return bool(
            getattr(self, "hardware_test_mode", False)
            and getattr(self, "bus", None) and self.bus.active
        )

    def _log_hardware_test_sound_hit(self, message: dict) -> None:
        """Log every ST7 hit without allowing sound to trigger arm motion."""
        self.hardware_test_sound_count += 1
        score = float(message.get("score", float("nan")))
        normality = message.get("normality_score")
        normality_text = (
            f"{float(normality):.1f}"
            if normality is not None and math.isfinite(float(normality)) else "unavailable"
        )
        phase_name = {
            HARDWARE_TEST_READY_PHASE: "waiting for button",
            HARDWARE_TEST_OUT_PHASE: "damped MIT freefall",
            HARDWARE_TEST_RETURN_PHASE: "predictive catch / continuous withdrawal",
            HARDWARE_TEST_HOLD_PHASE: "MIT hold settling",
            HARDWARE_TEST_RESTORE_PHASE: "leaving MIT session",
        }.get(self.phase, self.phase)
        detail = (
            f"ST7 HIT #{self.hardware_test_sound_count}: score={score:.3f}, "
            f"normality={normality_text}, phase={phase_name}"
        )
        sound_status = getattr(self, "hardware_test_sound_status", None)
        if sound_status is not None:
            sound_status.set("ST7 sound log: " + detail)
            self.hardware_test_sound_label.config(fg="#087f23")
        print("HARDWARE TEST SOUND LOG: " + detail, flush=True)

    def hihat_tick(self) -> None:
        if getattr(self, 'recording_only', False):
            self.hihat_status.set('Hi-hat ESP32: disabled in recording-only mode; no strikes')
            return
        if getattr(self, "test_mode", False):
            self.hihat_status.set(
                "Hi-hat ESP32: ignored in test mode — serial port not opened"
            )
            self.hihat_label.config(fg="#087f23")
            self._refresh_buttons()
            self.root.after(20, self.hihat_tick)
            return
        if getattr(self, "hardware_test_mode", False):
            self.hihat_status.set(
                "Hi-hat ESP32: not used in hardware test — serial port not opened"
            )
            self.hihat_label.config(fg="#087f23")
            self._refresh_buttons()
            self.root.after(20, self.hihat_tick)
            return
        try:
            if self.hardware:
                self.hihat.tick()
                calibration = getattr(self, 'hihat_calibration', None)
                if calibration is not None:
                    calibration.tick()
                if self.hihat.state == "error":
                    first_fault = self.hihat_fault_detail is None
                    self.hihat_fault_detail = self.hihat.detail
                    if first_fault and getattr(self, "continuous_strike_active", False):
                        self.status.set(
                            f"ESP32 hi-hat fault — stopping {self.strike_bpm:g} BPM striking, then "
                            "centering and relaxing: " + self.hihat.detail
                        )
                        self._request_continuous_stop()
                    elif first_fault and self._hihat_required_now():
                        self._program_failure(
                            "FAILED: ESP32 hi-hat error: " + self.hihat.detail
                        )
            if not self.hardware:
                text = "Hi-hat ESP32: offline preview — serial port not opened"
                color = "#8a5a00"
            elif self.hihat_fault_detail:
                text = "Hi-hat ESP32 ERROR: " + self.hihat_fault_detail
                color = "#b00020"
            elif getattr(self, 'hihat_calibration', None) is not None:
                text = 'Hi-hat ESP32: ' + self.hihat_calibration.detail
                color = '#087f23' if self.hihat_calibration.ready else '#8a5a00'
            elif self.hihat.ready():
                text = (
                    f"Hi-hat ESP32 READY — motor 2 alternates "
                    f"{MOTOR2_TARGET_DEGREES}°/0° on arm beats"
                )
                color = "#087f23"
            else:
                text = "Hi-hat ESP32: " + self.hihat.detail
                color = "#8a5a00"
            self.hihat_status.set(text)
            self.hihat_label.config(fg=color)
            self._refresh_buttons()
        except Exception as exc:
            self.hihat_fault_detail = str(exc)
            self.hihat_status.set("Hi-hat ESP32 ERROR: " + str(exc))
            self.hihat_label.config(fg="#b00020")
        self.root.after(20, self.hihat_tick)

    def _hihat_sound_messages(self, messages):
        calibration = getattr(self, 'hihat_calibration', None)
        if calibration is not None:
            calibration.feed(messages)
        sync = getattr(self, 'hihat_sync', None)
        if sync is not None:
            sync.feed('hihat', messages)

    def _cancel_hihat_calibration(self):
        calibration = getattr(self, 'hihat_calibration', None)
        if calibration is not None:
            calibration.cancel()

    def centered(self, now=None):
        if (isinstance(self.bus, (PlaybackMotors, SimulatedMotors))
                and not self.bus.left_center_ready()):
            self.status.set('Right centered — waiting for left center (J5 −50°, gripper raw target +14.16°)')
            return
        if getattr(self, 'single_run_mode', False):
            # No WAITING FOR LOAD phase and no inherited open-gripper check.
            # The center controller keeps holding until closure is observed.
            if self.phase != 'CENTERING':
                return
            now = time.monotonic() if now is None else now
            if not self.bus.fresh():
                self.status.set('Centered — waiting for fresh gripper feedback before recording')
                return
            if self.single_run_gripper_deadline is None:
                self.single_run_gripper_deadline = now + SINGLE_RUN_GRIPPER_TIMEOUT
            if abs(self.gripper_encoder() - RIGHT_GRIPPER_CLOSED) > GRIPPER_TOLERANCE:
                if now >= self.single_run_gripper_deadline:
                    self._program_failure('FAILED: gripper did not close to +7°; recording not started')
                else:
                    self.status.set('Centered — waiting for gripper to close to +7°, then recording')
                return
            if (not self._camera_ready_for_start(now) or not self._sound_ready_for_start(now)
                    or not self._hihat_ready_for_start()):
                self._program_failure('FAILED: camera, ST7 or hi-hat readiness lost before recording')
                return
            self.single_run_gripper_deadline = None
            self.begin_stage('MOVING TO RECORDING START', self.playback_trajectory.first_joints)
            self.status.set('Centered with gripper closed at +7° — moving to the recording\'s first pose')
            print('ONE RUN: center and gripper closure confirmed; starting recording without a loading pause', flush=True)
            return
        super().centered(now)
        if self.phase == "WAITING FOR LOAD":
            if getattr(self, 'recording_only', False):
                suffix = "only recording and a verified center return run next; no strikes"
            elif getattr(self, "test_mode", False):
                suffix = "camera is bypassed"
            elif getattr(self, "hardware_test_mode", False):
                suffix = "the recording and pink-zone alignment run next"
            else:
                return
            self.status.set(
                "Centered with gripper open at −3° — load the drumstick, then "
                f"press Continue ({suffix})"
            )

    def continue_motion(self):
        if not self.hardware or self.phase != "WAITING FOR LOAD":
            return
        if getattr(self, "test_mode", False) or getattr(self, 'recording_only', False):
            # Skip CameraSearchApp's live-camera gate while retaining the
            # shared Cartesian gripper-close transition beneath it.
            super(CameraSearchApp, self).continue_motion()
            return
        self.audio_receiver.poll()
        if not self._sound_ready_for_start():
            self.status.set("Cannot continue: waiting for ST7 on the TONOR microphone")
            return
        if not self._hihat_ready_for_start():
            self.status.set("Cannot continue: waiting for the ESP32 hi-hat controller")
            return
        super().continue_motion()

    def extra_control(self, now):
        if getattr(self, 'emergency_latched', False):
            return
        if getattr(self, "dual", None) is not None and self.dual.tick(now):
            return
        if not getattr(self, 'single_run_mode', False):
            self.continue_button.config(
                state="normal" if (
                    self.phase == "WAITING FOR LOAD" and self.bus.fresh()
                    and self._camera_ready_for_start(now) and self._sound_ready_for_start(now)
                    and self._hihat_ready_for_start()
                ) else "disabled"
            )
        if not self.bus.active:
            return
        if self.phase == HIHAT_CALIBRATION_WAIT_PHASE:
            calibration = getattr(self, 'hihat_calibration', None)
            if calibration is not None and calibration.ready:
                # Recheck the current endpoint and a fresh camera frame; do
                # not reuse an alignment observation made before this wait.
                self._begin_playback_alignment(now)
        self._sample_continuous_j7_minimum()
        self._advance_hardware_test_mit(now)
        hybrid_control.tick(self, now)
        if getattr(self, 'phase', None) == hybrid_control.FAULT:
            return
        self._maintain_hardware_test_fault_hold(now)
        self._maintain_hardware_test_workflow_fault(now)
        if self.phase == HARDWARE_TEST_WORKFLOW_FAULT_PHASE:
            return
        target = None
        if getattr(self, 'single_run_mode', False):
            # During setup the center generator alone configures the gripper.
            # Never send the legacy open target or write before drive setup.
            if self.gripper_closed_latched and self.setup is None:
                target = RIGHT_GRIPPER_CLOSED
        elif self.gripper_closed_latched:
            target = RIGHT_GRIPPER_CLOSED
        elif self.phase in ("CENTERING", "WAITING FOR LOAD"):
            target = RIGHT_GRIPPER_OPEN
        if target is not None and now - self.last_gripper_command >= GRIPPER_COMMAND_INTERVAL:
            self.bus.set_gripper(target)
            self.last_gripper_command = now

        if self.phase == "CLOSING GRIPPER":
            remaining = max(0.0, self.gripper_close_until - now)
            self.status.set(f"Closing gripper to +7° — recording starts in {remaining:.1f} s")
            if remaining <= 0.0:
                if not getattr(self, "test_mode", False):
                    self.receiver.poll()
                if (not getattr(self, "test_mode", False)
                        and (not self.receiver.fresh() or
                             (not self.receiver.cymbal and not getattr(self, 'recording_only', False)))):
                    self._program_failure(
                        "FAILED: fresh visible cymbal unavailable after gripper close"
                    )
                    return
                self.begin_stage("MOVING TO RECORDING START", self.playback_trajectory.first_joints)
                self.status.set("Gripper closed at +7° — moving to the recording's first pose")
        elif self.phase == "RECORDING PLAYBACK":
            self._advance_playback(now)

        if self.phase == STRIKE_SOUND_WAIT_PHASE and self.strike_sound_deadline is not None:
            if hybrid_control.enabled(self):
                self._advance_hybrid_audio_wait(now)
            elif now >= self.strike_sound_deadline:
                self._finish_no_hit_attempt()

        if getattr(self, "continuous_strike_active", False):
            self._advance_continuous_striking(now)

        if self.alignment_active:
            self._poll_hill_planner(now)
            self._advance_measurement(now)
            if (self.alignment_loss_deadline is not None
                    and now >= self.alignment_loss_deadline):
                self._program_failure(
                    "FAILED: no simultaneous cymbal + observed drumstick tip for 30 seconds"
                )

    def complete_stage(self, now=None):
        now = time.monotonic() if now is None else now
        if self.phase == HIHAT_CALIBRATION_WAIT_PHASE:
            return  # Keep the inherited feedback-supervised hold controller.
        if (getattr(self, 'single_run_mode', False)
                and isinstance(self.bus, PlaybackMotors)
                and self.phase in self.RETURN_PHASES | {'RECENTERING',
                                   'ZONE RECENTERING', 'FAULT RECENTERING'}):
            try:
                self.bus.center_evidence()
            except RuntimeError:
                self.status.set('Holding center target until all seven encoders verify settled center; NOT relaxing')
                return
        if getattr(self, 'dual', None) is not None and self.dual.returning:
            self.dual.right_center_reached()
            return
        if self.phase == "MOVING TO RECORDING START":
            self._begin_playback(now)
            return
        if self.phase == "SETTLING RECORDING END":
            if not self._restore_playback_speed("Recording endpoint reached"):
                return
            if getattr(self, "test_mode", False):
                self._begin_test_striking()
            else:
                self._begin_playback_alignment(now)
            return
        if self.phase == "STRIKE MOVING OUT":
            self.strike_attempt_motion_seen = True
            self._begin_strike_stage(
                "STRIKE RETURNING TO CYMBAL POSE",
                self.strike_plan.anchor_joints,
            )
            return
        if self.phase == "STRIKE RETURNING TO CYMBAL POSE":
            self.control = None
            self.strike_attempt_returned_at = now
            self._pause_strike_feedback()
            if getattr(self, "strike_hit_pending", None) is not None:
                self._finish_hit_strike_attempt()
                return
            self.phase = STRIKE_SOUND_WAIT_PHASE
            self.strike_sound_deadline = now + (HYBRID_AUDIO_BACKLOG_LIMIT
                if hybrid_control.enabled(self) else STRIKE_SOUND_WAIT_SECONDS)
            amount = self._current_strike_degrees()
            self.status.set(
                f"J7 −{amount}° attempt returned to the pink-zone pose — waiting "
                "for ST7's finalized audio interval before declaring a miss"
            )
            return
        if self.phase == CONTINUOUS_OUT_PHASE:
            self._sample_continuous_j7_minimum()
            self.continuous_j7_measurement_reached = True
            interval = self.continuous_current_interval_seconds
            if interval is None:
                self._program_failure("FAILED: swing strike interval was not set")
                return
            event_at = self.continuous_current_event_at
            if event_at is None:
                # The opening pickup establishes the clock when its strike
                # target is actually reached.  All following events retain
                # their triplet grid even if an individual target is late.
                event_at = now
                self.continuous_current_event_at = event_at
                snare_control.simulation_epoch(self, event_at + interval)
            self.continuous_last_beat_at = event_at
            self.continuous_next_beat_at = event_at + interval
            self._begin_continuous_stage(
                CONTINUOUS_RETURN_PHASE,
                self.strike_plan.anchor_joints,
                now,
            )
            return
        if self.phase == CONTINUOUS_RETURN_PHASE:
            self._log_continuous_j7_minimum()
            self.control = None
            self.phase = CONTINUOUS_WAIT_PHASE
            self._pause_strike_feedback()
            if self.continuous_stop_requested:
                self._finish_continuous_striking()
            else:
                next_label = self.swing_events[self.continuous_swing_index][0]
                self.status.set(
                    f"Swing strike {self.continuous_strike_count} returned to "
                    f"the pink-zone pose; {next_label} in "
                    f"{max(0.0, self.continuous_next_beat_at - now):.2f} s"
                )
            return
        if self.phase == CENTER_RELAX_PHASE:
            self.control = None
            self._clear_alignment_state()
            message = ("Center + Relax complete; simulated arm centered and relaxed"
                       if getattr(self, "test_mode", False) else
                       "Center + Relax complete; centered; right motors disabled")
            self.relax(message)
            return
        super().complete_stage(now)

    def _begin_playback(self, now: float) -> None:
        if isinstance(getattr(self, "playback_trajectory", None), SmoothRecording):
            self._begin_smooth_playback(now)
            return
        # This higher firmware velocity applies only to the recorded path.  Set
        # the flag first so a partial seven-motor write is restored on failure.
        self.playback_speed_fast = True
        try:
            self.bus.set_right_arm_speed(RIGHT_PLAYBACK_SPEED)
        except Exception as exc:
            self._program_failure(
                "FAILED: could not set right-arm recording playback speed: " + str(exc)
            )
            return
        self.control = None
        self.phase = "RECORDING PLAYBACK"
        self.playback_active = True
        self.playback_started_at = now
        self.playback_last_command = 0.0
        self.playback_follower = PlaybackFollower()
        self.playback_visual_samples = []
        self.playback_frame_ids = set()
        self.result_status.set(
            "PLAYBACK: following the selected recording; camera bypassed in test mode"
            if getattr(self, "test_mode", False) else
            "PLAYBACK: following the selected recording while observing the camera"
        )
        self.result_label.config(fg="navy")
        self.status.set(
            "Recording start reached — playing the recorded right-arm path "
            "with the temporary 0.8 rad/s motor limit"
        )

    def _advance_playback(self, now: float) -> None:
        if getattr(self, "smooth_playback_session", None) is not None:
            self._advance_smooth_playback(now)
            return
        elapsed = max(0.0, now - self.playback_started_at)
        desired, finished = self.playback_trajectory.joints_at(elapsed)
        command = self.playback_follower.update(self.arm(), desired, now)
        self.goal = desired.copy()
        self.target_point = self.hill_ik.position(desired)
        self.target_offset = self.target_point - self.ik.origin_tcp
        self.target_inside = True
        if now - self.playback_last_command >= PLAYBACK_COMMAND_INTERVAL:
            self.bus.set_positions(command)
            self.playback_last_command = now
        duration = self.playback_trajectory.duration_s
        self.status.set(
            f"Playing recording: {min(elapsed, duration):.2f}/{duration:.2f} s • "
            + ("camera bypassed (test mode)" if getattr(self, "test_mode", False) else
               f"camera observations {len(self.playback_visual_samples)}")
        )
        if finished:
            self.playback_active = False
            self.begin_stage("SETTLING RECORDING END", self.playback_trajectory.last_joints)
            self.status.set("Recorded path complete — settling at its final pose")

    def _begin_smooth_playback(self, now: float) -> None:
        self.control = None
        self.phase = "RECORDING PLAYBACK"
        self.playback_active = True
        self.playback_started_at = now
        self.playback_visual_samples = []
        self.playback_frame_ids = set()
        self.playback_follower = None
        self.playback_speed_fast = True
        try:
            if getattr(self, "smooth_playback_cleanup_error", None):
                raise RuntimeError(self.smooth_playback_cleanup_error)
            self.smooth_playback_session = RecordingSession(self.playback_trajectory, self.bus)
            if isinstance(self.bus, PlaybackMotors):
                self.bus.playback_session = self.smooth_playback_session
                # spawn()/trajectory serialization can exceed the ordinary
                # 300 ms feedback age while firmware holds the first pose.
                # Re-prime real readbacks before resuming the GUI watchdog;
                # never widen its timeout or accept stale encoder positions.
                recording_only_control.refresh_feedback(self.bus, .10)
            self.last_recording_worker_directory = str(self.smooth_playback_session.directory)
        except Exception as exc:
            self.playback_active = False
            self._program_failure("FAILED: smooth recording startup: " + str(exc))
            return
        self.result_status.set(
            "PLAYBACK: paced_precise 200 Hz; observing camera; exact recorded endpoint"
        )
        self.result_label.config(fg="navy")
        self.status.set("Recording start reached — starting smooth recording playback")

    def _advance_smooth_playback(self, now: float) -> None:
        update = self.smooth_playback_session.poll()
        desired, _ = self.playback_trajectory.joints_at(update["elapsed"])
        self.goal = desired.copy()
        self.target_point = self.hill_ik.position(desired)
        self.target_offset = self.target_point - self.ik.origin_tcp
        self.target_inside = True
        # Display/observe only. No Tk-clock arm targets compete with the worker.
        self.status.set(
            f"Smooth recording: {update['elapsed']:.2f}/{self.playback_trajectory.duration_s:.2f} s "
            f"• {update['phase']} • camera observations {len(self.playback_visual_samples)}"
        )
        if not update["done"]:
            return
        restored = self._stop_smooth_playback()
        self.playback_active = False
        result = update["result"]
        if not result["success"] or not restored:
            self._program_failure("FAILED: smooth recording: " + str(
                result.get("error") or self.smooth_playback_cleanup_error))
            return
        if getattr(self, 'recording_only', False):
            recording_only_control.finish(self, 'Recording endpoint verified', playback_success=True)
            return
        # Same endpoint -> settling -> camera alignment transition as before.
        # Never call playback.sh's recenter/relax workflow here.
        self.begin_stage("SETTLING RECORDING END", self.playback_trajectory.last_joints)
        self.status.set("Recorded path complete — original gains restored; settling at its final pose")

    def _stop_smooth_playback(self) -> bool:
        session = getattr(self, "smooth_playback_session", None)
        if session is None:
            return True
        result = session.stop()  # writer has exited before returning ownership
        self.smooth_playback_session = None
        if isinstance(getattr(self, 'bus', None), PlaybackMotors):
            self.bus.playback_session = None
        self.playback_active = False
        if result.get("cleanup_error") or not result.get("gains_restored", False):
            self.smooth_playback_cleanup_error = (
                "Smooth playback settings handoff failed: "
                + str(result.get("cleanup_error") or result.get("error"))
            )
            print(self.smooth_playback_cleanup_error, flush=True)
            return False
        return True

    def extra_markers(self):
        markers = super().extra_markers()
        dual = getattr(self, 'dual', None)
        if dual is None:
            return markers
        zone = dual.g.zone
        # Static hull: build once, not on every control-loop publication.
        if getattr(self, '_left_marker_zone', None) is not zone or zone._dirty:
            self._left_zone_markers = left_zone_markers(self.marker, zone)
            self._left_marker_zone = zone
        stamp = self.node.get_clock().now().to_msg()
        for marker in self._left_zone_markers:
            marker.header.stamp = stamp
        return markers + self._left_zone_markers

    def begin_stage(self, name, desired):
        if getattr(self, 'emergency_latched', False):
            raise RuntimeError('Motion blocked after emergency stop; restart required')
        dual = getattr(self, 'dual', None)
        if dual is not None:
            if name in self.RETURN_PHASES | {'RECENTERING', 'ZONE RECENTERING', 'FAULT RECENTERING'}:
                if not dual.returning:
                    return dual.begin_return(name)
                if 'right' in self.bus.center_disabled:
                    return
            if name == 'MOVING TO RECORDING START' and not dual.left_done:
                return dual.begin_left()
            if dual.session is not None and not dual.returning:
                dual.stop()
        # Covers ordinary stages and inherited zone/error recovery alike.
        if not self._stop_smooth_playback():
            raise RuntimeError(self.smooth_playback_cleanup_error)
        return super().begin_stage(name, desired)

    def _record_playback_observation(self, message: dict, now: float) -> None:
        observation = observation_from_message(message)
        frame_id = message.get("frame_id")
        if observation is None or frame_id in self.playback_frame_ids:
            return
        self.playback_frame_ids.add(frame_id)
        offset = self.tcp() - self.ik.origin_tcp
        self.playback_visual_samples.append(PlaybackVisualSample(
            time_s=now,
            tcp_offset_m=offset.copy(),
            center_score=normalized_center_distance(observation),
        ))
        if len(self.playback_visual_samples) > 2000:
            self.playback_visual_samples = self.playback_visual_samples[-2000:]

    def _begin_playback_alignment(self, now: float) -> None:
        if getattr(self, 'recording_only', False):
            recording_only_control.finish(self, 'Alignment bypassed', playback_success=True)
            return
        self.playback_active = False
        calibration = getattr(self, 'hihat_calibration', None)
        if calibration is not None and not calibration.ready:
            if calibration.engine.failure:
                self.fail('Hi-hat calibration failed: ' + calibration.engine.failure)
                return
            self.alignment_active = False
            self.phase = HIHAT_CALIBRATION_WAIT_PHASE
            self._reset_hold_controller()
            self.status.set('Holding recording endpoint — waiting for hi-hat calibration before ride search')
            print('ARM WAIT: hi-hat calibration must finish before ride calibration', flush=True)
            return
        actual_joints = self.arm().copy()
        joint2 = actual_joints[1]
        if joint2 < self.lower[1] - 3 * ENCODER_LSB_RAD or joint2 > self.upper[1] + 3 * ENCODER_LSB_RAD:
            self._program_failure("FAILED: recorded endpoint J2 is outside its limit")
            return
        self.playback_fixed_joint2 = float(np.clip(joint2, self.lower[1], self.upper[1]))
        order, detail = infer_direction_order(self.playback_visual_samples)
        self.direction_prior_detail = detail
        actual_offset = self.tcp() - self.ik.origin_tcp
        self.hill = GuidedHillClimber(actual_offset, actual_joints, order)
        self.alignment_active = True
        self.alignment_loss_deadline = now + DETECTION_LOSS_TIMEOUT_SECONDS
        self.measurement_gate_frame_id = self.receiver.frame_id
        self.phase = "CHECKING PLAYBACK END"
        self._reset_hold_controller()
        message = (
            f"Recording complete at X={actual_offset[0]:+.3f}, Y={actual_offset[1]:+.3f}, "
            f"Z={actual_offset[2]:+.3f}; {detail}"
        )
        self.result_status.set(message)
        self.result_label.config(fg="#087f23")
        self.status.set(
            message + " — waiting for one fresh tip check in the visible pink rectangle"
        )

    def _reset_hold_controller(self) -> None:
        if self.phase != HIHAT_CALIBRATION_WAIT_PHASE:
            super()._reset_hold_controller()
            return
        desired = self.control.desired.copy() if self.control is not None else self.arm().copy()
        self.control = HiHatCalibrationHold(desired, self.lower, self.upper, time.monotonic())
        self.last_command = 0.0

    def _process_alignment_frame(self, message: dict, now: float) -> None:
        observation = observation_from_message(message)
        if observation is None:
            return
        self.alignment_loss_deadline = now + DETECTION_LOSS_TIMEOUT_SECONDS
        frame_id = message["frame_id"]
        if self.phase == "CHECKING PLAYBACK END":
            if frame_id <= self.measurement_gate_frame_id:
                return
            if inside_outer_goal(observation):
                self._alignment_success(frame_id)
                return
            self.hill.anchor_score = normalized_center_distance(observation)
            self.hill.anchor_offset = (self.tcp() - self.ik.origin_tcp).copy()
            self.hill.anchor_joints = self.arm().copy()
            self.result_status.set(
                "Recorded endpoint is outside the visible pink rectangle — "
                "hill climbing until one settled camera frame enters the pink rectangle"
            )
            self.result_label.config(fg="#8a5a00")
            self._plan_next_candidate()
            return
        if self.phase in self.MEASUREMENT_PHASES:
            if (frame_id <= self.measurement_gate_frame_id
                    or frame_id in self.measurement_frame_ids):
                return
            # For this playback-only controller, the complete visible pink
            # rectangle is the goal. One fresh valid frame at a settled hill
            # pose is sufficient; no inner rectangle or timed hold is used.
            if inside_outer_goal(observation):
                self._alignment_success(frame_id)
                return
            self.measurement_frame_ids.add(frame_id)
            self.measurement_samples.append(observation)
            if self.measurement_first_valid_at is None:
                self.measurement_first_valid_at = now
            return

    def _advance_measurement(self, now: float) -> None:
        if self.phase not in self.MEASUREMENT_PHASES:
            return
        if self.measurement_first_valid_at is None:
            remaining = max(0.0, self.alignment_loss_deadline - now)
            self.status.set(
                "Holding still — waiting for cymbal + observed tip "
                f"({remaining:.1f} s before abort)"
            )
            return
        elapsed = now - self.measurement_first_valid_at
        if elapsed < MEASUREMENT_SECONDS:
            self.status.set(
                f"Holding still — robust camera sample {len(self.measurement_samples)}; "
                f"{MEASUREMENT_SECONDS - elapsed:.1f} s remaining"
            )
            return
        kind = self.measurement_kind
        # Pink-zone success is handled per fresh frame above. The robust
        # median remains useful only as a smooth score for choosing the next
        # hill-climber move when every sampled frame is outside the pink zone.
        _median, score, _inside = median_outer_result(self.measurement_samples)
        self.measurement_kind = None
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
            message = (
                f"IMPROVED {direction}: center distance {previous:.5f} → {score:.5f}; "
                f"continuing {direction}"
            )
            self.result_status.set(message)
            self.result_label.config(fg="#087f23")
            self._plan_next_candidate()
        else:
            direction = self.hill.direction_name
            message = (
                f"No improvement in {direction}: {score:.5f} versus "
                f"anchor {self.hill.anchor_score:.5f}; returning to anchor"
            )
            self.result_status.set(message)
            self.result_label.config(fg="#8a5a00")
            self.goal = self.hill.anchor_joints.copy()
            self.target_offset = self.hill.anchor_offset.copy()
            self.target_point = self.ik.origin_tcp + self.target_offset
            self.target_inside = True
            self.begin_stage("HILL RETURNING TO ANCHOR", self.goal)
            self.status.set(message)

    def _alignment_success(self, frame_id: int) -> None:
        """Prepare striking after one fresh visible-pink observation."""
        acceptance_phases = {"CHECKING PLAYBACK END"} | self.MEASUREMENT_PHASES
        if not self.alignment_active or self.phase not in acceptance_phases:
            return
        calibration = getattr(self, 'hihat_calibration', None)
        if calibration is not None and not calibration.ready:
            self._begin_playback_alignment(time.monotonic())
            return
        anchor = self.arm().copy()
        if getattr(self, "hardware_test_mode", False):
            self._prepare_hardware_test_manual_strikes(frame_id, anchor)
            return
        message = (
            f"PINK ZONE CONFIRMED ONCE (camera frame {frame_id}); "
            "starting the J7 hit search at 5°, then increasing by 1°"
        )
        self.result_status.set(message)
        self.result_label.config(fg="#087f23")
        print(message, flush=True)
        self.alignment_active = False
        self._cancel_planning()
        self.strike_index = 0
        self.strike_attempt_motion_seen = False
        self.strike_hit_pending = None
        self.strike_attempt_started_at = None
        self.strike_attempt_returned_at = None
        self.strike_sound_deadline = None
        self.hardware_test_session = None
        self.hardware_test_completed = 0
        self.hardware_test_anchor = None
        self.hardware_test_target = None
        self.hardware_test_degrees_value = None
        self.hardware_test_strike_count = 0
        self.hardware_test_sound_count = 0
        self.hardware_test_mit_active = False
        self.hardware_test_last_mit_command = 0.0
        self.hardware_test_csp_enabled_at = None
        self.hardware_test_restore_deadline = None
        self.hardware_test_last_enable_retry = 0.0
        sound_log = getattr(self, "hardware_test_sound_status", None)
        if sound_log is not None:
            sound_log.set("ST7 sound log: waiting for the manual strike section")
        self._cancel_continuous_striking()
        self.strike_active = True
        try:
            self.strike_plan = build_strike_plan(
                self.hill_ik.model,
                self.hill_ik.zone,
                self.lower.copy(),
                self.upper.copy(),
                anchor,
            )
        except ValueError as exc:
            self._program_failure("FAILED: cymbal-strike path rejected: " + str(exc))
            return
        except Exception as exc:
            self._program_failure("FAILED: cymbal-strike planning error: " + str(exc))
            return
        self._begin_strike_attempt()

    def _prepare_hardware_test_manual_strikes(
            self, frame_id: int, anchor: np.ndarray) -> None:
        """Hold the accepted pink-zone pose and expose the manual MIT control."""
        if getattr(self, 'recording_only', False):
            raise RuntimeError('Strikes are forbidden in recording-only mode')
        anchor = np.asarray(anchor, dtype=float).copy()
        self.alignment_active = False
        self._cancel_planning()
        self.control = None
        self.strike_active = False
        self.strike_plan = None
        self.strike_index = 0
        self.strike_attempt_motion_seen = False
        self.strike_hit_pending = None
        self.strike_attempt_started_at = None
        self.strike_attempt_returned_at = None
        self.strike_sound_deadline = None
        self._cancel_continuous_striking()
        self.hardware_test_anchor = anchor
        self.hardware_test_target = None
        self.hardware_test_degrees_value = None
        self.hardware_test_strike_count = 0
        self.hardware_test_sound_count = 0
        self.hardware_test_mit_active = False
        self.hardware_test_last_mit_command = 0.0
        self.hardware_test_csp_enabled_at = None
        self.hardware_test_restore_deadline = None
        self.hardware_test_last_enable_retry = 0.0
        self.hardware_test_hold_fault_message = None
        self.hardware_test_hold_delivery_error = None
        self.hardware_test_degrees.set(f"{HARDWARE_TEST_DEFAULT_DROP_DEGREES:g}")
        self.hardware_test_completed = 0
        self.hardware_test_mit_active = True
        self.phase = HARDWARE_TEST_HOLD_PHASE
        try:
            self.hardware_test_session = Joint7Session(
                self.bus, anchor[6], self.lower[6], self.upper[6],
                getattr(self, "hardware_test_settings", StrikeSettings()),
            )
        except Exception as exc:
            self.fail("Could not arm the MIT session: " + str(exc))
            return
        self.goal = anchor.copy()
        self.target_point = self.hill_ik.position(anchor)
        self.target_offset = self.target_point - self.ik.origin_tcp
        self.target_inside = True
        message = (
            f"PINK ZONE CONFIRMED ONCE (camera frame {frame_id}); holding the "
            "fixed pink-zone reference — arming MIT hold; the strike button becomes "
            "available after fresh encoder positions stabilize"
        )
        self.result_status.set(message)
        self.result_label.config(fg="#087f23")
        self.status.set(message)
        print(message, flush=True)
        self._refresh_buttons()

    def _begin_test_striking(self) -> None:
        """Repeat a fixed 10° target from the settled simulated endpoint."""
        anchor = self.arm().copy()
        self.alignment_active = False
        self._cancel_planning()
        self.strike_active = False
        self.strike_index = 0
        self.strike_attempt_motion_seen = False
        self.strike_hit_pending = None
        self.strike_attempt_started_at = None
        self.strike_attempt_returned_at = None
        self.strike_sound_deadline = None
        self._cancel_continuous_striking()
        try:
            self.strike_plan = build_strike_plan(
                self.hill_ik.model,
                self.hill_ik.zone,
                self.lower.copy(),
                self.upper.copy(),
                anchor,
            )
        except ValueError as exc:
            self._program_failure(
                f"FAILED: test-mode {TEST_STRIKE_DEGREES}-degree strike rejected: "
                + str(exc)
            )
            return
        except Exception as exc:
            self._program_failure("FAILED: test-mode strike planning error: " + str(exc))
            return
        target_index = (
            (TEST_STRIKE_DEGREES - STRIKE_START_DEGREES)
            // STRIKE_INCREMENT_DEGREES
        )
        if (TEST_STRIKE_DEGREES < STRIKE_START_DEGREES
                or (TEST_STRIKE_DEGREES - STRIKE_START_DEGREES)
                % STRIKE_INCREMENT_DEGREES
                or target_index >= len(self.strike_plan.targets)):
            self._program_failure(
                f"FAILED: fixed test-mode {TEST_STRIKE_DEGREES}-degree J7 target "
                "is not safe from the recording endpoint"
            )
            return
        self._begin_continuous_striking(
            TEST_STRIKE_DEGREES, self.strike_plan.targets[target_index]
        )

    def _current_strike_degrees(self) -> int:
        return STRIKE_START_DEGREES + self.strike_index * STRIKE_INCREMENT_DEGREES

    def _current_strike_target(self) -> np.ndarray:
        return self.strike_plan.targets[self.strike_index]

    def _begin_strike_attempt(self) -> None:
        calibration = getattr(self, 'hihat_calibration', None)
        if calibration is not None and not calibration.ready:
            self.fail('Ride search blocked: hi-hat calibration has not succeeded')
            return
        if hybrid_control.enabled(self):
            try:
                hybrid_control.begin_search(self)
            except Exception as exc:
                self.fail('Could not start hybrid search: '+str(exc))
            return
        self.strike_hit_pending = None
        self.strike_attempt_motion_seen = False
        self.strike_attempt_started_at = time.monotonic()
        self.strike_attempt_returned_at = None
        self.strike_sound_deadline = None
        self._begin_strike_stage(
            "STRIKE MOVING OUT", self._current_strike_target()
        )

    def _begin_strike_stage(self, name: str, desired) -> None:
        if getattr(self, 'recording_only', False):
            raise RuntimeError('Strikes are forbidden in recording-only mode')
        desired = np.asarray(desired, dtype=float)
        speed = (RIGHT_STRIKE_DOWN_SPEED if name == "STRIKE MOVING OUT"
                 else RIGHT_STRIKE_RETURN_SPEED)
        if not self._select_strike_leg_speed(speed, name):
            return
        self.hold_until = None
        self.control = StrikeControl(desired, self.lower, self.upper, time.monotonic())
        self.phase = name
        self.goal = desired.copy()
        self.target_point = self.hill_ik.position(desired)
        self.target_offset = self.target_point - self.ik.origin_tcp
        self.target_inside = True
        if name == "STRIKE RETURNING TO CYMBAL POSE":
            self.bus.set_right_joint7_position(desired[6])
        else:
            self.bus.set_positions(desired)
        self.last_command = 0.0
        amount = self._current_strike_degrees()
        if name == "STRIKE MOVING OUT":
            prefix = (
                f"Cymbal hit search {self.strike_index + 1}/"
                f"{len(self.strike_plan.targets)}"
            )
            self.status.set(
                f"{prefix}: J7 −{amount}° downward at "
                f"{RIGHT_STRIKE_DOWN_SPEED:.1f} rad/s"
            )
        else:
            self.status.set(
                f"J7 −{amount}° target reached — immediately returning at "
                f"{RIGHT_STRIKE_RETURN_SPEED:.1f} rad/s to the pink-zone pose"
            )

    def _select_strike_leg_speed(self, speed: float, phase: str) -> bool:
        try:
            if not getattr(self, "strike_feedback_fast", False):
                self.bus.set_right_joint7_feedback_rate(STRIKE_J7_FEEDBACK_HZ)
                self.strike_feedback_fast = True
            if getattr(self, "strike_selected_speed", None) != speed:
                self.bus.set_right_joint7_speed(speed)
                self.strike_selected_speed = speed
        except Exception as exc:
            self._pause_strike_feedback()
            self._program_failure(
                f"FAILED: could not prepare fast J7 control for {phase}: {exc}"
            )
            return False
        self.strike_speed_fast = True
        return True

    def _pause_strike_feedback(self) -> None:
        if not getattr(self, "strike_feedback_fast", False):
            return
        if self.bus and self.bus.active:
            self.bus.set_right_joint7_feedback_rate(None)
        self.strike_feedback_fast = False

    def _process_sound_hit(self, message: dict) -> None:
        # Defense in depth: optional hi-hat audio must never select a ride depth.
        if message.get('instrument', 'ride') != 'ride':
            return
        def rejected(reason):
            if hybrid_control.enabled(self):
                print('ST7 HIT NOT SELECTED: '+json.dumps(dict(
                    reason=reason, phase=self.phase, onset=message.get('event_at'),
                    detected_at=message.get('detected_at'),
                    release=getattr(self, 'strike_attempt_started_at', None),
                    returned=getattr(self, 'strike_attempt_returned_at', None))), flush=True)
        if hybrid_control.enabled(self) and not hybrid_control.sync_search(self):
            rejected('no active released search (swing hits are informational)')
            return
        if (not self.strike_active
                or self.phase not in {
                    "STRIKE MOVING OUT",
                    "STRIKE RETURNING TO CYMBAL POSE",
                    STRIKE_SOUND_WAIT_PHASE,
                }):
            rejected('not in an active search window')
            return
        if getattr(self, "strike_hit_pending", None) is not None:
            return
        started_at = self.strike_attempt_started_at
        if started_at is None:
            rejected('no confirmed release timestamp')
            return
        if abs(self.arm()[6] - self.strike_plan.anchor_joints[6]) >= math.radians(3.0):
            self.strike_attempt_motion_seen = True
        event_at = float(message["event_at"])
        if event_at < started_at - STRIKE_SOUND_EVENT_EARLY_TOLERANCE_SECONDS:
            rejected('onset precedes this release')
            return
        if (self.strike_attempt_returned_at is not None
                and event_at > (self.strike_attempt_returned_at
                                + STRIKE_SOUND_EVENT_LATE_TOLERANCE_SECONDS)):
            rejected('onset follows this completed return')
            return
        if not self.strike_attempt_motion_seen:
            rejected('no measured strike motion')
            return
        amount = self._current_strike_degrees()
        hit_message = (
            f"ST7 CYMBAL HIT DETECTED on J7 −{amount}° attempt "
            "— selecting this first detected depth"
        )
        self.result_status.set(hit_message)
        self.result_label.config(fg="#087f23")
        print(hit_message, flush=True)
        evidence = getattr(self, 'beat_evidence', None)
        if evidence is not None:
            evidence.selected(message, amount, started_at, self.strike_attempt_returned_at)
        # A hit never interrupts a strike leg. Finish the commanded depth and
        # the same return to the preserved anchor used by a no-hit
        # attempt. Only after that return is reached may the swing loop begin.
        # This guarantees anchor -> depth -> anchor for every search attempt.
        self.strike_hit_pending = {
            "message": hit_message,
            "degrees": amount,
        }
        if self.phase == STRIKE_SOUND_WAIT_PHASE:
            self._finish_hit_strike_attempt()
        else:
            self.status.set(
                hit_message + (" — completing the hybrid return to the pink-zone pose"
                               if hybrid_control.enabled(self) else
                               " — completing the fast return to the pink-zone pose")
            )

    def _advance_hybrid_audio_wait(self, now):
        """No miss until ST7 processed the entire strike, regardless of pipe lag."""
        finalized = getattr(getattr(self, 'audio_receiver', None), 'finalized_at', None)
        end = self.strike_attempt_returned_at + STRIKE_SOUND_EVENT_LATE_TOLERANCE_SECONDS
        if isinstance(finalized, (float, int)) and finalized >= end:
            self._finish_no_hit_attempt()
        elif now >= self.strike_sound_deadline:
            self._program_failure('FAILED: ST7 audio backlog exceeded 3 seconds; '
                                  'refusing to treat unprocessed audio as a missed hit')

    def _finish_no_hit_attempt(self) -> None:
        if self.phase != STRIKE_SOUND_WAIT_PHASE or self.strike_hit_pending is not None:
            return
        self.strike_sound_deadline = None
        self.strike_index += 1
        if self.strike_index < len(self.strike_plan.targets):
            self._begin_strike_attempt()
            return
        self._program_failure(
            "FAILED: every safe one-degree J7 cymbal-strike attempt from 5° "
            "completed without an ST7 sound HIT"
        )

    def _finish_hit_strike_attempt(self) -> None:
        hit = self.strike_hit_pending
        if hit is None:
            return
        degrees = float(hit["degrees"])
        self.strike_sound_deadline = None
        target = self._current_strike_target().copy()
        if hybrid_control.enabled(self):
            try:
                degrees, target = hybrid_control.boosted_swing_target(self, degrees)
            except ValueError as exc:
                self._program_failure(
                    f"FAILED: detected {hit['degrees']:g}° + "
                    f"{hybrid_control.SWING_DEPTH_BOOST_DEG:g}° swing boost is unsafe: {exc}"
                )
                return
            print(f"RIDE SWING DEPTH: detected {hit['degrees']:g}° + "
                  f"{hybrid_control.SWING_DEPTH_BOOST_DEG:g}° = {degrees:g}°", flush=True)
        self._begin_continuous_striking(degrees, target)

    def _begin_continuous_striking(self, degrees: float, target) -> None:
        if getattr(self, 'recording_only', False):
            raise RuntimeError('Strikes are forbidden in recording-only mode')
        target = np.asarray(target, dtype=float)
        if (not isinstance(degrees, (int, float, np.integer, np.floating))
                or isinstance(degrees, bool) or not math.isfinite(float(degrees))
                or degrees < STRIKE_START_DEGREES
                or target.shape != (7,) or not np.isfinite(target).all()):
            self._program_failure("FAILED: invalid detected cymbal-strike target")
            return
        hihat = getattr(self, "hihat", None)
        if hihat is not None:
            try:
                hihat.start_sequence()
            except Exception as exc:
                self.hihat_fault_detail = str(exc)
                self._program_failure(
                    "FAILED: could not start ESP32 hi-hat sequence: " + str(exc)
                )
                return
        self.strike_active = False
        self.strike_sound_deadline = None
        self.strike_hit_pending = None
        self.control = None
        self.continuous_strike_active = True
        self.continuous_strike_target = target.copy()
        self.continuous_strike_degrees = float(degrees)
        self.continuous_strike_count = 0
        self.continuous_last_beat_at = None
        self.continuous_next_beat_at = time.monotonic()
        self.continuous_swing_index = SWING_PICKUP_INDEX
        self.continuous_first_swing_hit = True
        self.continuous_current_swing_label = None
        self.continuous_current_main_beat = False
        self.continuous_current_interval_seconds = None
        self.continuous_current_event_at = None
        self.continuous_hihat_pending = False
        self.continuous_stop_requested = False
        self._reset_continuous_j7_measurement()
        self.phase = CONTINUOUS_WAIT_PHASE
        if getattr(self, "test_mode", False):
            message = (
                f"TEST DEFAULT: camera and sound bypassed; J7 "
                f"−{self.continuous_strike_degrees:g}° swing at "
                f"{self.strike_bpm:g} BPM until Stop"
            )
        elif hybrid_control.enabled(self):
            message = (
                f"SWING STRIKE: J7 −{self.continuous_strike_degrees:g}° "
                f"(first detected +{hybrid_control.SWING_DEPTH_BOOST_DEG:g}°); "
                f"starting triplet swing at {self.strike_bpm:g} BPM until Stop"
            )
        else:
            message = (
                f"FIRST DETECTED STRIKE: J7 −{self.continuous_strike_degrees:g}°; starting "
                f"triplet swing at {self.strike_bpm:g} BPM until Stop"
            )
        self.result_status.set(message)
        self.result_label.config(fg="#087f23")
        self.status.set(message)
        self._refresh_buttons()

        if hybrid_control.enabled(self):
            try:
                hybrid_control.start_swing(self, self.swing_events)
            except Exception as exc:
                self.fail('Could not start hybrid swing: '+str(exc))
        elif getattr(self, 'test_mode', False):
            snare_control.start_simulation(self)

    def _advance_continuous_striking(self, now: float) -> None:
        if hybrid_control.enabled(self):
            return  # The 500 Hz hybrid worker, not Tk, schedules releases.
        if not self.continuous_strike_active:
            return
        if self.continuous_stop_requested:
            if self.phase == CONTINUOUS_WAIT_PHASE:
                self._finish_continuous_striking()
            return
        if not self._send_pending_hihat(now):
            return
        if self.phase not in {CONTINUOUS_WAIT_PHASE, CONTINUOUS_RETURN_PHASE}:
            return
        event_at = self.continuous_next_beat_at
        if event_at is None:
            self._program_failure("FAILED: next swing strike deadline was not set")
            return
        actual = self.arm()
        distance = abs(float(actual[6]) - float(self.continuous_strike_target[6]))
        outbound_seconds = (
            distance / RIGHT_STRIKE_DOWN_SPEED + CONTINUOUS_STRIKE_LEAD_SECONDS
        )
        remaining = event_at - now
        rebound_ready = distance >= CONTINUOUS_MIN_REBOUND_RAD
        if remaining > outbound_seconds or not rebound_ready:
            next_label = ("pickup (extra before beat 1)"
                          if self.continuous_first_swing_hit else
                          self.swing_events[self.continuous_swing_index][0])
            if not rebound_ready:
                detail = (
                    f"returning at {RIGHT_STRIKE_RETURN_SPEED:.1f} rad/s; "
                    f"rebound {math.degrees(distance):.1f}°/"
                    f"{math.degrees(CONTINUOUS_MIN_REBOUND_RAD):.1f}° minimum"
                )
            elif self.phase == CONTINUOUS_RETURN_PHASE:
                detail = (
                    f"returning at {RIGHT_STRIKE_RETURN_SPEED:.1f} rad/s; "
                    f"predicted reversal in {max(0.0, remaining - outbound_seconds):.2f} s"
                )
            else:
                detail = "waiting at the full anchor"
            self.status.set(
                f"{self.strike_bpm:g} BPM swing J7 −{self.continuous_strike_degrees:g}°; "
                f"{detail}; {next_label} deadline in {max(0.0, remaining):.2f} s"
            )
            return
        interrupted = self.phase == CONTINUOUS_RETURN_PHASE
        self._begin_continuous_stage(
            CONTINUOUS_OUT_PHASE,
            self.continuous_strike_target,
            now,
        )
        if self.phase == CONTINUOUS_OUT_PHASE:
            action = "interrupted partial return" if interrupted else "left full anchor"
            self.status.set(
                f"{self.status.get()}; {action} to meet the scheduled strike deadline"
            )
            self._send_pending_hihat(now)

    def _begin_continuous_stage(self, name: str, desired, now: float) -> None:
        desired = np.asarray(desired, dtype=float)
        if name == CONTINUOUS_OUT_PHASE:
            # A return can be interrupted to start the next swing hit. Close
            # the preceding hit's encoder window before changing its label.
            self._log_continuous_j7_minimum()
        speed = (RIGHT_STRIKE_DOWN_SPEED if name == CONTINUOUS_OUT_PHASE
                 else RIGHT_STRIKE_RETURN_SPEED)
        if not self._select_strike_leg_speed(speed, name):
            return
        swing_label = self.continuous_current_swing_label
        main_beat = False
        if name == CONTINUOUS_OUT_PHASE:
            opening_pickup = self.continuous_first_swing_hit
            swing_label, main_beat, _interval = self._take_next_swing_event()
            self.continuous_current_main_beat = main_beat
            self.continuous_current_event_at = (
                None if opening_pickup else self.continuous_next_beat_at
            )
            self.continuous_hihat_pending = bool(
                main_beat and getattr(self, "hihat", None) is not None
            )
        self.control = StrikeControl(desired, self.lower, self.upper, now)
        self.phase = name
        self.goal = desired.copy()
        self.target_point = self.hill_ik.position(desired)
        self.target_offset = self.target_point - self.ik.origin_tcp
        self.target_inside = True
        if name == CONTINUOUS_RETURN_PHASE:
            self.bus.set_right_joint7_position(desired[6])
        else:
            self.bus.set_positions(desired)
        self.last_command = 0.0
        if name == CONTINUOUS_OUT_PHASE:
            self.continuous_strike_count += 1
            self._start_continuous_j7_measurement()
            detail = ("hi-hat ignored (test mode)"
                      if getattr(self, "test_mode", False) else
                      ("hi-hat unchanged on swing extra"
                       if not main_beat else
                       "hi-hat command scheduled at the main-beat deadline"))
            self.status.set(
                f"{self.strike_bpm:g} BPM swing {swing_label}, strike {self.continuous_strike_count}: J7 "
                f"−{self.continuous_strike_degrees:g}°; {detail}"
            )
        else:
            anchor_name = ("recording endpoint" if getattr(self, "test_mode", False)
                           else "pink-zone pose")
            self.status.set(
                f"{self.strike_bpm:g} BPM swing {swing_label}, strike "
                f"{self.continuous_strike_count}: target reached; "
                f"returning to the {anchor_name}"
            )

    def _reset_continuous_j7_measurement(self) -> None:
        self.continuous_j7_lowest_encoder_rad = None
        self.continuous_j7_measurement_anchor_rad = None
        self.continuous_j7_measurement_strike_count = None
        self.continuous_j7_measurement_label = None
        self.continuous_j7_measurement_reached = False

    def _start_continuous_j7_measurement(self) -> None:
        """Open one encoder-minimum window for the newly commanded swing hit."""
        self._reset_continuous_j7_measurement()
        plan = getattr(self, "strike_plan", None)
        if plan is None:
            return
        anchor = np.asarray(plan.anchor_joints, dtype=float)
        if anchor.shape != (7,) or not np.isfinite(anchor).all():
            return
        self.continuous_j7_measurement_anchor_rad = float(anchor[6])
        self.continuous_j7_measurement_strike_count = int(
            self.continuous_strike_count
        )
        self.continuous_j7_measurement_label = str(
            self.continuous_current_swing_label
        )
        self._sample_continuous_j7_minimum()

    def _sample_continuous_j7_minimum(self) -> None:
        """Retain the lowest sampled J7 encoder value during the current hit."""
        if getattr(self, "continuous_j7_measurement_strike_count", None) is None:
            return
        try:
            actual = np.asarray(self.arm(), dtype=float)
            value = float(actual[6])
        except (AttributeError, IndexError, TypeError, ValueError):
            return
        if actual.shape != (7,) or not math.isfinite(value):
            return
        lowest = getattr(self, "continuous_j7_lowest_encoder_rad", None)
        if lowest is None or value < lowest:
            self.continuous_j7_lowest_encoder_rad = value

    def _log_continuous_j7_minimum(self) -> None:
        """Print one completed swing hit's lowest observed J7 encoder sample."""
        count = getattr(self, "continuous_j7_measurement_strike_count", None)
        if count is None:
            return
        self._sample_continuous_j7_minimum()
        lowest = getattr(self, "continuous_j7_lowest_encoder_rad", None)
        anchor = getattr(self, "continuous_j7_measurement_anchor_rad", None)
        reached = getattr(self, "continuous_j7_measurement_reached", False)
        label = getattr(self, "continuous_j7_measurement_label", "swing hit")
        if reached and lowest is not None and anchor is not None:
            down_degrees = math.degrees(anchor - lowest)
            encoder_degrees = math.degrees(lowest)
            commanded_degrees = getattr(self, "continuous_strike_degrees", None)
            command_text = (
                f"{commanded_degrees:g} deg"
                if commanded_degrees is not None else "unknown"
            )
            print(
                f"J7 STRIKE LOW strike {count} [{label}]: "
                f"{down_degrees:.2f} deg down from anchor; "
                f"lowest J7 encoder: {encoder_degrees:.2f} deg; "
                f"commanded: {command_text}",
                flush=True,
            )
        self._reset_continuous_j7_measurement()

    def _take_next_swing_event(self):
        """Return and advance the next ride event in the repeating swing bar."""
        index = self.continuous_swing_index
        label, main_beat, interval = self.swing_events[index]
        if self.continuous_first_swing_hit:
            label = "pickup (extra before beat 1)"
            self.continuous_first_swing_hit = False
        self.continuous_swing_index = (index + 1) % len(self.swing_events)
        self.continuous_current_swing_label = label
        self.continuous_current_interval_seconds = interval
        return label, main_beat, interval

    def _send_pending_hihat(self, now: float) -> bool:
        """Send a main-beat hi-hat command at its beat, not at early J7 reversal."""
        if not self.continuous_hihat_pending:
            return True
        event_at = self.continuous_current_event_at
        if event_at is None or now < event_at:
            return True
        self.continuous_hihat_pending = False
        hihat = getattr(self, "hihat", None)
        if hihat is None:
            return True
        try:
            command = hihat.send_beat()
        except Exception as exc:
            self.hihat_fault_detail = str(exc)
            self.status.set(
                "ESP32 hi-hat beat command failed — stopping, centering, "
                "and relaxing: " + str(exc)
            )
            self._request_continuous_stop()
            return False
        amount = getattr(self.hihat, 'target_degrees', MOTOR2_TARGET_DEGREES)
        if not isinstance(amount, (int, float)):
            amount = MOTOR2_TARGET_DEGREES
        detail = f"CLOSE {amount}°" if command in (b"C", b"B") else "OPEN 0°"
        self.status.set(
            f"{self.strike_bpm:g} BPM swing {self.continuous_current_swing_label}: "
            f"hi-hat motor 2 {detail} on the main-beat deadline"
        )
        return True

    def _request_continuous_stop(self) -> None:
        if not self.continuous_strike_active:
            return
        if hybrid_control.enabled(self):
            hybrid_control.request_stop(self)
            return
        self.continuous_stop_requested = True
        snare_control.finish(self)
        self._stop_hihat_sequence()
        self.center_relax_button.config(state="disabled")
        if self.phase == CONTINUOUS_WAIT_PHASE:
            self._finish_continuous_striking()
        else:
            self.status.set(
                "Stop requested — finishing the current fast return, then "
                "centering and relaxing"
            )

    def _finish_continuous_striking(self) -> None:
        if not self.continuous_strike_active:
            return
        snare_control.finish(self)
        if not snare_control.finished(self):
            return
        self.continuous_strike_active = False
        self.continuous_stop_requested = False
        self.control = None
        if not self._restore_strike_speed(f"{self.strike_bpm:g} BPM striking stopped by user"):
            return
        try:
            self.begin_stage(CENTER_RELAX_PHASE, self.center_goal)
        except Exception as exc:
            self.fail(f"{self.strike_bpm:g} BPM Stop + Center command failed: " + str(exc))
            return
        self.status.set(
            f"{self.strike_bpm:g} BPM striking stopped — returning to customized right center, "
            "then relaxing"
        )

    def center_relax(self) -> None:
        """Cancel the active workflow, move from the live pose to center, then relax."""
        if getattr(self, 'emergency_latched', False):
            self.status.set('Emergency stop latched; restart required, no centering motion')
            return
        self._cancel_hihat_calibration()
        if getattr(self, 'phase', None) == hybrid_control.FAULT:
            if getattr(self, 'single_run_mode', False):
                hybrid_control.recover_to_center(self)
                return
            self.status.set('Hybrid controller fault: use Emergency Relax or physical stop, then restart')
            return
        if hybrid_control.engaged(self):
            hybrid_control.request_stop(self)
            return
        if not self.hardware or not self.bus:
            self.status.set("Center + Relax is unavailable in offline preview")
            return
        if not self.bus.active:
            self.status.set("Center + Relax is unnecessary: right motors are already disabled")
            return
        if not self.bus.fresh():
            self.status.set("Cannot center: waiting for fresh right-arm encoder feedback")
            return
        if (getattr(self, "hardware_test_mit_active", False)
                and not self._hardware_test_session_ready()):
            self.status.set(
                "Center + Relax is unavailable during the MIT freefall/rebound; "
                "wait for J7 to return or press EMERGENCY RELAX"
            )
            return
        if self.setup is not None:
            self.status.set(
                "Cannot center during motor setup; wait for setup or use Emergency Relax"
            )
            return
        if not self.zone.contains(
                self.planned_tcp(self.center_goal), MEMBERSHIP_BUFFER_M):
            self.status.set(
                "Cannot center: configured right center is outside right_zones/zone1.json"
            )
            return
        if getattr(self, "continuous_strike_active", False):
            self._request_continuous_stop()
            return
        if (self.phase in self.RETURN_PHASES
                or self.phase in {"FAULT RECENTERING", "ZONE RECENTERING", "RELAXING"}):
            self.status.set("Center + Relax is already in progress")
            return

        if getattr(self, "hardware_test_session", None) is not None:
            if not self._hardware_test_session_ready():
                self.status.set("Wait for fresh stationary MIT hold before centering")
                return
            try:
                self._stop_hardware_test_session()
                self.hardware_test_csp_enabled_at = self.bus.restore_right_joint7_csp(
                    float(self.hardware_test_anchor[6]))
                self.hardware_test_restore_deadline = time.monotonic() + 2.0
                self.hardware_test_last_enable_retry = time.monotonic()
                self.phase = HARDWARE_TEST_RESTORE_PHASE
                self.status.set("Leaving MIT hold; confirming position mode before centering")
            except Exception as exc:
                self.fail("Could not leave MIT session: " + str(exc))
            return

        self.setup = None
        self.hold_until = None
        self.search_active = False
        self.pending_detection_frame_id = None
        self.gripper_close_until = None
        self._clear_alignment_state()
        if not self._restore_playback_speed("Center + Relax requested"):
            return
        if not self._restore_strike_speed("Center + Relax requested"):
            return
        try:
            self.begin_stage(CENTER_RELAX_PHASE, self.center_goal)
        except Exception as exc:
            self.fail("Center + Relax command failed: " + str(exc))
            return
        self.status.set(
            "Center + Relax: BOTH arms return together; each relaxes at its own verified center"
            if getattr(self, 'dual', None) is not None else
            "Center + Relax requested — returning from the current pose to "
            "customized right center, then disabling the right arm"
        )

    def _restore_strike_speed(self, context: str) -> bool:
        speed_fast = getattr(self, "strike_speed_fast", False)
        feedback_fast = getattr(self, "strike_feedback_fast", False)
        if not speed_fast and not feedback_fast:
            self.strike_selected_speed = None
            return True
        try:
            if self.bus and self.bus.active:
                if feedback_fast:
                    self.bus.set_right_joint7_feedback_rate(None)
                if speed_fast:
                    self.bus.set_right_joint7_speed(SPEED)
            self.strike_feedback_fast = False
            self.strike_speed_fast = False
            self.strike_selected_speed = None
            return True
        except Exception as exc:
            self.strike_active = False
            failure = context + "; could not restore normal J7 speed: " + str(exc)
            self.result_status.set("FAILED: " + failure)
            self.result_label.config(fg="#b00020")
            print("FAILED: " + failure, flush=True)
            if getattr(self, "hardware_test_mode", False):
                self.fail(failure)
            else:
                self.relax("STOPPED: " + failure + "; right motors disabled")
            return False

    def _restore_playback_speed(self, context: str) -> bool:
        if not self._stop_smooth_playback():
            self.fail(self.smooth_playback_cleanup_error)
            return False
        if not getattr(self, "playback_speed_fast", False):
            return True
        try:
            if self.bus and self.bus.active:
                self.bus.set_right_arm_speed(SPEED)
            self.playback_speed_fast = False
            return True
        except Exception as exc:
            self.playback_active = False
            failure = context + "; could not restore normal right-arm speed: " + str(exc)
            self.result_status.set("FAILED: " + failure)
            self.result_label.config(fg="#b00020")
            print("FAILED: " + failure, flush=True)
            if getattr(self, "hardware_test_mode", False):
                self.fail(failure)
            else:
                self.relax("STOPPED: " + failure + "; right motors disabled")
            return False

    def _plan_next_candidate(self) -> None:
        if not self.alignment_active:
            return
        candidate = self.hill.candidate_offset()
        direction = self.hill.direction_name
        step = self.hill.step
        token = self.planner_token + 1
        self.planner_token = token
        future = self.planner_executor.submit(
            solve_playback_coordinate,
            self.hill_ik,
            candidate.copy(),
            self.hill.anchor_joints.copy(),
            self.playback_fixed_joint2,
        )
        self.planner_future = (token, future)
        self.pending_candidate_offset = candidate.copy()
        self.pending_candidate_result = None
        self.phase = "HILL PLANNING"
        self._reset_hold_controller()
        self.status.set(
            f"Holding endpoint — planning {direction} by {step:.4f} m, "
            "validating candidate/return paths, and retaining recorded J2"
        )

    def _clear_alignment_state(self) -> None:
        self._stop_smooth_playback()
        self._stop_hardware_test_session()
        super()._clear_alignment_state()
        self.playback_active = False
        self.playback_started_at = None
        self.playback_follower = None
        self.strike_active = False
        self.strike_plan = None
        self.strike_index = 0
        self.strike_attempt_motion_seen = False
        self.strike_hit_pending = None
        self.strike_attempt_started_at = None
        self.strike_attempt_returned_at = None
        self.strike_sound_deadline = None
        self.hardware_test_session = None
        self.hardware_test_completed = 0
        self.hardware_test_anchor = None
        self.hardware_test_target = None
        self.hardware_test_degrees_value = None
        self.hardware_test_strike_count = 0
        self.hardware_test_sound_count = 0
        self.hardware_test_mit_active = False
        self.hardware_test_last_mit_command = 0.0
        self.hardware_test_csp_enabled_at = None
        self.hardware_test_restore_deadline = None
        self.hardware_test_last_enable_retry = 0.0
        self.hardware_test_hold_fault_message = None
        self.hardware_test_hold_delivery_error = None
        self.hardware_test_workflow_fault_message = None
        self.hardware_test_workflow_hold_joints = None
        self.hardware_test_isolated_motor = None
        self.hardware_test_last_workflow_hold = 0.0
        sound_log = getattr(self, "hardware_test_sound_status", None)
        if sound_log is not None:
            sound_log.set("ST7 sound log: waiting for the manual strike section")
        self._cancel_continuous_striking()

    def _stop_hihat_sequence(self) -> None:
        sync = getattr(self, 'hihat_sync', None)
        if sync is not None:
            sync.stop()
        hihat = getattr(self, "hihat", None)
        if hihat is None:
            return
        try:
            hihat.stop_sequence()
        except Exception as exc:
            self.hihat_fault_detail = str(exc)

    def _cancel_continuous_striking(self) -> None:
        snare_control.finish(self)
        self._log_continuous_j7_minimum()
        self._stop_hihat_sequence()
        self.continuous_strike_active = False
        self.continuous_strike_target = None
        self.continuous_strike_degrees = None
        self.continuous_strike_count = 0
        self.continuous_last_beat_at = None
        self.continuous_next_beat_at = None
        self.continuous_swing_index = SWING_PICKUP_INDEX
        self.continuous_first_swing_hit = True
        self.continuous_current_swing_label = None
        self.continuous_current_main_beat = False
        self.continuous_current_interval_seconds = None
        self.continuous_current_event_at = None
        self.continuous_hihat_pending = False
        self.continuous_stop_requested = False
        self._reset_continuous_j7_measurement()
        button = getattr(self, "center_relax_button", None)
        if button is not None:
            button.config(text="CENTER + RELAX")

    def _program_failure(self, message: str) -> None:
        if getattr(self, 'dual', None) is not None and self.phase in dual_control.LEFT_PHASES:
            self.dual.fault(message)
            return
        self._cancel_hihat_calibration()
        if hybrid_control.engaged(self):
            if self.phase == hybrid_control.FAULT:
                return
            hybrid_control.request_stop(self, str(message))
            return
        if getattr(self, 'recording_only', False) and self.bus and self.bus.active:
            recording_only_control.finish(self, message)
            return
        if getattr(self, "hardware_test_mode", False):
            self.fail(message)
            return
        if getattr(self, "hardware_test_mit_active", False):
            self.fail(message)
            return
        self._cancel_continuous_striking()
        if not self._restore_playback_speed(str(message)):
            return
        if not self._restore_strike_speed(str(message)):
            return
        super()._program_failure(message)

    def fail(self, message):
        if getattr(self, 'emergency_latched', False):
            self.status.set('Stopped; restart required: '+str(message))
            return
        self._cancel_hihat_calibration()
        if (getattr(self, 'single_run_mode', False) and self.bus and self.bus.active
                and ('STALL:' in str(message) or 'too hot' in str(message).lower())):
            self.relax('MAJOR FAULT: '+str(message))
            return
        if (getattr(self, 'dual', None) is not None
                and self.phase in dual_control.LEFT_PHASES | {dual_control.FAULT}):
            self.dual.fault(message)
            return
        # During return use the original right-arm recovery below. Sending a
        # recoverable return error through dual.fault() replaces J7's center
        # goal with its current angle while J1-J6 retain their center targets.
        # That changes the path and can leave the wrist held off-center forever.
        if hybrid_control.engaged(self):
            if str(message).startswith('STALL:'):
                self.relax('Hybrid stall: emergency relaxation — '+str(message))
                return
            hybrid_control.fault(self, message)
            return
        if getattr(self, 'recording_only', False) and self.bus and self.bus.active:
            recording_only_control.finish(self, message)
            return
        if not self._stop_smooth_playback():
            message = str(message) + "; " + self.smooth_playback_cleanup_error
        self._stop_hardware_test_session()
        message = str(message)
        if getattr(self, "hardware_test_mode", False) and message.startswith("STALL:"):
            # A stalled drive may be pushing into an obstruction and heating.
            # This is the sole automatic whole-arm shutdown path in hardware test.
            self.hardware_test_mit_active = False
            super().fail(message)
            return
        if (getattr(self, "hardware_test_mode", False)
                and self._fault_motor_number(message) is not None):
            self.hardware_test_mit_active = False
            self._enter_hardware_test_workflow_fault(message)
            return
        if getattr(self, "hardware_test_mit_active", False):
            self._enter_hardware_test_hold_fault(message)
            return
        if getattr(self, "hardware_test_mode", False):
            self._enter_hardware_test_workflow_fault(message)
            return
        self._cancel_continuous_striking()
        if not self._restore_playback_speed(str(message)):
            return
        if not self._restore_strike_speed(str(message)):
            return
        super().fail(message)

    @staticmethod
    def _fault_motor_number(message: str) -> int | None:
        if re.search(r"\bleft\s+motor\s+[1-8]\b", message, re.I):
            return None
        match = re.search(
            r"(?:\bright\s+motor|^motor)\s+([1-8])\b", message, re.I
        )
        return None if match is None else int(match.group(1))

    def _enter_hardware_test_workflow_fault(self, message: str) -> None:
        """Lock motion, isolate a named failed drive, and keep healthy drives held."""
        if self.phase == HARDWARE_TEST_WORKFLOW_FAULT_PHASE:
            return
        hold = None
        try:
            candidate = np.asarray(self.arm(), dtype=float)
            if candidate.shape == (7,) and np.isfinite(candidate).all():
                hold = candidate.copy()
        except Exception:
            pass
        self.setup = None
        self.control = None
        self.hold_until = None
        self.playback_active = False
        self.alignment_active = False
        self._cancel_planning()
        self.strike_active = False
        self._cancel_continuous_striking()
        self.hardware_test_mit_active = False
        self.hardware_test_workflow_fault_message = str(message)
        self.hardware_test_workflow_hold_joints = hold
        self.hardware_test_isolated_motor = self._fault_motor_number(message)
        self.hardware_test_hold_delivery_error = None

        delivery_errors = []
        bus = getattr(self, "bus", None)
        if bus and bus.active:
            operations = []
            if self.hardware_test_isolated_motor is not None:
                operations.append((
                    "motor isolation",
                    lambda: bus.isolate_control_motor(self.hardware_test_isolated_motor),
                ))
            if hold is not None:
                operations.append(("healthy-joint hold", lambda: bus.set_positions(hold)))
            if (getattr(self, "gripper_closed_latched", False)
                    and self.hardware_test_isolated_motor != 8):
                operations.append((
                    "gripper hold", lambda: bus.set_gripper(RIGHT_GRIPPER_CLOSED)
                ))
            if getattr(self, "playback_speed_fast", False):
                operations.append((
                    "playback-speed restore", lambda: bus.set_right_arm_speed(SPEED)
                ))
            if getattr(self, "strike_feedback_fast", False):
                operations.append((
                    "J7 feedback restore", lambda: bus.set_right_joint7_feedback_rate(None)
                ))
            if (getattr(self, "strike_speed_fast", False)
                    and self.hardware_test_isolated_motor != 7):
                operations.append((
                    "J7 speed restore", lambda: bus.set_right_joint7_speed(SPEED)
                ))
            for label, operation in operations:
                try:
                    operation()
                except Exception as exc:
                    delivery_errors.append(f"{label}: {exc}")
        if delivery_errors:
            self.hardware_test_hold_delivery_error = "; ".join(delivery_errors)
        self.playback_speed_fast = False
        self.strike_feedback_fast = False
        self.strike_speed_fast = False
        self.strike_selected_speed = None
        self.phase = HARDWARE_TEST_WORKFLOW_FAULT_PHASE
        self.hardware_test_last_workflow_hold = time.monotonic()

        if self.hardware_test_isolated_motor is None:
            action = "healthy drives remain powered at their last commanded poses"
            prefix = "HARDWARE-TEST POWERED WORKFLOW FAULT"
        else:
            action = (
                f"motor {self.hardware_test_isolated_motor} disabled in isolation; "
                "the other drives remain powered at their captured poses"
            )
            prefix = "HARDWARE-TEST ISOLATED MOTOR FAULT"
        result = f"{prefix}: {action}; workflow locked — {message}"
        if self.hardware_test_hold_delivery_error:
            result += (
                "; hold/isolation delivery problem: "
                + self.hardware_test_hold_delivery_error
                + "; use the physical emergency stop if motion is uncontrolled"
            )
        if hasattr(self, "result_status"):
            self.result_status.set(result)
            self.result_label.config(fg="#b00020")
        if hasattr(self, "status"):
            self.status.set(result)
        print(result, flush=True)
        if hasattr(self, "center_relax_button"):
            self._refresh_buttons()

    def _enter_hardware_test_hold_fault(self, message: str) -> None:
        """Lock out strikes and hold J7 without guessing its active run mode."""
        self.setup = None
        self.control = None
        self.hold_until = None
        self.phase = HARDWARE_TEST_HOLD_FAULT_PHASE
        self.hardware_test_hold_fault_message = str(message)
        self.hardware_test_hold_delivery_error = None
        anchor = getattr(self, "hardware_test_anchor", None)
        try:
            if self.bus and self.bus.active and anchor is not None:
                if self.bus.right_joint7_mode_readback() == 5:
                    self.bus.reassert_right_joint7_csp_hold(float(anchor[6]))
                else:
                    self.bus.hold_right_joint7_unknown_mode(float(anchor[6]))
                self.hardware_test_last_mit_command = time.monotonic()
        except Exception as exc:
            # Do not turn a lost hold refresh into a whole-arm disable. The
            # drives retain their last accepted commands; the physical E-stop
            # remains the fallback if the mechanism is moving uncontrollably.
            self.hardware_test_hold_delivery_error = str(exc)
        result = (
            "MIT HARDWARE-TEST HOLD FAULT: right arm remains powered at the "
            f"pink-zone pose; strikes locked — {self.hardware_test_hold_fault_message}"
        )
        if self.hardware_test_hold_delivery_error:
            result += (
                "; hold refresh failed (drives retain their last command): "
                + self.hardware_test_hold_delivery_error
                + "; use the physical emergency stop if motion is uncontrolled"
            )
        if hasattr(self, "result_status"):
            self.result_status.set(result)
            self.result_label.config(fg="#b00020")
        if hasattr(self, "status"):
            self.status.set(result)
        print(result, flush=True)
        self._refresh_buttons()

    def _maintain_hardware_test_fault_hold(self, now: float) -> None:
        """Refresh both possible J7 hold protocols while mode is unconfirmed."""
        if self.phase != HARDWARE_TEST_HOLD_FAULT_PHASE:
            return
        anchor = getattr(self, "hardware_test_anchor", None)
        if (not self.bus or not self.bus.active or anchor is None
                or now - self.hardware_test_last_mit_command
                < HARDWARE_TEST_FAULT_HOLD_INTERVAL):
            return
        try:
            if self.bus.right_joint7_mode_readback() == 5:
                self.bus.reassert_right_joint7_csp_hold(float(anchor[6]))
            else:
                self.bus.hold_right_joint7_unknown_mode(float(anchor[6]))
            self.hardware_test_last_mit_command = now
            self.hardware_test_hold_delivery_error = None
        except Exception as exc:
            # Never auto-relax the other right-arm joints because a J7 hold
            # refresh or readback was lost.
            self.hardware_test_hold_delivery_error = str(exc)
            self.hardware_test_last_mit_command = now

    def _maintain_hardware_test_workflow_fault(self, now: float) -> None:
        """Refresh CSP holds for healthy drives after an isolated workflow fault."""
        if (self.phase != HARDWARE_TEST_WORKFLOW_FAULT_PHASE
                or not self.bus or not self.bus.active
                or now - self.hardware_test_last_workflow_hold
                < HARDWARE_TEST_FAULT_HOLD_INTERVAL):
            return
        try:
            hold = self.hardware_test_workflow_hold_joints
            if hold is not None:
                self.bus.set_positions(hold)
            if (self.gripper_closed_latched
                    and self.hardware_test_isolated_motor != 8):
                self.bus.set_gripper(RIGHT_GRIPPER_CLOSED)
            self.hardware_test_hold_delivery_error = None
        except Exception as exc:
            self.hardware_test_hold_delivery_error = str(exc)
        self.hardware_test_last_workflow_hold = now

    def encoder_text(self):
        feedback = getattr(self.bus, 'motor8_feedback', {}).get('left') if self.bus else None
        return super().encoder_text() + '\nLeft ' + gripper_feedback_text(feedback)

    def relaxed_feedback_sides(self):
        return ('left', 'right')

    def control_description(self):
        return 'RIGHT PLAYBACK + LEFT CENTER HOLD'

    def safety(self):
        if getattr(self, 'emergency_latched', False):
            return  # No motion controller survives emergency shutdown.
        if getattr(self, 'dual', None) is not None:
            if self.phase == dual_control.FAULT:
                return
            if self.dual.returning and 'right' in self.bus.center_disabled:
                return  # Left finishes independently; right is already off.
            if self.dual.returning:
                if not self.bus.fresh():
                    raise RuntimeError('Fresh feedback lost during right-arm centering')
                if any(self.bus.states['right', i][1] == 0 for i in range(1, 9)):
                    raise RuntimeError('Right motor stopped before verified center')
        if hybrid_control.engaged(self):
            inside = self.zone.contains(self.tcp(), MEMBERSHIP_BUFFER_M)
            self.zone_status.set('Zone: INSIDE' if inside else 'Zone: OUTSIDE')
            if not inside and self.phase != hybrid_control.FAULT:
                hybrid_control.fault(self, 'Right TCP left zone1 during hybrid control')
            return  # Never start inherited CSP recentering while J7 is in MIT.
        if (self.phase in {
                    HARDWARE_TEST_HOLD_FAULT_PHASE,
                    HARDWARE_TEST_WORKFLOW_FAULT_PHASE,
                }
                and self.bus and self.bus.active):
            inside = self.zone.contains(self.tcp(), MEMBERSHIP_BUFFER_M)
            self.zone_status.set("Zone: INSIDE" if inside else "Zone: OUTSIDE")
            return
        if (getattr(self, "hardware_test_mode", False)
                and self.bus and self.bus.active
                and not self.zone.contains(self.tcp(), MEMBERSHIP_BUFFER_M)):
            detail = (
                "right TCP left zone1 during the J7 MIT freefall/rebound"
                if getattr(self, "hardware_test_mit_active", False) else
                "right TCP left zone1 during the hardware-test workflow"
            )
            self.fail(detail)
            return
        if (getattr(self, "hardware_test_mit_active", False)
                and self.bus and self.bus.active):
            inside = self.zone.contains(self.tcp(), MEMBERSHIP_BUFFER_M)
            if not inside:
                self.fail("right TCP left zone1 during the J7 MIT freefall/rebound")
                return
        if (self.playback_speed_fast and self.bus and self.bus.active
                and not self.zone.contains(self.tcp(), MEMBERSHIP_BUFFER_M)):
            if not self._restore_playback_speed(
                    "right TCP left zone1 during recording playback"):
                return
        if (self.strike_speed_fast and self.bus and self.bus.active
                and not self.zone.contains(self.tcp(), MEMBERSHIP_BUFFER_M)):
            self._cancel_continuous_striking()
            if not self._restore_strike_speed("right TCP left zone1 during cymbal strike"):
                return
        super().safety()

    def emergency_relax(self, message='Operator emergency stop'):
        from .emergency import relax
        relax(self, message)

    def relax(self, message=None):
        dual = getattr(self, 'dual', None)
        fatal = 'STALL:' in str(message or '') or str(message or '').startswith('Hybrid stall:') or 'too hot' in str(message or '').lower()
        if fatal and getattr(self, 'single_run_mode', False):
            self.emergency_relax(message)
            return
        if dual is not None and self.bus and self.bus.active and not fatal:
            self.center_relax()
            return
        if dual is not None:
            dual.stop()
        self._cancel_hihat_calibration()
        if getattr(self, 'single_run_mode', False) and self.bus and self.bus.active:
            fatal = 'STALL:' in str(message or '') or str(message or '').startswith('Hybrid stall:')
            fatal = fatal or 'too hot' in str(message or '').lower()
            if isinstance(self.bus, PlaybackMotors):
                if fatal:
                    self.bus.emergency_relax_reason = str(message)
                else:
                    try:
                        self.bus.center_evidence()
                    except RuntimeError:
                        self.center_relax()
                        return
        if getattr(self, 'recording_only', False) and self.bus and self.bus.active:
            recording_only_control.finish(self, message or 'Controlled stop requested')
            return
        self._stop_smooth_playback()
        self._stop_hardware_test_session()
        hybrid_control.close(self)
        self.hybrid_stopping = self.hybrid_restoring = False
        hihat = getattr(self, "hihat", None)
        calibration = getattr(self, 'hihat_calibration', None)
        if hihat is not None and not (calibration is not None and calibration.busy):
            try:
                hihat.release_all()
            except Exception as exc:
                self.hihat_fault_detail = str(exc)
        self._cancel_continuous_striking()
        self.strike_active = False
        super().relax(message)
        if isinstance(self.bus, PlaybackMotors):
            self.bus.emergency_relax_reason = None
        self.hardware_test_mit_active = False
        self.hardware_test_last_mit_command = 0.0
        self.hardware_test_csp_enabled_at = None
        self.hardware_test_restore_deadline = None
        self.hardware_test_last_enable_retry = 0.0
        self.hardware_test_hold_fault_message = None
        self.hardware_test_hold_delivery_error = None
        self.hardware_test_workflow_fault_message = None
        self.hardware_test_workflow_hold_joints = None
        self.hardware_test_isolated_motor = None
        self.hardware_test_last_workflow_hold = 0.0
        if getattr(self, "test_mode", False):
            # The simulated bus disables synchronously; there is no physical
            # feedback confirmation phase to wait for or describe as motors.
            self.relax_at = None
            self.phase = "RELAXED"
            if message is not None:
                self.status.set(message)
        if not self.bus or not self.bus.active:
            self.strike_speed_fast = False
            self.strike_selected_speed = None
            self.strike_feedback_fast = False
            self.playback_speed_fast = False

    def request_safe_close(self):
        """First close requests center; repeat or deadline requests emergency exit."""
        self._cancel_recording_preflight()
        if getattr(self, 'close_requested', False):
            self.force_close = True
            self.emergency_relax('Repeated close/Ctrl-C: emergency shutdown')
            self.root.quit()
            return
        self.close_requested = True
        self.close_deadline = time.monotonic() + 35.
        print('CLOSE REQUEST: center then exit; repeat Ctrl-C for emergency disable; 35 s deadline', flush=True)
        self.root.after(50, self._safe_close_tick)
        self.center_relax()

    def _safe_close_tick(self):
        if time.monotonic() >= getattr(self, 'close_deadline', float('inf')):
            self.force_close = True
            self.emergency_relax('Shutdown deadline: emergency disable attempt')
            print('Exiting after emergency disable attempt; if disable is unconfirmed, USE PHYSICAL POWER CUTOFF', flush=True)
            self.root.quit()
            return
        calibration = getattr(self, 'hihat_calibration', None)
        if (not self.bus.active and self.phase != 'RELAXING'
                and (getattr(self, 'emergency_latched', False)
                     or not (calibration is not None and calibration.busy))):
            self.root.quit()
            return
        if self.phase not in self.RETURN_PHASES | {'RELAXING', 'FAULT RECENTERING', 'ZONE RECENTERING'}:
            self.center_relax()
        self.root.after(100, self._safe_close_tick)

    def run(self):
        # Parent run() relaxes/closes the bus immediately after mainloop exits.
        # Join the recording writer first on window close, signal, or exception.
        mainloop = self.root.mainloop
        def recording_safe_mainloop(*args, **kwargs):
            try:
                while True:
                    try:
                        result = mainloop(*args, **kwargs)
                    except Exception as exc:
                        if not (getattr(self, 'single_run_mode', False) and self.bus.active):
                            raise
                        self.fail('GUI loop failed: '+str(exc))
                        self.request_safe_close()
                        continue
                    calibration = getattr(self, 'hihat_calibration', None)
                    if getattr(self, 'force_close', False):
                        return result
                    if not (getattr(self, 'single_run_mode', False) and
                            (self.bus.active or (not getattr(self, 'emergency_latched', False)
                             and calibration is not None and calibration.busy))):
                        return result
                    self.request_safe_close()
            finally:
                if getattr(self, "dual", None) is not None and not getattr(self, 'emergency_latched', False):
                    self.dual.stop()
                if not getattr(self, 'emergency_latched', False):
                    self._stop_smooth_playback()
                    hybrid_control.close(self)
                if getattr(self, 'recording_only', False) and self.bus and self.bus.active:
                    recording_only_control.finish(self, 'Window closed: controlled center return')
        self.root.mainloop = recording_safe_mainloop
        try:
            super().run()
        finally:
            self.root.mainloop = mainloop
            self._cancel_recording_preflight()
            self.audio_receiver.close()
            monitor = getattr(self, 'hihat_sound_monitor', None)
            if monitor is not None:
                monitor.close()
            calibration = getattr(self, 'hihat_calibration', None)
            if calibration is not None:
                calibration.close()
            hihat = getattr(self, "hihat", None)
            if hihat is not None:
                hihat.close()


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--hardware", action="store_true")
    mode.add_argument("--test", action="store_true")
    mode.add_argument("--hardwaretest", action="store_true")
    parser.add_argument("--detection-socket", required=True)
    parser.add_argument("--audio-socket", required=True)
    parser.add_argument("--hihat-audio-socket", help="hi-hat detector socket (required for normal hardware calibration)")
    parser.add_argument("--esp-port", default=DEFAULT_ESP_PORT)
    parser.add_argument("--recording", type=Path, default=DEFAULT_RECORDING)
    parser.add_argument("--left-recording", type=Path, default=dual_control.DEFAULT_LEFT_RECORDING)
    parser.add_argument("--recording-only", action="store_true")
    parser.add_argument("--auto-run", action="store_true")
    parser.add_argument('--verify-playback', action='store_true',
                        help='authorized physical dual-arm playback check; skip alignment/beat and return to center')
    parser.add_argument('--verify-swing', action='store_true',
                        help='authorized physical acceptance run: auto RUN, 5 s swing, verified center/relax')
    parser.add_argument("--mit-fall-kd", type=float, default=StrikeSettings.fall_kd)
    parser.add_argument("--mit-inertia", type=float, default=StrikeSettings.inertia)
    parser.add_argument("--mit-brake-accel", type=float, default=StrikeSettings.brake_accel)
    parser.add_argument("--mit-latency-ms", type=float, default=StrikeSettings.command_latency*1000)
    args = parser.parse_args()
    if args.verify_playback and (not args.hardware or args.recording_only or args.verify_swing):
        parser.error('--verify-playback requires normal --hardware and no other verification mode')
    if args.verify_swing and (not args.hardware or args.recording_only):
        parser.error('--verify-swing requires normal --hardware')
    try:
        settings = StrikeSettings(fall_kd=args.mit_fall_kd, inertia=args.mit_inertia,
                                  brake_accel=args.mit_brake_accel,
                                  command_latency=args.mit_latency_ms/1000)
    except ValueError as exc:
        parser.error(str(exc))
    rclpy.init(args=[])
    try:
        app = App(
            args.hardware or args.hardwaretest,
            args.detection_socket,
            args.audio_socket,
            args.recording,
            args.esp_port,
            test_mode=args.test,
            hardware_test_mode=args.hardwaretest,
            hardware_test_settings=settings,
            recording_only=args.recording_only, auto_run=args.auto_run,
            hihat_audio_socket=args.hihat_audio_socket,
            left_recording_path=args.left_recording,
        )
        if args.verify_swing:
            from .beat_evidence import BeatEvidence
            app.beat_evidence = BeatEvidence(app)
        if args.verify_playback:
            from .playback_evidence import PlaybackEvidence
            app.playback_evidence = PlaybackEvidence(app)
        app.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
