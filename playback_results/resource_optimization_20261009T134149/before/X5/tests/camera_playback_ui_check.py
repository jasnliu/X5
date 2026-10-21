"""Offline UI integration for the separate camera-playback program."""
import argparse
from pathlib import Path
import tempfile

import numpy as np
import rclpy

from camera_playback.app import App
from camera_search.planner import INITIAL_OFFSET, solve_search_coordinate
from cartesian_goal.ik import CartesianIK
from centering.motors import SPEED
from motion_recording.recording import MotionRecording
from safe_zone.geometry import Model, RIGHT_TCP, Zone


ROOT = Path(__file__).resolve().parents[1]
model = Model(ROOT / "model/openarmx.urdf")
zone = Zone.load(ROOT / "right_zones/zone1.json", model.digest, RIGHT_TCP)
joints = {joint.get("name"): joint for joint in model.joints}
limits = [joints[f"openarmx_right_joint{i}"].find("limit") for i in range(1, 8)]
lower = np.array([float(limit.get("lower")) for limit in limits])
upper = np.array([float(limit.get("upper")) for limit in limits])
center = np.zeros(7)
center[6] = upper[6]
ik = CartesianIK(model, zone, lower, upper, SPEED, "right", RIGHT_TCP, center, np.zeros(7))
first = solve_search_coordinate(ik, INITIAL_OFFSET, center).joints
second = solve_search_coordinate(ik, INITIAL_OFFSET + np.array([.01, 0, 0]), first).joints


with tempfile.TemporaryDirectory() as directory:
    directory = Path(directory)
    recording = MotionRecording(model.digest)
    recording.start("2026-09-18T12:00:00.000Z")
    for stamp, values in ((1.0, first), (1.5, second)):
        state = {
            **{f"openarmx_right_joint{i + 1}": float(values[i]) for i in range(7)},
            "openarmx_right_finger_joint1": .02,
        }
        recording.add(stamp, state, ik.position(values))
    recording.stop()
    recording_path = recording.save(directory / "ui_path.json")
    socket_path = directory / "detections.sock"
    audio_socket_path = directory / "audio.sock"

    rclpy.init(args=[])
    app = None
    try:
        app = App(False, str(socket_path), str(audio_socket_path), recording_path)
        app.root.withdraw()
        assert app.playback_trajectory is not None
        assert app.playback_trajectory.sample_count == 2
        assert "Recorded camera Cartesian alignment" in app.root.title()
        assert "ui_path.json" in app.recording_status.get()
        assert "0.8 rad/s playback limit" in app.recording_status.get()
        assert not app.isolation_label.winfo_exists()
        assert "LOADING CAMERA" in app.loading_status.get()
        assert app.loading_label.winfo_manager() == "pack"
        app._show_recording_loading(
            "ui_path.json", "Checking representative recovery-to-center paths"
        )
        assert "LOADING RECORDING: ui_path.json" in app.loading_status.get()
        assert "recovery-to-center" in app.loading_status.get()
        app._hide_recording_loading()
        app.receiver.state = "ready"
        app._sync_camera_loading_label()
        assert app.loading_status.get() == ""
        assert app.loading_label.winfo_manager() == ""
        assert app.emergency_button.cget("text") == "EMERGENCY RELAX"
        assert app.center_relax_button.cget("text") == "CENTER + RELAX"
        assert app.emergency_button.master is app.relax_button_frame
        assert app.center_relax_button.master is app.relax_button_frame
        assert int(app.emergency_button.grid_info()["column"]) == 0
        assert int(app.center_relax_button.grid_info()["column"]) == 1
        assert str(app.center_relax_button.cget("state")) == "disabled"
        assert "ST7" in app.sound_status.get()
        assert "TONOR" in app.sound_status.get()
        assert "offline preview" in app.hihat_status.get()
        assert "serial port not opened" in app.hihat_status.get()
        assert "HYBRID search (5°, 6°, 7°…)" in app.header.cget("text")
        assert "ESP32 hi-hat" in app.header.cget("text")
        assert "100 BPM" in app.header.cget("text")
        assert "FIRST DEPTH HYBRID SWING + HI-HAT" in app.continue_button.cget("text")
        assert "100 BPM" in app.continue_button.cget("text")
        assert str(app.start_button.cget("state")) == "disabled"
        app.continuous_strike_active = True
        app._refresh_buttons()
        assert "STOP 100 BPM STRIKING" in app.center_relax_button.cget("text")
        app.continuous_strike_active = False
        app._refresh_buttons()
        assert app.center_relax_button.cget("text") == "CENTER + RELAX"
        assert app.bus is None
        app.start()
        assert "Offline preview only" in app.status.get()
    finally:
        if app is not None:
            app._cancel_planning()
            app.planner_executor.shutdown(wait=False, cancel_futures=True)
            app.receiver.close()
            app.audio_receiver.close()
            app.root.destroy()
            app.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

print("PASS: separate playback UI loaded/preflighted recording; offline only, no CAN or arm motion")
