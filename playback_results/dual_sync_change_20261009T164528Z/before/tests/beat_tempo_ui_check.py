"""Editable BPM Tk regression, complete simulated run; never opens real CAN."""
from pathlib import Path
import tempfile
import time
from unittest.mock import patch

import numpy as np
import rclpy
from PIL import ImageGrab
from camera_playback.tempo import swing_events

from camera_playback.app import App
from camera_playback.simulation import SimulatedMotors
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
    recording.start("2026-09-27T12:00:00.000Z")
    for stamp, values in ((1.0, first), (1.5, second)):
        state = {
            **{f"openarmx_right_joint{i + 1}": float(values[i]) for i in range(7)},
            "openarmx_right_finger_joint1": .02,
        }
        recording.add(stamp, state, ik.position(values))
    recording.stop()
    recording_path = recording.save(directory / "test_mode_path.json")

    app = None
    rclpy.init(args=[])
    try:
        with patch("goal_motion.app.Motors",
                   side_effect=AssertionError("real CAN Motors must not be constructed")):
            app = App(
                False,
                str(directory / "detections.sock"),
                str(directory / "audio.sock"),
                recording_path,
                test_mode=True,
            )
        app.root.geometry("+40+40")
        assert app.hardware is True
        assert app.test_mode is True
        assert isinstance(app.bus, SimulatedMotors)
        assert not hasattr(app.bus, "sockets")
        assert app.hihat is None
        assert "PURE SIMULATION" in app.root.title()
        assert "fixed 10° swing" in app.header.cget("text")
        assert "FIXED 10° SWING AT 100 BPM" in app.continue_button.cget("text")
        assert "ignored in test mode" in app.sound_status.get()
        assert "ignored in test mode" in app.hihat_status.get()
        assert app.loading_status.get() == ""
        assert app.loading_label.winfo_manager() == ""
        assert str(app.start_button.cget("state")) == "normal"
        assert app._camera_ready_for_start()
        assert app._sound_ready_for_start()
        assert app._hihat_ready_for_start()

        def wait_for(predicate, seconds, description):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                app.root.update()
                if predicate():
                    return
                time.sleep(.005)
            raise AssertionError(
                f"timed out waiting for {description}; phase={app.phase}; "
                f"status={app.status.get()}"
            )

        # Use the real text box and its write trace, not a bypassed setter.
        def enter(text):
            app.bpm_entry.delete(0, 'end')
            app.bpm_entry.insert(0, text)
            app.root.update_idletasks()

        def screenshot(name):
            app.root.update()
            time.sleep(.08)
            app.root.update()
            box = (app.root.winfo_rootx(), app.root.winfo_rooty(),
                   app.root.winfo_rootx()+app.root.winfo_width(),
                   app.root.winfo_rooty()+app.root.winfo_height())
            ImageGrab.grab(bbox=box).save(ROOT/'diagnostics/beat_tempo_20261005'/name)

        enter('nan')
        assert str(app.start_button.cget('state')) == 'disabled'
        assert 'finite BPM' in app.tempo_hint.cget('text')
        enter('87.5')
        assert app.strike_bpm == 87.5
        assert app.swing_events == swing_events(87.5)
        assert '87.5 BPM' in app.header.cget('text')
        assert '87.5 BPM' in app.continue_button.cget('text')
        assert str(app.start_button.cget('state')) == 'normal'
        screenshot('bpm_ready.png')
        app.start()
        assert str(app.bpm_entry.cget('state')) == 'disabled' 
        wait_for(lambda: app.phase == "WAITING FOR LOAD", 10.0, "simulated center")
        app.continue_motion()
        wait_for(lambda: app.continuous_strike_active, 20.0,
                 "recording and fixed 10-degree striking")
        assert app.continuous_strike_degrees == 10
        assert str(app.bpm_entry.cget('state')) == 'disabled'
        assert '87.5 BPM' in app.center_relax_button.cget('text')
        assert app.strike_bpm == 87.5
        screenshot('bpm_locked.png')
        wait_for(lambda: app.continuous_strike_count >= 2, 3.0,
                 "swing pickup plus beat 1 in simulation")
        app.center_relax()
        wait_for(lambda: app.phase == "RELAXED", 15.0,
                 "simulated Stop, Center, and Relax")
        assert app.bus.active is False
        app._refresh_buttons()
        assert str(app.bpm_entry.cget('state')) == 'normal'
        enter('120')
        assert app.strike_bpm == 120
        assert app.swing_events == swing_events(120)
    finally:
        if app is not None:
            app._cancel_planning()
            app.planner_executor.shutdown(wait=False, cancel_futures=True)
            app.receiver.close()
            app.audio_receiver.close()
            app.bus.close()
            app.root.destroy()
            app.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

print("PASS: --test UI uses only SimulatedMotors; real CAN constructor was forbidden")
print("PASS: camera, sound, ESP32, and all physical hardware are bypassed")
print("PASS: simulated center, recording, 10-degree 87.5 BPM swing, Stop, and Relax")

print("PASS: actual BPM textbox validates, locks during run, and unlocks after Center + Relax")
