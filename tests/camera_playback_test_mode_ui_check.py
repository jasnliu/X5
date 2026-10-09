"""Offline Tk check proving test mode never constructs the real CAN class."""
from pathlib import Path
import tempfile
import time
from unittest.mock import patch

import numpy as np
import rclpy

from camera_playback.app import App
from camera_playback.simulation import SimulatedMotors
from camera_playback.left_hold import LEFT_CENTER
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
        app.root.withdraw()
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

        app.start()
        wait_for(lambda: app.phase == "WAITING FOR LOAD", 10.0, "simulated center")
        assert app.bus.left_center_ready()
        np.testing.assert_allclose(app.bus._left_joints, LEFT_CENTER)
        assert abs(np.degrees(app.bus.motor8_feedback['left']['raw_rad']) - 14.16) < .022
        app.continue_motion()
        wait_for(lambda: app.continuous_strike_active, 20.0,
                 "recording and fixed 10-degree striking")
        assert app.continuous_strike_degrees == 10
        wait_for(lambda: app.continuous_strike_count >= 2, 3.0,
                 "swing pickup plus beat 1 in simulation")
        np.testing.assert_allclose(app.bus._left_joints, LEFT_CENTER)
        assert all(app.bus.states['left', i][1] == 2 for i in range(1, 9))
        app.center_relax()
        wait_for(lambda: app.phase == "RELAXED", 15.0,
                 "simulated Stop, Center, and Relax")
        assert app.bus.active is False
        assert all(state[1] == 0 for state in app.bus.states.values())
        print('PASS: shared Start centers both arms; left J5 -50 degrees and raw gripper target +14.16° '
              'hold through right playback/strikes; both arms relax (simulation only)')
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
print("PASS: simulated center, recording, 10-degree 100 BPM swing, Stop, and Relax")
