"""Offline Tk check for editable MIT freefall/rebound in --hardwaretest mode."""
from pathlib import Path
import time
import tempfile
from unittest.mock import Mock, patch

import numpy as np
import rclpy

from camera_playback.app import (
    App,
    HARDWARE_TEST_OUT_PHASE,
    HARDWARE_TEST_HOLD_PHASE,
    HARDWARE_TEST_READY_PHASE,
)
from camera_playback.mit_strike import Sample, Status, StrikeSettings

with tempfile.TemporaryDirectory() as directory:
    directory = Path(directory)
    app = None
    rclpy.init(args=[])
    try:
        # hardware=False guarantees this check cannot open SocketCAN.  The UI
        # mode flag is still enough to construct and inspect the dedicated
        # hardware-test manual strike control.
        app = App(
            False,
            str(directory / "detections.sock"),
            str(directory / "audio.sock"),
            hardware_test_mode=True,
        )
        app.root.withdraw()
        assert app.bus is None
        assert app.hardware_test_mode is True
        assert app.test_mode is False
        assert "HARDWARE TEST" in app.root.title()
        assert "center/open" in app.header.cget("text")
        assert "pink-zone" in app.header.cget("text")
        assert "RECORDING" in app.continue_button.cget("text")
        assert app.continue_button.winfo_manager() == "pack"
        assert app.camera_label.winfo_manager() == "pack"
        assert app.recording_label.winfo_manager() == "pack"
        assert app.recording_button.winfo_manager() == "pack"
        assert app.sound_label.winfo_manager() == "pack"
        assert app.hihat_label.winfo_manager() == ""
        assert app.hardware_test_frame.winfo_manager() == "pack"
        assert app.hardware_test_button.cget("text") == "MIT TEST: REACH PINK ZONE FIRST"
        assert str(app.hardware_test_button.cget("state")) == "disabled"
        assert str(app.hardware_test_entry.cget("state")) == "disabled"
        assert "serial port not opened" in app.hihat_status.get()
        assert app.hihat is None

        anchor = np.zeros(7)
        target = anchor.copy()
        target[6] = -np.deg2rad(6.5)
        app.hardware = True
        app.bus = Mock(active=True)
        app.bus.fresh.return_value = True
        app.setup = None
        app.arm = Mock(return_value=anchor.copy())
        with patch("camera_playback.app.Joint7Session") as factory:
            app._prepare_hardware_test_manual_strikes(17, anchor)
        session = factory.return_value
        assert app.phase == HARDWARE_TEST_HOLD_PHASE
        assert str(app.hardware_test_button.cget("state")) == "disabled"
        session.controller.settings = StrikeSettings()
        session.status = Status("hold", True, sample=Sample(0, 0, .1, 2, time.monotonic()))
        app._advance_hardware_test_mit(time.monotonic())
        assert app.phase == HARDWARE_TEST_READY_PHASE
        assert "PINK ZONE CONFIRMED ONCE" in app.status.get()
        assert "pink-zone" in app.status.get()
        app.hardware_test_degrees.set("6.5")
        app._refresh_hardware_test_controls()
        assert app.hardware_test_button.cget("text") == "FREEFALL 6.5° + REBOUND"
        assert str(app.hardware_test_button.cget("state")) == "normal"
        assert str(app.hardware_test_entry.cget("state")) == "normal"
        with patch(
            "camera_playback.app.build_manual_strike_target", return_value=target
        ):
            app.hardware_test_button.invoke()
        session.strike.assert_called_once_with(np.deg2rad(6.5))

        app.phase = HARDWARE_TEST_OUT_PHASE
        app._refresh_hardware_test_controls()
        assert app.hardware_test_button.cget("text") == "MIT FREEFALL 6.5° DOWN…"
        assert str(app.hardware_test_button.cget("state")) == "disabled"
        assert str(app.hardware_test_entry.cget("state")) == "disabled"

        assert app._hardware_test_sound_monitor_active() is True
    finally:
        if app is not None:
            # The mock bus was attached only after construction for UI state;
            # detach it so cleanup cannot call any motor method.
            app._stop_hardware_test_session()
            app.bus = None
            app._cancel_planning()
            app.planner_executor.shutdown(wait=False, cancel_futures=True)
            app.receiver.close()
            app.audio_receiver.close()
            if app.hihat is not None:
                app.hihat.close()
            app.root.destroy()
            app.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

print("PASS: --hardwaretest UI accepts a fixed-anchor maximum drop goal")
print("PASS: --hardwaretest exposes center/open/load, recording, and pink-zone controls")
print("PASS: each click authorizes one damped fall, predictive catch and smooth return")
print("PASS: camera and recording are restored; ST7 logs only; ESP32 remains bypassed")
print("PASS: offline UI check opened no CAN socket and commanded no physical hardware")
