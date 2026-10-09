"""Real --hardware App/Tk routing with only substituted motor/audio/ESP32 I/O.

This is NOT a physical test. AF_CAN is forbidden, including accidental calls.
"""
from dataclasses import replace
import math
from pathlib import Path
import socket
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import rclpy

def forbid_can(event, args):
    if event == 'socket.__new__' and len(args) > 1 and args[1] == socket.AF_CAN:
        raise AssertionError('Physical CAN forbidden by hybrid UI check')

sys.addaudithook(forbid_can)

from camera_playback.app import App, SWING_EVENTS, CENTER_RELAX_PHASE
from camera_playback.hybrid_strike import HybridStatus
from camera_playback.mit_strike import Sample
from camera_playback.simulation import SimulatedMotors
from camera_playback.left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET
from camera_playback import hybrid_workflow as hw
from camera_search.planner import INITIAL_OFFSET, solve_search_coordinate
from centering.motors import RIGHT_GRIPPER_CLOSED


with tempfile.TemporaryDirectory() as directory:
    path = Path(directory)
    bus = SimulatedMotors(np.array([0.]*6+[1.4]))
    session = Mock()
    session.status = HybridStatus()
    session.request.return_value = 3
    app = None
    rclpy.init(args=[])
    try:
        with patch('goal_motion.app.Motors', return_value=bus), \
                patch('camera_playback.app.HiHatController'), \
                patch('camera_playback.app.HiHatCalibrationRuntime'):
            app = App(True, str(path/'camera.sock'), str(path/'audio.sock'))
        app.root.withdraw()
        assert app.hybrid_enabled and not app.test_mode and not app.hardware_test_mode
        assert app.single_run_mode
        assert 'HYBRID' in app.header.cget('text')
        assert app.start_button.winfo_manager() == ''
        assert app.continue_button.winfo_manager() == 'pack'
        assert app.continue_button.cget('text') == 'RUN: CENTER + CLOSE → RECORDING → HYBRID SWING'
        assert not bus.active  # Opening the application never starts motion.
        app.q = bus.positions()  # Match the normal encoder tick before a press.
        anchor = solve_search_coordinate(app.hill_ik, INITIAL_OFFSET, app.center_goal).joints
        # Exercise the actual Tk button binding through center + close and the
        # recording entry point. Only then inject the later alignment result.
        app.playback_trajectory = SimpleNamespace(first_joints=anchor.copy(),
                                                  tcp_positions=[app.planned_tcp(anchor)])
        with patch.object(app, 'receiver', Mock(cymbal=True)), \
                patch.object(app, '_camera_ready_for_start', return_value=True), \
                patch.object(app, '_sound_ready_for_start', return_value=True), \
                patch.object(app, '_hihat_ready_for_start', return_value=True), \
                patch.object(app, '_begin_playback') as playback, \
                patch.object(bus, 'center', wraps=bus.center) as center:
            app._refresh_buttons()
            assert str(app.continue_button.cget('state')) == 'normal'
            app.continue_button.invoke()  # The only press in the whole workflow.
            assert app.phase == 'PLAYBACK PREFLIGHTED'
            assert str(app.continue_button.cget('state')) == 'disabled'
            app.continue_button.invoke()  # Disabled; must not schedule a second run.
            app._start_preflighted_recording()
            # Model the existing post-RUN ESP32 reply gate, not just readiness.
            app.hihat.telemetry_at = time.monotonic()
            app._start_preflighted_recording()
            center.assert_called_once_with(app.center_goal, RIGHT_GRIPPER_CLOSED)
            list(app.setup)
            app.setup = None
            app.begin_stage('CENTERING', app.center_goal)
            bus._joints = app.center_goal.copy()
            bus._gripper = RIGHT_GRIPPER_CLOSED
            bus._refresh_states(time.monotonic())
            app.q = bus.positions()
            app.complete_stage(time.monotonic())
            assert app.phase == 'CENTERING'  # Right arrival alone cannot play.
            bus._left_joints = LEFT_CENTER.copy()
            bus._left_gripper = LEFT_GRIPPER_TARGET
            now = time.monotonic()
            for stamp in (now, now + .7):
                bus._refresh_states(stamp)
                bus.left_monitor.update(bus.states, stamp)
            app.q = bus.positions()
            app.complete_stage(now + .7)
            assert app.phase == 'MOVING TO RECORDING START'
            assert app.gripper_closed_latched
            app.complete_stage(time.monotonic())
            playback.assert_called_once()
        print('PASS: one actual RUN button waits for both centers, left J5 -25 degrees, '
              'left gripper raw target +14.16°, right gripper +7 degrees, and '
              'automatically enters recording; no Start control or loading pause')
        app.arm = Mock(return_value=anchor)
        bus.active = True
        bus._joints = bus._targets = anchor.copy()
        app.q = bus.positions()
        app.phase = 'CHECKING PLAYBACK END'
        app.alignment_active = True
        with patch.object(hw, 'HybridSession', return_value=session):
            app._alignment_success(21)
        assert app.strike_active and app.hybrid_session is session
        session.request.assert_called_once_with('search', math.radians(5))
        assert app.control is None
        now = time.monotonic()
        session.status = HybridStatus(phase='coast', count=1, released_at=now-.1,
            peak_drop=math.radians(4), sample=Sample(anchor[6]-.07, -.5, 0, 2, now), at=now)
        app._process_sound_hit({'event_at': now-.01})
        assert app.strike_hit_pending['degrees'] == 5
        assert not app.continuous_strike_active
        session.status = replace(session.status, phase='hold', ready=True, completed=1,
            returned_at=now, sample=Sample(anchor[6], 0, .745, 2, now))
        hw.tick(app, now)
        assert app.continuous_strike_active and app.continuous_strike_degrees == 5.5
        assert session.request.call_args.args == ('swing', math.radians(5.5), SWING_EVENTS)
        assert '100 BPM' in app.status.get()
        app.center_relax()
        assert session.request.call_args.args == ('finish',)
        assert app.phase == hw.FINISHING
        bus.restore_right_joint7_csp = Mock(return_value=now)
        bus.right_joint7_mode_readback = Mock(return_value=0)
        bus.begin_mit_center_return = Mock()
        session.status = replace(session.status, request_id=3, swing=False)
        hw.tick(app, now)
        session.stop.assert_called_once()
        bus.restore_right_joint7_csp.assert_not_called()
        bus.begin_mit_center_return.assert_called_once_with(.745)
        assert app.phase == CENTER_RELAX_PHASE and app.control is not None
        assert bus.active  # No relaxation before center completion.
        bus._joints = app.center_goal.copy()
        app.complete_stage(now+.1)
        assert not bus.active
        print('PASS: real App(hardware=True) selects hybrid, boosts first-hit depth by 0.5 degrees, '
              'starts 100 BPM swing, and joins/centers in MIT before simulated relax')
        print('PASS: Tk widgets show hybrid; motor, microphone and ESP32 I/O were substituted; '
              'no physical hardware operated')
    finally:
        if app is not None:
            hw.close(app)
            app._cancel_planning()
            app.planner_executor.shutdown(wait=False, cancel_futures=True)
            app.receiver.close()
            app.audio_receiver.close()
            bus.close()
            app.root.destroy()
            app.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
