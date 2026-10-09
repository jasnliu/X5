"""Run from either workspace; real App/transport, only fake motor feedback.

Compare identical right starting poses and recordings, not differing defaults.
Never opens CAN, serial, cameras or audio. Writes results only to supplied path.
"""
import json
import math
from pathlib import Path
import socket
import struct
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path.cwd().resolve()
sys.path.insert(0, str(ROOT))
mode, output = sys.argv[1:]


def audit(event, args):
    if event == 'socket.__new__' and args[1] == socket.AF_CAN:
        raise AssertionError('Physical CAN forbidden')
    if event == 'open' and isinstance(args[0], str) and args[0].startswith(('/dev/tty', '/dev/video', '/dev/snd')):
        raise AssertionError('Physical devices forbidden')


sys.addaudithook(audit)
import numpy as np
import rclpy
from camera_playback.app import App
from camera_playback import hybrid_workflow
from camera_playback.left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET, LeftCenterMonitor
from camera_playback.playback_transport import PlaybackMotors
from centering.motors import RIGHT_GRIPPER_CLOSED
from safe_zone.encoder import FRAME, encoder_to_joint
from tests.test_left_center_hold import fake_bus

clock = [100.]
trace, positions = [], []
rclpy.init(args=[])
app = None
try:
    with tempfile.TemporaryDirectory() as directory:
        hihat = Mock(connect=Mock(return_value=False), ready=Mock(return_value=False),
                     state='disconnected', detail='mocked devices')
        with patch('goal_motion.app.Motors', return_value=fake_bus()), \
                patch('camera_playback.app.HiHatController', return_value=hihat):
            app = App(True, str(Path(directory)/'cam.sock'), str(Path(directory)/'sound.sock'),
                      recording_path=ROOT/'recordings/record3.json')
        app.root.withdraw()
        if app.recording_preflight:
            app.recording_preflight.thread.join(120)
            app._poll_recording_preflight('record3.json')
        dual = getattr(app, 'dual', None)
        if dual and dual.job:
            dual.job.thread.join(120)
            dual.poll_load()
        for callback in app.root.tk.call('after', 'info'):
            app.root.after_cancel(callback)
        assert app.playback_trajectory is not None
        bus = app.bus
        bus.active = bus.left_drive.active = True
        right = np.r_[app.playback_trajectory.last_joints, RIGHT_GRIPPER_CLOSED]
        left = np.r_[dual.recording.last_joints if dual else LEFT_CENTER, LEFT_GRIPPER_TARGET]
        actual = {'right': right.copy(), 'left': left.copy()}
        targets = {s: q.copy() for s, q in actual.items()}
        running = {s: np.full(8, 2) for s in actual}
        speeds = {s: np.full(8, .4) for s in actual}
        bus.modes = {i: (0 if mode == 'mit' and i == 7 else 5, clock[0]) for i in range(1, 9)}
        bus.left_monitor = None
        bus.last_query = 0.
        last_poll = [clock[0]]
        disabled_at = [None]

        def feedback():
            for side in actual:
                for i in range(1, 9):
                    bus.states[side, i] = (float(actual[side][i-1]), int(running[side][i-1]), clock[0])

        def send(side, frame):
            advance()  # Integrate the OLD target only up to this command time.
            cid, _, data = FRAME.unpack(frame)
            kind, motor = (cid >> 24) & 31, cid & 255
            if side == 'right' and kind in (1, 3, 4, 18) and disabled_at[0] is None:
                trace.append({'t':clock[0], 'kind':kind, 'motor':motor, 'frame':frame.hex()})
            if kind == 18:
                index = int.from_bytes(data[:2], 'little')
                value = struct.unpack('<f', data[4:])[0]
                if index == 0x7016:
                    targets[side][motor-1] = encoder_to_joint(side, motor, value) if motor < 8 else -value
                elif index == 0x7017:
                    speeds[side][motor-1] = value
            elif kind == 1:
                raw = int.from_bytes(data[:2], 'big')/65535*25.14-12.57
                targets[side][motor-1] = -raw
            elif kind == 4:
                running[side][motor-1] = 0
                if side == 'right' and disabled_at[0] is None:
                    disabled_at[0] = clock[0]
            feedback()
            return 16

        def no_receive(*args):
            raise BlockingIOError

        for side in actual:
            bus.sockets[side] = SimpleNamespace(send=lambda frame, s=side: send(s, frame), recv=no_receive)

        def advance():
            dt = max(0., clock[0]-last_poll[0])
            last_poll[0] = clock[0]
            for side in actual:
                step = np.clip(targets[side]-actual[side], -speeds[side]*dt, speeds[side]*dt)
                actual[side] += step*(running[side] != 0)
            feedback()

        def poll():
            advance()
            PlaybackMotors.poll(bus)  # Real queries, MIT updates and center evidence.
            if disabled_at[0] is None:
                positions.append({'t':clock[0], 'q':actual['right'][:7].tolist()})

        bus.poll = poll
        app.phase = 'SETTLING RECORDING END'
        app.hihat_calibration = None  # No unrelated calibration in a return trial.
        app.gripper_closed_latched = True
        app.last_gripper_command = 0.
        app.control = app.setup = None
        app.playback_active = app.alignment_active = False
        app.q = bus.positions()
        feedback()
        with patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch('time.sleep', side_effect=lambda seconds: clock.__setitem__(0, clock[0]+seconds)):
            if mode == 'mit':
                hybrid_workflow.begin_center_return(app)
            else:
                app.center_relax()
            assert app.control is not None, app.status.get()
            dispatched_at = next(row['t'] for row in trace if row['kind'] == 18 and row['motor'] == 1)
            mit_at = bus.mit_center_return.at if mode == 'mit' else None
            assert mit_at is None or abs(mit_at-dispatched_at) < 1e-9, (mit_at, dispatched_at)
            # Binary-exact fixed callback periods avoid float-threshold artifacts.
            for index in range(1, 801):
                clock[0] = dispatched_at+index/32.
                if index == 32:
                    from camera_playback.hihat_calibration_runtime import HiHatCalibrationRuntime
                    app.hihat_calibration = HiHatCalibrationRuntime(app, directory=Path(directory)/'hihat_fault')
                    app.hihat_calibration.engine.failure = 'INJECTED: Calibrated ESP32 status became stale'
                    app.hihat_calibration._report_failure()
                    app.hihat_calibration.close()
                    assert app.dual.returning
                    assert app.control is not None
                    assert app.bus.mit_center_return.goal == app.center_goal[6]
                app.tick()
                assert 'FAULT' not in app.phase, app.status.get()
                if disabled_at[0] is not None:
                    break
            assert disabled_at[0] is not None, app.status.get()
        movements = [dict(row, t=round(row['t']-dispatched_at, 9)) for row in trace
                     if row['t'] <= disabled_at[0] and row['kind'] != 4]
        path = [dict(row, t=round(row['t']-dispatched_at, 9)) for row in positions if row['t'] >= dispatched_at]
        result = dict(root=str(ROOT), mode=mode, recording='record3.json',
                      right_start=right[:7].tolist(), movements=movements, positions=path,
                      mit_start_to_j1_dispatch_s=None if mit_at is None else dispatched_at-mit_at,
                      right_disabled_after_s=disabled_at[0]-dispatched_at,
                      no_physical_devices=True, final_status=app.status.get())
        Path(output).write_text(json.dumps(result, indent=2)+'\n')
        print('PASS:',mode,len(movements),'right movement frames;',len(path),'poses; no physical devices')
finally:
    if app is not None:
        app._cancel_recording_preflight()
        app._cancel_planning()
        app.planner_executor.shutdown(wait=True,cancel_futures=True)
        app.receiver.close();app.audio_receiver.close();app.hihat_sound_monitor.close()
        app.root.destroy();app.node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
