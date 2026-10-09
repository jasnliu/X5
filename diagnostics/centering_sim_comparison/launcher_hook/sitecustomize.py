"""Automation installed ONLY in isolated --test launcher smoke processes.

The production modules and both shell launchers stay unchanged. Sets an equal
initial simulated pose, presses Center, records completion, then exits the UI.
"""
import os
if os.environ.get('BEAT_SIM_SMOKE'):
    import inspect
    import json
    from pathlib import Path
    import socket
    import sys
    import time
    import tkinter

    def no_devices(event, args):
        if event == 'socket.__new__' and args[1] == socket.AF_CAN:
            raise AssertionError('CAN forbidden in simulation launcher')
        if event == 'open' and isinstance(args[0], str) and args[0].startswith(('/dev/tty', '/dev/serial', '/dev/video', '/dev/snd')):
            raise AssertionError('Physical device forbidden')
    sys.addaudithook(no_devices)
    original = tkinter.Tk.mainloop
    def automated_mainloop(root, *args, **kwargs):
        frame = inspect.currentframe()
        app = None
        while frame:
            candidate = frame.f_locals.get('self')
            if getattr(candidate, 'root', None) is root and hasattr(candidate, 'center_relax'):
                app = candidate
                break
            frame = frame.f_back
        if app is None:
            return original(root, *args, **kwargs)
        assert app.test_mode and type(app.bus).__name__ == 'SimulatedMotors'
        start = time.monotonic()
        launched = [False]
        poses = []
        def advance():
            try:
                if time.monotonic()-start > 180:
                    raise RuntimeError('Simulation launcher automation timed out')
                bus = app.bus
                if not launched[0]:
                    dual = getattr(app, 'dual', None)
                    if app.playback_trajectory is None or (dual and dual.recording is None):
                        root.after(100, advance)
                        return
                    from camera_playback.left_hold import LEFT_CENTER, LEFT_GRIPPER_TARGET
                    from centering.motors import RIGHT_GRIPPER_CLOSED
                    bus._joints = app.playback_trajectory.last_joints.copy()
                    bus._targets = bus._joints.copy()
                    bus._left_joints = (dual.recording.last_joints.copy() if dual else LEFT_CENTER.copy())
                    bus._left_targets = bus._left_joints.copy()
                    bus._gripper = bus._gripper_target = RIGHT_GRIPPER_CLOSED
                    bus._left_gripper = LEFT_GRIPPER_TARGET
                    bus.active = True
                    bus.center_goal = app.center_goal.copy()
                    bus._last_poll = time.monotonic()
                    bus._refresh_states(bus._last_poll)
                    app.phase = 'SETTLING RECORDING END'
                    app.gripper_closed_latched = True
                    app.control = app.setup = None
                    app.playback_active = app.alignment_active = False
                    app.center_relax()
                    launched[0] = True
                poses.append(bus._joints.tolist())
                if not bus.active:
                    result = dict(completed=True, test_mode=app.test_mode,
                        bus=type(bus).__name__, root=str(Path.cwd()),
                        final_phase=app.phase, right_final=bus._joints.tolist(),
                        center_goal=app.center_goal.tolist(), poses=poses)
                    Path(os.environ['BEAT_SIM_SMOKE']).write_text(json.dumps(result, indent=2)+'\n')
                    print('SIMULATION LAUNCHER CENTER COMPLETE; simulated drives relaxed', flush=True)
                    root.quit()
                    return
                root.after(20, advance)
            except Exception as exc:
                Path(os.environ['BEAT_SIM_SMOKE']).write_text(json.dumps(dict(completed=False,error=str(exc))))
                print('SIMULATION LAUNCHER FAILURE:', exc, flush=True)
                root.quit()
        root.after(100, advance)
        return original(root, *args, **kwargs)
    tkinter.Tk.mainloop = automated_mainloop
