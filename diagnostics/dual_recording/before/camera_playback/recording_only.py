"""Explicit no-strike physical test mode for the real start_beat application."""
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid

from smooth_playback.runner import Runner
from smooth_playback.trajectory import ROOT
from smooth_playback.trajectory import transition


def refresh_feedback(bus, seconds=.08):
    """Request/read fresh feedback after CPU-only geometric preflight.

    Geometry can take longer than the 300ms freshness window. Temporarily
    suspending the *software* active check permits query/readback only; it does
    not send a disable/mode/position command or change physical motor state.
    No movement resumes until fresh feedback has been independently checked.
    """
    active = bus.active
    try:
        bus.active = False
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            bus.poll()
            time.sleep(.002)
    finally:
        bus.active = active
    if not bus.fresh():
        raise RuntimeError('Fresh feedback unavailable after center-path preflight')


def evidence_directory():
    path = ROOT/'playback_results'/'start_beat_recording_only'/(
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:8])
    path.mkdir(parents=True)
    return path


def finish(app, reason, playback_success=False):
    """Return using a checked rest-to-rest path; disable only at measured center.

    This also handles close/Escape/error in the opt-in no-strike test. No fresh
    feedback or a stopped drive means HOLD, not a blind return or relaxation.
    """
    if getattr(app, '_recording_only_finishing', False):
        return
    app._recording_only_finishing = True
    runner = None
    result = dict(reason=str(reason), playback_success=playback_success,
                  success=False, center=None, relaxed_verified=False,
                  worker_directory=getattr(app, 'last_recording_worker_directory', None))
    try:
        app._stop_smooth_playback()
        app.setup = app.control = None
        app.playback_active = app.alignment_active = False
        app.phase = 'RECORDING ONLY RECENTERING'
        app.status.set('No-strike test: returning to verified center before relaxing')
        app.root.update_idletasks()
        bus = app.bus
        if not bus.active:
            raise RuntimeError('Arm was not active; no physical playback/return is claimed')
        directory = app.recording_only_directory/('return-'+uuid.uuid4().hex[:6])
        directory.mkdir()
        recording = app.playback_trajectory
        runner = Runner(recording.geometry, recording.smooth_motion, directory,
                        gripper=False, video=False)
        runner.bus = bus
        runner.enabled = True
        runner.recovering = True
        # Drain queued copies after joining the playback writer, then request
        # current feedback before using a measured pose for the return path.
        refresh_feedback(bus, .15)
        runner.poll()
        bus.set_right_arm_speed(.4)
        goal = recording.geometry.center
        q = runner.poll()
        runner.event('CHECKING RECORDING ONLY RECENTER')
        recording.geometry.check_line(q, goal, measured_start=True)
        refresh_feedback(bus)
        current = runner.poll()
        if max(abs(current-q)) > .0035:
            raise RuntimeError('Arm moved during stationary center-path validation')
        # Include the measured validation drift without a second expensive FK
        # sweep. The first target remains the validated measured hold pose.
        duration, at = transition(q, goal)
        runner.event('RECORDING ONLY RECENTER', duration_s=duration)
        runner.stream(duration, at)
        result['center'] = runner.settle(goal, 'RECORDING ONLY RECENTER SETTLED')
        if getattr(app, 'smooth_playback_cleanup_error', None):
            raise RuntimeError(app.smooth_playback_cleanup_error)
        deadline = time.monotonic()+.15
        while time.monotonic() < deadline:
            runner.poll()
            time.sleep(.002)
        # The transport independently checks fresh <=0.20 degree center error,
        # <=0.12 degree span, and 0.6 seconds settling on EVERY disable packet.
        bus.relax()
        deadline = time.monotonic()+2.
        while time.monotonic() < deadline:
            bus.poll()
            if bus.fresh() and all(s[1] == 0 for s in bus.states.values()):
                result['relaxed_verified'] = True
                result['success'] = bool(playback_success)
                result['disable_center_evidence'] = bus.last_center_evidence
                app.phase = 'RELAXED'
                app.status.set('Recording-only complete: centered, then relaxed; no alignment or strikes')
                break
            time.sleep(.005)
        else:
            raise RuntimeError('Disabled feedback was not confirmed')
    except Exception as exc:
        result['error'] = str(exc)
        app.phase = 'SAFE HOLD - NOT RELAXED'
        app.status.set('NOT RELAXED: '+str(exc))
        print('RECORDING ONLY: NOT RELAXED; operator attention required: '+str(exc), flush=True)
    finally:
        if runner:
            runner.trace_file.close()
        app.recording_only_result = result
        with open(app.recording_only_directory/'results_history.jsonl', 'a') as out:
            out.write(json.dumps(result)+'\n')
        (app.recording_only_directory/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print('RECORDING ONLY RESULT: '+json.dumps(result), flush=True)
        app._recording_only_finishing = False


def auto_tick(app):
    """Opt-in unattended test: original Run/Continue, never any strike action."""
    now = time.monotonic()
    phase = app.phase
    if phase != getattr(app, '_auto_last_phase', None):
        app._auto_last_phase = phase
        row = dict(t=now, phase=phase, status=app.status.get())
        print('RECORDING ONLY PHASE: '+json.dumps(row), flush=True)
        with open(app.recording_only_directory/'phases.jsonl', 'a') as out:
            out.write(json.dumps(row)+'\n')
    if not getattr(app, '_auto_started', False):
        if (phase == 'READY' and app.bus.fresh() and app._camera_ready_for_start()
                and app._sound_ready_for_start() and app._hihat_ready_for_start()):
            app._auto_started = True
            app.start()
    elif phase == 'WAITING FOR LOAD':
        if not hasattr(app, '_auto_load_at'):
            app._auto_load_at = now+5.
            print('RECORDING ONLY: gripper closes in 5 seconds; keep hands clear', flush=True)
        if now >= app._auto_load_at:
            app.continue_motion()
    elif phase == 'RELAXED' and getattr(app, 'recording_only_result', None):
        app.root.quit()
        return
    app.root.after(50, lambda: auto_tick(app))
