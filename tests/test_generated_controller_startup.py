"""Reproduce the cold-spawn heartbeat race without opening any devices."""
import multiprocessing as mp
import os
from pathlib import Path
import time
from unittest.mock import patch

import numpy as np
import pytest

from camera_playback import generated_stream as streaming
from camera_playback.generated_simulation import SimulatedGeneratedMotors, SimulatedStreamMotors
from camera_playback.generated_transport import GeneratedMotors
from tests.test_generated_stream import make_curve, wait_ready


def restore_slow_curve(curve, delay):
    # Child-side unpickling/import work happens BEFORE _worker is entered, just
    # like the failed launch. Delaying inside the motion loop is not this bug.
    time.sleep(delay)
    return curve


class SlowCurve:
    def __init__(self, curve, delay=.7):self.curve, self.delay = curve, delay
    def __getattr__(self, key):return getattr(self.curve, key)
    def __reduce__(self):return restore_slow_curve, (self.curve, self.delay)


def bus(tmp_path):
    b = SimulatedGeneratedMotors({s:np.zeros(7) for s in ('left', 'right')}, streaming=True)
    b.active = True; b.poll(); b.stream_geometry = {}; b.stream_directory = tmp_path
    return b


def finish(session):
    until = time.monotonic()+5
    while not (update := session.poll())['done']:
        assert time.monotonic() < until
        time.sleep(.01)
    assert update['result']['success'], update
    return update['result']


def cleanup(b, s):
    s.stop(); s.detach(); b.active=False; b.close()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_cold_spawn_over_old_watchdog_does_not_fault_or_move_before_ack(tmp_path, side):
    b = bus(tmp_path); c = make_curve(side)
    s = b.start_stream(SlowCurve(c), side, None)
    try:
        assert s.ready.wait(5)  # Intentionally do NOT call poll/renew heartbeat.
        assert time.monotonic()-s.heartbeat.value > .5
        assert s.process.is_alive() and not s.run.is_set()
        assert s.state.read()[0] == 0. and not s.owns_feedback
        assert not (s.directory/'trace.csv').exists()
        for q in b.q.values():np.testing.assert_array_equal(q, np.zeros(7))
        result = finish(s)  # First live GUI acknowledgement now starts the clock.
        assert result['ready_after_s'] >= .7
        assert result['start_ack_after_s'] >= result['ready_after_s']
        assert result['heartbeat_timeout_s'] == .5  # Not a weakened watchdog.
        assert result['executor'] == 'plan_drums.baseline.motion.Controller'
        np.testing.assert_array_equal(b.q[side], c.at(c.duration))
    finally:cleanup(b, s)


def test_real_hardware_start_method_and_read_only_refresh_use_same_handshake(tmp_path):
    # Run GeneratedMotors.start_stream itself, substituting ONLY device I/O.
    # This covers the 80 ms parent feedback drain missed by the old sim adapter.
    b = bus(tmp_path); c = make_curve(); session_class = streaming.StreamSession
    def simulated_session(owner, curve, side, step, **kw):
        return session_class(owner, curve, side, step, **kw,
                             factory=SimulatedStreamMotors, shared=b.shared)
    with patch.object(streaming, 'StreamSession', side_effect=simulated_session):
        s = GeneratedMotors.start_stream(b, SlowCurve(c), 'left', None)
    try:
        assert s.ready.wait(5)
        assert not s.run.is_set()
        result = finish(s)
        assert result['success'] and result['ready_after_s'] >= .7
    finally:cleanup(b, s)


def test_ready_worker_waits_for_gui_not_a_stale_pre_spawn_heartbeat(tmp_path):
    b=bus(tmp_path);s=b.start_stream(make_curve(), 'left', None)
    try:
        assert s.ready.wait(5)
        time.sleep(.65)  # GUI absent before acknowledgement: stay at rest.
        assert s.process.is_alive() and not s.run.is_set()
        np.testing.assert_array_equal(b.q['left'], np.zeros(7))
        assert s.state.read()[0] == 0.
        finish(s)
    finally:cleanup(b,s)


def test_hold_cancels_ready_worker_without_starting_a_trajectory(tmp_path):
    b=bus(tmp_path);s=b.start_stream(make_curve(), 'left', None)
    try:
        assert s.ready.wait(5)
        stopped=s.stop()
        assert stopped['done'] and stopped['elapsed'] == 0.
        assert stopped['result']['cancelled'] and not s.run.is_set()
        assert stopped['result']['held'] == [0.]*7 and b.active
        assert not (s.directory/'trace.csv').exists()
    finally:cleanup(b,s)


def test_relax_during_unpickling_prevents_any_later_start(tmp_path):
    b=bus(tmp_path);s=b.start_stream(SlowCurve(make_curve(), 2.), 'left', None)
    try:
        b.emergency_release()
        assert not b.active and not s.process.is_alive()
        assert not s.run.is_set() and s.state.read()[0] == 0.
        np.testing.assert_array_equal(b.q['left'], np.zeros(7))
    finally:cleanup(b,s)


def test_unacknowledged_startup_is_bounded_and_does_not_disable(tmp_path):
    # Exercise worker failure deterministically without waiting the production
    # 10 seconds. The running 0.5 second watchdog is NOT used for initialization.
    b=bus(tmp_path);ctx=mp.get_context('spawn');c=make_curve()
    cancel, ready, run=ctx.Event(),ctx.Event(),ctx.Event()
    heartbeat=ctx.Value('d', time.monotonic(), lock=False)
    state=streaming.Progress(ctx,[0.,0.,*c.at(0),0.])
    with patch.object(streaming,'STARTUP_TIMEOUT',.08), patch.object(streaming.signal,'signal'):
        streaming._worker(c,None,'left',{},b.target,tmp_path,[],None,b.shared,
            cancel,heartbeat,ready,run,state,os.getppid(),time.monotonic(),SimulatedStreamMotors)
    import json
    result=json.loads((tmp_path/'result.json').read_text())
    assert not result['success'] and 'acknowledgement timed out' in result['error']
    assert result['held'] == [0.]*7 and not run.is_set() and b.active
    assert state.read()[0] == 0.
    b.active=False;b.close()


def test_large_cold_spawn_does_not_block_parent_held_worker_heartbeat(tmp_path):
    b=bus(tmp_path);b.stream_geometry={'not_used_before_cancel':bytes(2_000_000)}
    started=time.monotonic()
    s=b.start_stream(SlowCurve(make_curve(),.8),'left',None)
    try:
        # Previously a large bootstrap pipe waited for the child's imports
        # and slow unpickling, killing already-powered MIT owners at 500 ms.
        assert time.monotonic()-started<.4
        assert s.parent_startup_s<.4 and not s.run.is_set()
        assert s.inputs_path.stat().st_mode&0o777==0o600
        s.stop();s.detach()
        assert not s.inputs_path.exists() and b.active
        np.testing.assert_array_equal(b.q['left'],np.zeros(7))
    finally:cleanup(b,s)
