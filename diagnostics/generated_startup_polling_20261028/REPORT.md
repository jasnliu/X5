# Generated beat startup feedback regression — 2026-10-28

## Result

Fixed the startup regression introduced by the preceding CAN query-scheduling
change. Reproduced the actual error in the **real program's Tk Start/ARMING
workflow with packet-level fake devices**, then completed startup, both outbound
paths, Hold, both return paths, rest release and Stop without that error.

**No physical CAN devices, motors, camera, microphone or ESP32 were accessed.**
Physical socket/device access was prohibited; the actual hardware constructors
were patched to raise if called. Fake device state changes are not physical
motor commands. The user's separate running hardware application was untouched.

## Why it failed

The user's two saved failures were in ARMING:

- `planner_runs/generated-beat-20261028T220838-a66d4bc5/last_fault.json`
- `planner_runs/generated-beat-20261028T221006-81fb4d62/last_fault.json`

Some received encoder samples were **136–137 ms old**, exceeding the existing
**120 ms** `Motion.fresh` requirement. Other motors had just replied. This was
not a reason to widen the freshness threshold or blame the encoders.

The prior change allowed only two parent queries per bus per callback, assuming
Tk's `after(20)` meant a fixed 50 Hz schedule. It does not: the delay starts after
the callback's work. The real log has roughly 33 ms intervals; the loaded test
GUI had roughly 51 ms intervals. Spreading eight drives across several callbacks
left readings too old during setup, before the path worker was available.

The previous ideal simulator refreshed all sixteen states at every poll and
enabled them in one fake step. It could not reproduce this packet-scheduling
failure. Passing those tests was insufficient startup verification.

Packet-level testing also exposed why partial sweeps were fragile at worker
handoff: the parent's old query order was not the order of the oldest *received*
feedback, and suppressing parent queries until readings reached 100 ms left
less than one normal GUI callback before the 120 ms guard.

## Minimal runtime correction

Only two existing runtime files were changed:

- `camera_playback/generated_can.py`: configurable bounded sweep size; query
  priority follows actual received age, then last request as a tie-breaker.
- `camera_playback/generated_transport.py`: restore a complete eight-drive
  sweep for the slower parent GUI. Recent per-joint replies suppress duplicate
  queries directly; no all-or-nothing 100 ms child handoff delay.

The busy **200 Hz path worker still staggers two queries per bus per tick**.
Both parent and worker skip drives with recent replies. The earlier bounded
ENOBUFS/EAGAIN handling and TX-versus-RX distinction remain intact. A four-query
parent sweep was tested but rejected: it passed setup yet left insufficient
margin at callback/worker handoffs. Restoring complete low-rate GUI sweeps was
the smaller, more reliable adaptation of the original working behavior.

The 120 ms feedback check, other watchdogs, drive faults, limits, gains, path
generation, timing, motor ownership, actual confirmed-CSP startup and emergency
controls were **not changed**. Truly missing encoder replies still cause a fault.

## Verification

### Reproduction before the correction

`before-ui/ui_report.json` and `before_ui.txt` show the actual Start button and
ARMING workflow failing with:

```
FAULT: Fresh feedback missing; powered hold cannot be confirmed;
HOLD UNCONFIRMED: Fresh feedback missing; powered hold cannot be confirmed
```

No fake drive had even been enabled. Maximum sample age after Start was
**168.734 ms**. The pre-fix unit study also failed with 40/50 ms GUI intervals
(`before_tests.txt`).

### Full modified-program run after the correction

`after-ui/ui_report.json`, `after_ui.txt`, `after-ui/panel.png`:

1. Startup acquires disabled feedback through state-query packets only.
2. The real confirmed setup generator configures/enables all **16 fake drives**
   at the acquired resting poses, with mode/readback verification and no center.
3. Real isolated 200 Hz workers, real polling/decoding and position send gates,
   but fake sockets, play the saved LEFT and RIGHT trajectories.
4. Emergency Hold retains both simulated arms.
5. Cached RIGHT then LEFT returns reach captured rest; disable replies confirm.
6. The unchanged 120 ms feedback guard passes throughout the sequence.
7. Stop/Close completes through the normal controls.

**All seven checks passed; zero faults.** All four trajectory worker results
reported success (`verified_streams.json`). Median parent polling interval was
**51.128 ms**; maximum feedback age after Start was **97.264 ms**, including
startup, playback, Hold, return and release. ARMING's maximum was **87.073 ms**.

The test does not invoke physical/audio calibration after the goals. It uses the
latest failed run's saved mission/camera scene, not a new camera acquisition.
The packet double replies only to queries/commands, preserves unread replies,
and supplies independent parent/child receive streams. It does not mark every
joint fresh on a GUI callback. A fixture regression checks that a pending newer
reply cannot erase an already delivered one.

### Automated regression suite

**349 tests passed** in `regression_tests.txt`. This includes setup at fixed
20, 33.5, 40, 50 and 60 ms callback intervals and mixed jitter; the existing
finite-CAN-queue congestion tests; real missing-encoder rejection; worker handoff
priority; exact path/controller, ownership, signs, display, Hold/Return/Stop and
normal strike/snare/centering regressions. These are fresh results, not previous
test counts.

`preservation.json` verifies **264 existing inventoried files unchanged**, with
only the two runtime files above differing. The queue-pressure retry helper's
AST is unchanged from the previous correction. Recordings, saved destinations,
planners, timing, calibration, audio models and launchers were not edited.

## Re-run the no-device program check

```bash
cd /home/jason/Proyectos3/X5
source /opt/ros/jazzy/setup.bash
source install/setup.bash
DISPLAY=:0 ROS_DOMAIN_ID=105 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
STRIKE_LAB_OFFLINE_ONLY=1 .venv-planner-experiment/bin/python \
  -m tests.generated_packet_startup_ui_check \
  --report-dir /tmp/generated-packet-startup-check
```

This is the normal `GeneratedApp` with fake device connections, not an arm test.
No physical movement is necessary to verify this software regression. These
results do not certify physical tracking or recovery through a genuine hardware
failure; no hardware acceptance test was performed.
