# Smooth recording playback in start_beat

`start_beat.sh --hardware` and `start_beat.sh --hardwaretest` now use the
`paced_precise` recording method selected by `playback.sh`. The shell launcher,
flags, default `record1.json`, recording selector, and original recordings are
unchanged. For record3, use the existing option:

```bash
./start_beat.sh --hardware --recording recordings/record3.json
# Or the existing manual-strike workflow:
./start_beat.sh --hardwaretest --recording recordings/record3.json
```

Only the recording segment changes. The existing center/open/load/close-gripper,
move-to-start, endpoint-settling, camera alignment, audio/ESP32 behavior, and
mode-specific striking remain in their existing workflow. In particular, the
standalone player's final recenter/relax sequence is **not** inserted here.
`--test` and offline preview keep their original playback implementation.

## Playback implementation

- `smooth_recording.py` keeps the existing file/schema/zone/return preflight,
  then applies the tested constrained quintic smoothing and joint-specific
  arc-length retiming. Selected recordings are not replaced with record3.
- Movement duration is the source duration multiplied by `4.8 / 3.841315`.
  Record3 therefore remains **4.8 seconds**, plus start/endpoint holding and gain
  restoration. Preflight rejects paths that cannot meet the corridor and dynamic
  bounds at this near-original duration; it does not stretch them arbitrarily.
- Record3's commanded path agrees with the standalone tested method to within
  `2e-14` radians in the numerical regression check. Exact original first and
  last targets are retained. Record1 and record2 also pass preflight.
- A spawned worker schedules targets at **200 Hz**, independently of GUI rendering.
  This is a scheduler ceiling, not a guaranteed physical delivery rate: CAN
  backpressure can delay a write, and the scheduler skips stale deadlines rather
  than replaying a backlog or lengthening the recording.
  It uses the same feedback guards, scheduler, endpoint settling, and temporary
  position-gain cap **10** as the standalone method. Firmware speed ceilings
  are `[.4, .4, .4, .8, .4, .4, .4]` rad/s.
- At the endpoint it performs the tested soft settle, guarded **0.5 s** original
  gain restoration, then the **0.05 degree** endpoint check. Normal **0.4 rad/s**
  settings and original gains are restored before the existing workflow resumes.

## Ownership and errors

The GUI retains its existing CAN locks, feedback polling, gripper commands, and
workflow. The worker receives duplicated lock descriptors and opens independent
receive sockets. Its command allowlist is limited to right joints 1–7 position,
speed, volatile position gain, and read-only CSP-mode checks. It cannot enable,
disable, center, change motor modes, command a gripper/left arm, or save parameters.

The GUI joins the worker before subsequent stage/recovery/relax commands and
before window-close cleanup. Cancellation stops advancement and attempts a
fresh measured-pose hold and guarded restoration; it never independently centers
or relaxes. Lost GUI heartbeat also stops the worker. Failed restoration is
reported and prevents successful alignment handoff; it is not reported as a
completed playback. A forced worker stop explicitly reports unverified settings.

Each live invocation records trajectory metadata, motor feedback, commands,
gain readbacks, and its result beneath `playback_results/start_beat/`.

## Verification boundary

Integration tests use fake CAN sockets, including the spawned worker, with an
audit hook that forbids real PF_CAN sockets. They cover both physical-mode
branches, trajectory equivalence, exact endpoints, 200 Hz scheduling during a
GUI pause, cancellation, watchdog, readback failure, and ownership handoff.
Existing regression and offline Tk checks preserve the later workflows.

Those initial integration checks were **offline**, not physical runs. Subsequent
physical validation of both start_beat modes is described below. Earlier physical
evidence for the standalone method remains in `PLAYBACK_RESULTS.md`. Initial
integration logs and the original app backup remain under
`playback_results/start_beat_integration/`.

## CAN queue-pressure repair and no-strike physical test

The GUI now stops issuing duplicate state queries once the playback worker has
established its independent feedback stream. Both GUI and worker socket writes
retry transient `ENOBUFS`/`EAGAIN` for a bounded period instead of dropping a
command or treating one full transmit queue as a fatal motor fault. Other CAN
errors still fail explicitly. These changes are local to camera playback; the
shared motor driver and standalone player are not modified.

To exercise the real start_beat program without alignment or any strike:

```bash
./start_beat.sh --hardwaretest --recording recordings/record3.json --recording-only
# Same recording transport in the normal hardware mode:
./start_beat.sh --hardware --recording recordings/record3.json --recording-only
```

This explicit test option keeps the usual center/open/load/close/start sequence,
then plays the recording, returns along a validated smooth path, and relaxes
**only after fresh measured center settling**. It disables the ESP32 and strike
entry points, forbids MIT commands, and allows the cymbal to be removed (fresh
camera frames are still required). Close/Escape/errors request the same center
return; missing feedback prevents blind movement or relaxation. Its transport
gate independently checks every actual disable command against a <=0.20 degree
center envelope held for >=0.6 seconds with <=0.12 degree span.

`--auto-run` additionally starts the sequence, continues after a **five-second
loading pause**, and exits after confirmed relaxation. It is accepted **only**
with `--recording-only`; keep hands clear before using it. Normal modes without
`--recording-only` retain their existing alignment/strike workflow.

No-strike run results, phase history, return traces, per-disable center evidence,
and transport retry counts are saved in `playback_results/start_beat_recording_only/`.
The referenced worker directory contains the original-endpoint verification,
recording trace, feedback, and gain restoration evidence.

### Physical verification: 2026-10-03

After ntfy warnings, the actual `start_beat.sh` completed record3 in **both**
`--hardwaretest` and `--hardware`, each with `--recording-only --auto-run`.
Both runs played the full **4.8-second** trajectory (original 3.841315 seconds),
verified the exact original endpoint to **0.02198 degrees** maximum joint error,
returned to center, verified settling, and only then relaxed. Maximum joint
error at disable was **0.03297 degrees** and **0.02148 degrees**, respectively.
Neither entered pink-zone alignment or striking. The cymbal was not hit.

Both final launches exited cleanly without CAN/control faults. The read-only
postflight confirmed all sixteen motors relaxed, original position gains restored,
and both buses ERROR-ACTIVE with zero current TX/RX error counters. All **385**
offline regression tests and four real-Tk/no-hardware mode checks passed.

Logs, videos, failed intermediate attempts, source snapshots, and the reproducible
offline evidence audit are retained in `playback_results/buffer_fix/`;
see `playback_results/buffer_fix/RESULTS.md`. These tests deliberately did **not**
exercise alignment or strikes. Without `--recording-only`, those original later
stages remain enabled. All original recordings, `start_beat.sh`, the shared motor
driver, and the standalone player are unchanged by this repair.
