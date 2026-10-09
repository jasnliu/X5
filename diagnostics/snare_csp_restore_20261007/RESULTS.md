# Snare CSP handoff fix — 2026-10-07

## Cause and correction

The old post-strike handoff could accept a cached MIT-mode RUNNING state before
the disable/configure/enable transition was acknowledged. It also swallowed
setup exceptions. Reverse playback could then encounter a stopped J6 and retry.
This code defect explains the reported timing; the original log alone does not
identify a specific electrical fault.

`snare_lab/runner.py` now uses the existing confirmed motor-setup handshake:
CSP mode readback before enable, fresh running feedback after enable, then a
short stable-running confirmation window before reverse playback. Setup errors
propagate and unconfirmed handoffs block reverse playback. Recovery only skips
restoration when J6 is running in CSP. Successful final relaxation now requires
fresh disabled feedback from all eight left motors.

Strike depth, the curved downward trajectory, return trajectory, dynamic limits,
recordings, and shared motor code were not changed by this fix. Preserved source
hashes are recorded in `preservation.sha256`.

## Physical verification

Exactly one physical trial was performed, after a successfully delivered motion
notification and query-only checks that all 16 motors were disabled:

```bash
./snare.sh --hardware --10
```

- Commanded strike: **10 degrees**, with no other physical strike depth tested.
- Measured peak excursion: **9.956640 degrees** (encoder measurement, not a changed command).
- Complete sequence: center → close gripper → recording → strike → confirmed
  CSP handoff → reverse recording → center → confirmed relax.
- Program exit: **0**, approximately **30.595 seconds** elapsed.
- Program fault/retry messages: **0**.
- Passive CAN audit: **16,229 feedback frames; 0 motor-fault frames**.
- Handoff: one disable and one enable, with three CSP mode-5 readbacks.
- No nonrunning left feedback during reverse playback.
- Maximum final centering joint error before relaxation: **0.032969 degrees**.
- Independent postflight state queries: **all 16 motors disabled**.
- Right-arm motor writes: **0**.

Evidence: `run1/program.log`, `run1/summary.json`, `run1/can.log`,
`run1/can_audit.json`, `run1/postflight_disabled.json`, and
`run1/physical_video.avi`. The video contains 919 readable frames through
confirmed relaxation; extracted before-strike, peak-strike and centered images
were visually inspected.

Recording-only caveat: after the robot had exited successfully, termination of
the separate camera process interrupted its read and produced `Camera stopped`
in `run1/camera.log`. The finalized video is readable and includes completion.
This was not a robot fault. The raw log is retained; the capture helper now
recognizes its requested shutdown instead of reporting that interruption as an
error.

This verifies one real, complete 10-degree run, not long-term endurance or drum
loudness/contact speed.

## Offline checks

- Eight mocked handoff/relaxation regression tests pass, covering stale feedback,
  wrong mode, stopped feedback, setup exceptions, and bounded timeouts.
- All 26 existing centering tests pass.
- A 10-degree simulation completes successfully.
- Preserved-file SHA-256 checks pass.

See `handoff_tests.log`, `centering_tests.log`, and `simulation_10deg.log`.
