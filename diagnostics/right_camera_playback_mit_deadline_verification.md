# Hardware-test deadline repair — 2026-09-30

## Changes
- J7 control runs in a spawned process, not in the Tk/ROS interpreter's thread.
- 500 Hz active strikes / 100 Hz idle hold. Active control and feedback limits
  remain 20 ms. Idle gaps up to 250 ms keep the same hold, reset readiness and
  require fresh settling before another strike; longer gaps still fault.
- Deadline failures include phase, measured interval and threshold.
- Parent state-only J7 queries continue during process startup.
- Session entry confirms disable, mode selection and running state. It retries
  ignored mode writes and enables while arming only; never retries a strike.

## Physical findings and verification
The initial record1 attempt failed and is NOT counted as passing. The first
record3 capture identified an ignored MIT-mode write: the disabled drive kept
returning mode 5 (CSP). Waiting for an acknowledgement without retrying the
write left J7 disabled. This explains the observed limp joint; it was not a
successful hold. Subsequent power-off also caused the later CAN/ENOBUFS errors.

The final record3 attempt succeeded using the production controller, physical
CAN/motors, normal recording playback, RViz, camera and ST7/audio processes.
The existing ntfy notifier delivered an alert before each movement test/recovery.
No stick was loaded and no strike was requested. A temporary external test
subclass skipped human loading and visual acceptance, using the recorded endpoint
for MIT hold. Production visual acceptance and loading were not bypassed.

Evidence: `right_camera_playback_mit_record3_physical.txt`,
`right_camera_playback_mit_record3_verified.json`,
`right_camera_playback_mit_record3_can.txt` and
`right_camera_playback_mit_record3_metrics.json`.

- Recording: `recordings/record3.json` (unchanged).
- Real endpoint reached; J7 entered MIT and held for approximately 88 seconds.
- No deadline, stale-feedback, drive or zone fault during the successful run.
- After the first second, 8,690 captured J7 feedback frames all reported running;
  maximum measured anchor error in those samples was 0.044 degrees.
- Successful handoff capture shows four mode writes before mode 0 confirmation.
- Entry was not motionless: the unloaded J7 initially dipped about 2.7 degrees
  while changing modes, then recovered. This test does not establish loaded
  strike accuracy, sound consistency, or a jitter-free readiness indicator.
- Motor velocity noise made the ready indicator toggle. The test harness's
  first automatic Center request met a stale GUI phase; invoking the normal
  enabled Center + Relax button completed CSP restoration and centering.
- All 16 drives confirmed disabled at test completion; a separate state-only
  postflight rechecked that result. Both CAN buses were ERROR-ACTIVE, counters 0.
- Camera KeyboardInterrupt/exit -2 appears during normal launcher teardown;
  this is not a motor-control failure.

## Offline verification
- 271 unit/regression tests passed, including a real spawned process tested
  under parent interpreter/GIL contention, idle-gap recovery, strict active
  deadline, startup queries, ignored mode writes, and ignored enables.
- All three playback GUI checks passed without physical CAN access.
- Logs: `right_camera_playback_mit_deadline_tests.txt`,
  `right_camera_playback_mit_deadline_ui.txt`,
  `right_camera_playback_mit_deadline_postflight.txt`.

The test automation snapshot is preserved as `...record3_harness.py.txt` for
review, not as an executable/default test. Future physical playback tests should
explicitly select record3. No recording data or standard application recording
default was changed.
