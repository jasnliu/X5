# Hi-hat calibration to arm-start stale-status fix

## Cause and implementation

Recording geometry validation/retiming used to execute synchronously on the Tk control loop. After calibration, loading a recording could suspend ESP32 queries/reads longer than the 1.5-second status timeout; the next tick sent a query and faulted before its reply arrived.

Normal `--hardware` now runs the identical smooth-recording loader in a calculation-only background worker. It receives no motors, serial ports, sockets, widgets or GUI callbacks. Progress/results cross a queue to the Tk thread. A pending/failed replacement load clears the previous recording; RUN is gated until successful validation. Cancellation/close discards late results. RUN additionally requests and requires fresh ESP32 telemetry before enabling the arm. The calibration watchdog itself and all actual motion/model parameters are unchanged. No automatic ntfy alert/countdown was reintroduced.

The existing UI layout, ride search, +0.5 degree boost, swing scheduling, hi-hat synchronization, calibration 90..115 by 5 degrees, 2-second holds, firmware and other launch modes are preserved. The launcher now describes the hi-hat detector's actual calibration/timing role rather than calling it visual-only.

## Physical acceptance: PASS

A delivered ntfy warning preceded hardware use (`notification.json`). The ordinary launch graph ran with the Logitech camera, TONOR input, unmodified ST7 ride v2 and hi-hat v1, actual arm and actual hi-hat. The test wrapper only deferred the recording choice until hi-hat calibration succeeded, then called the ordinary `load_recording(record3)` and existing bounded `--verify-swing` RUN/Center+Relax helper. No synthetic feedback, assisted detections or altered motion parameters were used.

- Hi-hat calibrated at **90 degrees**, with a real hi-hat ST7 HIT. Closed hold: 2.034 s; subsequent open hold: 2.028 s.
- Then record3 preflight took **38.365 s** under full camera/detector load, with **73 ESP32 status replies** and **289 heartbeats** received/sent during validation. Maximum heartbeat gap: 0.243 s (below the unchanged 400 ms firmware watchdog). Arm remained disabled during loading.
- RUN centered/closed the gripper, played record3 and verified its endpoint; camera alignment and the unchanged ride search followed.
- Ride ST7 detected **10 degrees**; the swing used **10.5 degrees**, preserving the existing boost.
- **5.037 seconds of continuous 100 BPM swing**, **14 encoder strikes**, **14 matched real ride ST7 detections**, **zero faults**. Hi-hat closures/detections and synchronization ran concurrently.
- Verified customized right center before relax: maximum center error 0.0769 degrees, settled 0.615 s. Only then were the motors disabled.
- Post-exit query-only audit confirmed all 16 drives disabled/fault-free. No test controllers or serial/camera owners remained running.

Evidence: `physical_launch.log`, `physical_background.jsonl`, `physical_background_result.json`, `physical_summary.json`, `query_before.json`, `query_after.json`.

Raw physical calibration: `playback_results/hihat_calibration/20261005T233429-d9cf185e/events.jsonl`.
Raw physical swing evidence: `/home/jason/Proyectos3/X5/playback_results/swing_verification/20261005T233429-0b99b0fc`.
Camera recording: `/home/jason/Proyectos3/Y2/recordings/20261005_163429_630834.mp4`.

## Offline checks

- The actual normal-hardware Tk App with mocked devices completed calibration followed by real record3 validation: 13.244 s validation, 26 status queries during load, max GUI gap 0.120 s. Unlike the old test, telemetry only becomes fresh after a queried reply; it is not refreshed unconditionally every tick.
- Added regression tests for nonblocking calculation, queued failure, cancellation, old-recording rejection, fresh-before-enable, missing/future replies, and preserving real stale-status faults.
- **582 tests passed** in 136.933 s (`full_tests_final.log`), including 11 added regression tests. The first full pass also passed all 579 tests before the final three coverage additions.
- Syntax checks passed. 522 protected file hashes matched, including original recordings, datasets, models, firmware and ride/timing code (`protected_verification.json`).

`before/` preserves original edited files; `changes.patch` records the change.
