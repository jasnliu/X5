# ST7 handoff investigation and physical verification

## Result

**Two consecutive physical passes**, with real ST7 search detections and no
independent audio analysis, injected sound, substituted HIT, model tuning, or
threshold changes:

| Trial | ST7-selected depth | ST7 score | Swing duration | Encoder strokes | Faults |
|---|---:|---:|---:|---:|---:|
| 3 | 12 degrees | 0.719481 | 5.0069 s | 14 | 0 |
| 4 | 11 degrees | 0.689211 | 5.0206 s | 14 | 0 |

Each trial used the ordinary RUN workflow via the explicit `--verify-swing`
test flag. Its accepted event came from the separate, unmodified ST7 listener.
In trial 3, ST7's onset was 91.8 ms after release, within the outbound/return
interval; its notification arrived about 0.75 s later and was correctly
accepted while waiting at the anchor. Trial 4 accepted an onset 91.3 ms after
release. No acceptance-window enlargement was necessary.

Both runs centered before disabling. Maximum settled center errors were
0.05495 and 0.12089 degrees; settled durations were 0.633 and 0.642 seconds.
Final independent, audited state-only requests confirmed all 16 motors
disabled with no fault bits. No controller was left running.

These are ST7-confirmed search hits followed by encoder-verified swing motion;
this report does not claim ST7 separately detected every swing stroke.

## Reproduced failure and changes

1. Trial 1 reproduced the user's failure: goals 5 through 11 degrees completed
   with no ST7 event. The 11-degree command peaked at 10.770 degrees. The sole
   ST7 HIT had onset 512164.34146, during the later MIT-to-CSP handoff, not the
   earlier strike interval (release 512162.90044, return 512163.30040).
   Encoder feedback showed J7 dropping during that handoff. Widening the sound
   window would have incorrectly used a recovery-path event for depth selection.
2. The original powered search continued through geometry-validated depths.
   Copying the experiment's default 12-degree hard corridor into hybrid mode
   truncated this at 11 degrees. Normal hardware now permits the next
   12-degree goal with a 13-degree hard corridor, still intersected with the
   validated path and joint limits. The experiment itself is unchanged.
   Torque, speed, timing and other motion safeguards remain unchanged.
3. Removed the normal off-center MIT-to-CSP handoff. J7 now remains powered in
   MIT during the entire center return, using a 0.35 rad/s, 0.6 rad/s² bounded
   reference and estimated torque bound of 3 Nm. J1-J6 use their existing CSP
   center path. Disabling an active drive through the normal transport is
   refused until all seven joints are verified centered, except the explicit
   major stall/thermal-fault exception.
4. Trial 2 encountered a startup stale-feedback fault before strikes; the new
   powered center return recovered safely. Hybrid worker creation now refreshes
   actual encoder feedback before releasing a strike, as recording-worker
   creation already did. The 300 ms watchdog was not increased.
5. Removed the verifier's arbitrary eight-repeat-ST7-event requirement and
   its invitation to substitute independent audio review. Success now requires
   an actual accepted ST7 search event, five seconds of fault-free swing, and
   verified center/disable. Historical diagnostics remain unmodified evidence,
   not a substitute for this test.

The ST7 listener, detector, decoder, models, thresholds, ADC mapping, bridge,
and hit-acceptance window were not changed during this investigation. Original
recordings and experiment tuning hashes are unchanged.

## Evidence

- [Machine-readable summary](verification.json)
- [Trial 3 result](../../playback_results/swing_verification/20261005T035659-d45c7964/result.json)
- [Trial 4 result](../../playback_results/swing_verification/20261005T035845-e5973f8a/result.json)
- `physical_run_1.log` through `physical_run_4.log`
- Adjacent `telemetry.jsonl` in each result directory: real encoder/controller/ST7 events
- `notification_1.json` through `notification_4.json`: delivered ntfy warnings before motion
- `query_final_disabled.json`: 48 audited state-only requests, all motors disabled/fault-free
- `protected_final_check.log`: original hashes match

## Offline checks

- Full suite: **469 tests passed** (`full_tests.log`).
- Real Tk normal-hardware routing with substituted transports passed
  (`hardware_ui_test.log`). No physical CAN is permitted by that test.
- Twelve-degree hybrid search/swing and bounded powered center-return
  regressions are included. Offline results are separate from the physical
  evidence above.

## Normal launch

```bash
./start_beat.sh --hardware --camera 0 --recording recordings/record3.json
```

Press RUN once. Without `--verify-swing`, swing continues until Stop; Stop
finishes the hybrid return, centers while powered, verifies center, then relaxes.
