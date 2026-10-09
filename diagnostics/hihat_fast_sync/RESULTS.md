# Faster hi-hat synchronization — offline verification

## Change

- Two consecutive valid estimates must agree within 20 ms before updating.
- Use the full absolute correction from a three-estimate median (two estimates
  for the first correction), instead of accumulating 35%/10 ms increments.
- Actual upcoming close/open pairs may shift by at most `min(150 ms, period/4)`.
  At 100 BPM this is 150 ms; at 120 BPM it is 125 ms. The scheduled open interval
  remains at least 75% of a quarter note; the closed interval remains a full
  quarter note. No already-scheduled edge is moved.
- Keep the 15 ms deadband, original signed advance bounds, acoustic-onset pairing,
  detector finalization/freshness checks, ambiguous-event rejection, deadline
  faults and controlled stop. Missing/ambiguous pairs reset confirmation history.
- Continue using each historical command's **actual** advance to reconstruct an
  absolute latency estimate. Delayed errors cannot compound an already applied
  correction. New start/match log fields identify the policy and confirmation.
- Ride timing, snare strikes, motor speed/angle settings, calibration and firmware
  are unchanged; see `unchanged.sha256`.

## Deterministic before/after simulation

At 100 BPM, times below run from the **first close's grid target** until the first
close using a correction within the existing 15 ms tolerance. Simulated detector
delivery delays and finalization gates are included, not removed.

| Scenario | Before | After | First corrected closure, before → after |
| --- | ---: | ---: | ---: |
| 100 ms late | 14.4 s | 3.6 s | 13 → 4 |
| 135 ms late, longer delivery delay | 20.4 s | 4.8 s | 18 → 5 |
| 80 ms early | 12.0 s | 3.6 s | 11 → 4 |
| 100 ms late, 2.4+ s delivery lag | 16.8 s | 6.0 s | 15 → 6 |

All four new-controller scenarios reached the exact simulated offset without
overshooting it. See `comparison.json`. This is **simulation, not physical timing
validation or proof of a maximum stable hardware setting**. Real detection gaps
and latency changes can still postpone a correction.

The pre-change source is preserved in `before/`. To reproduce the comparison:

```bash
cd /home/jason/Proyectos3/X5
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 diagnostics/hihat_fast_sync/compare.py
```

## Regression coverage

Tests exercise full signed corrections, confirmation, isolated outliers,
conflicting estimates, timing jitter, real offset reversals, actual-versus-planned
send time, missing/ambiguous events, stale detectors, signed bounds, delayed
feedback, 100 ms settling time, tempo-scaled step limits, close/open pairing and
unchanged ride timing. Tests use mock sends and synthetic acoustic events only.

**95 focused tests passed; the full repository suite passed all 796 tests**
(242.95 seconds). Test outputs: `targeted_tests.log` and `full_tests.log`.
No robot movement, serial connection or CAN connection was initiated for this
change. A running application must be stopped normally and restarted to load it.
