# Physical smoothing comparison protocol

Updated after the user's speed restriction and before cap-20/cap-30 trials.

- Preserve every method, source snapshot, successful run and failed attempt.
- Speed eligibility: recording segment no longer than 1.25 × record3's original
  3.841315 seconds (4.801644 seconds). This is a working interpretation of
  "close", not a user-specified numerical tolerance. The old player's 7.09575 s
  is a comparator, not the allowable final speed. Slow earlier experiments
  remain recorded but cannot win.
- Keep the same endpoint and the already validated 1.5-degree / 10-mm spatial
  corridor. Current fast trajectory reaches exact source targets in 4.8 s.
- Primary comparison: seven-joint vector RMS of actual encoder position in
  3–15 Hz, 100 Hz interpolation, trim 0.25 s at each end.
- Independent supporting comparison: same processing of motor-reported
  velocity (not differentiation of positions). Lower is better for both.
- Do not select solely from one run or hide a worsening primary metric.
  Screen bounded gain-cap variants, then repeat the promising candidate and
  both comparator types at least three times where hardware is available.
- Comparators: ideal 50 Hz old trajectory; physical replay of old GUI command
  timing captured without hardware. The latter is not a live start_beat run.
- Record feedback, command packets, timestamps, trajectory constraints, video,
  endpoint/center settling, original and restored gains. A camera showing only
  the cymbal is not whole-arm visual evidence.
- Gain writes are volatile, reductions only, within 25–100% of saved originals.
  Save/read back originals before changes; restore/read back at verified center
  before any relax. No firmware flash saves, motor zero or calibration writes.
- Ntfy accepted + 10-second countdown before every physical cycle. Hardware
  faults block motion. Any controlled cancellation must recenter before relax.

## Refinement after the nine-cycle campaign (2026-10-03)

The cap-20 median improved position ripple by 4.5–6.1% and velocity ripple by
11.4–13.5% versus the two contemporaneous comparators. Before further physical
runs, add one cap-10 refinement (same path, 4.8 s, same speed/current ceilings).
It halves the tested cap-20 outer position gains; inner velocity/current gains
remain untouched. At peak commanded joint speed, the simple velocity/gain
following-error estimate is 0.0693 rad (3.97 degrees), below the unchanged
8-degree runtime tracking guard. This estimate is not a physical guarantee.
The write envelope is explicitly expanded to 12.5–100% of saved originals;
only named cap 10/20/30 and factor .5/.75 methods are exposed. All originals
must still be saved, read back, restored and verified at center. No persistent
save, higher speed/current or wider trajectory corridor is authorized by this
refinement. Screen once, then repeat if promising; retain the cap-20 fallback.

## Endpoint-accuracy refinement

The cap-10 screen reduced position/velocity ripple but reached the ordinary
hold criterion with 0.154-degree endpoint error. Add `paced_precise` separately;
retain raw `paced_kp10` unchanged. Playback remains the same 4.8 s cap-10 curve.
At the endpoint it first settles within the ordinary 0.20-degree / 0.12-degree
span bounds, then smoothly ramps position gains to the saved originals over
at least 0.5 s (10 smoothstep stages, all read back). Every stage holds the exact
recording target and aborts the ramp if feedback leaves a 0.25-degree envelope.
It then requires 0.05-degree endpoint accuracy over 0.6 s before recentering.
No relax occurs at the endpoint. On interruption/failure, the existing controlled
center return restores any still-modified gains at center. Other methods keep
their original gain-restoration-at-center behavior.
