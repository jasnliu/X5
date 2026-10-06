# Lighter fall damping and continuous turnaround — offline verification

## Scope
Only the `--hardwaretest` J7 trajectory and its wording/default documentation
changed. No physical test, CAN query, motor command, camera/audio capture, or
hardware launch was performed. Existing physical reports predate this change.

## Implementation
- Default fall KD reduced from 0.08 to 0.04. KP and feed-forward torque remain
  zero during gravity descent. The existing `--mit-fall-kd` override remains.
- Catch velocity reaches zero with positive upward acceleration, inherited by
  the return polynomial. There is no bottom settling gate or rest-to-rest restart.
- The return still ends at the fixed upper anchor with zero velocity and
  acceleration; existing speed/acceleration limits and readiness gates remain.
- Predictive catching uses the new trajectory's actual planned minimum position.
  The additional catch distance is join_acceleration * catch_duration^2 / 12.
- Join acceleration is bounded so the return remains monotonic and respects
  existing limits, including speed-limited, small-stroke and low-velocity cases.
- No impact detection, audio trigger, new control thread, or safety layer added.

## Verification
- 51 focused controller/transport/lifecycle tests passed.
- 288 full offline regression tests passed.
- All three offline camera-playback UI checks passed (hardwaretest, test,
  ordinary playback); hardware disabled or mocked.
- Python compilation, shell syntax, and launcher help/default checks passed.
- New regressions cover position/velocity/acceleration continuity at turnaround,
  positive upward motion on the next control tick, the updated catch prediction,
  endpoint rest, and late-tick time carry-through without a bottom hold.
- Trajectory-bound tests cover 120 combinations of stroke, incoming velocity,
  control rate, braking acceleration, return acceleration and return speed.
- Existing inertial simulations still pass for 1/2/5/10-degree strokes, command
  delays, repeated strikes, inertia mismatch and slower encoder feedback.

These tests verify software/reference trajectories and synthetic dynamics, not
physical contact duration, sound, loaded-arm accuracy, or the best damping value.
0.04 is an initial tuning choice, not a measured physical calibration.

Logs: `right_camera_playback_mit_turnaround_tests.txt`,
`right_camera_playback_mit_turnaround_regressions.txt`,
`right_camera_playback_mit_turnaround_ui.txt`.
