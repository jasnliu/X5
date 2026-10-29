> **Approved follow-up complete:** ±25° was authorized and implemented. Both
> actual-camera replay missions and the standalone destination now pass. See
> [APPROVED_REPORT.md](APPROVED_REPORT.md). The investigation below retains its
> original before-approval/pending statements as historical evidence.

# DT1 startup IK rejection — investigation in progress

No arm/ESP32 actuation. Application aiming policy remains **±20 degrees**.
Changing it requires the operator's pending approval, not a silent solver fallback.

## Inputs and reproduction

Original failed run:
`planner_runs/generated-beat-20261028T083720-5ac2c7a8/plan-dd96e4f6/right/dt1/checkpoint_1/`.
Exact green point `[0.5456778764161799, -0.3773627777464458, 0.48015632163673116]`.
Unmodified generated model, goal reserves, exact tip, above-parallel, J7 roll,
negative-J7 direction and inward ±20-degree constraints.

- Original 68 least-squares starts reproduced zero acceptable endpoints.
- Analytic Jacobian agrees with numerical differentiation to 6.5e-11.
- Some least-squares solutions point vertically down the disc normal: the cone
  residual is zero but projected inward direction is undefined. Final geometry
  checking correctly rejects these. They are NOT usable destinations.
- An independent directed-orientation formulation (290 starts, no pole/roll-sign
  shortcut) also found no allowed endpoint. This is bounded numerical evidence,
  not a global impossibility proof.
- Diagnostic minimization of inward yaw with all other endpoint constraints and
  reserves retained found +21.169033 degrees, with J2 lower, J6 lower and J7 upper
  **goal reserve** boundaries active. Exact tip error 3.6e-14 m, exact J7 roll.
  This candidate is not exported or accepted at the production ±20-degree range.
- Non-exportable isolated sensitivity at ±25 degrees found 19 collision/zone-valid
  endpoints and a densely checked approach. No source setting was changed.

## Fresh no-actuation program run

`./start_beat.sh --hardware --prepare-only`

Fresh result directory:
`planner_runs/generated-prepare-20261028T085139-7a80836a/`.
The left path passed at 3.723570 s movement. Right DT1 reproduced the ±20-degree
rejection at a fresh green point `[0.5482295140636008,-0.3794690842285847,0.4812309977071252]`.
No full mission is marked ready. Query audit: 728 state queries per bus,
zero enable/disable/hold/motion/gripper commands, query connections closed.
There was no ESP32 or audio connection.

An earlier read-only capture attempt returned corrupt RGB / no synchronized
pair; the next attempt succeeded without modifying camera code or calibration.
All 3,712 sends in that failed capture were state queries only, also closed.

## Software changes so far

- `--hardware --prepare-only`: same acquisition and normal bounded planning
  workers; no controller, no hi-hat calibration, no microphone, no execution.
- More specific non-exportable IK failure diagnosis reports the best exact-tip
  aiming angle found and active goal reserves. It does not change acceptance.
- Tests cover the recorded failure, rejected outside-range candidate, retained
  limits, launcher routing, worker cleanup/failure, and all relevant old controls.

Initial combined regression: 257 passed (two benign SLSQP trial-bound warnings).
Final rerun and any approved actual fix will be appended below.

**Pending:** operator decision on allowing ±25-degree aim versus keeping ±20.
Joint/motor bounds and reserves must not be widened to hide this conflict.

Final current-scope regression rerun: **258 passed**, two SLSQP trial-bound
warnings, 12.66 s. See `tests.log`. Production solver and ±20-degree setting
remain unchanged. The specific failure is not claimed fixed pending the operator's
constraint decision; no ready normal-hardware mission was fabricated.
