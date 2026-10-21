# Baseline error review — 2026-10-19

Scope: the production three-window `./plan_drums.sh baseline` workflow, its
read-only capture/check workers, independent motor owner and return checker.
No change to `start_beat.sh`, experimental algorithms, collision margins, zone3,
recordings, motor gains or motion/settling limits.

## Confirmed in the saved runs

- `20261020T000726Z-baseline-3e6a64`: right playback entered FAULT_HOLD solely
  because the relaxed left arm moved. Its return subsequently completed.
- `20261020T000335Z-baseline-b2a953`: two Start checks rejected paths after new
  depth acquisitions; a later attempt completed. These checks were testing a
  DIFFERENT snapshot from Calculate/preview. This is not proof that either
  reported collision was fake, but it is a mismatch with the frozen-scene workflow.

## Removed / downgraded

| Former blocker | New behavior |
|---|---|
| Left-joint/gripper change since Calculate | Display it; do not invalidate the right plan. |
| Left-joint variation in camera synchronization or post-plan observation | Only right joints must be stationary. Both arms are still measured; no clipping or invented left pose. Shared non-baseline capture keeps its previous default. |
| Left pose/opening differs at prepare | Not a right-start mismatch. Collision checking retains the acquired left pose. |
| Static-left movement during outward/hold/normal return | Not a playback fault; no forced pause. |
| Left sway while the recovery checker runs or during recovery | No movement veto. The checker still checks geometry at its measured snapshot. |
| New depth capture and scene recheck on every Start | Calculate captures/plans/densely validates once; Start uses that exact scene/proof. |
| Prepared scene expires after 30 seconds | No timer-based invalidation. User recalculates if obstacles/camera change. |
| Runtime event log, CAN audit or status-file write failure | Warning surfaced in owner status, not a motion stop or recovery veto. |
| Offline demo claims local simulated pose became stale during rendering | Sample the local simulated pose on demand. Actual hardware/owner feedback timestamps remain unchanged. |

Left motor mode/fault/temperature/feedback validity checks are not **motion**
checks; ordinary passive swinging does not trip them. Left stays query-only.
The snapshot collision model includes the measured left pose; this is not a
claim of live inter-arm collision avoidance or arbitrary left-arm movement safety.

## Remaining checks, reviewed by category

- **Actual motor/communication health:** missing/stale feedback, malformed CAN,
  drive fault/overtemperature, unexpectedly disabled right motor, unexpected
  competing controller traffic. These are not evidence of ordinary left sway.
- **Right-arm execution:** finite/bounded positions, closed custom-gripper
  target, start-pose match, enable confirmation, scheduler deadline, target-step
  continuity, existing sustained tracking/stall monitor. These prevent sending
  a different or discontinuous motion than the checked curve.
- **Hold and release:** right stationary hold before return; checked traveled
  prefix and small tracking-offset connector; settled original right pose before
  normal release. No warning/fault silently relaxes an arm in the air.
- **Collision/zone and plan data:** dense full arm/stick path checks on the
  captured scene, zone3's gripper point, finite trajectory/timing/limits,
  correct model/calibration/units/hashes/boot, exact endpoints, return proof
  identity. These checks are retained, not bypassed to make failures disappear.
- **Acquisition:** usable calibrated depth, five ordered frames, right-stationary
  encoder/camera bracketing, finite complete disabled-motor measurements. Missing
  data cannot become a fabricated scene or starting pose.
- **Lifecycle:** one CAN owner, explicit Start, cancel does not queue motion,
  GUI connection loss holds, disconnected feedback is labeled stale. Wrong-phase
  clicks explain the next action; all controls remain clickable and retries work.
- **Required files/resources:** missing or corrupt plan/config/proof, failed
  planner/worker/dependency, memory limit or unavailable device remains a visible
  retryable error. Optional runtime diagnostic writes no longer veto motion.

All actual-motion tests for this change are simulated; physical testing must be
reported separately. Changes do not promise a particular real-hardware success rate.
