# Right experiment: camera-generated dt1 destination

Completed 2026-10-26 local time. Scope: normal experiment only, right-only virtual
destination. No physical motion, enable/disable, hold, relax or gripper commands.

## Implementation

- `experiment/rgbd_camera.py` reuses DT1's native paired camera bridge in its
  existing isolated environment. One short five-pair acquisition, then release.
- `experiment/rgbd.py` retains the existing five-frame confirmed-depth fusion,
  all native pixels, 5 mm occupancy, calibration, margins and observed-obstacle
  policy. The final pair supplies RGB/depth for DT1. Honest paired metadata uses
  a new experiment-only schema; shared/hardware scene acceptance is unchanged.
- `experiment/dt1_goal.py` reuses the existing standalone perception and endpoint
  solver (v4, directed inward yaw +/-20 degrees). It checks the pose and approach
  against the exact five-frame planner world, not a separate single-frame scene.
  Normal workspace path generation and the selected timing policy run afterward.
- The right checkpoint menu contains `dt1`; the left menu does not. No destination
  file is written to the user's destination folders. CLI supports `--destinations
  dt1` and mixed saved/generated checkpoint lists. Later dt1 poses rank approaches
  from the preceding checkpoint. The original acquired pose remains the mask.
- Selection/Calculate fresh generates anew from the frozen pair, with bounded
  serial, no-CAN goal and route jobs. Cancel/switch discards old generations;
  Recapture regenerates dt1. Missing RGB/detection/IK failure has no old-goal fallback.
- Left acquisition, hardware, separate --fast, standalone cymbal generator and
  sibling DT1/Y2 sources are unchanged. No limit/path/timing algorithm tuning.

## Actual-camera and route evidence

1. `fresh_right/`: new paired camera capture and both-arm query-only encoder
   acquisition via the actual experiment CLI. Capture completed in **4.373 s**.
   Five distinct frames, sequence 1–5, encoder brackets for all five. Camera and
   telemetry closed before generation. `can_query_audit.jsonl` independently
   audited: **1,408 sent packets, all exact allowed state queries**; no mode or
   movement commands. See `independent_verification.json`.
2. Same frozen camera data generated dt1 in **16.105 s**, then a full normal
   workspace route in **5.171 s** (Fast timing test). Outward movement **2.788254 s**.
   Default Balanced GUI replay of the same acquisition also passed and produced
   the same movement duration. Times are component observations, not a speedup
   claim or a GPU benchmark; the destination solver is additional work.
3. `native_ui_final/`: real Qt/VTK controls checked right dt1 availability,
   unselected/no-work startup, Cancel, Calculate fresh retry, playable result,
   goal seek/play and absence of dt1 from the simulated-start list. Screenshot:
   `native_ui_final/right_dt1_goal.png`.
4. The actual **Recapture scene** button acquired another new paired scene with
   no CAN. The UI invalidated the prior endpoint, regenerated dt1 from the new
   pair, and planned a new **2.749826 s** outward movement. The acquired arm pose
   and self-mask remained unchanged. This different-scene duration is NOT an
   algorithm comparison. `native_ui_final/ui_report.json` records the checks.
5. Left native window opened, retained saved choices and started unselected;
   no dt1 entry and no calculation. Screenshot `native_ui_final/left_saved_only.png`.
   Mock acquisition tests separately prove the left still uses depth-only capture.
6. Actual launcher offline integration:
   ```bash
   ./plan_drums.sh experiment --right \
     --state diagnostics/experiment_dt1_20261026/fresh_right/acquisition/measured_start.json \
     --scene diagnostics/experiment_dt1_20261026/fresh_right/acquisition/camera/scene.json \
     --headless --destinations R1 dt1 \
     --output diagnostics/experiment_dt1_20261026/checkpoints
   ```
   All three independently checked legs passed: start→R1 **3.777010 s**,
   R1→dt1 **1.374877 s**, dt1→start **3.538406 s** (movement only). Both endpoint
   holds remain two seconds. Full mission in `checkpoints/results.json`.
7. `verify.py`, run inside a 3 GiB no-CAN job, independently reopened **five final
   timing laws**, rechecked exact geometry, dense timing limits and native full-arm
   collision/zone edges. All passed. Independently recomputed **three dt1 endpoint
   geometries**, exact green target and directed inward aiming. Yaw +20 degrees;
   shaft elevations 9.0643, 8.7680 and 16.6851 degrees. Native/model-space precision
   does not certify real-world camera or motor accuracy.

No robot execution was performed. Captured poses and scene are historical as soon
as acquisition finishes; this feature is simulation-only, not hardware readiness.

## Software tests and preservation

`regressions_final.log`: **388 passed, 2 deselected** across:
`test_experiment_dt1`, `test_experiment_acquisition`, `test_planner_experiment`,
`test_experiment_motion_controls`, `test_cymbal_destination`,
`test_planner_camera_diagnostics`, `test_experiment_checkpoints`,
`test_singlearc_arms`, and `test_current_plan`.

Coverage includes paired RGB/last-depth identity, five-frame fusion parity,
tampered data/geometry rejection, legacy depth-only dispatch, right-only CLI/menu,
job serialization/cancellation/retry/stale-result rejection, no-RGB failure,
source/result identity, busy-camera cause/owner reporting, capture timeout cleanup,
and both-arm acquisition cleanup. Existing motor/path/timing preservation tests
pass. The motion-controls historical byte test was updated only to exempt the two
now-authorized acquisition/input-adapter files, retaining all other protections.

Two obsolete historical SHA tests were excluded in the final run after recording
failures: `test_hardware_timing_generators_and_saved_data_unchanged` and
`test_single_leg_worker_and_pool_identity_unchanged_without_checkpoint_fields`.
They compare the user-replaced R1 and previously changed depth.py against old
October 21/22 fixtures. This task's **before** hashes already differ from those
historical values; neither file was modified here. Their assertions were not
weakened to hide those historical mismatches.

An initial native UI diagnostic stopped on a test-harness `KeyError: passed`:
collision proof uses `samples_checked`, not a `passed` field. The test was corrected
and the full UI workflow rerun successfully in `native_ui_final`; the initial
incomplete run is preserved in `native_ui/` and not counted as a pass.

`preservation_audit.json`: 338 initial files hashed; exactly nine intended existing
files changed (experiment adapters/controller/CLI, two tests, documentation),
**329 unchanged**. New implementation files/tests are additive. Standalone cymbal
and sibling source/weight behavior, saved destinations and existing unrelated
working-tree changes were preserved. `git diff --check` passed. Nothing staged,
committed, pushed, deleted or reset.
