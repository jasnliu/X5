# Occupied-volume clearance correction — 2026-10-21

## Exact incident

Input: `planner_runs/20261021T170415Z-baseline-8b9ce3/calculate-2e584558`.
The endpoint checker returned `[false, true]`: the **start** collided, while the
record3 goal was clear. The route search therefore never began. This was not a
feedback timeout, missing seed, invalid joint limit, or bad goal.

Five retained Gemini voxels near the starting drumstick tip caused the result.
`colliders.json` links them to the original depth pixels and five readings.
These are temporally supported returns, not a justification to erase obstacles
or enlarge the robot self-mask.

The closest voxel center is 38.320 mm from the nominal stick surface. A 10 mm
cube's half-diagonal is 8.660 mm; therefore every occupied cube in this scene
is **at least 29.660 mm** from the nominal starting stick. The original URDF,
start angles, 14.8 mm stick diameter and 409.4 mm modeled length were retained.

The old checker used robot padding of **20 + 9 + 5 = 34 mm** against full
10 mm OctoMap boxes. The 9 mm voxel-radius allowance counted voxelization again,
even though the occupied boxes already enclose the original returns. This
turned the clear start into a collision. See `endpoint_report.json`.

## Correction and boundaries

New requests explicitly select `world_padding_policy: occupied_volume_v1`:

- Full occupied OctoMap cells, not point obstacles or shrunken cells.
- Same **20 mm requested clearance**, plus the same **5 mm edge-sampling allowance**.
- No extra robot dilation for voxel radius: that extent is already in the cells.
- All 122,667 camera voxels are supplied unchanged. Existing initial-pose
  self-mask/shadow rules and their removals are identical.
- No collision exemptions, clipped starting joints, shifted endpoints, altered
  calibration, smaller stick model, or unchecked initial escape segment.
- Same Single arc candidate generation/ranking, latest timing and motion limits.
- Planning oracle, dense final checker, preparation and recovery use the same
  request. Native margin metadata is checked and bound into prepared packets.
- Candidate cache keys include the policy. Old requests lacking the field keep
  `legacy_voxel_radius_v1` and their original 34 mm padding; archived plans are
  not silently reinterpreted. Unknown policy identifiers fail explicitly.

## Software verification

`incident_case.json` is a small, standalone five-voxel regression fixture.
`tests/test_voxel_clearance.py` checks the exact old failure/new success, a
conservative distance bound, identical self filtering, legacy compatibility,
real-obstacle rejection, blocked connecting edges despite clear endpoints,
oracle/final-check parity, cache separation, both-arm request propagation,
prepared margin proofs and invalid/mismatched policy rejection.

The full Calculate harness was also run with **the exact incident's saved five
camera frames and encoder counts**, substituting only the physical sensor I/O.
The real synchronizer, query decoder/monitor, planner, timing solver, and dense
native collision/zone checks all ran. Result:

- Complete calculation and preparation **passed**, in 123.22 seconds.
- **4.036636-second** outward Single arc trajectory (candidate 13).
- **2,020** exact-curve states plus connecting edges passed final checks.
- **2,427** simulated feedback batches, no feedback errors; maximum batch
  duration 10.14 ms and maximum start gap 50.71 ms.
- Raw depth archive, fused depth, free-depth image and obstacle-cloud files are
  **byte-identical** to the failed capture; start/goal/passive-arm values and
  generated URDF are identical. See `preservation_report.json`.
- Simulated acquisition flags were revoked on disk after validation, so this
  test result cannot be mistaken for a fresh hardware-authorized calculation.

All tests are device-free. No real camera, CAN bus, arm, gripper, or motor mode
was accessed. These results do not certify live tracking or physical clearance.

To rerun the self-contained checks, source ROS/install and run
`pytest tests/test_voxel_clearance.py`. `run_tests.sh` runs the broader suite.
`verify_calculation.py OUTPUT` uses the locally archived incident (not a runtime
input dependency). `run_ui.sh` uses the generated software result with the
native three-window UI and strictly simulated motor-owner services.

Native GUI verification: **21 checks passed**, covering separate live/
planned previews, play/pause/scrub/speed, all five buttons, invalidation/retry,
full simulated cycle, emergency hold/reverse, injected-fault recovery, and no
automatic midair release. All three simulated owners ended returned/relaxed.
The real launcher also passed in `baseline --simulate` mode.

Final regression results: **920 tests passed in 298.48 seconds**, including
16 new occupied-volume tests. The focused planner/hardware-policy suite passed
228 tests; the final clean-artifact/voxel rerun passed 26 tests. No unexpected
failures remained. Intentionally injected faults were checked and recovered,
not suppressed. Native UI renderers reported no rendering errors.

The calculation cgroup (including one-time native compilation) peaked at
1,254.6 MiB, below the unchanged 3 GiB limit. Test-owned services were fully
reaped. No physical movement or device access took place.
