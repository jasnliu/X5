# Final-checkpoint return correction — 2026-10-21

Multi-checkpoint experiment playback now plans an additional independent leg from
its final selected checkpoint to the original saved start. It no longer reverses
the outward checkpoint sequence or revisits earlier checkpoints as required stops.
The same selected planner, timing policy, original frozen scene/self-mask, passive
arm, zones and collision margins apply to the return. Two-second outward checkpoint
holds remain. A blocked return is reported without silently substituting a reversed
checkpoint route; retry and selection controls remain usable.

Single-destination previews and all hardware modes are unchanged. Endpoint-only
return targets are private result artifacts, not additions to user destinations.
Old reverse-route aggregate previews use a different schema and are not reused;
unchanged individual-leg searches and timing remain cacheable.

## Verification

- 459 regression tests passed; source-preservation checks cover hardware,
  path generators, timing, model, scene, calibration, saved poses and destinations.
  One existing SLSQP trial-step bounds warning remains (also in the prior report);
  no test failures. This is not robot encoder clipping.
- 30 native Qt/VTK full-cycle playback checks: five methods, both arms, one/two/three
  destinations. All planned and played successfully. Twenty multi-checkpoint cycles
  used newly planned returns with complete collision/zone proofs.
- All 30 outward sample sequences and analytic timing laws equal the previous
  successful checkpoint runs exactly; all single-destination cycles also match.
- Exact return-law evaluation, final-to-original endpoints, original snapshot mask,
  outward holds, end-at-start, play/pause, scrub, speed, loop, Go to goal/start,
  add/remove/max/exclusion, cancellation/retry and stale-result rejection checked.
- Failure injection verifies blocked-return recovery without partial playback.
- No camera access, CAN access or motor commands. Left's additional endpoints are
  small test-only variations because only one user destination currently exists.
- Worker memory cap remains 3 GiB; observed peak 370.8 MiB. Serial work and
  cancellation remain intact. Multi-checkpoint time ceiling includes the new return
  leg, without reducing solver refinement quality.

The older `diagnostics/checkpoints_20261021/REPORT.md` records the superseded
reverse-through-checkpoints behavior. This report and `verification.json` document
the correction. Large plans/caches stay local under `planner_runs/`.

## Full-cycle timing evidence

| Arm | Method | Checkpoints | To final incl. holds (s) | New return (s) | Full cycle (s) |
|---|---|---:|---:|---:|---:|
| right | progress | 1 | 7.690 | legacy reverse | 17.379 |
| right | sweep | 1 | 5.722 | legacy reverse | 13.444 |
| right | single_arc | 1 | 4.020 | legacy reverse | 10.040 |
| right | clearance | 1 | 4.114 | legacy reverse | 10.228 |
| right | workspace_curve | 1 | 4.205 | legacy reverse | 10.410 |
| right | progress | 3 | 15.456 | 3.839 | 21.295 |
| right | sweep | 3 | 13.727 | 2.590 | 18.317 |
| right | single_arc | 3 | 12.459 | 2.739 | 17.197 |
| right | clearance | 3 | 11.891 | 2.739 | 16.630 |
| right | workspace_curve | 3 | 12.910 | 3.843 | 18.753 |
| right | progress | 2 | 11.279 | 14.500 | 27.779 |
| right | sweep | 2 | 9.545 | 2.953 | 14.498 |
| right | single_arc | 2 | 8.105 | 2.820 | 12.926 |
| right | clearance | 2 | 7.710 | 14.500 | 24.210 |
| right | workspace_curve | 2 | 8.515 | 3.526 | 14.041 |
| left | progress | 1 | 2.610 | legacy reverse | 7.219 |
| left | sweep | 1 | 2.610 | legacy reverse | 7.219 |
| left | single_arc | 1 | 2.834 | legacy reverse | 7.668 |
| left | clearance | 1 | 2.610 | legacy reverse | 7.219 |
| left | workspace_curve | 1 | 2.840 | legacy reverse | 7.679 |
| left | progress | 3 | 7.472 | 2.623 | 12.095 |
| left | sweep | 3 | 7.472 | 2.720 | 12.192 |
| left | single_arc | 3 | 7.697 | 2.865 | 12.562 |
| left | clearance | 3 | 7.472 | 2.720 | 12.192 |
| left | workspace_curve | 3 | 7.756 | 2.838 | 12.594 |
| left | progress | 2 | 5.041 | 3.185 | 10.226 |
| left | sweep | 2 | 5.041 | 2.677 | 9.718 |
| left | single_arc | 2 | 5.266 | 2.773 | 10.038 |
| left | clearance | 2 | 5.041 | 2.677 | 9.718 |
| left | workspace_curve | 2 | 5.298 | 2.838 | 10.136 |
