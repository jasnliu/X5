# Current inward-aim range — version 8 (2026-10-28)

The operator approved restoring **±25°** after the generated beat startup's
DT1 rejection. All joint bounds/reserves, the exact green point, disc normal,
above-parallel rule, J7 alignment and collision policies are retained.

See [the failure replay and fresh prepare-only verification](../diagnostics/dt1_ik_recovery_20261028/APPROVED_REPORT.md).
The entries below are preserved historical evidence, not current-version
standalone reopen commands. Recalculate old captures instead of relabeling them.

# Historical J2 operating-limit verification — version 6

The v6 destination and right-arm 0° operating floor supersede the older reserve
policy below. J2 goals use +1° minimum; other endpoint bounds keep 3°.
See [the controlled offline study](../diagnostics/right_j2_operating_limit_20261028/REPORT.md)
for saved-camera comparisons, native checks and verification scope.

```bash
./plan_cymbal_destination.sh --open diagnostics/right_j2_operating_limit_20261028/standalone_latest
```

Earlier captures/results remain historical evidence, not current executable
plans. Their old standalone previews require recalculation under v6.

# Historical: endpoint joint reserve — version 5

New generated goals have a 3° position reserve at every calibrated joint bound.
This is shared by the standalone preview, experiment dt1 and normal right-arm
hardware dt1. The green target and all orientation/collision rules are retained;
timing, physical limits and motor recovery are not changed.

See [the saved fault-capture study](../diagnostics/cymbal_goal_reserve_20261027/REPORT.md)
for current offline numerical/native checks. Reopen its new result with:

```bash
./plan_cymbal_destination.sh --open diagnostics/cymbal_goal_reserve_20261027/standalone_fault
```

The evidence below is historical v4 verification, not reserve verification.
Its old preview must be **recalculated**, not relabeled or overwritten, for v5.

## Historical inward range ±20° — version 4 verification

The accepted inward aiming range is now **−20° through +20°**, tightened from
±25°. Only the range/policy version and dependent text/tests/documentation were
changed. The IK/search/ranking algorithms, inward-only rule, edge preference,
green target, above-parallel rule, J7 alignment, original limits, collision checks,
acquisition and shared X5/DT1/Y2 programs are unchanged. The existing preference
scales to the new ±20° edges; it does not force a particular edge or sign.

All verification here is **offline**, using saved actual-camera captures and
saved both-arm poses. No camera, encoder or motor was accessed or moved.

Evidence root: [`diagnostics/cymbal_aim20_20261026/`](../diagnostics/cymbal_aim20_20261026/).

## Results

| Saved source | New run | Inward yaw | Shaft-above elevation | Full offline replay |
| --- | --- | ---: | ---: | ---: |
| `cymbal_destination_runs/20261027T054238Z-923ea3` | `replay_latest` | **+20.00°** | 8.1945° | 10.64 s |
| `cymbal_destination_runs/20261027T052841Z-bbe06f` | `replay_previous` | **+20.00°** | 7.5276° | 10.38 s |

Both complete launcher replays independently recomputed detection and generated
valid destinations at their original green dots. Each compared three feasible
geometric approaches; endpoint and approach passed original collision/zone and
joint-bound checks. Two separately initialized native collision/FK checkers also
passed, including independently computed inward angles. Output remains one
posture; approach curves are discarded and no strike or timing law is produced.
These are numerical geometry checks, not measured physical accuracy/performance.

- **204 tests passed**, 8.76 s: standalone destination tests plus unchanged
  fast-mode/workspace-recovery regressions. Six SciPy warnings concern internal
  optimization trial iterates, not clipping accepted endpoints or widening limits.
- New boundary/rejection tests first failed on the old ±25° code (**13 failures**),
  then passed: ±20° accepted, ±20.01° and ±25° rejected across multiple elevations.
  Outward/reversed sticks remain rejected.
- Saved-policy validation rejects old ±25° results, even if their version field
  is relabeled as current. Previous outputs are preserved, not silently reused.
- Native Qt/VTK preview verified the **±20°** status and explanatory text,
  unchanged red/green markers, static pose switching, overlays, fit/camera views,
  export/no-overwrite/tamper handling and failure state. No rendering errors.
  Screenshots were visually inspected.
- Current headless reopen passes; old ±25° reopen gives a recalculate-in-new-folder
  message. Python AST and launcher shell syntax checks pass.
- Input comparison verifies identical red/green/disc coordinates, state/depth/
  point-cloud/display/RGB bytes, calibrated URDF, zone, both-arm geometry, original
  policies and self-mask for each corresponding capture. No targets were shifted.
- Preservation audit checked **2,845 existing files**: only the intended standalone
  config/UI/diagnostic text, tests and docs changed. Solver, aim mathematics,
  acquisition, earlier recordings/results, models and calibration remain intact.

## Open the checked result without devices

```bash
cd /home/jason/Proyectos3/X5
./plan_cymbal_destination.sh --open diagnostics/cymbal_aim20_20261026/replay_latest
```

Normal `./plan_cymbal_destination.sh` uses the new range on subsequent launches.
Existing open windows must be restarted; old ±25° runs require recalculation.

Recompute offline from the same capture:

```bash
./plan_cymbal_destination.sh --headless \
  --replay cymbal_destination_runs/20261027T054238Z-923ea3/acquisition/camera/pair.npz \
  --state cymbal_destination_runs/20261027T054238Z-923ea3/acquisition/measured_start.json
```

Evidence includes `regression_before.log`, `pytest.log`, both replay logs/runs,
`*_native/report.json`, `ui/ui_checks.json`, `ui/ui_actual_target_success.png`,
`input_comparison.json`, `cli_checks.json` and `preservation_audit.json`.
Previous version-3 source/tests/docs are retained in this study's `before/`.
The prior ±25° study remains unchanged at `diagnostics/cymbal_inward_aim_20261026/`;
its results are historical, not validation of the tightened range.
