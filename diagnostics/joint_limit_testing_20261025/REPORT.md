# Joint-limit testing: offline software stage

2026-10-25 UTC. New standalone `test_joint_limits.sh` / `joint_limit_lab/`; original programs, raw recordings, destinations, calibration and hardware code remain unchanged.

## Scope and remaining boundary

This release prepares and analyzes tests without accessing devices. **No live motor runner or telemetry acquisition backend is implemented or invoked.** It accepts recorded physical traces through a defined format, but that is not equivalent to having a live collector. The choice of adding a guarded live runner now (mock-tested only) versus leaving it to the physical stage remains separate. No `--hardware` option exists.

The complete proposed physical procedure, future runner/recovery requirements, data contract and copyable offline commands are in `JOINT_LIMIT_TESTING.md`. Physical limits cannot be inferred while the arm is off, and finite trials cannot prove absolute maxima or a global fastest route.

## Implemented

- Per-arm study configurations with explicit current V/A/J ceilings, ordered test levels, bounded displacements, repeated trials, payload/controller/supply labels, acceptance criteria and a time-scaling reserve.
- C3 one-joint out/hold/back/hold probes with analytic derivative extrema. Saved-scene preparation uses the original native full-arm/stick/zone checker, calibrated bounds and acquired self-mask; blocked excursions are listed, never clamped or forced.
- Typed/checksummed trace loading, strict timestamps and command identity, position/velocity tracking, held-joint drift, settled return, feedback sequence/age, current freshness/headroom, temperature and drive-fault checks. Missing telemetry is not silently replaced by zero.
- Repeated-level coverage and first-failure stopping of escalation in synthetic tests. Missing/duplicate/mixed-origin evidence cannot produce a complete candidate profile. Commanded analytic peaks—not noisy numerical jerk maxima—form the tested envelope. Two measured derivative-filter windows are diagnostic only.
- Explicitly labelled synthetic end-to-end demos; candidate profiles are compatible with the normal experiment UI. A separate evidence file says componentwise probe extrema do not establish simultaneous route capability.
- Fresh re-timing of the unchanged saved route, native validation and independent continuous-root checking; output is a forecast, not an executable hardware plan.
- Pure recorded RobStride frame decoder using installed X5 sign/offset and SDK decoding ranges. Encoding ranges are never used as operating limits.
- One serial contained worker (3 GiB, AF_CAN excluded), bounded traces/allocations/storage, cancellation cleanup and a success receipt. A zero systemd exit alone is not proof of completed work.

## Verification

**107 selected tests passed** (64 new laboratory tests plus 43 existing experiment-motion-control tests). Includes both arms, analytical extrema, scaling, profile compatibility, malformed/stale/missing/duplicate telemetry, wrong-arm data, simulated fault classes, missing coverage, mixed provenance, protocol sign/range conversion, no accidental hardware CLI, extreme input bounds and existing-file preservation.

**11 actual new CLI cases passed**, plus an actual Ctrl-C cancellation/worker-cleanup test. All used saved inputs or generated data, never an actual camera/encoder/motor. The release command logs and completion receipts identify final runs.

| Arm | Prepared trials | Blocked excursions | Synthetic traces passing | Unchanged-profile route forecast | Fresh timing + checks |
|---|---:|---:|---:|---:|---:|
| right | 212 | 3 | 212 | 1.656984 s | 4.893 s |
| left | 204 | 5 | 204 | 0.459337 s | 3.338 s |

The right path is current R1→R2. The left path is the previous synthetic L1→L1+1°J6 verification path—not a full drum transfer. Those identical movement times are parity checks, **not newly measured hardware results**. Calculation times exclude acquisition and path generation.

Both saved-scene analyses correctly refused profile export because actual payload/controller/supply conditions were unspecified. Complete fake demonstrations did export files explicitly named `SYNTHETIC_NOT_FOR_HARDWARE_*`. The injected tracking-failure demonstration retained only the completed 0.75 level, never accepted the failed 1.0 level and skipped remaining trials in each affected group.

Native preparation rejected 3 right and 5 left excursions; smaller clear alternatives still provided all nominal joint/pose/direction cohorts. This is geometric software evidence only. Short probes did not reach all requested velocity ceilings; the analysis reports achieved excitation rather than claiming those ceilings were measured.

Cancellation testing initially exposed an incorrect test assumption: a requested systemd service stop may return exit code zero. The final test checks the cancellation flag, terminal service state and absence of a completion receipt. The new CLI also refuses zero-exit work with no matching receipt and prints a clean cancellation message instead of a traceback.

## Evidence

- `audit.json`, `before_hashes.json`, `summary.json`: current-task source/data preservation and resource/no-device boundaries.
- `tests_final.log`, `study_release.log`, `release/cli_results.json`, `cancel_result.json`: fresh validation.
- `study.py release`, `cancel_check.py`, `audit.py`: reproduction/auditing helpers. Choose a new study tag rather than overwriting an old session.
- `release/right_config.json`, `release/left_config.json`, original-limit profiles and route reports: exact configurations and forecasts.
- Full raw campaigns, trace NPZs and detailed per-trial reports remain locally under `release/`. Only concise reproducible evidence is allowlisted; bulk sessions and runtime credentials are not curated.

No packages were installed, hardware limits changed, camera calibration adjusted, Git history rewritten, files staged or commits made. Actual hardware measurements, load-dependent operating envelopes, realistic drumming duty-cycle validation and fastest physically demonstrated routes remain future work.
