# Approved DT1 ±25-degree restoration — 2026-10-28

Operator approved the range change after the bounded numerical study reported
an exact-tip fit at +21.169 degrees under all retained goal reserves, outside
±20. Production now uses ±25, version 8, shared by standalone/experiment/
hardware and generated beat startup. No joint bound/reserve, green dot, collision
margin, path generator, timing limit or motor setting was modified.

The original investigation is preserved in REPORT.md; its pending status and
±20 production statements describe the earlier turn, not this approved update.

## Verified results

Both complete missions were recalculated from the preserved real camera/encoder
captures using the unchanged normal generated-planning worker, with AF_CAN
excluded from the worker's allowed socket families. No result-cache reuse.

| Input | Left outward | Right outward | Full calculation | Result |
|---|---:|---:|---:|---|
| Original failing capture | 3.527501 s | 2.814549 s | 39.913 s | Both paths and right→left return ready |
| Second capture acquired during diagnosis | 3.723570 s | 2.833885 s | 34.516 s | Both paths and right→left return ready |

Output folders:
- `planner_runs/dt1-ik-recovery-20261028/approved-saved/mission.json`
- `planner_runs/dt1-ik-recovery-20261028/approved-second-capture/mission.json`

Independent proof readback: `approved_verification.json`, reproduced by
`python -m diagnostics.dt1_ik_recovery_20261028.verify_approved` under the planner
Python/environment. Green point, measured red point and fitted disc are exactly
unchanged from each original capture. Right yaw +25°, tip residual below
3e-16 m numerically, J2 goal ≥+1°, other goal reserves ≥3°. This is model-space
numerical consistency, NOT absolute camera/hardware accuracy. Dense native
outward and return samples/edges all passed full-arm/stick collision and zone
checks with unchanged 20 mm clearance plus 5 mm edge allowance.

The standalone command also passed on the original saved pair and poses:
`planner_runs/dt1-ik-recovery-20261028/approved-standalone`.
It generated one v8 destination, not a movement or recording.

## Tests and native UI

- **365 pytest regressions passed** in 74.09 s (`approved_tests.log`); four
  SLSQP trial-bound warnings, no rejected final goal accepted/clipped.
- The original ±20 failure remains a test under its explicit old policy; the
  same target now succeeds at the approved ±25 range with unchanged reserves.
- Directed ±25 boundaries pass; ±25.01, reversed/outward and undefined aiming
  remain rejected. Old v7/20-degree metadata remain recovery-readable and are
  not relabeled as v8 new-generation results.
- Actual Tk generated control panel and spawned 200 Hz writer with **fake
  motors**: seven checks passed, including full outward, cached return, Hold,
  unconditional Emergency Relax and Stop; no physical transport or ESP32.
- Actual Qt/VTK windows: eight checks passed, frozen camera mesh, both tools,
  outward and return preview, scrubbing independent of saved rest, play/pause,
  no rendering errors. Screenshots and reports in `approved-ui/`.
- Standalone native UI confirms ±25 status/instructions, exact target, all three
  markers, start/goal switching and export availability, without devices.

## Preservation and hardware boundary

Of 195 protected source/data files, only `cymbal_destination/config.py` and
`cymbal_destination/output.py` changed in this approval turn. The latter retains
legacy recovery metadata. Path/timing/motor code, calibration, models, raw
recordings, and saved destinations were byte-identical. Related tests and docs
were updated; there are no firmware/dependency changes.

A new `./start_beat.sh --hardware --prepare-only` acquisition attempt stopped
before camera capture: both buses were ERROR-PASSIVE (TX error counter 128),
no motor replies. Run: `planner_runs/generated-prepare-20261028T100757-c634fe31/`.
Audit shows only 8 state requests per bus, zero enable/disable/motion/gripper
commands, and closed connections. No bus reset or suppression of the CAN fault
was attempted. This separate live-feedback problem is not claimed repaired.
The operator was asked to confirm power/connectivity for an optional fresh rerun.

All successful route tests above use **preserved actual camera/pose captures**,
not invented geometry and not a new live acquisition. No real arm or hi-hat motor
commands were sent. Simulation passes do not certify physical tracking or strikes.

## Reproduction

```bash
# Normal operator workflow (auto hi-hat calibration; arm motion only on Start)
./start_beat.sh --hardware

# Fresh capture + complete calculation only; NO actuator owner or ESP32
./start_beat.sh --hardware --prepare-only

# Already checked saved-camera mission; software-only preview/replay
./start_beat.sh --generated-simulate planner_runs/dt1-ik-recovery-20261028/approved-saved/mission.json
```

