# Baseline feedback isolation — 2026-10-21

## Failure and diagnosis

The two saved user calculations ended their encoder streams while adaptive
retiming was running, not because all distal encoders had failed together:

- `20261021T163651Z-baseline-c04d74/calculate-6f024196`: 198.8 ms between
  reading left motor 3 and right motor 3; motors 4–8 on both buses were reported
  missing after the 150 ms query deadline expired.
- `20261021T163337Z-baseline-44f5f6/calculate-d8cb37d0`: a 381.5 ms reader pause,
  leaving right motors 7–8 reported missing.

The previous worker ran native SciPy SLSQP in the main thread and the encoder
observer in another thread of **the same Python process**. Native optimization
can retain that process's GIL longer than the query deadline. See `incident.json`.

`reproduce.py` ran the real optimizer on the failing candidate alongside the real
`Observer.sample` loop, substituting only healthy in-memory CAN replies. It
reproduced `Missing fresh feedback` while seven replies per bus were **already
queued**. Thus a scheduling failure alone explains this error; encoder movement
or encoder failure is not necessary. This does not certify the live electrical
CAN connection, which was not accessed. `reproduction.json` records the result.

## Correction

- `baseline/work.py` retains exclusive, uninterrupted query monitoring and the
  same captured-scene and right-arm movement guards.
- `baseline/computation.py` runs the unchanged planner, adaptive timing and dense
  collision/zone checks in a separate fresh Python process. It inherits no CAN
  descriptors and installs the existing CAN-denying audit guard.
- Terminal progress still streams directly. Computation errors retain their
  messages. A real monitor failure aborts computation promptly; it is not ignored
  or retried using stale data. Cancellation/errors reap native child processes.
- Both processes remain in the existing calculation cgroup: 3 GiB maximum memory,
  128 MiB maximum swap, one numerical worker thread, full-group cancellation.
- No UI, timing policy, trajectory generation, motor controller, relaxation
  behavior, encoder deadline, or freshness threshold was changed.

## Reproducing software verification

`tests/test_baseline_computation.py` is self-contained and opens no devices.
It tests GIL-blocking native computation, original encoder query deadlines,
process and descriptor isolation, fault/cancel/descendant cleanup, child errors,
existing-plan checks, and the CAN guard. `run_tests.sh` runs the broader suite.

`verify_calculation.py OUTPUT` uses the locally saved incident's five raw depth
frames and motor encoder counts. Only the sensor backends are substituted; real
capture synchronization, query decoding/audits, movement guards, Single arc,
full adaptive timing, dense collision/zone validation and packet validation run.
Use the experiment `Job` wrapper with `allow_can=False` to enforce the device
boundary in the kernel. Simulated acquisition is explicitly revoked from
hardware eligibility afterwards. This script needs the locally saved incident;
it is not a new runtime dependency.

`run_ui.sh` uses that software result with the existing native three-window GUI
and **simulated** motor-owner services only. It never starts physical playback.

## Verified results

- Full production Calculate pipeline on the saved failing scene/pose: **passed**
  in 262.87 seconds, with 5,207 complete encoder batches and
  **no feedback errors**. Maximum batch duration 9.48 ms;
  maximum batch-start gap 58.56 ms.
- Selected Single arc path timed to 3.944697 seconds. Final
  collision/zone check: 1,974 exact-curve samples plus edges, all valid.
- Peak calculation cgroup memory 468.6 MiB, well below
  the unchanged 3 GiB cap. Kernel address-family policy excluded CAN.
- Native GUI: **21 workflow checks passed**, including all three windows,
  no overlapping buttons, play/pause/scrub/speed, independent measured preview,
  movement invalidation/retry, and always-available controls.
- Three independent **simulated** motor cycles finished returned to rest and
  relaxed: full cycle, emergency hold/reverse, and injected CAN fault/recovery.
  No automatic midair relaxation; no test-owned services left running.
- Actual `./plan_drums.sh baseline --simulate ... --seconds 3` launcher passed.
- No physical CAN, camera, actuator, gripper or motor-mode access during these
  tests. This is software validation, not a live hardware reliability claim.
- Broad software regression suite: **904 passed in 291.92 seconds**, including
  13 new process-isolation regressions. No unexpected failures. Deliberately
  injected faults were asserted and recovered in the tests, not hidden.
- Source preservation checks passed for the planner/timing, model/recording,
  native collision engine, UI, motor controller and encoder/freshness code.
