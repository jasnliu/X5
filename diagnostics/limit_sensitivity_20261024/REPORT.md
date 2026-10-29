# Fixed-path limit sensitivity — 2026-10-24

## Outcome

**The current motion limits, not remaining timing-optimizer quality, prevent
sub-second movement on the historical R1 → R4 benchmark.** The limits interact:
raising only velocity, acceleration, or jerk soon gives diminishing returns.
Increasing all three by the same factor eventually leaves jerk dominant.

All work was software-only. **No camera, CAN, serial, motor, enable/disable,
hold, centering, gripper or relax operations occurred.** The installed limits,
programs, calibration, raw recordings and destinations were not edited. A hash
audit of 386 pre-existing files reports zero changes. Only this new diagnostics
directory was created. The arm being powered did not confer motor authority.

## Controlled inputs and important scope distinction

Primary input: `diagnostics/fast_gpu_20261024/evidence/checkpoint_2.json` — the
exact frozen path behind the previously reported **3.502570 s** R1 → R4 result.
Its original plan is `diagnostics/fast_gpu_20261024/verified3/checkpoint_2/plan.json`.
The current optimizer reproduces the 3.502570 s baseline exactly.

Every primary case retains identical joint-curve coefficients, knots, endpoints,
source geometry hash, zero endpoint velocity/acceleration, 0.1% reserve, and no
internal stops. Two-second demonstration holds are excluded from every reported
transfer duration. No path generation or corner smoothing was performed, and
record4's interior motion did not seed or shape any route.

**This is an archived route, not a fresh plan for the current scene.** Current
`destinationR1.json` differs from that frozen endpoint by up to 25.14 degrees;
`destinationR4.json` is no longer present. The calibration/scene used when this
route was generated predates the camera move. Reusing that exact route is
intentional for a controlled sensitivity comparison, not permission to execute
it against today's scene. No historical collision evidence was relabeled as
current readiness, and no new physical collision clearance is claimed.

## Primary results: identical R1 → R4 geometry

Baseline uniform per-joint bounds are V=0.6 rad/s, A=1.2 rad/s², J=6 rad/s³.
Multipliers below affect only a disposable research process's in-memory copy.

| Hypothetical change | Movement time | Saving vs baseline |
|---|---:|---:|
| Current limits | **3.502570 s** | — |
| Velocity ×2 only | 3.244935 s | 7.36% |
| Acceleration ×2 only | 3.435929 s | 1.90% |
| Jerk ×2 only | 3.265298 s | 6.77% |
| All three ×2 | 2.525420 s | 27.90% |
| All three ×4 | 1.997177 s | 42.98% |
| All three ×8 | 1.585160 s | 54.74% |
| All three ×16 | 1.258144 s | 64.08% |
| V×4, A×16, J×64 | 0.875641 s | 75.00% |
| V×8, A×64, J×512 | **0.437821 s** | 87.50% |

The last two rows are **kinematic experiments, NOT recommended hardware limits**.
The 0.438 s case permits V=4.8 rad/s, A=76.8 rad/s² and J=3072 rad/s³.
It omits strike, rebound and settling time and is not proof of powered tracking,
torque feasibility, mechanical safety or complete 120 BPM performance.

The full 29-case table is in `results.csv` and `primary2/results.json`.
Single-limit sweeps included ×1.1, ×1.5, ×2, ×4, ×8 and ×64; paired sweeps also
tested V+A, V+J and A+J. Increasing velocity beyond roughly ×1.5 makes practically
no further difference without raising the other limits. Acceleration alone
also saturates near ×1.5. Even jerk ×64 alone only reaches 3.157 s.

At baseline, joint 4 reaches approximately 99.7% of the velocity bound, 99.6%
of acceleration and 99.7% of jerk. It is not just one inactive ceiling.
After all limits ×4, peak velocity uses only 60% of the new speed allowance,
whereas jerk remains nearly saturated and is near its bound for about 59% of
sampled movement time. Raising all limits further equally therefore mostly
produces cube-root rather than linear time improvements.

## Lower bounds: why another optimizer cannot solve this alone

The endpoint displacement of joint 4 is **1.580481 rad = 90.555 degrees**.
Even ignoring acceleration/braking, the 0.6 rad/s speed bound alone requires
at least **2.634134 s**. Adding the current acceleration/jerk bounds and the
existing rest-to-rest endpoint conditions gives an exact single-joint S-curve
lower bound of **3.334134 s**.

This bound applies to any route between these same joint endpoints, not merely
this particular curve. Other joints/path coupling can only make it slower.
Therefore, compared with the delivered 3.502570 s clock, even a hypothetical
perfect optimizer/path could save **at most 0.168435 s (4.81%)** while retaining
these limits, endpoints and rest conditions. It cannot yield 0.5 s.

Eight main cases were rerun at 192 controls, up to 300 iterations and 3 seconds
per sparse solve, versus the usual 96 controls/90 iterations/1 second. All eight
converged, and changes were below 1%:

| Case | Normal solver | Higher-resolution check |
|---|---:|---:|
| Baseline | 3.502570 s | 3.492562 s |
| Velocity ×2 | 3.244935 s | 3.236610 s |
| Acceleration ×2 | 3.435929 s | 3.429397 s |
| Jerk ×2 | 3.265298 s | 3.255900 s |
| All ×4 | 1.997177 s | 1.988822 s |
| All ×16 | 1.258144 s | 1.252879 s |
| V×8, A×64, J×512 | 0.437821 s | 0.436570 s |

These are practical robustness checks, not certificates of global optimality.
The endpoint lower bound is independent of the sparse solver's convergence.

## Meaning for 120 BPM

Quarter notes allow 0.5 s, eighth notes 0.25 s, and eighth-note triplets 1/6 s.
Those intervals must also accommodate the necessary strike/rebound behavior.
For an unchanged clock shape, compressing time by k multiplies velocity by k,
acceleration by k² and jerk by k³. This identity was independently checked against
the original polynomial-root checker; freshly optimizing the corresponding
scaled-limit cases reproduced the predicted time scaling.

| Target transfer time | Sufficient V limit | Sufficient A limit | Sufficient J limit |
|---|---:|---:|---:|
| 1.000 s | 2.102 rad/s | 14.722 rad/s² | 257.817 rad/s³ |
| 0.500 s | 4.203 rad/s | 58.886 rad/s² | 2062.536 rad/s³ |
| 0.250 s | 8.406 rad/s | 235.546 rad/s² | 16500.292 rad/s³ |
| 0.167 s | 12.609 rad/s | 529.977 rad/s² | 55688.485 rad/s³ |

These are **sufficient for a time-compressed version of this clock, not minimum
necessary limits and not proposed hardware settings**. A different clock may
need less. `target_times.json` also separates necessary individual lower bounds.
For example, a 0.5 s rest-to-rest transfer necessarily requires at least
V≥3.161, A≥25.288 and J≥404.603 in the same units, considered independently;
meeting all three lower bounds is not itself sufficient. None of these equations
model motor torque, inertia, gravity, payload, thermal limits or controller tracking.

## What record4 actually demonstrates

`recordings/record4.json` has 1,111 samples over 22.403 s and explicitly records
**query-only disabled-motor feedback**. It is a desired manual-motion reference,
not evidence of the powered controller's capability.

The fast part of the recording is concentrated in joints 6–7, while the large
joint-4 repositioning occurs much more slowly later. With cubic Savitzky–Golay
derivatives over an 11-sample, 0.20 s span:

- J4 peak speed: approximately **0.858 rad/s (49 degrees/s)**.
- J6 peak speed: approximately **5.761 rad/s (330 degrees/s)**.
- J7 peak speed: approximately **6.001 rad/s (344 degrees/s)**.

Over any 0.5 s interval, the largest observed J4 net displacement is only
**17.88 degrees**, versus **90.56 degrees** required by the frozen R1 → R4 pair.
For 0.25 s / 1/6 s windows, its maximum net change is 10.01 / 6.80 degrees.
Thus the record does not demonstrate this exact large-arm maneuver in one beat.
Neither frozen endpoint is closely reached by record4: closest maximum-joint
errors are 12.57 and 22.02 degrees respectively.

Derivative sensitivity was evaluated at 7, 11 and 15 samples. Estimated J6 peak
speed spans 5.18–6.05 rad/s, J7 4.69–6.23, and J4 0.72–1.03. Acceleration and jerk
are much more smoothing-sensitive, so their estimated peaks are retained as
diagnostics only, not motor-limit recommendations. See `record4_analysis.json`
and `record4_joints.png`.

## Secondary paths, computation and verification

Five profiles were also checked on the archived rest → R1 route and an archived
left-arm route (10 extra cases). Both show the same qualitative interaction:
raising all three limits helps more than raising any one alone. All ×4 gives
**4.213 → 1.913 s** on the other right path and **2.857 → 1.594 s** on the left.
The left acceleration-only ×2 case hit its normal solver wall-time cap; its
reported 2.820 s incumbent passed independent continuous checks but is not
described as converged. Other primary/higher-resolution cases had no warnings.

All **47 final timing results** were checked using the independent original
continuous polynomial-root checker, plus 10,001-point derivative checks,
endpoint rest checks, exact path identity and no-internal-stop checks.
All 29 primary cases converged. Primary optimization took **0.678–1.435 s**,
median **0.944 s**. Each case was a fresh Python process, with fresh optimization
and no timing-result cache. These numbers measure timing optimization, not camera
acquisition/path generation; those are deliberately excluded by this study.
Cold-process/import/independent-check totals are separately recorded.

Work was serial in bounded systemd jobs: 3 GiB memory cap, 128 MiB swap cap,
bounded threads/runtime, **AF_CAN excluded at the OS level**. Python guards also
forbid camera/USB/serial/GPU device access. No device attempts were observed.
Observed primary/check peak resident group memory was approximately 207/215 MiB.
The first partial diagnostic attempt was discarded after discovering a filename
collision for decimal case labels; the corrected `primary2` retains all 29
distinct cases and has a regression test for that bug.

**11 diagnostic tests passed**, covering output naming, hardware guards, all
47 result proofs, exact geometry/endpoints, monotonicity, time scaling,
higher-resolution consistency, resource containment and source/data preservation.

## Recommended next step — not performed here

Do not blindly install large global multipliers or spend another round trying
to save a few percent with the same limits. First distinguish a fast strike or
nearby drum transfer from this large 90-degree elbow relocation. Use representative
playing endpoints/postures consistent with record4's fast section, then investigate
joint-specific capability profiles: distal joints need very different speeds
from the shoulder/elbow. Validate those profiles through separately authorized,
incremental physical tracking tests before changing deployed limits.

After capabilities are measured, shorter/less-curved routes and coordinated
approach/strike/rebound motion may be needed to fit the beat, avoiding unnecessary
large-arm repositioning and full stops where the task permits. The present study
does not change any path, endpoint, limit or stop policy in the applications.

## Reproduce

```bash
cd /home/jason/Proyectos3/X5
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
D=diagnostics/limit_sensitivity_20261024
STAMP=$(date +%s)
.venv-planner-experiment/bin/python diagnostics/fast_gpu_20261024/run_job.py \
  "$D/repeat-$STAMP-job" "$D/study.py" --output "$D/repeat-$STAMP"
# Add --refined for the uniformly higher-resolution research configuration.
# Optional --path left or --path checkpoint_1 changes the frozen benchmark.
.venv-planner-experiment/bin/python -m pytest "$D/test_study.py" -q -p no:cacheprovider
```

Evidence: `primary2/`, `checks/`, their `*-job/{job,containment}.json`,
`summary.json`, `results.csv`, `target_times.json`, `record4_analysis.json`,
`before_hashes.json`, `test-console.log`, and the two plots. All artifacts remain
local; no commit/staging, package installation, memory update or changes to the
artifact-retention policy were made.
