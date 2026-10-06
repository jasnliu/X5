# record3 physical playback results

## Conclusion

**Use `./playback.sh`. The selected default is `paced_precise`.**
The recording movement takes **4.8 s**, versus **3.841315 s** in record3 (24.96% longer). The old fitted playback takes 7.09575 s; the new movement is 32.35% shorter than that. Centering and approximately two seconds of endpoint verification are separate.

Across **4 successful physical runs**, the selected method's all-run median position ripple was **11.5% lower**, and independently reported motor-velocity ripple **17.8% lower**, than physical replay of the old GUI command timing. Against an ideal 50 Hz old-trajectory comparator, the reductions were **8.5%** and **13.6%**. Both comparators also have four successful physical runs.

All four selected-method runs reached the original endpoint within **0.02198 degrees** (approximately one encoder count), then returned to verified center before relaxing. Final command targets match record3 exactly; hardware accuracy is finite, not mathematically exact. Center error before relaxation was at most **0.02148 degrees**.

The choice is the best tested balance, not a claim of zero vibration or a globally optimal controller. The lower-gain raw method reduced ripple in its screening run but had a 0.154-degree endpoint error. `paced_precise` retains its 4.8-second motion, then softly restores original gains while holding the exact endpoint and verifies 0.05-degree accuracy before recentering. No relax command is sent at the endpoint.

## Measurement and comparison limits

- Scores use actual right-motor CAN feedback, not simulated positions or commanded-curve jerk: seven-joint vector RMS after a third-order 3–15 Hz bandpass, 100 Hz interpolation, trimming 0.25 s at each end. The velocity score uses separately reported motor velocity, not numerical differentiation.
- Timestamps are host receipt times, not hardware timestamps. These are motor-side ripple proxies; there is no accelerometer and no whole-arm visual verification. The camera sees the cymbal area. Different durations change the motion spectrum, so the comparison does not isolate one cause.
- `baseline` physically replays the unchanged old fitted curve at ideal 50 Hz. `legacy_gui` physically replays command timing captured from the unchanged GUI in simulation (median 32.25 ms, maximum 48.78 ms). It is **not** a full live `start_beat.sh` camera/audio workload test.
- A balanced nine-cycle campaign repeated baseline, cap-20 and GUI timing three times each. Three subsequent precise-method runs established the selection; a fourth plain `./playback.sh` run verified the delivered default. The earlier progress estimate of 13%/16% used the three-run validation cohorts. The headline above uses all four successful runs of each method, including screening/final runs.
- Small sample sizes and overlapping ideal-baseline position ranges limit generalization. All four selected-method velocity scores were below both comparator ranges, and all four selected-method position scores were below the GUI-timing comparator range. No formal population-level statistical claim is made.

## Path, timing and safety evidence

- Commanded path stayed within 1.310 degrees per joint and 3.732 mm TCP of the source path at corresponding retimed phase; exact first/last joint targets were preserved. Validated peaks: 0.693 rad/s, 1.265 rad/s², 37.964 rad/s³ piecewise jerk. These are command checks, not the measured improvement score.
- Actual feedback sampled at 50 Hz stayed within **1.110 degrees per joint / 8.839 mm TCP** of the nearest raw-recorded-path phase across the four selected runs. This permits intentional local retiming and is sampled proximity, not a continuous bound or collision certification.
- Speed/current ceilings were not increased. Temporary outer position gains were capped at 10; the saved original values were [80,80,60,60,30,30,30]. A guarded 0.5-second restoration ramp runs only after soft settling near the endpoint. All readbacks and writes are logged; no flash save, zero or calibration command is used.
- Ntfy HTTP acceptance and a 10-second countdown preceded every physical cycle. This verifies service acceptance, not independently observed phone delivery. Left-arm commands were prohibited; left motors stayed disabled in successful-run feedback audits.
- **2 deliberate physical SIGINT tests passed**: one 1.25 s into playback, one 0.1 s into the endpoint gain ramp. Both recentered, restored gains and relaxed; neither disabled at the interruption pose. Their expected `Cancelled` summaries are excluded from efficacy scores, not discarded.
- Final query-only postflight verified all 16 motors relaxed, original gains restored, both CAN buses ERROR-ACTIVE with zero current TX/RX error counters, and runtime source hashes identical to the final physical run.
- All **265 protected existing files** remained byte-for-byte unchanged. Original record3 SHA256: `138c4cd724f33135b33a1ba96c40a4b2a412ba195d47a02d55f5ea8a964afa7e`.

## All retained physical methods

All-run medians below; a single screening run is not equivalent to repeated validation. Every method, source snapshot and raw log is retained. Slow variants and comparators cannot be selected under the final speed requirement.

| Method | Successful runs | Movement s | Position ripple ° RMS | Velocity ripple rad/s RMS | Timing eligible |
|---|---:|---:|---:|---:|:---:|
| `baseline` | 4 | 7.096 | 0.078365 | 0.074680 | no |
| `legacy_gui` | 4 | 7.096 | 0.080986 | 0.078466 | no |
| `matched` | 1 | 13.641 | 0.058527 | 0.058661 | no |
| `paced` | 1 | 4.600 | 0.091825 | 0.076285 | yes |
| `paced_200hz` | 1 | 4.600 | 0.081183 | 0.073780 | yes |
| `paced_damped` | 1 | 4.800 | 0.081615 | 0.067614 | yes |
| `paced_damped200` | 1 | 4.800 | 0.083287 | 0.070008 | yes |
| `paced_kp10` | 1 | 4.800 | 0.071094 | 0.063525 | yes |
| `paced_kp20` | 4 | 4.800 | 0.076654 | 0.065731 | yes |
| `paced_kp30` | 1 | 4.800 | 0.074752 | 0.067921 | yes |
| `paced_precise` | 4 | 4.800 | 0.071685 | 0.064514 | yes |
| `paced_soft` | 1 | 4.800 | 0.086954 | 0.073292 | yes |
| `paced_soft200` | 1 | 4.800 | 0.078266 | 0.072514 | yes |
| `retimed` | 1 | 34.382 | 0.026631 | 0.050508 | no |
| `smooth` | 1 | 13.641 | 0.061624 | 0.058504 | no |
| `smooth_200hz` | 1 | 13.641 | 0.053103 | 0.058434 | no |
| `smooth_csp04` | 1 | 13.641 | 0.052474 | 0.057203 | no |
| `smooth_slow` | 1 | 27.281 | 0.032971 | 0.052257 | no |

### Method definitions

- `retimed`: original cubic with global velocity/acceleration/jerk scaling. `matched`: original cubic slowed to the smooth-path duration.
- `smooth`: Gaussian filtering plus endpoint-exact quintic path. `_csp04` lowers the firmware speed ceiling; `_200hz` also increases update rate. `smooth_slow` doubles that already-slow duration.
- `paced`: locally retimed smooth path at 4.6 s; `_200hz` changes update rate. `paced_soft` uses 4.8 s and joint-specific firmware ceilings; `paced_soft200` increases update rate.
- `paced_damped` / `paced_damped200`: same 4.8 s path with original position gains halved at 100/200 Hz.
- `paced_kp10` / `paced_kp20` / `paced_kp30`: common position-gain caps during the same 4.8 s, 200 Hz motion, restored at center.
- `paced_precise`: cap-10 motion plus soft endpoint settle, guarded original-gain restoration and tight endpoint verification before return. Original gains are independently checked again at center.

## Retained failures and checks

There were **30 successful full cycles**, **2 early failed motion attempts** and **2 intentional cancellation tests**. Both early failures happened before enabling: camera startup made feedback stale, and passive J4 sag exceeded the strict modeled limit. These were corrected with a fresh pre-enable state batch and bounded inward-only startup recovery. No failed data was deleted.
Separate query-only checks failed while the arm was powered off; the user confirmed power had been off. After power returned, state-only requests brought the old CAN error counters back to zero; no privileged interface reconfiguration was needed.

- New offline suite: 24 tests passed, with physical CAN creation prohibited.
- Existing repository suite: 358 tests passed. An initial stdin-launched attempt failed four multiprocessing tests because `<stdin>` could not be reopened; the real-file entrypoint fixed the test invocation, without changing existing tests.
- Final delivered-command log: `playback_results/final_default_console.txt`.
- Final full cycle: `playback_results/20261003T054749-paced_precise-c1515a`.
- Raw/aggregate evidence: `playback_results/comparison.json`, `.csv`, `.png`; `selection.json`; `final_verification.json`; `measured_path_proximity.json`; every per-run directory and cancellation verdict.
- Reproduce analysis: `python3 -m smooth_playback.report`, then `python3 -m smooth_playback.path_evidence`, then `python3 -m smooth_playback.make_results` (system Python with NumPy/SciPy/Matplotlib). No hardware access is used by these commands.

Position-gain register 0x701E was checked against the [manufacturer RS04 manual](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS04/RS04User%20Manual260713.pdf); the downloaded reference is retained under `playback_results/reference/`. The chosen gains are empirical local tests, not a manufacturer tuning recommendation.
