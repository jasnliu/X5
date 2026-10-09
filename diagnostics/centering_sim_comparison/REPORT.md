# Right-arm centering comparison

## Result

**12/12 matched exactly:** right J1–J7 simulated positions, transmitted center command bytes, command timestamps relative to center dispatch, and right disable time. Both simulated arms finished and relaxed in every current-program case.

Both actual shell launchers were also run with `--test`, from the same right record3 endpoint, and exited cleanly after simulated centering/relaxation. The detailed MIT/CSP comparisons additionally run each launcher’s actual App/control/transport code with fake motor sockets, because ordinary `--test` does not exercise the hardware MIT return controller.

## Change

Only production file changed this turn: `camera_playback/app.py`, `App.fail`. Recoverable faults during a dual-arm return now reach the original right-arm fault-recentering code, rather than `DualRecording.fault`, which replaced J7’s center goal with the measured off-center angle. Left playback-fault handling and existing checks/tolerances are unchanged.

Before the fix, an injected transient return-feedback failure left J7 off-center in powered hold for the entire 25-second trial; the reference completed. After the fix, the same injected fault completed and matched the reference exactly. This demonstrates a software failure mechanism, not proof that the exact same transient occurred in the physical incident.

## Test matrix

| Case | Poses | Center commands | Right disable (s) | Max difference |
|---|---:|---:|---:|---:|
| csp_feedback_fault_record3 | 158 | 222 | 4.96875 | 0° |
| csp_jitter_record3 | 143 | 190 | 4.96875 | 0° |
| csp_normal_record3 | 159 | 208 | 4.96875 | 0° |
| csp_send_fault_record3 | 159 | 215 | 4.96875 | 0° |
| mit_deadline_fault_record3 | 159 | 343 | 4.96875 | 0° |
| mit_feedback_fault_record3 | 158 | 354 | 4.96875 | 0° |
| mit_jitter_record3 | 143 | 311 | 4.96875 | 0° |
| mit_normal_record1 | 43 | 102 | 1.34375 | 0° |
| mit_normal_record2 | 116 | 253 | 3.62500 | 0° |
| mit_normal_record3 | 159 | 343 | 4.96875 | 0° |
| mit_post_strike_record3 | 159 | 343 | 4.96875 | 0° |
| mit_send_fault_record3 | 158 | 354 | 4.96875 | 0° |

## Scope and preservation

- Compare identical right starting poses/recordings, not the different launcher defaults.
- Timing is relative to the first right center command. The current program still performs extra left-arm path/freshness checks before dispatch. In the hybrid-deadline case its stationary fault-hold frames precede center dispatch by 0.316 simulated seconds; the reference dispatches immediately. Those identical hold-frame bytes and their different timestamps are retained separately in `pre_center_commands`. The centering motion itself is identical.
- Simulated transport executes actual controller/encoder-frame logic with finite-speed motor tracking, fixed callbacks, a 0.5-second delayed callback case, and one-shot feedback/send faults. It does not model full torque dynamics, gravity, collisions, or physical CAN timing.
- All processes ran without host networking/CAN interfaces and without real device nodes (`bwrap --unshare-net --dev /dev`); the reference/root filesystems were read-only. Python audit guards additionally reject CAN/device access.
- All 681 regression tests passed. All 309 protected reference/recording files match the pre-run SHA256 manifest. `start_beatTest.sh` and its reference runtime were not edited.
- No physical arm was commanded or tested. Do not interpret simulation agreement as physical clearance/safety validation.

## Reproduction

From `/home/jason/Proyectos3/X5`:

```bash
bash diagnostics/centering_sim_comparison/run_matrix.sh
bash diagnostics/centering_sim_comparison/run_launchers.sh
python3 diagnostics/centering_sim_comparison/summarize.py
```

Raw traces: `matrix/*.json`; shell logs: `start_beat*.sh.log`; full suite: `full_tests.txt`; comparison: `comparison.json`.
