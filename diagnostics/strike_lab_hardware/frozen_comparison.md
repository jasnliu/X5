# Hardware measurement tables

All measured attempts included, not just passing strokes. Durations in milliseconds.

| Mode | Pass / attempts | Depth min–max ° | Zone median / P95 ms | Cycle median ms | Upper overshoot max ° | Failures |
|---|---:|---:|---:|---:|---:|---|
| powered | 197/200 | 9.957–10.067 | 54.77 / 55.56 | 477.4 | 0.066 | excessive_command_limiting: 3 |
| impedance | 199/200 | 9.913–10.110 | 54.93 / 55.82 | 483.3 | 0.066 | feedback_gap: 1 |
| optimized | 196/200 | 9.957–10.067 | 54.40 / 55.27 | 478.2 | 0.066 | excessive_command_limiting: 4 |
| gravity | 200/200 | 9.891–10.023 | 55.88 / 57.88 | 401.1 | 0.110 |  |
| variable_damping | 199/200 | 9.891–10.023 | 56.33 / 58.29 | 397.6 | 0.110 | feedback_gap: 1 |
| hybrid | 200/200 | 9.847–10.001 | 56.07 / 57.66 | 392.1 | 0.110 |  |
| cosine | 192/200 | 9.935–10.045 | 63.95 / 64.75 | 441.8 | 0.110 | excessive_command_limiting: 8 |
| torque | 183/200 | 9.847–10.198 | 66.27 / 67.84 | 495.9 | 0.154 | upper_overshoot: 1, missed_start_interval: 16 |
| baseline | 200/200 | 9.891–10.001 | 79.85 / 81.46 | 395.8 | 0.132 |  |

## Session history

| Session | Attempts | Pass |
|---|---:|---:|
| experiment_results/hardware_calibration/20261002T154900-1c47eb41 | 1 | 0 |
| experiment_results/hardware_calibration/20261002T155153-8c136341 | 0 | 0 |
| experiment_results/hardware_calibration/20261002T155322-ab583d64 | 1 | 0 |
| experiment_results/hardware_calibration/20261002T155636-3a055574 | 0 | 0 |
| experiment_results/hardware_calibration/20261002T155755-bbbf2fc6 | 5 | 4 |
| experiment_results/hardware_confirmation/20261002T162437-7ee5911a | 135 | 135 |
| experiment_results/hardware_confirmation/20261002T162932-793ae6f0 | 20 | 18 |
| experiment_results/hardware_confirmation/20261002T163507-a134d16c | 15 | 15 |
| experiment_results/hardware_final/20261002T163910-1a369985 | 1800 | 1766 |
| experiment_results/hardware_tuning/20261002T160025-7b98f42c | 18 | 4 |
| experiment_results/hardware_tuning/20261002T160233-c1302ece | 16 | 7 |
| experiment_results/hardware_tuning/20261002T160628-677a5095 | 81 | 58 |
| experiment_results/hardware_tuning/20261002T160922-f91f85d0 | 162 | 51 |
| experiment_results/hardware_tuning/20261002T161439-3dac7c8d | 160 | 86 |
| experiment_results/hardware_tuning/20261002T161950-bf39e23c | 180 | 158 |
