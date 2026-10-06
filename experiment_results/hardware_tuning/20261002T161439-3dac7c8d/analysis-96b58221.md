# Strike lab results

Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|
| gravity | 42e18dcb99ed4285 | hardware_refine_2 | 5 | 3 | 62.539 | 63.339 | 10.110 | 383.088 |
| gravity | 61877c8d539138b6 | hardware_refine_2 | 5 | 5 | 72.169 | 72.239 | 10.132 | 403.144 |
| gravity | 15fe4681633dd315 | hardware_refine_2 | 5 | 3 | 55.867 | 56.742 | 10.067 | 360.720 |
| gravity | e36e5c63f1f33271 | hardware_refine_2 | 5 | 5 | 63.549 | 64.428 | 10.089 | 400.205 |
| variable_damping | f4dd23ef304eb78f | hardware_refine_2 | 5 | 0 | 61.908 | 63.100 | 10.089 | 362.194 |
| variable_damping | 59b3e7cebdabb91a | hardware_refine_2 | 5 | 5 | 71.786 | 72.371 | 10.089 | 402.311 |
| variable_damping | 6abdb0230fd21562 | hardware_refine_2 | 5 | 1 | 56.242 | 57.021 | 10.067 | 364.188 |
| variable_damping | a4059053fa1408de | hardware_refine_2 | 5 | 5 | 64.854 | 65.235 | 10.067 | 400.907 |
| hybrid | 68a3b3986344cb72 | hardware_refine_2 | 5 | 0 | 62.003 | 63.620 | 10.067 | 359.472 |
| hybrid | f3b13f7b47271e62 | hardware_refine_2 | 5 | 5 | 71.981 | 72.529 | 10.067 | 400.266 |
| hybrid | 6184903781897f31 | hardware_refine_2 | 5 | 1 | 56.682 | 57.361 | 10.045 | 358.830 |
| hybrid | 55e8b8585ec75125 | hardware_refine_2 | 5 | 5 | 62.876 | 64.824 | 10.067 | 398.566 |
| baseline | 6eb03892cae3cd92 | hardware_refine_2 | 5 | 5 | 86.566 | 86.948 | 10.023 | 416.411 |
| baseline | 4948b0c6f5966d8b | hardware_refine_2 | 5 | 5 | 84.555 | 84.641 | 9.891 | 416.793 |
| baseline | 0d7edeaf399144a2 | hardware_refine_2 | 5 | 0 | 77.307 | 77.442 | 10.067 | 387.028 |
| baseline | 490b5c6625bd9898 | hardware_refine_2 | 5 | 5 | 75.313 | 75.624 | 9.935 | 382.490 |
| baseline | ed3bbc6a5b1a9221 | hardware_refine_2 | 5 | 0 | 73.550 | 74.136 | 10.067 | 369.257 |
| baseline | 204e541441d9945a | hardware_refine_2 | 5 | 0 | 71.404 | 72.166 | 9.935 | 361.367 |
| powered | 640f0f82a838eae3 | hardware_refine_2 | 5 | 4 | 53.926 | 54.870 | 10.089 | 441.942 |
| powered | 90fc020fd36129d2 | hardware_refine_2 | 5 | 5 | 51.743 | 51.899 | 10.001 | 440.882 |
| impedance | 65c7eb7cc16349eb | hardware_refine_2 | 5 | 5 | 53.564 | 53.646 | 10.089 | 441.887 |
| impedance | a8ba46491a8948f1 | hardware_refine_2 | 5 | 5 | 51.672 | 51.863 | 10.001 | 442.316 |
| optimized | 241ef92ca8f08bec | hardware_refine_2 | 5 | 3 | 52.669 | 53.163 | 10.045 | 441.343 |
| optimized | f4b3b3bf0739dbff | hardware_refine_2 | 5 | 4 | 51.059 | 52.029 | 9.979 | 441.276 |
| torque | 023c81ef2d51330f | hardware_refine_2 | 5 | 1 | 65.380 | 66.788 | 9.759 | 531.753 |
| torque | c40003795da27da6 | hardware_refine_2 | 5 | 0 | 58.543 | 60.572 | 9.627 | 530.901 |
| torque | 7b0148a1b033800c | hardware_refine_2 | 5 | 0 | 61.713 | 65.437 | 9.693 | 531.783 |
| torque | d76ca468d7587b52 | hardware_refine_2 | 5 | 0 | 49.602 | 52.057 | 9.451 | 530.808 |
| cosine | 81dcc1337831dd36 | hardware_refine_2 | 5 | 0 | 58.884 | 59.234 | 10.067 | 401.034 |
| cosine | 2e053c122698e4ea | hardware_refine_2 | 5 | 3 | 61.481 | 62.449 | 10.045 | 423.066 |
| cosine | 2c84f5232f4cea13 | hardware_refine_2 | 5 | 0 | 57.762 | 58.204 | 10.001 | 400.958 |
| cosine | f5e8a120495c2196 | hardware_refine_2 | 5 | 3 | 59.849 | 60.500 | 10.001 | 422.409 |

## Comparisons
```json
[
  {
    "comparison": [
      "hardware",
      "603334b9dfb2a230",
      "cfb13686bf2638de",
      "hardware_refine_2"
    ],
    "depth_matched": false,
    "endurance_complete": false,
    "campaign_complete": false,
    "winner": null,
    "provisional_leader": null,
    "candidates": [
      "impedance",
      "powered",
      "impedance",
      "hybrid",
      "gravity",
      "variable_damping",
      "variable_damping",
      "hybrid",
      "gravity",
      "baseline",
      "baseline",
      "baseline"
    ],
    "tied": [
      "impedance",
      "powered",
      "impedance"
    ],
    "note": "Depth matching required before declaring a winner",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
