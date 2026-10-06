# Strike lab results

Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|
| powered | 9227cc5feb2a5c09 | frozen_profile_confirmation | 15 | 15 | 53.732 | 54.256 | 10.001 | 461.398 |
| impedance | 87f67162675a1b8a | frozen_profile_confirmation | 15 | 15 | 53.917 | 54.449 | 10.001 | 461.979 |
| optimized | 2a1fc6e27dd57c7b | frozen_profile_confirmation | 15 | 15 | 53.346 | 53.899 | 9.979 | 461.350 |
| gravity | 66632940ab35eb3c | frozen_profile_confirmation | 15 | 15 | 56.803 | 57.546 | 9.979 | 400.602 |
| variable_damping | 384e0b28c3bee0e1 | frozen_profile_confirmation | 15 | 15 | 57.296 | 58.343 | 9.979 | 398.200 |
| hybrid | d7b35f6ba2d09e1e | frozen_profile_confirmation | 15 | 15 | 57.340 | 58.099 | 9.979 | 392.893 |
| baseline | 7518d3671ae4728c | frozen_profile_confirmation | 15 | 15 | 80.171 | 80.916 | 9.979 | 395.664 |
| cosine | bdab0bba51d7bbfa | frozen_profile_confirmation | 15 | 15 | 63.548 | 64.116 | 10.001 | 441.486 |
| torque | f1470ac9ec240cdf | frozen_profile_confirmation | 15 | 15 | 73.826 | 75.450 | 10.001 | 546.744 |

## Comparisons
```json
[
  {
    "comparison": [
      "hardware",
      "603334b9dfb2a230",
      "cfb13686bf2638de",
      "frozen_profile_confirmation"
    ],
    "depth_matched": true,
    "endurance_complete": false,
    "campaign_complete": false,
    "winner": null,
    "provisional_leader": null,
    "candidates": [
      "optimized",
      "powered",
      "impedance",
      "gravity",
      "variable_damping",
      "hybrid",
      "cosine",
      "torque",
      "baseline"
    ],
    "tied": [
      "optimized",
      "powered",
      "impedance",
      "gravity",
      "variable_damping",
      "hybrid"
    ],
    "note": "Differences within measurement brackets are treated as ties",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
