# Strike lab results

Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|
| powered | 9227cc5feb2a5c09 | validate | 100 | 99 | 55.013 | 55.592 | 10.023 | 482.721 |
| impedance | 87f67162675a1b8a | validate | 100 | 100 | 55.138 | 55.773 | 10.045 | 485.515 |
| hybrid | d7b35f6ba2d09e1e | validate | 100 | 100 | 55.674 | 56.687 | 9.913 | 390.820 |
| gravity | 66632940ab35eb3c | validate | 100 | 100 | 55.247 | 56.300 | 9.913 | 399.872 |
| cosine | bdab0bba51d7bbfa | validate | 100 | 93 | 64.162 | 64.851 | 9.990 | 441.805 |
| variable_damping | 384e0b28c3bee0e1 | validate | 100 | 100 | 55.861 | 57.009 | 9.935 | 396.729 |
| baseline | 7518d3671ae4728c | validate | 100 | 100 | 78.846 | 80.276 | 9.913 | 393.244 |
| torque | 347521193e9aad83 | validate | 100 | 99 | 66.646 | 67.850 | 10.045 | 488.974 |
| optimized | 2a1fc6e27dd57c7b | validate | 100 | 98 | 54.574 | 55.325 | 10.023 | 480.807 |
| gravity | 66632940ab35eb3c | endurance | 100 | 100 | 56.849 | 58.116 | 9.957 | 406.550 |
| hybrid | d7b35f6ba2d09e1e | endurance | 100 | 100 | 56.719 | 57.921 | 9.979 | 403.620 |
| optimized | 2a1fc6e27dd57c7b | endurance | 100 | 98 | 54.079 | 54.960 | 10.001 | 463.657 |
| variable_damping | 384e0b28c3bee0e1 | endurance | 100 | 99 | 56.963 | 58.739 | 9.979 | 409.207 |
| cosine | bdab0bba51d7bbfa | endurance | 100 | 99 | 63.700 | 64.602 | 10.001 | 441.890 |
| baseline | 7518d3671ae4728c | endurance | 100 | 100 | 80.558 | 81.689 | 9.957 | 397.192 |
| impedance | 87f67162675a1b8a | endurance | 100 | 99 | 54.668 | 55.987 | 10.001 | 462.345 |
| powered | 9227cc5feb2a5c09 | endurance | 100 | 98 | 54.529 | 55.386 | 10.023 | 462.809 |
| torque | 347521193e9aad83 | endurance | 100 | 84 | 65.496 | 67.782 | 10.001 | 502.584 |

## Comparisons
```json
[
  {
    "comparison": [
      "hardware",
      "603334b9dfb2a230",
      "cfb13686bf2638de",
      "validate"
    ],
    "depth_matched": false,
    "endurance_complete": false,
    "campaign_complete": true,
    "winner": null,
    "provisional_leader": null,
    "candidates": [
      "impedance",
      "gravity",
      "hybrid",
      "variable_damping",
      "baseline"
    ],
    "tied": [
      "impedance",
      "gravity",
      "hybrid",
      "variable_damping"
    ],
    "note": "Depth matching required before declaring a winner",
    "validation_complete": true
  },
  {
    "comparison": [
      "hardware",
      "603334b9dfb2a230",
      "cfb13686bf2638de",
      "endurance"
    ],
    "depth_matched": true,
    "endurance_complete": true,
    "campaign_complete": true,
    "winner": null,
    "provisional_leader": null,
    "candidates": [
      "hybrid",
      "gravity",
      "baseline"
    ],
    "tied": [
      "hybrid",
      "gravity"
    ],
    "note": "Differences within measurement brackets are treated as ties",
    "validation_complete": true
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
