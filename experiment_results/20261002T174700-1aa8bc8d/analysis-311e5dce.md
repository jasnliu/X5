# Strike lab results

Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|
| gravity | 66632940ab35eb3c | manual | 15 | 15 | 56.808 | 57.476 | 9.957 | 399.850 |
| variable_damping | 384e0b28c3bee0e1 | manual | 6 | 6 | 57.500 | 58.859 | 9.990 | 397.208 |
| hybrid | d7b35f6ba2d09e1e | manual | 24 | 24 | 56.911 | 57.853 | 9.979 | 394.349 |
| cosine | ebfb12854abf6b98 | manual | 9 | 9 | 66.172 | 66.544 | 9.979 | 461.150 |
| optimized | 7a7b6e637a1a39e5 | manual | 6 | 6 | 56.166 | 56.487 | 9.979 | 460.864 |
| baseline | 7518d3671ae4728c | manual | 3 | 3 | 80.407 | 80.538 | 9.979 | 392.224 |
| powered | 6703ce90a2b96a8d | manual | 18 | 18 | 56.779 | 57.281 | 9.979 | 461.311 |
| torque | 8987bb893be816d1 | manual | 3 | 3 | 60.728 | 60.969 | 10.001 | 476.734 |
| impedance | 87f67162675a1b8a | manual | 3 | 3 | 53.509 | 53.733 | 9.979 | 462.626 |

## Comparisons
```json
[
  {
    "comparison": [
      "hardware",
      "603334b9dfb2a230",
      "cfb13686bf2638de",
      "manual"
    ],
    "depth_matched": true,
    "endurance_complete": false,
    "campaign_complete": false,
    "winner": null,
    "provisional_leader": null,
    "candidates": [
      "impedance",
      "optimized",
      "powered",
      "gravity",
      "hybrid",
      "variable_damping",
      "torque",
      "cosine",
      "baseline"
    ],
    "tied": [
      "impedance",
      "optimized",
      "powered",
      "gravity",
      "hybrid",
      "variable_damping",
      "torque"
    ],
    "note": "Differences within measurement brackets are treated as ties",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
