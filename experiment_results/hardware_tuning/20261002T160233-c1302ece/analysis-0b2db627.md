# Strike lab results

Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|
| cosine | 11871f8e8b01580d | family_calibration | 2 | 2 | 86.503 | 86.768 | 10.045 | 590.404 |
| impedance | 37e2e93b92f1edf6 | family_calibration | 2 | 2 | 130.865 | 131.108 | 9.968 | 642.920 |
| optimized | 1ab0f046bd8ca567 | family_calibration | 2 | 2 | 129.421 | 129.518 | 9.957 | 641.127 |
| torque | ab5ed847f3a03124 | family_calibration | 2 | 0 | 129.874 | 136.733 | 9.638 | 641.166 |
| gravity | 866c97570bd7ea33 | family_calibration | 2 | 0 | 120.011 | 122.644 | 10.495 | 501.440 |
| variable_damping | a613a45b6718c5b3 | family_calibration | 2 | 0 | 119.192 | 120.232 | 10.429 | 502.324 |
| hybrid | 79be1f4d92d27c86 | family_calibration | 2 | 0 | 117.624 | 118.569 | 10.385 | 496.800 |
| baseline | d342cd850761b866 | family_calibration | 2 | 1 | 93.289 | 95.951 | 10.154 | 427.794 |

## Comparisons
```json
[
  {
    "comparison": [
      "hardware",
      "603334b9dfb2a230",
      "cfb13686bf2638de",
      "family_calibration"
    ],
    "depth_matched": true,
    "endurance_complete": false,
    "campaign_complete": false,
    "winner": null,
    "provisional_leader": "cosine",
    "candidates": [
      "cosine",
      "optimized",
      "impedance"
    ],
    "tied": [
      "cosine"
    ],
    "note": "Differences within measurement brackets are treated as ties",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
