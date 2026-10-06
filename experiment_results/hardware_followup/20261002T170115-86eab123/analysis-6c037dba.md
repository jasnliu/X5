# Strike lab results

Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|
| powered | 6703ce90a2b96a8d | bounded_robustness_followup | 20 | 20 | 57.025 | 57.815 | 10.001 | 462.865 |
| optimized | 7a7b6e637a1a39e5 | bounded_robustness_followup | 20 | 20 | 56.725 | 57.686 | 9.990 | 462.940 |
| cosine | ebfb12854abf6b98 | bounded_robustness_followup | 20 | 20 | 66.653 | 67.750 | 10.001 | 461.855 |
| torque | 2e5b0d45a8bab20c | bounded_robustness_followup | 20 | 9 | 57.397 | 58.961 | 9.957 | 479.055 |

## Comparisons
```json
[
  {
    "comparison": [
      "hardware",
      "603334b9dfb2a230",
      "cfb13686bf2638de",
      "bounded_robustness_followup"
    ],
    "depth_matched": true,
    "endurance_complete": false,
    "campaign_complete": false,
    "winner": null,
    "provisional_leader": null,
    "candidates": [
      "optimized",
      "powered",
      "cosine"
    ],
    "tied": [
      "optimized",
      "powered"
    ],
    "note": "Differences within measurement brackets are treated as ties",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
