# Strike lab results

Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|
| gravity | 66632940ab35eb3c | manual | 6 | 6 | 56.149 | 57.106 | 9.979 | 399.270 |
| variable_damping | 384e0b28c3bee0e1 | manual | 3 | 3 | 57.288 | 57.791 | 9.979 | 397.386 |
| hybrid | d7b35f6ba2d09e1e | manual | 9 | 9 | 56.420 | 57.396 | 9.979 | 396.323 |

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
      "gravity",
      "hybrid",
      "variable_damping"
    ],
    "tied": [
      "gravity",
      "hybrid",
      "variable_damping"
    ],
    "note": "Differences within measurement brackets are treated as ties",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
