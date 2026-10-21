# Strike lab results

Encoder lower-zone timing only, not measured cymbal contact or sound quality; simulation is not physical evidence.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Goal ° | Zone >° | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| gravity | 66632940ab35eb3c | manual | 10 | 9 | 3 | 3 | 59.561 | 59.958 | 10.176 | 399.226 |
| powered | 6703ce90a2b96a8d | manual | 10 | 9 | 3 | 1 | 62.695 | 62.954 | 10.154 | 461.350 |

## Comparisons
```json
[
  {
    "comparison": [
      "hardware",
      "603334b9dfb2a230",
      "cfb13686bf2638de",
      "manual",
      10.0,
      9.0
    ],
    "depth_matched": true,
    "endurance_complete": false,
    "campaign_complete": false,
    "winner": null,
    "provisional_leader": "gravity",
    "candidates": [
      "gravity"
    ],
    "tied": [
      "gravity"
    ],
    "note": "Differences within measurement brackets are treated as ties",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
