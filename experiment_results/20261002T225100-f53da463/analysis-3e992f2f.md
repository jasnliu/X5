# Strike lab results

Encoder lower-zone timing only, not measured cymbal contact or sound quality; simulation is not physical evidence.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Goal ° | Zone >° | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| gravity | 66632940ab35eb3c | manual | 10 | 9 | 6 | 6 | 60.603 | 61.538 | 10.176 | 400.004 |
| hybrid | d7b35f6ba2d09e1e | manual | 10 | 9 | 3 | 3 | 60.155 | 60.645 | 10.154 | 393.288 |
| hybrid | d7b35f6ba2d09e1e | manual | 11 | 9.9 | 9 | 0 | 56.244 | 57.832 | 10.858 | 400.258 |

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
    "provisional_leader": null,
    "candidates": [
      "hybrid",
      "gravity"
    ],
    "tied": [
      "hybrid",
      "gravity"
    ],
    "note": "Differences within measurement brackets are treated as ties",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
