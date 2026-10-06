# Strike lab results

Encoder lower-zone timing only, not measured cymbal contact or sound quality; simulation is not physical evidence.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Goal ° | Zone >° | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| gravity | 66632940ab35eb3c | manual | 10 | 9 | 6 | 6 | 60.603 | 61.538 | 10.176 | 400.004 |
| hybrid | d7b35f6ba2d09e1e | manual | 10 | 9 | 3 | 3 | 60.155 | 60.645 | 10.154 | 393.288 |
| hybrid | d7b35f6ba2d09e1e | manual | 11 | 9.9 | 25 | 1 | 56.498 | 57.969 | 10.858 | 399.444 |
| gravity | 66632940ab35eb3c | manual | 11 | 9.9 | 8 | 0 | 57.126 | 58.249 | 10.891 | 407.702 |
| powered | 6703ce90a2b96a8d | manual | 11 | 9.9 | 8 | 0 | 56.269 | 57.459 | 10.869 | 461.274 |
| powered | 6703ce90a2b96a8d | manual | 11.5 | 10.35 | 8 | 0 | 51.652 | 53.464 | 11.166 | 461.774 |

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
