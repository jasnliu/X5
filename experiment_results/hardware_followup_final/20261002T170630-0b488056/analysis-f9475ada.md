# Strike lab results

Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|
| torque | 8987bb893be816d1 | validate | 100 | 100 | 61.422 | 62.115 | 10.001 | 486.377 |
| powered | 6703ce90a2b96a8d | validate | 100 | 100 | 57.694 | 58.186 | 10.023 | 479.793 |
| optimized | 7a7b6e637a1a39e5 | validate | 100 | 100 | 57.366 | 57.938 | 10.001 | 479.329 |
| gravity | 66632940ab35eb3c | validate | 100 | 100 | 55.530 | 56.684 | 9.913 | 399.328 |
| cosine | ebfb12854abf6b98 | validate | 100 | 99 | 67.382 | 67.943 | 10.001 | 478.350 |
| cosine | ebfb12854abf6b98 | endurance | 100 | 98 | 66.756 | 67.551 | 10.001 | 462.207 |
| powered | 6703ce90a2b96a8d | endurance | 100 | 98 | 57.214 | 58.198 | 10.001 | 462.851 |
| optimized | 7a7b6e637a1a39e5 | endurance | 100 | 100 | 56.851 | 57.602 | 10.001 | 462.392 |
| torque | 8987bb893be816d1 | endurance | 100 | 98 | 60.085 | 62.030 | 9.935 | 490.961 |
| gravity | 66632940ab35eb3c | endurance | 100 | 100 | 56.874 | 58.299 | 9.979 | 413.341 |
| gravity | 66632940ab35eb3c | extended_cadence | 30 | 30 | 57.077 | 58.404 | 9.957 | 412.404 |

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
    "endurance_complete": true,
    "campaign_complete": true,
    "winner": null,
    "provisional_leader": null,
    "candidates": [
      "gravity",
      "optimized",
      "powered",
      "torque"
    ],
    "tied": [
      "gravity",
      "optimized",
      "powered"
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
      "optimized",
      "gravity"
    ],
    "tied": [
      "optimized",
      "gravity"
    ],
    "note": "Differences within measurement brackets are treated as ties",
    "validation_complete": true
  },
  {
    "comparison": [
      "hardware",
      "603334b9dfb2a230",
      "cfb13686bf2638de",
      "extended_cadence"
    ],
    "depth_matched": true,
    "endurance_complete": true,
    "campaign_complete": true,
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
