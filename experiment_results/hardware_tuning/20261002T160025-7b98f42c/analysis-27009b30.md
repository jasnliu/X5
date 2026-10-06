# Strike lab results

Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|
| powered | 917c4bd3da1a5d8b | speed_screen | 3 | 3 | 130.539 | 131.058 | 9.957 | 640.549 |
| powered | 6ea58d810a250087 | speed_screen | 3 | 1 | 110.217 | 110.669 | 10.001 | 541.306 |
| powered | 3566264b4142b075 | speed_screen | 3 | 0 | 99.993 | 100.421 | 10.067 | 485.680 |
| powered | f64a78adf70b1efc | speed_screen | 3 | 0 | 88.683 | 89.201 | 10.132 | 431.113 |
| powered | a5bf10591d0fb4df | speed_screen | 3 | 0 | 81.279 | 81.676 | 10.198 | 396.396 |
| powered | 6086fba0b16ed208 | speed_screen | 3 | 0 | 73.403 | 74.826 | 10.198 | 374.244 |

## Comparisons
```json
[
  {
    "comparison": [
      "hardware",
      "603334b9dfb2a230",
      "cfb13686bf2638de",
      "speed_screen"
    ],
    "depth_matched": true,
    "endurance_complete": false,
    "campaign_complete": false,
    "winner": null,
    "provisional_leader": "powered",
    "candidates": [
      "powered"
    ],
    "tied": [
      "powered"
    ],
    "note": "Differences within measurement brackets are treated as ties",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
