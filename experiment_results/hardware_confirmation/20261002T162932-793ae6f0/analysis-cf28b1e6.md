# Strike lab results

Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|
| torque | 5a2eae3aee09def3 | torque_cadence_check | 5 | 3 | 68.734 | 69.441 | 10.198 | 481.652 |
| torque | ea29672c8aa2a08a | torque_cadence_check | 5 | 5 | 71.180 | 71.363 | 10.176 | 501.265 |
| torque | 4823c31e191465f3 | torque_cadence_check | 5 | 5 | 66.489 | 67.085 | 10.110 | 481.242 |
| torque | 921e3b974887344d | torque_cadence_check | 5 | 5 | 68.623 | 68.864 | 10.067 | 500.714 |

## Comparisons
```json
[
  {
    "comparison": [
      "hardware",
      "603334b9dfb2a230",
      "cfb13686bf2638de",
      "torque_cadence_check"
    ],
    "depth_matched": false,
    "endurance_complete": false,
    "campaign_complete": false,
    "winner": null,
    "provisional_leader": null,
    "candidates": [
      "torque",
      "torque",
      "torque"
    ],
    "tied": [
      "torque",
      "torque",
      "torque"
    ],
    "note": "Depth matching required before declaring a winner",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
