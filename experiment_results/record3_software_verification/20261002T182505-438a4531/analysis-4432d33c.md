# Strike lab results

Encoder lower-zone timing only, not measured cymbal contact or sound quality; simulation is not physical evidence.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Goal ° | Zone >° | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | e5939ebc598f8e87 | manual | 3 | 2.7 | 3 | 3 | 40.762 | 40.762 | 2.956 | 298.000 |
| gravity | 1f411f9b4cca8693 | manual | 3 | 2.7 | 3 | 3 | 55.762 | 55.762 | 2.956 | 370.000 |
| variable_damping | 677394c5c34e0f31 | manual | 3 | 2.7 | 3 | 3 | 57.096 | 57.096 | 2.978 | 370.000 |
| powered | 6dc662cdb561f212 | manual | 3 | 2.7 | 3 | 3 | 88.629 | 88.629 | 3.000 | 462.000 |
| cosine | e9b26969ac77dcb6 | manual | 3 | 2.7 | 3 | 3 | 68.972 | 68.972 | 3.000 | 520.000 |
| torque | 6874be10814c41f7 | manual | 3 | 2.7 | 3 | 3 | 90.315 | 90.906 | 3.022 | 460.000 |
| hybrid | 1bcadfbf67624594 | manual | 3 | 2.7 | 3 | 3 | 56.762 | 56.762 | 2.978 | 352.000 |
| impedance | 6652a0301f2347c2 | manual | 3 | 2.7 | 3 | 3 | 88.629 | 88.629 | 3.000 | 462.000 |
| optimized | 056745070db6529b | manual | 3 | 2.7 | 3 | 3 | 88.629 | 88.629 | 3.000 | 462.000 |

## Comparisons
```json
[
  {
    "comparison": [
      "simulation",
      "603334b9dfb2a230",
      "bef118a0205ca0a8",
      "manual",
      3.0,
      2.7
    ],
    "depth_matched": true,
    "endurance_complete": false,
    "campaign_complete": false,
    "winner": null,
    "provisional_leader": "baseline",
    "candidates": [
      "baseline",
      "gravity",
      "hybrid",
      "variable_damping",
      "cosine",
      "impedance",
      "optimized",
      "powered",
      "torque"
    ],
    "tied": [
      "baseline"
    ],
    "note": "Differences within measurement brackets are treated as ties",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
