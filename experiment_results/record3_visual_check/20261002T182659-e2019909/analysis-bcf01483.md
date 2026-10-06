# Strike lab results

Encoder lower-zone timing only, not measured cymbal contact or sound quality; simulation is not physical evidence.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Goal ° | Zone >° | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | e5939ebc598f8e87 | manual | 10 | 9 | 1 | 1 | 78.122 | 78.122 | 9.946 | 520.000 |
| gravity | 1f411f9b4cca8693 | manual | 10 | 9 | 1 | 1 | 72.820 | 72.820 | 9.968 | 486.000 |
| variable_damping | 677394c5c34e0f31 | manual | 10 | 9 | 1 | 1 | 72.885 | 72.885 | 9.968 | 482.000 |
| powered | 6dc662cdb561f212 | manual | 10 | 9 | 1 | 1 | 88.820 | 88.820 | 9.990 | 460.000 |
| cosine | e9b26969ac77dcb6 | manual | 10 | 9 | 1 | 1 | 69.220 | 69.220 | 9.990 | 522.000 |
| torque | 6874be10814c41f7 | manual | 10 | 9 | 1 | 1 | 88.722 | 88.722 | 9.990 | 462.000 |
| hybrid | 1bcadfbf67624594 | manual | 10 | 9 | 1 | 1 | 72.418 | 72.418 | 9.968 | 458.000 |
| impedance | 6652a0301f2347c2 | manual | 10 | 9 | 1 | 1 | 89.122 | 89.122 | 9.990 | 462.000 |
| optimized | 056745070db6529b | manual | 10 | 9 | 1 | 1 | 88.820 | 88.820 | 9.990 | 462.000 |

## Comparisons
```json
[
  {
    "comparison": [
      "simulation",
      "603334b9dfb2a230",
      "bef118a0205ca0a8",
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
      "cosine",
      "hybrid",
      "gravity",
      "variable_damping",
      "baseline",
      "torque",
      "optimized",
      "powered",
      "impedance"
    ],
    "tied": [
      "cosine",
      "hybrid",
      "gravity",
      "variable_damping"
    ],
    "note": "Differences within measurement brackets are treated as ties",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
