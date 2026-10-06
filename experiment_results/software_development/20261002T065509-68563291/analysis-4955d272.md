# Strike lab results

Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.

| Method | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---:|---:|---:|---:|---:|---:|
| baseline | manual | 1 | 1 | 80.142 | 80.142 | 9.990 | 520.000 |
| gravity | manual | 1 | 1 | 75.035 | 75.035 | 10.012 | 488.000 |
| variable_damping | manual | 1 | 1 | 74.901 | 74.901 | 10.012 | 482.000 |
| powered | manual | 1 | 1 | 88.838 | 88.838 | 9.990 | 460.000 |
| cosine | manual | 1 | 1 | 69.238 | 69.238 | 9.990 | 522.000 |
| torque | manual | 1 | 1 | 89.542 | 89.542 | 10.012 | 462.000 |
| hybrid | manual | 1 | 1 | 76.835 | 76.835 | 10.078 | 462.000 |
| impedance | manual | 1 | 1 | 89.142 | 89.142 | 9.990 | 462.000 |
| optimized | manual | 1 | 1 | 88.838 | 88.838 | 9.990 | 462.000 |

## Comparisons
```json
[
  {
    "comparison": [
      "simulation",
      "85d17b3dc5693ca2",
      "bef118a0205ca0a8",
      "manual"
    ],
    "depth_matched": true,
    "winner": null,
    "provisional_leader": "cosine",
    "candidates": [
      "cosine",
      "variable_damping",
      "gravity",
      "hybrid",
      "baseline",
      "optimized",
      "powered",
      "impedance",
      "torque"
    ],
    "tied": [
      "cosine"
    ],
    "note": "Differences within measurement brackets are treated as ties",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
