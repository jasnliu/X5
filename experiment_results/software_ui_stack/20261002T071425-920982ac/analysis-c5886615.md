# Strike lab results

Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|
| baseline | 6fa8880273dcbaa1 | manual | 1 | 1 | 80.142 | 80.142 | 9.990 | 520.000 |
| gravity | 5e798abb6eb9d830 | manual | 1 | 1 | 75.035 | 75.035 | 10.012 | 488.000 |
| variable_damping | be2ab18f7a6fd231 | manual | 1 | 1 | 75.301 | 75.301 | 10.034 | 482.000 |
| powered | a64337238518791f | manual | 1 | 1 | 88.838 | 88.838 | 9.990 | 460.000 |
| cosine | cff117c61ec6921a | manual | 1 | 1 | 69.238 | 69.238 | 9.990 | 522.000 |
| torque | 5debded42403dbc0 | manual | 1 | 1 | 89.542 | 89.542 | 10.012 | 462.000 |
| hybrid | 20357d2ba10a420e | manual | 1 | 1 | 76.835 | 76.835 | 10.078 | 462.000 |
| impedance | 1d4a1205cad2bcd3 | manual | 1 | 1 | 89.142 | 89.142 | 9.990 | 462.000 |
| optimized | a9d3fef24697d380 | manual | 1 | 1 | 88.838 | 88.838 | 9.990 | 462.000 |

## Comparisons
```json
[
  {
    "comparison": [
      "simulation",
      "603334b9dfb2a230",
      "bef118a0205ca0a8",
      "manual"
    ],
    "depth_matched": true,
    "endurance_complete": false,
    "winner": null,
    "provisional_leader": "cosine",
    "candidates": [
      "cosine",
      "gravity",
      "variable_damping",
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
