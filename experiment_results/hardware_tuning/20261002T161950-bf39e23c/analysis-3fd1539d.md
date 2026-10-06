# Strike lab results

Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|
| powered | 90fc020fd36129d2 | hardware_final_calibration | 8 | 8 | 52.525 | 52.899 | 10.067 | 440.859 |
| powered | c2bbf988e7de8e80 | hardware_final_calibration | 8 | 8 | 53.926 | 54.306 | 10.001 | 461.921 |
| impedance | a8ba46491a8948f1 | hardware_final_calibration | 8 | 8 | 52.947 | 53.661 | 10.067 | 441.717 |
| impedance | 74746168054bf69e | hardware_final_calibration | 8 | 8 | 53.972 | 54.556 | 10.001 | 461.360 |
| optimized | e5e8894047165424 | hardware_final_calibration | 8 | 8 | 51.501 | 51.896 | 10.023 | 442.282 |
| optimized | 5fafdd2530531cc7 | hardware_final_calibration | 8 | 8 | 53.484 | 53.965 | 9.979 | 461.713 |
| gravity | e36e5c63f1f33271 | hardware_final_calibration | 5 | 5 | 63.406 | 64.309 | 10.045 | 401.241 |
| gravity | 6a950caf1245c33b | hardware_final_calibration | 5 | 5 | 58.240 | 58.791 | 10.067 | 399.248 |
| gravity | f3738538dfc8fca2 | hardware_final_calibration | 5 | 5 | 61.345 | 63.259 | 9.979 | 401.346 |
| gravity | c00ee4de48c641c1 | hardware_final_calibration | 5 | 5 | 57.379 | 57.750 | 9.979 | 400.717 |
| variable_damping | f6d1c5504b23d615 | hardware_final_calibration | 5 | 5 | 64.406 | 65.085 | 10.067 | 399.089 |
| variable_damping | f9e308f11325b27c | hardware_final_calibration | 5 | 5 | 59.264 | 60.690 | 10.067 | 398.920 |
| variable_damping | 47f513b72eacfff4 | hardware_final_calibration | 5 | 5 | 62.530 | 63.286 | 10.001 | 398.827 |
| variable_damping | 85002e5f12b7384c | hardware_final_calibration | 5 | 5 | 58.180 | 58.951 | 10.001 | 398.253 |
| hybrid | eb30b6d85e8681aa | hardware_final_calibration | 5 | 5 | 64.125 | 65.417 | 10.067 | 394.300 |
| hybrid | 8e01baafa8c97dea | hardware_final_calibration | 5 | 5 | 58.574 | 58.800 | 10.045 | 391.769 |
| hybrid | 2ae020f366b75631 | hardware_final_calibration | 5 | 5 | 62.937 | 63.112 | 9.957 | 393.935 |
| hybrid | 71ad8d61785c6dbb | hardware_final_calibration | 5 | 5 | 57.740 | 57.864 | 9.979 | 393.436 |
| baseline | 10560cef05a51ceb | hardware_final_calibration | 8 | 0 | 76.417 | 76.797 | 9.990 | 383.875 |
| baseline | 19b45692e7c9b6fb | hardware_final_calibration | 8 | 6 | 76.110 | 76.333 | 9.979 | 384.702 |
| torque | c9729430e6e6a9e5 | hardware_final_calibration | 8 | 6 | 76.909 | 84.990 | 10.100 | 551.112 |
| torque | 634cd29846cd815a | hardware_final_calibration | 8 | 8 | 69.002 | 70.024 | 9.836 | 540.279 |
| torque | 72feb668897c5264 | hardware_final_calibration | 8 | 0 | 64.302 | 65.009 | 9.726 | 541.891 |
| cosine | 1d241c53ba7cfef5 | hardware_final_calibration | 8 | 6 | 61.536 | 62.214 | 9.990 | 432.048 |
| cosine | 190c0fe7a4398ce5 | hardware_final_calibration | 8 | 8 | 63.423 | 63.967 | 9.990 | 441.270 |
| cosine | af435d3f7efa7792 | hardware_final_calibration | 8 | 8 | 61.950 | 62.284 | 10.001 | 431.259 |
| cosine | e994b592614a649f | hardware_final_calibration | 8 | 8 | 63.286 | 63.717 | 9.979 | 441.010 |

## Comparisons
```json
[
  {
    "comparison": [
      "hardware",
      "603334b9dfb2a230",
      "cfb13686bf2638de",
      "hardware_final_calibration"
    ],
    "depth_matched": false,
    "endurance_complete": false,
    "campaign_complete": false,
    "winner": null,
    "provisional_leader": null,
    "candidates": [
      "optimized",
      "powered",
      "impedance",
      "optimized",
      "powered",
      "impedance",
      "gravity",
      "hybrid",
      "variable_damping",
      "gravity",
      "hybrid",
      "variable_damping",
      "gravity",
      "cosine",
      "variable_damping",
      "hybrid",
      "cosine",
      "gravity",
      "cosine",
      "hybrid",
      "variable_damping",
      "torque"
    ],
    "tied": [
      "optimized",
      "powered",
      "impedance",
      "optimized",
      "powered",
      "impedance",
      "gravity",
      "hybrid",
      "variable_damping",
      "gravity",
      "hybrid",
      "variable_damping"
    ],
    "note": "Depth matching required before declaring a winner",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
