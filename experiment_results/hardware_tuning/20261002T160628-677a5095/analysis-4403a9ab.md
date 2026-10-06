# Strike lab results

Unloaded lower-zone timing only; simulation is not physical evidence or sound quality.

Statistics include every measured attempt, even failed strokes; missing measurements are not zeros.
Only all-pass groups can enter comparisons. See JSON for measurement counts and failure reasons.

| Method | Parameters | Stage | Attempts | Pass | Median zone ms | P95 zone ms | Median depth deg | Cycle ms |
|---|---|---|---:|---:|---:|---:|---:|---:|
| cosine | 19b4b27d7a81c09b | hardware_screen_2 | 3 | 3 | 78.213 | 79.603 | 10.110 | 523.612 |
| cosine | 4911a43f8c8650b6 | hardware_screen_2 | 3 | 3 | 69.053 | 69.476 | 10.132 | 462.161 |
| cosine | 95bd0f749863c25d | hardware_screen_2 | 3 | 0 | 61.453 | 61.822 | 10.198 | 402.310 |
| cosine | ecb799b1de0e6fd3 | hardware_screen_2 | 3 | 0 | 57.619 | 57.977 | 10.286 | 437.471 |
| cosine | c232afd5d94d171d | hardware_screen_2 | 3 | 0 | 54.935 | 55.190 | 10.352 | 351.401 |
| powered | 49eac29d27db90e6 | hardware_screen_2 | 2 | 2 | 100.692 | 101.385 | 10.110 | 571.276 |
| powered | eb401f405ecacdd2 | hardware_screen_2 | 2 | 2 | 89.844 | 89.852 | 10.089 | 509.506 |
| powered | 084755dfd704df08 | hardware_screen_2 | 2 | 2 | 79.945 | 80.252 | 10.089 | 460.928 |
| powered | 17143d703bad38c3 | hardware_screen_2 | 2 | 2 | 81.009 | 81.095 | 10.056 | 572.528 |
| powered | 7ac9a3a8420f2030 | hardware_screen_2 | 2 | 2 | 72.661 | 72.839 | 10.056 | 509.188 |
| powered | 77eb105bbb0a32db | hardware_screen_2 | 2 | 2 | 64.865 | 65.096 | 10.078 | 462.492 |
| powered | c69536c371546da5 | hardware_screen_2 | 2 | 2 | 69.026 | 69.212 | 10.056 | 573.519 |
| powered | 7632c8ce700559ab | hardware_screen_2 | 2 | 2 | 61.663 | 61.668 | 10.034 | 509.032 |
| powered | dca748e1dfdca598 | hardware_screen_2 | 2 | 2 | 55.422 | 55.593 | 10.078 | 461.911 |
| torque | f95d7cb824bf5454 | hardware_screen_2 | 2 | 0 | 131.258 | 132.776 | 10.484 | 602.462 |
| torque | 5a16538a2217212f | hardware_screen_2 | 2 | 1 | 123.242 | 124.183 | 10.242 | 602.893 |
| torque | 41c9084c8f740da6 | hardware_screen_2 | 2 | 2 | 116.775 | 117.559 | 10.110 | 590.757 |
| torque | 45dd0004236eb604 | hardware_screen_2 | 2 | 2 | 115.697 | 116.414 | 9.891 | 591.886 |
| torque | 7e1596bb0d70959f | hardware_screen_2 | 2 | 1 | 110.517 | 114.926 | 9.869 | 591.309 |
| torque | 17ee463b0a2740ca | hardware_screen_2 | 2 | 0 | 108.279 | 108.463 | 9.726 | 595.372 |
| gravity | d850ccd7321843e4 | hardware_screen_2 | 3 | 3 | 105.904 | 106.487 | 10.045 | 473.062 |
| gravity | 64676ee90edb7d71 | hardware_screen_2 | 3 | 3 | 99.673 | 99.846 | 10.067 | 458.426 |
| gravity | 0827b47e88a4095d | hardware_screen_2 | 3 | 3 | 95.945 | 96.331 | 10.067 | 448.612 |
| variable_damping | 488dc057f29d7a05 | hardware_screen_2 | 3 | 3 | 106.579 | 107.360 | 10.067 | 476.611 |
| variable_damping | d1599364608af334 | hardware_screen_2 | 3 | 3 | 100.554 | 100.607 | 10.067 | 460.757 |
| variable_damping | f370ddd77765e055 | hardware_screen_2 | 3 | 3 | 94.486 | 95.140 | 10.089 | 443.771 |
| hybrid | be3ca5d8aad2674a | hardware_screen_2 | 3 | 3 | 106.952 | 107.506 | 10.067 | 472.943 |
| hybrid | c0203cae17fed2e2 | hardware_screen_2 | 3 | 3 | 100.414 | 100.639 | 10.067 | 451.651 |
| hybrid | 89f9e082a99ce35e | hardware_screen_2 | 3 | 3 | 96.196 | 96.206 | 10.045 | 441.072 |
| baseline | c4661d8852ee9694 | hardware_screen_2 | 3 | 0 | 114.026 | 120.492 | 10.352 | 472.223 |
| baseline | b46406c4e335206a | hardware_screen_2 | 3 | 0 | 113.250 | 121.092 | 10.638 | 449.629 |
| baseline | d342cd850761b866 | hardware_screen_2 | 3 | 1 | 101.180 | 102.698 | 10.396 | 432.978 |

## Comparisons
```json
[
  {
    "comparison": [
      "hardware",
      "603334b9dfb2a230",
      "cfb13686bf2638de",
      "hardware_screen_2"
    ],
    "depth_matched": false,
    "endurance_complete": false,
    "campaign_complete": false,
    "winner": null,
    "provisional_leader": null,
    "candidates": [
      "powered",
      "powered",
      "powered",
      "powered",
      "cosine",
      "powered",
      "cosine",
      "powered",
      "powered",
      "powered",
      "variable_damping",
      "gravity",
      "hybrid",
      "gravity",
      "hybrid",
      "variable_damping",
      "powered",
      "gravity",
      "variable_damping",
      "hybrid",
      "torque",
      "torque"
    ],
    "tied": [
      "powered",
      "powered",
      "powered"
    ],
    "note": "Depth matching required before declaring a winner",
    "validation_complete": false
  }
]
```

Failed, interrupted, and rejected attempts remain in their individual trial directories.
