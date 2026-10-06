# Record3 cymbal workflow: software verification, 2026-10-02

**Offline only. No physical arm operation, CAN sockets, camera, microphone, or live motion notification.**

## Implemented behavior

- Prepare validates a session snapshot of record3, closes the gripper, centers, plays J1–J7, then holds the exact recording endpoint.
- Recorded gripper openings are ignored for commands. The closed gripper goal persists throughout enabled setup, hold and strikes.
- All nine methods accept the same editable goal relative to the recording endpoint. Goal changes do not replay the recording or recapture the reference.
- UI, graph, scoring, immutable trial metadata, comparisons, campaigns and rescores use the selected goal. Different depths are not pooled.
- Existing beat programs, recording JSON files, motor helper and hardware-tuned profile remain byte-for-byte unchanged.

## Verification

- Full regression suite: **358 tests passed** (`full_tests.txt`). Includes hardware-sequence mocks and a process-level invalid-goal rejection followed by a valid custom-depth batch without re-preparation.
- Real Tk + ROS UI: all nine modes at 10°, then change entry to 3° and run all nine again. Checked entry locking, visible invalid-input message, changing graph markers, trial goal metadata, clean shutdown (`ui_check.txt`, `ui.png`).
- Real launcher + Tk + RViz: record3 preparation, all nine 10° simulated modes, saved traces, renderer startup, clean process-group shutdown (`launch_check.txt`, `visual_launch.txt`, `rviz.png`).
- Headless custom-depth batch: 27/27 synthetic passes at 3°; original recording endpoint retained across every method.
- Python compile and shell syntax checks passed. All protected checksums match (`protected_check.txt`).

## Preserved synthetic sessions

| Session | Attempts | Synthetic passes | Goal degrees |
|---|---:|---:|---|
| `experiment_results/record3_software_verification/20261002T182505-438a4531` | 27 | 27 | [3.0] |
| `experiment_results/record3_visual_check/20261002T182310-c79b68bf` | 9 | 9 | [10.0] |
| `experiment_results/record3_visual_check/20261002T182659-e2019909` | 9 | 9 | [10.0] |

## Evidence limits

These checks verify software routing, commands against mocked hardware, simulation and UI behavior. They do not establish real cymbal contact time, sound quality, loaded strike accuracy, or physical playback/hold performance.
The historical hardware preset remains calibrated only for the unloaded centered arm. Its gains, timings and fixed-load feedforward have not been retuned for record3 or the attached stick.
Stop, Finish, faults and shutdown retain the existing right-arm relaxation behavior (including the gripper).
Earlier diagnostic logs are retained; the final full regression run above supersedes the initial fake-adapter fixture failure.
