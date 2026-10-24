# Right/left experiment modes verification

- `./plan_drums.sh experiment --right`: original right start, record3 endpoint and right zone3.
- `./plan_drums.sh experiment --left`: saved current left snapshot, left_recordings/record1 endpoint and left zone1.
- Same frozen Gemini scene and four endpoint-only algorithms. Opposite arm stays fixed in simulation.
- Left zone1 and identical tool dimensions are provisional defaults pending the user's confirmation.
- Right fitted joint offsets are not mirrored onto left joints. No physical collision certification is claimed.

## Verified

- **780 regression tests passed** (complete log: `regression.log`).
- Real CLI planning succeeded for all four methods on each arm, with final dense checks.
- Both native Qt/VTK windows exercised all four plans: play/pause, seek, goal/start,
  all speed options, loop/end, exact return, method switching, mesh/points/color,
  camera navigation, screenshot, settings-dialog cancel, and real worker cancel/resume.
- Unselected-arm transforms stayed bit-identical during playback. Camera mesh uploaded once.
- Actual left launcher auto-cached all four methods and completed an autoplay cycle during a 65-second UI run.
- No render errors or OOM. Observed planning worker peaks were below 400 MiB; 3 GiB caps and OS CAN exclusion verified.
- Initial left pose acquisition sent exactly **160 state queries on can1**, no other-bus access,
  and **zero enable/motion/gripper/relax commands**. Its socket closed before planning.
- Recordings, original URDF, encoder conversion, zones, old right defaults, and four algorithm sources were preserved.

`verification.json` contains results and paths to detailed local runs. Routine full runs remain ignored by Git.
`verify_arm_ui.py SESSION` is the saved native UI verification driver (run with ROS sourced and the experiment Python environment).
This is software validation plus a read-only encoder snapshot, not physical motion testing.
