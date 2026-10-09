# start_beatTest.sh — original pre-left-recording program

Run from the main X5 workspace:

```bash
./start_beatTest.sh --hardware
```

For simulation without physical devices:

```bash
./start_beatTest.sh --test
```

This is the actual backup saved immediately before adding left-arm recording
playback. It is **not** the newer two-arm program with its centering patched.
VS Code contained no X5 local-history entries; use of this original backup was
explicitly approved instead.

All 131 archived files were copied byte-for-byte and verified against the
pre-change hash manifest. Other unchanged local dependencies have independent
copies here. The historical camera_playback and smooth_playback packages are
restored in full, with no dual_recording module. No historical Python source or
original launcher was edited.

Original behavior and defaults are preserved:

- Only the right arm plays a recording; the left retains its original center hold.
- The original right-arm centering and Center + Relax code are restored.
- The original default is recordings/record1.json, not the newer record3 default.
- Persistent recording-preflight caching is retained as it was in that version.
- Existing mode flags and camera selection remain unchanged.

The copied recordings are private to this snapshot. To explicitly use another
current recording, pass its absolute path with --recording.

The runtime lives in `/home/jason/Proyectos3/X5_beatTest`; the `X5` link in this
folder points there. Keeping it alongside the original X5 preserves the exact
original sibling Y2/ST7 paths without changing historical source code.
The ROS installation/vendor resources and sibling Y2/ST7 detector environments
remain shared dependencies. All beat-control Python packages, zones, tuning,
robot-model source, firmware source and recording files are independent copies.
RESTORE_MANIFEST.json records every copied file, source, and SHA-256 checksum.

The main start_beat.sh program and its files remain untouched. Do not run both
programs together. Restoration checks are offline/simulated only; no physical
motion or obstacle-clearance trial was performed.
