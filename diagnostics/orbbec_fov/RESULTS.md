# Maximum Orbbec RGB FOV — 2026-10-04

## Finding

The previous camera selection correctly chose the Gemini RGB interface, but left
resolution at the driver's **640x480** default. That is a cropped 4:3 mode, not a
full-view resize. No V4L2 zoom/pan/tilt controls are exposed by this RGB interface.

The [manufacturer's specification](https://www.orbbec.com.cn/index/Product/info.html?cate=38&id=51)
lists nominal RGB FOV as **86 degrees horizontal x 55 degrees vertical for 16:9**,
versus 63 x 50 degrees for 4:3. These angles are manufacturer specifications;
we did not perform absolute angular calibration.

## Physical mode comparison

The RGB interface was enumerated using V4L2. All four advertised native MJPEG
sizes were captured on the connected USB 2 camera at approximately 30 FPS. The
camera remained stationary; sample images were visually inspected, and matching
static scene features were fit with a robust affine transform into 1920x1080:

| Native mode | Matched static features | Relative image-width coverage | Relative image-height coverage |
| --- | ---: | ---: | ---: |
| 640x480 (old) | 505 | 66.66% | 88.88% |
| 640x360 | 343 | 100.01% | 100.01% |
| 1280x720 | 951 | 100.01% | 100.01% |
| 1920x1080 | reference | 100% | 100% |

Values slightly over 100% are subpixel fitting error, not additional FOV.
The old image corresponds approximately to x=320..1600 and y=60..1020 inside
the 1920x1080 full frame. The 16:9 modes all show the same complete view.

**Selected default: 1280x720 MJPEG at 30 FPS.** This provides maximum RGB FOV
with less processing/storage than 1080p. `compare_modes.py`, `comparison.log`,
`modes.json`, `mode_comparison.json`, `relative_coverage.json`, and the four JPGs
retain the comparison code and data. No alternate tested mode or data was deleted.

## Implementation coverage

`config/camera.json` records the full-view default and verified full-FOV modes.
The existing shared `open_camera` applies it to camera-search, camera-playback
(both start_beat hardware modes), and standalone playback/experiment video.
The same enforcement applies if `--camera N` numerically selects this Gemini.

- Missing width/height defaults to 1280x720.
- A single requested dimension implies its 16:9 partner.
- Allowed explicit full-FOV sizes: 640x360, 1280x720, 1920x1080.
- Cropped or unverified sizes fail before the camera opens.
- Driver width/height readback must match the requested full-FOV mode; silent
  fallback is rejected and capture is closed.
- Frames are not software-cropped, stretched, or synthetically zoomed out.
- Unrelated camera overrides retain their original behavior. Camera-free modes
  stay camera-free. No arm-control or saved motion recording changes.

## Live integration checks

Both actual camera entry points were run with the Y2 model, preprocessing,
preview, Unix detection messages, and video writer. Only the quit key was
automated after ten seconds. Both produced **1280x720** images and videos:

- Search: 298 displayed frames, 271 inference-result messages in 10.03 s.
- Playback: 299 displayed frames, 270 inference-result messages in 10.04 s.
- Standalone `Video`: 91 full-view frames spanning 3.00 s, no capture error.
- Numeric RGB index override: verified 1280x720 capture.
- `--width 640` equivalent: verified 640x360 capture, not cropped 640x480.

The final scene became very dark after the illuminated mode comparison. Both
the old and new native modes were subsequently checked and were equally dark
(mean pixel values about 3/255). Read-only controls showed auto exposure enabled;
no exposure/gain/laser settings were changed. Thus the illuminated comparisons
establish expanded scene coverage; later tests establish full-size capture and
pipeline operation, **not recognition accuracy in darkness**. The lighting cause
was not independently confirmed. Raw checks and controls are in
`lighting_check.json` and `lighting_check_*.jpg`.

The existing optional low-light FPS-priority control remains unsupported (EINVAL)
and follows its existing warning path. Existing Qt font warnings remain nonfatal.
No CAN or arm-control application was run; camera integration tests explicitly
forbid CAN socket creation. The arm was not moved.

## Verification records

**Final result: all 402 regression tests passed**, including seventeen camera
selection/full-FOV tests. The thirteen transport tests also passed independently.

`verify_evidence.py` checks captured results, relative coverage, unchanged
protected files, syntax, and the final full regression result. It writes
`verification.json` and `changes.diff`. `before/` retains pre-change source.

The first two regression runs had one pre-existing wall-clock assertion fail:
`test_duplicate_gui_queries_are_suppressed_only_after_worker_is_ready` observed
19 ms and 17 ms against a 10 ms assertion. Both logs are retained as
`regressions_first.log` and `regressions_second.log`. That test now mocks the
clock and asserts the **exact timestamp** set when the worker owns feedback,
rather than measuring host scheduling/GC delay. This is a test-only determinism
fix, not a relaxed timeout or change to motor behavior; its original source is
backed up. The final full-suite result is in `regressions.log`.

To verify saved evidence again without accessing hardware:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 diagnostics/orbbec_fov/verify_evidence.py
```
