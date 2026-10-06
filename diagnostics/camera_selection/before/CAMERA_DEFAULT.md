# Shared default camera

All active camera users in X5 default to the connected **Orbbec Gemini 2 RGB**
camera:

- `start_beat.sh` / `launch_right_camera_playback.py`, in both hardware modes;
- `start_right_camera_cartesian.sh` / `launch_right_camera_cartesian.py`;
- direct `camera_search/camera.py` and `camera_playback/camera.py` invocations;
- optional camera evidence in `playback.sh` and the smooth-playback experiments.

Programs that do not use a camera remain camera-free. In particular,
`start_beat.sh --test` remains pure simulation, and `playback.sh --no-video`
still omits camera capture. Camera defaults do not change arm motion, recording
selection, calibration, alignment, strikes, audio, or relaxation behavior.

## Stable RGB selection

`config/camera.json` identifies the current Gemini by USB vendor/product,
serial **AY6R463004K**, and RGB interface **04**. `camera_search/device.py` resolves
its capture node at runtime, excluding the paired metadata node. The live-tested
node was initially `/dev/video4` and later `/dev/video2`, demonstrating why that
number is **not hardcoded** and may change after reconnecting or adding cameras.

Depth and infrared are separate interfaces. Do not use the first Orbbec node or
the generic `/dev/v4l/by-id` link: on this camera those can select depth/IR rather
than RGB. The shared opener requests native color MJPEG, **1280x720 at 30 FPS**,
compatible with the currently observed USB 2 connection. This is a **full-FOV
16:9 mode**, replacing the cropped 640x480 driver default.

## Maximum RGB field of view

The shared configuration now explicitly selects the entire RGB view rather than
accepting the driver's cropped 4:3 mode. Orbbec specifies approximately
**86 degrees horizontal x 55 degrees vertical** for 16:9, compared with
63 x 50 degrees in 4:3 ([manufacturer specification](https://www.orbbec.com.cn/index/Product/info.html?cate=38&id=51)).
These are nominal manufacturer angles, not a new calibrated angular measurement.

Live stationary-scene comparisons confirmed that **640x360, 1280x720, and
1920x1080 show the same complete view**. 1280x720 is the default to retain that
maximum FOV without the additional processing/storage cost of 1080p. The former
640x480 frame covered only the middle two-thirds of the full image width and
about eight-ninths of its height; it was not simply a lower-resolution full view.

All X5 camera users share this policy, including standalone video capture. It
also applies when a numeric `--camera N` points to the configured Gemini RGB node.
For the Orbbec, `--width`/`--height` may select one of the three verified 16:9
sizes; if only one dimension is supplied, the other is inferred at 16:9. Cropped
or unverified sizes (including 640x480) are rejected before opening the camera.
Actual driver resolution is checked after configuration; an unexpected fallback
closes capture and reports an error instead of silently reducing FOV.

This does not zoom out artificially, stretch or crop frames, change the lens,
or substitute the wider depth/IR sensor for RGB. Normal camera/model coordinates
use the complete captured image. The raw saved video has the same full coverage.

Existing numeric overrides still work on the camera launchers/processes:

```bash
./start_beat.sh --hardware --camera orbbec   # same camera as the default
./start_beat.sh --hardware --camera 0        # explicit override, not necessarily RGB
./start_right_camera_cartesian.sh --camera orbbec
```

These are normal program commands, **not camera-only tests**; hardware modes
retain their original motor workflow. To change the default physical device
later, update `config/camera.json` after identifying its actual RGB interface.

Close OrbbecViewer or other programs using the camera before launching X5.
The SDK viewer can temporarily remove the Linux video nodes while it owns the
interfaces. If the configured camera is absent or unavailable, X5 reports a clear
error instead of silently using another camera. No SDK/firmware update or new
system driver was required for this change.

## Verification

Evidence and backups are under `diagnostics/orbbec_default/`. Tests open only the
camera; they do not launch an arm controller. Live integration checks exercise the
real capture, preprocessing, YOLO pose model, preview, saved video, and detection
messages in each camera process. Their automatic quit key is test-only.

The shared selector has offline tests for device renumbering, identity matching,
depth/IR/metadata exclusion, no fallback, numeric overrides, all parser defaults,
capture cleanup, standalone-video failure handling, full-FOV defaults and partial
dimensions, cropped-mode rejection, and driver-readback enforcement.

Full-FOV comparisons and repeat live integration checks are retained separately
under `diagnostics/orbbec_fov/`, preserving the original camera-default evidence.

The installed Orbbec SDK/Viewer was consulted only to identify the device and its
ownership of the streams; X5 continues to use its existing OpenCV/V4L2 pipeline.
For SDK-based applications, Orbbec documents its separate color-only pipeline in
the [official quick-start guide](https://orbbec.github.io/pyorbbecsdk/source/3_QuickStarts/QuickStart.html).
