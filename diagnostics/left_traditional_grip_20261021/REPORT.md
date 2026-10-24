# Left traditional-grip draft — 2026-10-21 UTC

## Scope

Official model update for `./plan_drums.sh experiment --left`. No CAN, camera
capture, physical motion, gripper commands, or calibration changes during these
tests. Reuses the saved left encoder snapshot and frozen Gemini scene. The four
algorithms, endpoint, zone1 TCP constraint and collision margins are unchanged.
Right-only generated URDF/SRDF remain byte-identical to the previous model.

## Geometry

- User STL files remain untouched, interpreted in millimetres.
- `gripper2left.stl` and `gripper2right.stl` have matching radius 7.4 mm bores
  along CAD Y, with cross-section centers at CAD X=-27.15, Z=12.4 mm.
- Opposite 0/180-degree CAD yaw and +/-27.15 mm X translation make the two
  half bores share the same axis and projecting face. The two semicircles
  complement one another, rather than overlap or point in opposite directions.
- Sleeve spans hand Y=-30.75 to +95.75 mm; bore center is hand Z=27.4 mm.
- Stick tip is 260.35 mm (10.25 inches) beyond the +95.75 mm forward face.
  Total modeled cylinder length is 386.85 mm, diameter 14.8 mm. Rear end is
  flush with the sleeve, as requested. Native cylinder +Z points along hand +Y.
- Visual geometry and collision geometry use the same STL transforms/cylinder.
  Python renderers now read the selected cylinder's actual length, matching
  native MoveIt FK and scoring instead of reusing the right-arm length.
- Closed CAD assembly corresponds to existing raw motor8 +14.16 degrees
  (negative internal coordinate). This does not infer slider travel or modify
  hardware settings. The preserved CAD attachment foot matches the old claw's
  foot geometry; this does not prove registration to the URDF mounting screws.

## Deliberate approximation

The nominal hand mount plane is Z=15 mm. Clocking along hand +Y was chosen
because it points forward at the current custom center and puts the tip over
the snare area at the saved endpoint. At that endpoint the draft tip is about
[0.3990, 0.2003, 0.2106] m in robot-base coordinates. `goal_preview.png` shows
that model against the saved scene. This is a rough placement, NOT independently
measured physical mounting calibration; the user will refine it visually.
The original robot URDF, recordings, motor conventions and calibration were
not altered to obtain this position. No recording interior was used as a seed.

## Verification

- All four methods generated validated software trajectories with the new
  attachment. Invalid search proposals were rejected normally, not accepted by
  reducing margins. Full final paths retained collision and zone checks.
- Native Qt/VTK tested method switching, play/pause, timestamps, exact goal and
  return, speed, looping, scene layers/colors, navigation, saving, settings
  cancel and real worker cancel/resume. No render/UI errors; scene uploaded once.
- At every tested preview timestamp the displayed tip matches the actual left
  cylinder transform/length and the opposite arm remains fixed.
- Geometry regressions check matching half bores/front faces, perpendicular
  stick axis, exact exposed length, closure metadata, right model preservation,
  native left FK and tip-only obstacle rejection, and CAD-content cache invalidation.
- **787 tests passed** in 242.91 seconds. `regression.log` contains the full result; `verification.json`
  and `ui_verification.json` retain model/input hashes and detailed results.
- Existing running windows/archived plans keep their historical model. Restart
  the experiment to generate a new model and fresh candidate cache.
