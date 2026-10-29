"""Local paths and explicit numerical/design tolerances, not motor settings."""
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
DT1=ROOT.parent/'dt1'
CAMERA_PYTHON=DT1/'.venv/bin/python'
# Numerical model-space tolerances, NOT camera accuracy claims.
POSITION_TOL_M=1e-5
ROLL_TOL_DEG=.1
# First try exact parallel, then solve the continuous above-parallel half-space.
# These are search modes, not a maximum permitted angle.
IK_TILT_MODES=(0.,None)
ANGLE_CONVENTION='shaft_tip_to_grip_above_disc_tangent_v2'
AIM_CONVENTION='disc_plane_grip_to_tip_inward_radial_v1'
# Operator-approved 2026-10-28: restores the original inward aiming window.
# This changes orientation preference/acceptance only, never joint limits.
MAX_INWARD_YAW_DEG=25.
EDGE_PREFERENCE_FRACTION=.02  # At most 2% preference among otherwise comparable approaches.
APPROACH_SHORTLIST=6
MAX_FEASIBLE_APPROACHES=3
# Endpoint-only design reserve, not a new physical limit or encoder tolerance.
# Leaves room around the former boundary goals (notably J7 and J2). This is
# not a measured guarantee against arbitrary tracking error/overshoot.
GOAL_JOINT_RESERVE_DEG=3.
# J2 already has a 1-degree operating clearance from its reported -1-degree
# stop. Avoid stacking another 3 degrees: retain 1 additional degree at goals.
# This changes ONLY J2's lower goal reserve; the path floor stays exactly zero.
GOAL_J2_LOWER_RESERVE_DEG=1.
VERSION='cymbal-prime-destination-v8'
