"""No-CAN end-to-end timing and motion check for right Cartesian updates."""
import json
from pathlib import Path
import time

import numpy as np

from cartesian_goal.ik import CartesianIK, IK_TOLERANCE_M
from centering.motors import SPEED
from goal_motion.control import GoalControl
from safe_zone.geometry import Model, RIGHT_TCP, Zone


ROOT = Path(__file__).resolve().parents[1]
model = Model(ROOT / "model/openarmx.urdf")
zone = Zone.load(ROOT / "right_zones/zone1.json", model.digest, RIGHT_TCP)
joints = {joint.get("name"): joint for joint in model.joints}
limits = [joints[f"openarmx_right_joint{index}"].find("limit") for index in range(1, 8)]
lower = np.array([float(limit.get("lower")) for limit in limits])
upper = np.array([float(limit.get("upper")) for limit in limits])
center = np.zeros(7)
center[6] = upper[6]
ik = CartesianIK(model, zone, lower, upper, SPEED, "right", RIGHT_TCP, center, np.zeros(7))


def simulated_move(start, goal, target):
    """Ideal 0.4 rad/s firmware response driven by the real goal controller."""
    actual = start.copy()
    control = GoalControl(goal, lower, upper, 0.0)
    dt = 0.02
    for step in range(1, 1501):
        now = step * dt
        command, _, reached = control.update(actual, now)
        actual += np.clip(command - actual, -SPEED * dt, SPEED * dt)
        tcp_error = float(np.linalg.norm(ik.position(actual) - target))
        if reached:
            return actual, now, tcp_error
    raise AssertionError("simulated update did not settle within 30 seconds")


initial_offset = np.array([0.4, 0.02, 0.25])
initial = ik.solve(initial_offset)
actual = initial.joints.copy()
checks = []
for offset in (
    np.array([0.41, 0.02, 0.25]),
    np.array([0.41, 0.03, 0.25]),
    np.array([0.40, 0.03, 0.26]),
):
    started = time.perf_counter()
    result = ik.solve_update(offset, actual)
    planning_seconds = time.perf_counter() - started
    if planning_seconds >= 2.0:
        raise AssertionError(f"update preflight took {planning_seconds:.3f} seconds")
    if result.error_m > IK_TOLERANCE_M:
        raise AssertionError("IK result missed its requested coordinate")
    actual, motion_seconds, tcp_error = simulated_move(actual, result.joints, result.target)
    checks.append({
        "offset_m": offset.tolist(),
        "planning_seconds": round(planning_seconds, 3),
        "simulated_motion_seconds": round(motion_seconds, 3),
        "final_tcp_error_mm": round(tcp_error * 1000, 6),
    })

print(json.dumps(checks, indent=2))
print("PASS: fast safe updates reach each RViz Cartesian marker; no CAN or motor transport used")
