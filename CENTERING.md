# Simple left-arm centering

Close the previous arm programs, then:

```bash
cd /home/jason/X5
./start_centering.sh --hardware
```

- **CENTER LEFT ARM:** command existing encoder position **0 radians** to left
  motors **1–7**, using the motors' built-in position control. They hold until Relax.
- **EMERGENCY RELAX** or **Esc** in the panel: cancel pending commands and disable
  all left motors (1–8). Closing the program also disables motors it enabled.
- Left gripper stays disabled. Right arm receives **state queries only**.
- No heartbeat, separate control subprocess, custom correction loop or simulation
  physics. Without `--hardware`, the same buttons give a simple offline preview.

Verify **can1 = LEFT, can0 = RIGHT**. Both arms/grippers must start relaxed.
Nothing enables on launch. The goal is existing zero, not encoder recalibration.
The firmware speed setting is 0.4 rad/s; current settings are 6,6,4,4,2,2,2.

**Relax removes torque; it does not cut electrical power.** A physical power cut
requires hardware. Support the arm—it can drop when disabled. A failed computer
or CAN connection can prevent software relaxation. Keep physical cutoff available.

There is no collision checking or saved-zone enforcement. Keep the entire path
clear. RViz shows measured joint angles, not a claim that commanded zero was reached.
Hardware movement has not been tested by the assistant. Position-mode packet bytes
were checked against the installed OpenArmX SDK; offline checks are not physical proof.

Implementation: `centering/app.py` (buttons/display), `centering/motors.py`
(direct commands). The safe-zone recorder is unchanged.
