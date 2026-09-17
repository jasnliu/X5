# OpenArmX Gravity Feedforward Compensation

[English](#overview) | [中文](README_CN.md)

---

### Overview

This package provides real-time gravity feedforward compensation for the OpenArmX dual-arm robot. In MIT control mode, pure PD control results in approximately 6° steady-state position error due to gravity. This package reduces the error to less than 1° by injecting KDL-computed gravity compensation torques into the motor's `τ_ff` (feedforward torque).

### Implementation Principle

#### Problem Background

The motor control formula in MIT mode is:

```
τ_output = kp × (pos_cmd - pos_actual) + kd × (vel_cmd - vel_actual) + τ_ff
```

When `τ_ff = 0` (pure PD control), gravity causes steady-state error:

```
Steady-state error = τ_gravity / kp ≈ 5.3 / 50 ≈ 0.106 rad ≈ 6°
```

#### Solution

The `gravity_comp_node` subscribes to `/joint_states`, and upon receiving joint angles, uses the KDL recursive Newton-Euler algorithm to compute gravity compensation torques for each joint in real-time, writing them to the hardware's `τ_ff` via `forward_command_controller`.

**Feedforward torque is not a fixed value** - it varies in real-time with the manipulator's posture. When joints are extended, the gravity arm is longest and torque is maximum; when joints are folded, torque decreases.

#### Complete Data Flow

```
/joint_states  (sensor_msgs/JointState, containing 14 joint positions)
        │
        ▼
gravity_comp_node
  ├─ Find left arm 7 joint angles q[7] by name
  │       ↓
  │   Dynamics::GetGravity(q, tau_g)
  │     └─ KDL::ChainDynParam::JntToGravity()  ← Recursive Newton-Euler algorithm
  │       ↓
  │   tau_out[j] = clamp(g_scale × tau_g[j], ±TAU_LIMITS[j])
  │       ↓
  │   /left_forward_effort_controller/commands  (Float64MultiArray)
  │       ↓
  │   left_forward_effort_controller
  │     └─ Write to effort command interface → tau_commands_[i]
  │       ↓
  │   v10_simple_hardware.cpp  write()
  │     └─ param.torque = tau_commands_[i] × direction_multipliers[i]
  │       ↓
  │   MIT CAN packet → Left arm motor τ_ff
  │
  └─ Right arm follows same path → /right_forward_effort_controller/commands → Right arm motors
```

#### Gravity Direction Convention

When the manipulator base is mounted with ±90° rotation around the X-axis, the world coordinate system gravity `[0, 0, -9.81]` becomes Y-direction in each arm's link0 coordinate system after rotation:

| Arm | Actual gravity vector in link0 | Code setting value |
|----|---------------------|-----------|
| Right arm | `[0, +9.81, 0]` | `RIGHT_ARM_GY = -9.81` |
| Left arm | `[0, -9.81, 0]` | `LEFT_ARM_GY  = +9.81` |

The code setting values are opposite to the actual direction because the hardware `write()` uniformly multiplies by `-1`, resulting in correct direction after two negations. No additional direction correction is needed within the node.

#### Safety Clamping (TAU_LIMITS)

Prevents excessive feedforward output in abnormal postures:

| Joint | Motor Model | Current Limit |
|------|---------|---------|
| joint1, 2 | RS04 | 20 Nm |
| joint3, 4 | RS03 | 7 Nm |
| joint5, 6, 7 | RS00 | 2 Nm |

---

### Package Structure

```
openarmx_gravity_comp/
├── include/
│   └── dynamics.hpp          # Dynamics class declaration (KDL dynamics wrapper)
├── src/
│   ├── dynamics.cpp          # KDL dynamics implementation (gravity, Coriolis, inertia, Jacobian)
│   └── gravity_comp_node.cpp # ROS2 node main body, subscribes to joint states and publishes feedforward torques
├── CMakeLists.txt
├── package.xml
├── GRAVITY_COMP_NOTES.md     # Detailed design notes (notation conventions, error analysis, etc.)
└── README_CN.md
```

**Dynamics Class Main Interface:**

| Method | Description |
|------|------|
| `Init()` | Parse URDF, build KDL kinematic chain, create dynamics solver |
| `SetGravityVector(gx, gy, gz)` | Set gravity vector (in link0 coordinate system) |
| `GetGravity(q, tau_g)` | Compute gravity compensation torque for each joint |
| `GetColiori(q, q_dot, tau_c)` | Compute Coriolis torque |
| `GetMassMatrixDiagonal(q, diag)` | Get diagonal elements of joint-space inertia matrix |
| `GetJacobian(q, J)` | Compute Jacobian matrix |
| `GetNullSpace(q, N)` | Compute null-space projection matrix |
| `GetEECordinate(q, R, p)` | Forward kinematics, get end-effector pose |

---

### Dependencies

- ROS 2 Humble
- `orocos_kdl`
- `kdl_parser`
- `urdf` / `urdfdom`
- `Eigen3`
- `forward_command_controller` (launched in `openarmx_bringup` or `openarmx_bimanual_moveit_config`)

---

### Installation and Build

```bash
colcon build --packages-select openarmx_gravity_comp openarmx_bringup
source install/setup.bash
```

---

### Usage

The node in this package is managed by bringup or MoveIt demo launch files, controlled by the `enable_forward_effort` parameter. **It is disabled by default.**

#### Method 1: Launch via Bringup

Suitable for teleoperation, teaching, and other scenarios that don't require MoveIt.

**Without gravity compensation (default):**

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 \
    left_can_interface:=can1 \
    control_mode:=mit \
    robot_controller:=forward_position_controller
```

**With gravity compensation enabled:**

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 \
    left_can_interface:=can1 \
    control_mode:=mit \
    robot_controller:=forward_position_controller \
    enable_forward_effort:=true
```

#### Method 2: Launch via MoveIt Demo

Suitable for scenarios using MoveIt for motion planning.

**Without gravity compensation (default):**

```bash
ros2 launch openarmx_bimanual_moveit_config demo.launch.py \
    control_mode:=mit
```

**With gravity compensation enabled:**

```bash
ros2 launch openarmx_bimanual_moveit_config demo.launch.py \
    control_mode:=mit \
    enable_forward_effort:=true
```

#### Verify Normal Operation

```bash
# Check if effort controller is active
ros2 control list_controllers | grep effort

# View real-time feedforward torque output
ros2 topic echo /left_forward_effort_controller/commands
ros2 topic echo /right_forward_effort_controller/commands
```

Normal output example (near zero position):

```
data: [16.1, 15.9, 0.0, 5.0, 0.0, 0.0, 0.0]
```

---

### Key Parameters

#### g_scale (Runtime Adjustable)

KDL calculations are based on URDF inertia parameters, which differ from the actual hardware. `g_scale` is used for overall scaling:

| Value | Effect |
|----|------|
| < 1.0 | Under-compensation, joints still have downward deviation |
| 1.0 | Theoretically complete compensation |
| > 1.0 | Over-compensation, joints shift upward |

**Current optimal value: `1.05`, error < 1°.**

Adjust at runtime without restarting the node:

```bash
ros2 param set /gravity_comp_node g_scale 1.05
ros2 param get /gravity_comp_node g_scale
```

#### Node Parameters Summary

| Parameter | Type | Default | Description |
|------|------|--------|------|
| `urdf_path` | string | Required | URDF file path (automatically written to `/tmp/v10_bimanual_gravity.urdf` by launch file) |
| `g_scale` | double | `1.05` | Overall gravity torque scaling factor |
| `enable_left` | bool | `true` | Enable left arm compensation |
| `enable_right` | bool | `true` | Enable right arm compensation |
| `verbose` | bool | `false` | Print real-time torque for each joint (1 second throttle) |

---

### Residual Error Analysis

| Stage | Error |
|------|------|
| No gravity compensation (pure PD) | ~6.4° (0.111 rad) |
| g_scale = 0.975 | ~3.5° |
| g_scale = 1.05 | < 1° |

Main sources of residual error:

1. **Inaccurate URDF inertia**: Mass and center of mass positions in `inertials.yaml` are design values. `g_scale` can only scale overall, not correct relative errors between joints.
2. **Steady-state error formula**: `Steady-state error = τ_residual / kp`. To reduce error from 2° to 1°, requires `τ_residual < 0.87 Nm`.

Directions for further error reduction:
- Continue fine-tuning `g_scale`
- Increase `kp` (must synchronously increase `kd` to prevent oscillation)
- Calibrate inertia parameters in `inertials.yaml` (most fundamental)

---

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

For more details, see the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Author

- **Li Yongqi** (李永旗)
- Company: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- Website: https://openarmx.com/

## Version

**Current Version**: 1.0.0

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.

| Contact           | Information                                                                                                  |
| ----------------- | ------------------------------------------------------------------------------------------------------------ |
| 📧 Email          | [openarmrobot@gmail.com](mailto:openarmrobot@gmail.com)                                                      |
| 📱 Phone / WeChat | +86-17746530375                                                                                              |
| 🌐 Website        | [https://openarmx.com/](https://openarmx.com/)                                                               |
| 📍 Address        | Huacheng Machinery Plant, No.11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| 👤 Contact Person | Mr. Wang                                                                                                     |
