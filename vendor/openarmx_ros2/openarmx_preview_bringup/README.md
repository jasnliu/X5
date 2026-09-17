# openarmx_preview_bringup

## Package Overview

`openarmx_preview_bringup` is an independent wrapper bringup package used to launch our robot joint control plugin for easier user operation.

When this package starts, it always launches RViz2 and preconfigures the following displays:

- `openarmx_joint_slider_panel/JointSliderPanel`
- `RobotModel` display from `/robot_description`

This package does **not** modify any existing bringup package, and can be integrated directly into existing systems for debugging, demos, and visualization validation.

## Simulation Mode (OpenArmX)

```bash
source install/setup.bash
ros2 launch openarmx_preview_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=true
```

## Real Robot Mode (OpenArm)

```bash
source install/setup.bash
ros2 launch openarmx_preview_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=false
```

The following alternative launch file names are also supported:

- `openarmx.preview.bimanual.launch.py`
- `openarm.preview.bimanual.launch.py`

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.

For details, see [LICENSE_CN.md](LICENSE_CN.md) or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Author

- **Li QingRan**
- Company: Chengdu Changshu Robot Co., Ltd.
- Website: https://openarmx.com/

## Version

**Current Version**: 1.0.0

---

## Contact Us

### Chengdu Changshu Robotics Co., Ltd.

| Contact           | Information                                                                                                  |
| ----------------- | ------------------------------------------------------------------------------------------------------------ |
| 📧 Email          | [openarmrobot@gmail.com](mailto:openarmrobot@gmail.com)                                                      |
| 📱 Phone / WeChat | +86-17746530375                                                                                              |
| 🌐 Website        | [https://openarmx.com/](https://openarmx.com/)                                                               |
| 📍 Address        | Huacheng Machinery Plant, No.11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| 👤 Contact Person | Mr. Wang                                                                                                     |
