# openarmx_preview_bringup

## 包介绍

`openarmx_preview_bringup` 是一个独立的封装启动（wrapper bringup）包，用于启动我们的机器人关节控制插件，方便用户使用。

该包启动时会始终同时拉起 RViz2，并预配置以下显示项：

- `openarmx_joint_slider_panel/JointSliderPanel`
- 来自 `/robot_description` 的 `RobotModel` 显示

该包不会修改任何现有 bringup 包，适合直接接入现有系统用于联调、演示和可视化验证。

## 仿真模式（OpenArmX）

```bash
source install/setup.bash
ros2 launch openarmx_preview_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=true
```

## 实机模式（OpenArm）

```bash
source install/setup.bash
ros2 launch openarmx_preview_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=false
```

同时也支持以下备用 launch 文件名：

- `openarmx.preview.bimanual.launch.py`
- `openarm.preview.bimanual.launch.py`



## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE_CN.md](LICENSE_CN.md) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 作者

- **Li QingRan** (李青燃)
- 公司: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 网站: https://openarmx.com/

## 版本

**当前版本**：1.0.0

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | <https://openarmx.com/> |
| 📍 地址 | 天津经济技术开发区西区新业八街11号华诚机械厂 |
| 👤 联系人 | 王先生 |
