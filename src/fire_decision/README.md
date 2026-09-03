# Fire Decision

这个模块负责把“视觉瞄准是否允许开火”从简单的角度门控升级为更接近真实 RoboMaster 场景的发射决策。

## 设计目标

- 目标必须可见、有效且稳定，才允许开火。
- 传感器数据必须足够新鲜，不能因延迟、丢包或异步导致错误开火。
- 目标在未来射击窗口中仍然有效，不能只看当前帧的一瞬间。
- 发射必须遵守冷却时间和机械安全边界。
- 所有关键阈值都必须通过真实机器人场景测量后写入 YAML 配置。

## 真实入口

当前项目的实际发射门控入口在：

- `sp_vision_25/tasks/auto_aim/shooter.cpp`
- `sp_vision_25/tasks/auto_aim/shooter.hpp`

最终命令通过：

- `src/serical_device_ros2/robot_ctrl_node.cpp`
- `src/auto_aim_interfaces/msg/RobotCtrl.msg`

传入串口 / 底层控制链路，最终由 `target_lock` 与 `fire_command` 控制发射。

## 必须实测的参数

以下参数不能只靠猜，必须在真实机器人上测量：

- `first_tolerance`：近距离容差（deg）
- `second_tolerance`：远距离容差（deg）
- `judge_distance`：近远距离判定阈值（m）
- `yaw_offset` / `pitch_offset`：云台机械零偏
- `bullet_speed`：真实弹速
- `high_speed_delay_time` / `low_speed_delay_time`：目标速度和系统延迟补偿
- `min_detect_count` / `max_temp_lost_count`：目标稳定性门槛
- `fire_cooldown`：发射最小间隔
- `max_sensor_stale_time`：数据陈旧判定阈值
- `max_pnp_residual`：PnP 误差阈值
- `max_track_uncertainty`：目标状态不确定度阈值

## 决策顺序

1. 目标有效性检查
   - 目标不为空
   - 视觉 aim point 有效
   - 距离合理
   - 目标未 diverged

2. 稳定性检查
   - 目标角度误差在允许窗口中
   - 连续有效帧数达到门槛
   - 目标不出现明显噪声/抖动

3. 数据新鲜度检查
   - 相机、IMU、状态数据不能超过 stale 时间
   - 若任何信息源过期，进入 hold-fire

4. 冷却与节流检查
   - 发射间隔大于 `fire_cooldown`
   - 不允许在目标刚切换/重锁定时立即发射

5. 发射出口
   - 满足条件后输出 `target_lock` 与 `fire_command`
   - 否则维持锁定但不发射

## 要求

- 所有阈值必须通过车辆配置或 ROS 参数管理，而不是硬编码在判定逻辑中。
- 在真实机器人上通过样本打靶、遮挡、重锁定等测试回归后再微调参数。
- 若某一项数据可疑（延迟、丢包、模糊、畸变、云台超限），优先保守，不开火。

## 相关文件

- `sp_vision_25/tasks/auto_aim/shooter.cpp`
- `sp_vision_25/tasks/auto_aim/tracker.cpp`
- `sp_vision_25/tasks/auto_aim/aimer.cpp`
- `sp_vision_25/configs/example.yaml`
- `src/serical_device_ros2/robot_ctrl_node.cpp`
