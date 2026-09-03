# game_26 ROS 2 自动瞄准 / 视觉桥接项目说明

## 项目概览

本仓库是一个基于 ROS 2 的自动瞄准视觉项目，包含以下几个核心部分：

- `src/auto_aim_ros2`：ROS 2 节点，负责订阅图像和视觉消息，调用 `sp_vision_25` 视觉逻辑，输出 `/auto_aim/armors`、`/auto_aim/detection_count` 等 topic。
- `src/auto_aim_interfaces`：ROS 2 接口定义，包含 `Vision.msg` 和 `RobotCtrl.msg` 等消息。
- `src/serical_device_ros2`：串口/下位机桥接节点，把底层串口协议转为 ROS 消息。
- `sp_vision_25`：底层视觉检测逻辑，完成装甲板检测、分类、目标提取等工作。
- `src/ros2-hik-camera`：海康相机 ROS 2 接口包，依赖海康 SDK。

---

## 本次处理的核心工作

### 1. 确认运行环境

已确认当前系统为 Ubuntu 22.04，ROS 2 Humble 已安装在：

- `/opt/ros/humble`

同时确认这里是 WSL/Ubuntu 环境，不是直接在 Windows 上运行该项目。

### 2. 安装 ROS 2 相关依赖

在 Ubuntu 环境中安装了必要的构建工具：

- `colcon`
- `rosdep`
- `build-essential`
- `python3-vcstool`

完成了 rosdep 的初始化与更新，确保后续 `colcon build` 能解析依赖。

### 3. 建立 ROS 工作空间

创建了工作空间：

- `~/game26_ws`

并把项目同步到：

- `~/game26_ws/src`

确保 `auto_aim_ros2`、`serical_device_ros2`、`ros2-hik-camera` 等 ROS 包位于工作区中。

### 4. 修复编译问题

在编译过程中，发现并修复了几个关键问题：

#### 4.1 海康 SDK 链接问题

`hik_camera` 包在 Ubuntu 环境下，原本依赖短库名 `-lFormatConversion`、`-lMediaProcess` 等方式链接，导致链接器找不到 SDK 库。

已修正为显式定位实际库目录：

- `hikSDK/lib/amd64` 或 `hikSDK/lib/arm64`

并通过 `find_library()` 绑定到具体 `.so` 文件。

#### 4.2 `sp_vision_25` 路径问题

`auto_aim_ros2` 的 CMake 里原先写死了一个很强绑定的路径，导致在 Ubuntu 环境下无法找到 `sp_vision_25`。

已修正为支持自动搜索多个路径：

- `../sp_vision_25`
- `../../sp_vision_25`
- `/home/guanjiu/桌面/game_26/sp_vision_25`

这样可以在当前工作树里正确找到视觉工程。

### 5. 成功编译

已在 ROS 2 环境中执行：

```bash
source /opt/ros/humble/setup.bash
source /home/guanjiu/ros2_hik_ws/install/setup.bash
cd ~/game26_ws
colcon build --packages-up-to auto_aim_ros2
```

并验证编译成功，输出结果为：

- `auto_aim_interfaces` 成功
- `hik_camera` 成功
- `auto_aim_ros2` 成功

### 6. 验证离线视频模式

项目中自带离线演示功能，关键文件：

- `src/auto_aim_ros2/launch/offline_demo.launch.py`
- `src/auto_aim_ros2/scripts/video_replay_node.py`

离线演示视频：

- `sp_vision_25/assets/demo/demo.avi`

已验证该模式能够正常运行，日志中输出持续 `detections=...`，说明视觉检测逻辑确实在处理图像帧，并且 ROS 节点工作正常。

启动命令示例：

```bash
source /opt/ros/humble/setup.bash
source /home/guanjiu/ros2_hik_ws/install/setup.bash
source /home/guanjiu/game26_ws/install/setup.bash
export LD_LIBRARY_PATH=/home/guanjiu/.local/lib/python3.10/site-packages/openvino/libs:$LD_LIBRARY_PATH

cd ~/game26_ws
ros2 launch auto_aim_ros2 offline_demo.launch.py \
  project_root:=/home/guanjiu/桌面/game_26/sp_vision_25 \
  video_path:=/home/guanjiu/桌面/game_26/sp_vision_25/assets/demo/demo.avi \
  config_path:=configs/demo.yaml \
  image_topic:=/image_raw
```

随后可以检查 topic：

```bash
ros2 topic list
ros2 topic echo /auto_aim/detection_count -n 5
ros2 topic echo /auto_aim/armors -n 1
```

### 7. 检查通讯协议结构

已确认项目中有统一的通讯分层：

#### ROS 消息层

- `src/auto_aim_interfaces/msg/Vision.msg`
- `src/auto_aim_interfaces/msg/RobotCtrl.msg`

两者分别表示：

- 下位机 -> 上位机：视觉状态
- 上位机 -> 下位机：控制命令

#### 串口协议层

- `protocol_new.hpp`（项目根目录中的唯一协议定义）

这里定义了底层结构：

- `FrameHeader`
- `VisionData`
- `RobotCtrlData`
- `TARGET_LOCKED` / `TARGET_UNLOCKED`

#### 桥接节点

- `vision_pub_node.cpp`：从串口读入下位机数据，发布 `/Vision_data`
- `robot_ctrl_node.cpp`：订阅 `/Robot_ctrl_data`，转成 `RobotCtrlData`，发往下位机

---

## 结论

### 已完成的状态

- ROS 2 环境已确认并可用
- 代码已成功编译
- 离线视频 demo 已可运行
- 视觉检测链路已验证正常
- 项目里存在统一的 ROS 消息协议和串口协议层

### 当前的限制

- 真实相机模式依赖真实海康相机设备和正确的 USB/设备访问权限
- 当前环境没有检测到真实海康相机，所以 `live_detection.launch.py` 常见日志为 `No camera found!`
- 当前代码中并没有真正实现完整的“发射决策”，而是属于检测-only 安全模式
- 也就是说：视觉检测可以跑通，但完整开火逻辑还需要进一步补全和验证

---

## 参考命令

### 进入 ROS 环境

```bash
source /opt/ros/humble/setup.bash
source /home/guanjiu/ros2_hik_ws/install/setup.bash
source /home/guanjiu/game26_ws/install/setup.bash
```

### 离线 demo

```bash
ros2 launch auto_aim_ros2 offline_demo.launch.py \
  project_root:=/home/guanjiu/桌面/game_26/sp_vision_25 \
  video_path:=/home/guanjiu/桌面/game_26/sp_vision_25/assets/demo/demo.avi \
  config_path:=configs/demo.yaml \
  image_topic:=/image_raw
```

### 查看 topic

```bash
ros2 topic list
ros2 topic echo /auto_aim/detection_count -n 5
ros2 topic echo /auto_aim/armors -n 1
```

---

## 备注

- 这次处理的重点是把项目从“能编译、能跑离线 demo”的状态推进到“ROS 2 运行链路确认完成”。
- 真实相机和下位机发射控制仍然需要在对应硬件/设备环境中进一步测试。

## 部署文档

- [车载小电脑部署清单（x64 / Ubuntu 22.04）](./DEPLOYMENT_CHECKLIST_x64_Ubuntu22.md)

### 部署脚本

- `scripts/setup_x64_ubuntu22.sh`：安装 x64 + Ubuntu 22.04 所需依赖
- `scripts/build_game26_ws.sh`：编译 ROS2 工作区
- `scripts/run_offline_demo.sh`：启动离线视频 demo

## 当前 Orin 一键流程

当前比赛流程使用以下目录：

- 源码：`~/game_26_current`
- 构建产物：`~/game_26_build/ros2`

Windows 端先在 `orin_target.psd1` 中填写当前车的 SSH 地址和用户目录。
之后可直接使用：

- `sync_to_orin.cmd`：只同步代码并打开 VS Code，不编译。
- `deploy_to_orin.cmd`：同步代码并在 Orin 上增量编译。
- `aim_status.cmd`：检查相机、`/dev/robomaster`、构建和运行状态。
- `aim_safe.cmd`：启动相机、串口和自瞄，但只发布控制预览。
- `aim_on.cmd`：确认安全后开启无发弹云台控制；仍要求下位机模式为 33。
- `aim_fire.cmd`：经两次手动确认后启用云台与脉冲连发。

任一启动命令运行期间，按 `Ctrl+C` 停止全部节点。除
`aim_fire.cmd` 外，`fire_command` 均保持为 0。开火模式还要求目标角度、
云台速度、PnP 重投影误差、模式和数据新鲜度同时通过门控；任一
条件失效会立即取消剩余连发。

### 换车标定

在 NoMachine 的 Orin 终端中执行：

```bash
cd ~/game_26_current
bash scripts/aim calibrate
```

图形界面中采集完成后依次点击 `CALIBRATE`、`SAVE`，再按 `Ctrl+C`。
回到 Windows 双击 `fetch_calibration.cmd`，新内参会保存为
`src/ros2-hik-camera/config/camera_info_vehicle.yaml`。然后再次双击
`deploy_to_orin.cmd` 上传并编译。

标定板参数位于 `config/autoaim_vehicle.env`，默认是 `11x8` 个内角点、
方格边长 `0.020 m`。如果实际标定板不同，必须先修改这两个值。

---
