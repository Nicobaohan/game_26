# 车载小电脑部署清单（x64 / Ubuntu 22.04）

## 1. 目标平台

- 架构：x86_64 / AMD64
- 操作系统：Ubuntu 22.04 LTS
- ROS 2：Humble
- 目标用途：在车载小电脑上部署 `auto_aim_ros2`、`serical_device_ros2`、相机节点和视觉链路

## 2. 适用部署方式

本平台不建议做交叉编译，推荐直接在车载机上编译：

- 方案 A：直接在目标设备上部署源码并运行 `colcon build`（最稳妥）
- 方案 B：在开发机编译后拷贝产物到目标机（不推荐，除非目标机不能联网）
- 方案 C：用 Docker 容器运行（适合复现环境，适合固定依赖版本）

当前目标环境为 `x64 + Ubuntu22.04`，因此直接部署在本机最适合。

## 3. 目标机必须安装的软件

### 3.1 基础开发工具

```bash
sudo apt update
sudo apt install -y build-essential cmake git curl wget pkg-config \
  python3 python3-pip python3-venv python3-dev \
  python3-colcon-common-extensions
```

### 3.2 ROS 2 Humble

如果目标机还没有安装 ROS 2 Humble，先安装：

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y ros-humble-desktop
```

如果系统中已装好 ROS 2，则跳过。

### 3.3 视觉/推理/数学依赖

```bash
sudo apt install -y libopencv-dev libeigen3-dev libyaml-cpp-dev \
  nlohmann-json3-dev libceres-dev libgflags-dev libgoogle-glog-dev \
  libboost-all-dev libssl-dev
```

### 3.4 OpenVINO 依赖

如果推理部分依赖 OpenVINO，需要确认 OpenVINO 已安装在系统中，并设置环境变量：

```bash
find / -name 'OpenVINOConfig.cmake' 2>/dev/null | head
find / -name 'libopenvino.so*' 2>/dev/null | head -n 20
```

若找到 OpenVINO 的 CMake 配置和库文件，可在每次编译时设置：

```bash
export OpenVINO_DIR=/path/to/openvino/cmake
export LD_LIBRARY_PATH=/path/to/openvino/libs:$LD_LIBRARY_PATH
```

也可以把该路径写入 shell 配置文件：

```bash
echo 'export OpenVINO_DIR=/path/to/openvino/cmake' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/path/to/openvino/libs:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

## 4. 工作区准备

```bash
mkdir -p ~/game26_ws/src
cd ~/game26_ws
```

把当前仓库同步到车载小电脑的 `~/game26_ws/src` 下，例如：

```bash
# 方式一：git clone
cd ~/game26_ws/src
git clone https://github.com/<your-user>/<repo>.git

# 方式二：若已在开发机准备好源码，直接拷贝
# rsync -avz /path/to/local/repo/ ~/game26_ws/src/
```

确保目录结构类似：

```text
~/game26_ws/
  src/
    auto_aim_ros2/
    auto_aim_interfaces/
    serical_device_ros2/
    ros2-hik-camera/
    sp_vision_25/
```

## 5. 编译步骤

先加载 ROS 2 环境：

```bash
source /opt/ros/humble/setup.bash
cd ~/game26_ws
```

如果需要加载其他工作区（例如相机 SDK 或中间工作区）：

```bash
source /home/guanjiu/ros2_hik_ws/install/setup.bash
```

开始编译：

```bash
colcon build --packages-up-to auto_aim_ros2
```

如果项目整体需要编译：

```bash
colcon build
```

编译完成后加载环境：

```bash
source ~/game26_ws/install/setup.bash
```

## 6. 常见编译问题处理

### 6.1 找不到 OpenVINO

报错类似：

```text
Could not find OpenVINOConfig.cmake
```

解决：

```bash
export OpenVINO_DIR=/home/<user>/.local/lib/python3.10/site-packages/openvino/cmake
```

### 6.2 找不到 nlohmann_json

报错类似：

```text
Could not find package configuration file provided by 'nlohmann_json'
```

解决：

```bash
sudo apt install -y nlohmann-json3-dev
```

### 6.3 找不到 Ceres

报错类似：

```text
Could not find package 'Ceres'
```

解决：

```bash
sudo apt install -y libceres-dev
```

### 6.4 串口协议头文件找不到

确保项目中仅保留一份规范头文件：

```text
/home/guanjiu/桌面/game_26/protocol_new.hpp
```

并且各个 CMake / include 统一引用此文件，不要出现多份副本分散在多个目录中。

## 7. 运行顺序

### 7.1 先启动相机节点

```bash
source /opt/ros/humble/setup.bash
source /home/guanjiu/ros2_hik_ws/install/setup.bash
source ~/game26_ws/install/setup.bash

ros2 run hik_camera hik_camera_node --ros-args -p camera_info_url:=file:///home/guanjiu/download/calibrationdata/ost.yaml
```

### 7.2 再启动视觉节点

```bash
ros2 launch auto_aim_ros2 offline_demo.launch.py \
  project_root:=/home/guanjiu/桌面/game_26/sp_vision_25 \
  video_path:=/home/guanjiu/桌面/game_26/sp_vision_25/assets/demo/demo.avi \
  config_path:=configs/demo.yaml \
  image_topic:=/image_raw
```

### 7.3 检查 topic

```bash
ros2 topic list
ros2 topic echo /auto_aim/detection_count -n 5
ros2 topic echo /auto_aim/armors -n 1
```

### 7.4 串口桥接节点

如果目标机需要接串口 / 下位机：

```bash
ros2 run serical_device_ros2 robot_ctrl_main
```

或运行对应 launch 文件，按实际包名调整。

## 8. 运行验证清单

部署完成后，建议按以下顺序确认：

- [ ] ROS2 环境加载正常
- [ ] `colcon build` 成功完成
- [ ] `ros2 topic list` 能看到相关 topic
- [ ] 相机节点启动正常
- [ ] 图像回流正常
- [ ] 视觉检测有输出（`detection_count` / `armors`）
- [ ] 串口节点能正常打开设备
- [ ] 协议头文件能被编译器找到
- [ ] 下位机控制命令和视觉消息链路可通信

## 9. 生产部署建议

- 在车载小电脑上固定使用同一份 ROS 2 环境，避免混用多个工作区
- 把 `OpenVINO` 路径、`LD_LIBRARY_PATH` 固定到 `~/.bashrc` 中
- 保持唯一 `protocol_new.hpp` 规范来源，避免多个副本导致协议不一致
- 先上线离线 demo 和 mock 视觉链路，再接真实相机和下位机
- 尽量在目标机上保留一个可重现的 `build.sh` / `run.sh` 脚本，便于后续重装

## 10. 建议的快速启动脚本

在车载机上可以准备一个简单脚本：

```bash
#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /home/guanjiu/ros2_hik_ws/install/setup.bash
source ~/game26_ws/install/setup.bash

export OpenVINO_DIR=/path/to/openvino/cmake
export LD_LIBRARY_PATH=/path/to/openvino/libs:$LD_LIBRARY_PATH

ros2 launch auto_aim_ros2 offline_demo.launch.py \
  project_root:=/home/guanjiu/桌面/game_26/sp_vision_25 \
  video_path:=/home/guanjiu/桌面/game_26/sp_vision_25/assets/demo/demo.avi \
  config_path:=configs/demo.yaml \
  image_topic:=/image_raw
```

## 11. 结论

当前 `x64 + Ubuntu 22.04` 平台最适合直接部署，不需要交叉编译。只要按本清单安装依赖、同步代码、配置 ROS2 环境和 OpenVINO 路径，通常可以在车载小电脑上直接完成编译和运行。
