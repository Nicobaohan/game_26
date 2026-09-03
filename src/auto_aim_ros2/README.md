# auto_aim_ros2

This package wraps the `sp_vision_25` armor detector in ROS 2 without opening
the camera or serial device itself.

## Topics

- Subscribes: `/image_raw` (`sensor_msgs/msg/Image`)
- Subscribes: `/Vision_data` (`auto_aim_interfaces/msg/Vision`)
- Publishes: `/auto_aim/armors` (`auto_aim_interfaces/msg/Armors`)
- Publishes: `/auto_aim/detection_count` (`std_msgs/msg/Int32`)
- Publishes: `/auto_aim/debug_image` (`sensor_msgs/msg/Image`)
- Publishes: `/auto_aim/control_preview` (`auto_aim_interfaces/msg/RobotCtrl`)
- Each armor detection includes an optional camera-frame PnP position,
  distance, yaw, pitch, and reprojection error. PnP becomes valid after the
  node receives a matching `/camera_info` message.
- Reuses the upstream `Solver`, `Tracker`, and `Aimer` for world-frame target
  tracking, motion prediction, and ballistic compensation.
- `/auto_aim/control_preview` is always available for no-motion validation.
  `/Robot_ctrl_data` and firing are disabled by default. Firing additionally
  requires the explicit `enable_fire` gate.

## Offline demo

```bash
ros2 launch auto_aim_ros2 offline_demo.launch.py
```

## Live camera detection

```bash
ros2 launch auto_aim_ros2 live_detection.launch.py
```

For the temporary hero debug-vehicle calibration:

```bash
ros2 launch auto_aim_ros2 live_detection.launch.py \
  camera_info_url:=package://hik_camera/config/camera_info_hero_debug.yaml
```

After checking the preview topic and clearing the test area, gimbal output can
be enabled explicitly. The node still requires fresh `/Vision_data`, a valid
IMU quaternion, MCU auto-aim mode, and a stable tracked target:

```bash
ros2 launch auto_aim_ros2 live_detection.launch.py \
  camera_info_url:=package://hik_camera/config/camera_info_hero_debug.yaml \
  enable_control_output:=true
```

Keep `enable_control_output:=false` while testing detection/PnP/tracking or
when the serial bridge is connected unexpectedly. `enable_control_output`
alone never enables firing.

When `enable_fire:=true`, each shot is a single command pulse. A burst is
allowed only after the target remains inside the tighter fire-angle window,
the gimbal velocity is low, PnP reprojection is acceptable, vision data is
fresh, and MCU mode 33 is active. Losing any gate cancels the rest of the
burst immediately. Use `bash scripts/aim fire` rather than enabling the launch
arguments manually; it requires two explicit confirmations.

The gimbal command applies a low-pass filter and yaw/pitch deadbands before it
is sent. The defaults are intended to stop a stationary target from producing
small continuous corrections. They can be tuned at launch time:

```bash
ros2 launch auto_aim_ros2 live_detection.launch.py \
  control_target_filter_alpha:=0.25 \
  control_yaw_deadband_deg:=0.25 \
  control_pitch_deadband_deg:=0.20
```

A smaller filter alpha is smoother but follows motion more slowly. Increase a
deadband slightly if the corresponding axis still chatters after the camera and
target are fixed.

The live camera publishes `/image_raw` with reliable QoS so both the auto-aim
subscriber and default ROS 2 GUI viewers can connect. To inspect the processed
image, select `/auto_aim/debug_image` in `rqt_image_view`.

## One-command pipeline

The combined launch starts the camera and auto-aim nodes. Serial nodes remain
off until explicitly requested:

```bash
ros2 launch auto_aim_ros2 full_pipeline.launch.py \
  project_root:=/home/hero/game_26_current/sp_vision_25 \
  camera_info_url:=package://hik_camera/config/camera_info_hero_debug.yaml
```

After `/dev/robomaster` exists, add `start_serial:=true` to receive MCU state.
Keep `enable_control_output:=false` until the preview is correct and the test
area is clear. Only then add `enable_control_output:=true` for a no-fire gimbal
test.

All launch files default to `ROS_LOCALHOST_ONLY=1`, preventing identically
named topics from other robots on the LAN from entering this control chain.
Use `local_only:=0` only when multi-computer ROS communication is intentional
and every robot has an isolated ROS domain or namespace.

Do not run the standalone `sp_vision_25` camera program at the same time as
the ROS camera node.

## Short operator commands

The repository-level `scripts/aim` wrapper uses the active calibration in
`hik_camera/config/camera_info_vehicle.yaml`:

```bash
bash scripts/aim build
bash scripts/aim calibrate
bash scripts/aim status
bash scripts/aim safe
bash scripts/aim on
bash scripts/aim fire
```

Run `calibrate` from a NoMachine terminal because it opens a GUI. `on` asks
for one confirmation and keeps `fire_command` at zero; `fire` asks separately
for `AIM` and `FIRE` before it arms the launcher.
