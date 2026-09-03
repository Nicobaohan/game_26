# Data received from the RoboMaster controller.
std_msgs/Header header

uint16 AUTO_AIM_MODE=33

uint16 id
uint16 mode
# MCU feedback Euler angles are in degrees and angular rates are in deg/s.
# Quaternion order is w, x, y, z.
float32 yaw
float32 yaw_vel
float32 pitch
float32 pitch_vel
float32 roll
float32[4] quaternion
float32 shoot_speed
uint16 bullet_count
uint8 game_progress
