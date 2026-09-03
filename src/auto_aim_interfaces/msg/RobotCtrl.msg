# Command sent from vision to the RoboMaster controller.
std_msgs/Header header

int8 TARGET_LOCKED=49
int8 TARGET_UNLOCKED=50

# Absolute command angles use radians; rates use rad/s and rad/s^2.
float32 yaw
float32 yaw_vel
float32 yaw_acc
float32 pitch
float32 pitch_vel
float32 pitch_acc
int8 target_lock
int8 fire_command
