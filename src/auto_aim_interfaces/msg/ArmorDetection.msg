uint8 COLOR_RED=0
uint8 COLOR_BLUE=1
uint8 COLOR_EXTINGUISH=2
uint8 COLOR_PURPLE=3

uint8 TYPE_BIG=0
uint8 TYPE_SMALL=1

uint8 color
uint8 number
uint8 armor_type
float32 confidence
geometry_msgs/Point32 center
geometry_msgs/Point32[4] corners

# Debug PnP result in the OpenCV optical camera frame:
# x points right, y points down, z points forward.
bool pose_valid
geometry_msgs/Point position_camera
float32 distance_m
float32 yaw_rad
float32 pitch_rad
float32 reprojection_error_px
