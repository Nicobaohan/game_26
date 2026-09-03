cmake_minimum_required(VERSION 3.8)
project(auto_aim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmorDetection.msg"
  "msg/Armors.msg"
  "msg/RobotCtrl.msg"
  "msg/Vision.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
