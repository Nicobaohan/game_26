cmake_minimum_required(VERSION 3.10)
project(serical_device_ros2)

# 设置 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# clangd 支持
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 安装路径兼容
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 编译警告（可选）
# add_definitions(-Wall -Werror)

#########################################
# ✅ 自动依赖 + 头文件包含
#########################################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
)

# 协议文件的唯一来源在项目根目录，不在 ROS 包目录下。
# 兼容两种情况：
# 1) 包被拷贝到同一工作区根目录下；
# 2) 包位于独立仓库目录中，真实协议头文件在 /home/guanjiu/桌面/game_26/protocol_new.hpp。
set(PROTOCOL_HEADER_SEARCH_PATHS
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../..
  /home/guanjiu/桌面/game_26
)

foreach(_protocol_root IN LISTS PROTOCOL_HEADER_SEARCH_PATHS)
  if(EXISTS "${_protocol_root}/protocol_new.hpp")
    include_directories("${_protocol_root}")
    set(PROTOCOL_HEADER_SOURCE "${_protocol_root}/protocol_new.hpp")
    break()
  endif()
endforeach()

if(NOT DEFINED PROTOCOL_HEADER_SOURCE)
  message(FATAL_ERROR "protocol_new.hpp not found. Please ensure the canonical protocol header exists in the project root.")
endif()
message(STATUS "protocol_new.hpp source: ${PROTOCOL_HEADER_SOURCE}")

#########################################
# ✅ 构建组件库
#########################################
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/serial_main.cpp
  src/serial_device.cpp
  src/crc.cpp
  vision_pub_node.cpp
  robot_ctrl_node.cpp
)

#########################################
# ✅ 注册组件并自动生成执行文件
#########################################
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::VisionPub
  EXECUTABLE vision_pub_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::RobotCtrlSub
  EXECUTABLE robot_ctrl_main
)

#########################################
# ✅ 安装头文件（如果有 include 文件夹）
#########################################
install(DIRECTORY include/
  DESTINATION include/
)

# protocol_new.hpp 位于工作空间根目录，是上下位机协议的唯一来源。
install(FILES "${PROTOCOL_HEADER_SOURCE}"
  DESTINATION include/
)

#########################################
# ✅ 打包
#########################################
ament_auto_package()
