cmake_minimum_required(VERSION 3.16)
project(auto_aim_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

option(AUTO_AIM_ENABLE_TENSORRT "Enable the NVIDIA TensorRT detector backend" ON)
set(AUTO_AIM_TENSORRT_FOUND OFF)
if(AUTO_AIM_ENABLE_TENSORRT)
  find_path(TENSORRT_INCLUDE_DIR
    NAMES NvInfer.h
    HINTS /usr/include /usr/local/include /usr/include/aarch64-linux-gnu
  )
  find_library(TENSORRT_NVINFER_LIBRARY
    NAMES nvinfer
    HINTS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu
  )
  find_path(CUDA_RUNTIME_INCLUDE_DIR
    NAMES cuda_runtime_api.h
    HINTS /usr/local/cuda/include /usr/local/cuda/targets/aarch64-linux/include
  )
  find_library(CUDA_RUNTIME_LIBRARY
    NAMES cudart
    HINTS
      /usr/local/cuda/lib64
      /usr/local/cuda/targets/aarch64-linux/lib
      /usr/lib/aarch64-linux-gnu
  )
  if(
    TENSORRT_INCLUDE_DIR AND TENSORRT_NVINFER_LIBRARY AND
    CUDA_RUNTIME_INCLUDE_DIR AND CUDA_RUNTIME_LIBRARY)
    set(AUTO_AIM_TENSORRT_FOUND ON)
    message(STATUS "TensorRT detector backend enabled")
  else()
    message(WARNING "TensorRT/CUDA runtime not found; TensorRT detector backend disabled")
  endif()
endif()

# The pip OpenVINO package used on the Orin ships its CMake config inside the
# Python package, which is not part of CMAKE_PREFIX_PATH by default.
find_package(OpenVINO QUIET COMPONENTS Runtime)
if(NOT OpenVINO_FOUND)
  execute_process(
    COMMAND python3 -c
      "import pathlib, openvino; print(pathlib.Path(openvino.__file__).resolve().parent / 'cmake')"
    OUTPUT_VARIABLE OPENVINO_PYTHON_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE OPENVINO_PYTHON_RESULT
  )
  if(OPENVINO_PYTHON_RESULT EQUAL 0 AND EXISTS "${OPENVINO_PYTHON_CMAKE_DIR}")
    list(APPEND CMAKE_PREFIX_PATH "${OPENVINO_PYTHON_CMAKE_DIR}")
  endif()
  find_package(OpenVINO REQUIRED COMPONENTS Runtime)
endif()

set(SP_VISION_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../sp_vision_25")
if(NOT EXISTS "${SP_VISION_ROOT}/tasks/auto_aim/yolo.cpp")
  message(FATAL_ERROR "sp_vision_25 was not found at ${SP_VISION_ROOT}")
endif()

# Build only the detector portion of sp_vision_25.  Camera and serial I/O stay
# in their ROS nodes, so this target deliberately does not link the upstream io
# library or open either hardware device.
add_library(sp_vision_detector STATIC
  ${SP_VISION_ROOT}/tasks/auto_aim/armor.cpp
  ${SP_VISION_ROOT}/tasks/auto_aim/classifier.cpp
  ${SP_VISION_ROOT}/tasks/auto_aim/detector.cpp
  ${SP_VISION_ROOT}/tasks/auto_aim/aimer.cpp
  ${SP_VISION_ROOT}/tasks/auto_aim/solver.cpp
  ${SP_VISION_ROOT}/tasks/auto_aim/target.cpp
  ${SP_VISION_ROOT}/tasks/auto_aim/tracker.cpp
  ${SP_VISION_ROOT}/tasks/auto_aim/yolo.cpp
  ${SP_VISION_ROOT}/tasks/auto_aim/yolos/yolo11.cpp
  ${SP_VISION_ROOT}/tasks/auto_aim/yolos/yolov5.cpp
  ${SP_VISION_ROOT}/tasks/auto_aim/yolos/yolov8.cpp
  ${SP_VISION_ROOT}/tools/img_tools.cpp
  ${SP_VISION_ROOT}/tools/extended_kalman_filter.cpp
  ${SP_VISION_ROOT}/tools/logger.cpp
  ${SP_VISION_ROOT}/tools/math_tools.cpp
  ${SP_VISION_ROOT}/tools/trajectory.cpp
)

if(AUTO_AIM_TENSORRT_FOUND)
  target_sources(sp_vision_detector PRIVATE
    ${SP_VISION_ROOT}/tasks/auto_aim/yolos/yolov5_trt.cpp
  )
  target_compile_definitions(sp_vision_detector PUBLIC SP_VISION_HAS_TENSORRT=1)
  target_include_directories(sp_vision_detector PUBLIC
    ${TENSORRT_INCLUDE_DIR}
    ${CUDA_RUNTIME_INCLUDE_DIR}
  )
  target_link_libraries(sp_vision_detector PUBLIC
    ${TENSORRT_NVINFER_LIBRARY}
    ${CUDA_RUNTIME_LIBRARY}
  )
endif()

target_include_directories(sp_vision_detector PUBLIC
  ${SP_VISION_ROOT}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(sp_vision_detector PUBLIC
  ${OpenCV_LIBS}
  Eigen3::Eigen
  fmt::fmt
  OpenMP::OpenMP_CXX
  openvino::runtime
  spdlog::spdlog
  yaml-cpp
)

add_executable(auto_aim_node src/auto_aim_node.cpp)
target_include_directories(auto_aim_node PRIVATE include)
target_link_libraries(auto_aim_node sp_vision_detector)
ament_target_dependencies(auto_aim_node
  auto_aim_interfaces
  cv_bridge
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS auto_aim_node
  DESTINATION lib/${PROJECT_NAME}
)

# Git on the Windows development machine cannot preserve the Linux executable
# bit.  Make it explicit so --symlink-install also produces a runnable node.
file(CHMOD ${CMAKE_CURRENT_SOURCE_DIR}/scripts/video_replay_node.py
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

file(CHMOD ${CMAKE_CURRENT_SOURCE_DIR}/scripts/extrinsic_capture_node.py
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

install(PROGRAMS
  scripts/video_replay_node.py
  scripts/extrinsic_capture_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(fire_controller_test test/fire_controller_test.cpp)
  target_include_directories(fire_controller_test PRIVATE include)
  add_test(NAME fire_controller_test COMMAND fire_controller_test)
endif()

ament_package()
