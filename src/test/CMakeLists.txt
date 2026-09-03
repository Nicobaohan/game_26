cmake_minimum_required(VERSION 3.8)
project(gimbal_delay_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Serial classes from serical_device_ros2 (pure C++, no ROS dependency):
#   serial_main.cpp / serial_device.cpp / crc.cpp
# protocol_new.hpp lives at the repo root (game_26_current/).
set(SERIAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../serical_device_ros2)
set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_executable(gimbal_delay_test
  gimbal_delay_test.cpp
  ${SERIAL_DIR}/src/serial_main.cpp
  ${SERIAL_DIR}/src/serial_device.cpp
  ${SERIAL_DIR}/src/crc.cpp)

target_include_directories(gimbal_delay_test PRIVATE
  ${SERIAL_DIR}/include
  ${REPO_ROOT})

find_package(Threads REQUIRED)
target_link_libraries(gimbal_delay_test PRIVATE Threads::Threads)
