cmake_minimum_required(VERSION 3.16)
project(robomaster_protocol_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(
  robomaster_protocol_test
  ../robomaster_protocol_test.cpp
  ../../io/gimbal/robomaster_protocol.cpp
  ../../tools/crc.cpp
)
target_include_directories(robomaster_protocol_test PRIVATE ../..)

enable_testing()
add_test(NAME robomaster_protocol_test COMMAND robomaster_protocol_test)
