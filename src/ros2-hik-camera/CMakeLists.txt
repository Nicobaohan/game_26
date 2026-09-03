cmake_minimum_required(VERSION 3.8)
project(hik_camera)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/hik_camera_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC hikSDK/include)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/amd64)
  install(
    DIRECTORY hikSDK/lib/amd64/
    DESTINATION lib
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  # Jetson/Orin installations normally place the ARM64 MVS runtime here.
  # The repository does not ship these large proprietary shared libraries.
  set(HIK_MVS_ARM64_LIB_DIR "/opt/MVS/lib/aarch64")
  if(EXISTS "${HIK_MVS_ARM64_LIB_DIR}/libMvCameraControl.so")
    target_link_directories(${PROJECT_NAME} PUBLIC ${HIK_MVS_ARM64_LIB_DIR})
    set_target_properties(${PROJECT_NAME} PROPERTIES
      BUILD_RPATH "${HIK_MVS_ARM64_LIB_DIR}"
      INSTALL_RPATH "${HIK_MVS_ARM64_LIB_DIR}"
    )
  elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/arm64/libMvCameraControl.so")
    target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/arm64)
    install(
      DIRECTORY hikSDK/lib/arm64/
      DESTINATION lib
    )
  else()
    message(FATAL_ERROR
      "Hikrobot ARM64 SDK libraries were not found in /opt/MVS/lib/aarch64 "
      "or hikSDK/lib/arm64")
  endif()
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(${PROJECT_NAME}
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN hik_camera::HikCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
