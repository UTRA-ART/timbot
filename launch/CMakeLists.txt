cmake_minimum_required(VERSION 3.8)
project(timbot_launch)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

###########
## Build ##
###########

# zed_open_capture_node is the real-rover ZED camera driver: it talks to the
# camera over raw USB via Linux UVC extension units (videocapture.cpp includes
# <linux/usb/video.h>, calibration.hpp includes <sys/vfs.h>), both Linux-only
# kernel headers with no macOS equivalent. Sim never uses this node (Gazebo's
# camera bridge publishes the same topics instead — see CLAUDE.md §5 sensor_drivers/),
# so it's simply not built on macOS rather than attempting a port.
if(NOT APPLE)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
  find_package(CURL REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

  add_executable(zed_open_capture_node
    src/zed_open_capture_node.cpp
    ../sensor_drivers/zed_open_source/src/videocapture.cpp
  )

  target_include_directories(zed_open_capture_node PRIVATE
    ../sensor_drivers/zed_open_source/include
    ../sensor_drivers/zed_open_source/examples/include
    ${OpenCV_INCLUDE_DIRS}
    ${LIBUSB_INCLUDE_DIRS}
  )

  target_compile_definitions(zed_open_capture_node PRIVATE VIDEO_MOD_AVAILABLE)

  ament_target_dependencies(zed_open_capture_node
    rclcpp
    sensor_msgs
    cv_bridge
  )

  target_link_libraries(zed_open_capture_node
    ${OpenCV_LIBS}
    ${LIBUSB_LIBRARIES}
    CURL::libcurl
    pthread
  )
endif()

#############
## Install ##
#############

# Install C++ executables
if(NOT APPLE)
  install(TARGETS
    zed_open_capture_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()