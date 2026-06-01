cmake_minimum_required(VERSION 3.8)
project(octomap_2d_server)

# Default to C++17
if(CMAKE_CXX_STANDARD LESS 17)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)

install(PROGRAMS
  src/pointcloud_merge_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
