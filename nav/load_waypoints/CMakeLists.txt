cmake_minimum_required(VERSION 3.8)
project(load_waypoints)

# Export compile commands for IDEs (VSCode, CLion)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use C++17 (ROS2 standard)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)

# Your executable
add_executable(load_waypoints src/ramp_navigate.cpp)

# Link ROS2 & Eigen dependencies
ament_target_dependencies(load_waypoints
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  std_msgs
  nav2_msgs
)

target_include_directories(load_waypoints PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS load_waypoints
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
