cmake_minimum_required(VERSION 3.8)
project(load_waypoints)

# Export compile commands for IDEs (VSCode, CLion)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use C++17 (ROS2 standard)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Configure Python for Conda environment (if active)
if(DEFINED ENV{CONDA_PREFIX})
  set(Python3_ROOT_DIR "$ENV{CONDA_PREFIX}")
  set(Python3_FIND_VIRTUALENV FIRST)
  message(STATUS "Using Conda Python from: $ENV{CONDA_PREFIX}")
endif()

# Find required ROS2 packages
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)

############################################################
#                    SERVICE GENERATION                    #
############################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RoverNavigation.srv"
  DEPENDENCIES geometry_msgs
)

# Allow other targets to use generated service code
ament_export_dependencies(rosidl_default_runtime)

# C++ executable (renamed to avoid conflict with package name)
add_executable(ramp_navigate src/ramp_navigate.cpp)

# Link ROS2 & Eigen dependencies
ament_target_dependencies(ramp_navigate
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  std_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_include_directories(ramp_navigate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

install(
  TARGETS ramp_navigate
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/nav_options.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
