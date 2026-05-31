cmake_minimum_required(VERSION 3.8)
project(simple_nav)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Configure Python for Conda environment (if active)
if(DEFINED ENV{CONDA_PREFIX})
  set(Python3_ROOT_DIR "$ENV{CONDA_PREFIX}")
  set(Python3_FIND_VIRTUALENV FIRST)
  message(STATUS "Using Conda Python from: $ENV{CONDA_PREFIX}")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

install(PROGRAMS
  src/simple_nav_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
