cmake_minimum_required(VERSION 3.8)
project(load_waypoints)

# Export compile commands for IDEs (VSCode, CLion)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

############################################################
#                    SERVICE GENERATION                    #
############################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RoverNavigation.srv"
  DEPENDENCIES geometry_msgs
)

# Allow other targets to use generated service code
ament_export_dependencies(rosidl_default_runtime)

# Install Python scripts as executables
install(PROGRAMS
  src/navigate_waypoints.py
  src/nav_options.py
  src/ramp_navigate.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install JSON waypoint files
install(DIRECTORY
  jsons
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
