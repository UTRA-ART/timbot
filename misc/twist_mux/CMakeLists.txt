cmake_minimum_required(VERSION 3.8)
project(timbot_twist_mux)

# Default to C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

# Include directories
include_directories(include)

# twist_mux executable
add_executable(twist_mux
  src/twist_mux.cpp
  src/twist_mux_node.cpp
  src/twist_mux_diagnostics.cpp
  src/topic_handle.cpp)

ament_target_dependencies(twist_mux
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  diagnostic_updater)

# twist_marker executable
add_executable(twist_marker
  src/twist_marker.cpp)

ament_target_dependencies(twist_marker
  rclcpp
  geometry_msgs
  visualization_msgs)

# Install executables
install(TARGETS
  twist_mux
  twist_marker
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/joystick_relay.py
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py")

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml")

# Install header files
install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

