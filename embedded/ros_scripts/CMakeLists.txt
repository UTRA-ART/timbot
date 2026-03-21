cmake_minimum_required(VERSION 3.8)
project(ros_scripts)

find_package(ament_cmake REQUIRED)

# Install Python executables
install(PROGRAMS
  src/read_odom_arduino.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
