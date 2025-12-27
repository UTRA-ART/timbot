cmake_minimum_required(VERSION 3.8)
project(motor_control)

find_package(ament_cmake REQUIRED)

# Install Python executables
install(PROGRAMS
  src/simple_motor_echo.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()