cmake_minimum_required(VERSION 3.8)
project(teleop_twist_keyboard)

find_package(ament_cmake REQUIRED)

# Install Python executables
install(PROGRAMS
  src/teleop_twist_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()