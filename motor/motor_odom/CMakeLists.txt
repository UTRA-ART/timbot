cmake_minimum_required(VERSION 3.8)
project(motor_odom)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  src/odom_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()