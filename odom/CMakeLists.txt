cmake_minimum_required(VERSION 3.8)
project(odom)

find_package(ament_cmake REQUIRED)

# Install launch directory
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install config directory  
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()