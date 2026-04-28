cmake_minimum_required(VERSION 3.8)
project(depth_detection)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/depth_to_pointcloud.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
