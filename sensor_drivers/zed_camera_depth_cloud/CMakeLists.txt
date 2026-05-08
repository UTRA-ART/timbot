cmake_minimum_required(VERSION 3.8)
project(zed_camera_depth_cloud)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/depth_to_pointcloud.py
  src/pointcloud_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
