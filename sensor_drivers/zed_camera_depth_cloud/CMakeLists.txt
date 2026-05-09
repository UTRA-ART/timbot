cmake_minimum_required(VERSION 3.8)
project(zed_camera_depth_cloud)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/pointcloud_filter_from_rviz.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
