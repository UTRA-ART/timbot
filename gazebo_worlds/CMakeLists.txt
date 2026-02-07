cmake_minimum_required(VERSION 3.8)
project(gazebo_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  pavement/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()