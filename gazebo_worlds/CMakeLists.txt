cmake_minimum_required(VERSION 3.8)
project(gazebo_worlds)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files and assets
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()