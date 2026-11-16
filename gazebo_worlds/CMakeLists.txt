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

# Patched ign-rendering media files, laid over the system copy at launch
install(DIRECTORY
  rendering
  DESTINATION share/${PROJECT_NAME}/
)

# Publishes the world's scenery as markers (used by the Docker/Foxglove sim)
install(PROGRAMS
  scripts/world_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()