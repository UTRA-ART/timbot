cmake_minimum_required(VERSION 3.8)
project(zed_custom)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

###########
## Build ##
###########

# Uncomment to build C++ libraries
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/zed_custom.cpp
# )
# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   sensor_msgs
#   cv_bridge
# )

# Uncomment to build C++ executables
# add_executable(${PROJECT_NAME}_node src/zed_custom_node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp
#   sensor_msgs
#   cv_bridge
# )

# Uncomment to link libraries
# target_link_libraries(${PROJECT_NAME}_node
#   ${PROJECT_NAME}
# )

#############
## Install ##
#############

# Install Python scripts
install(PROGRAMS
  src/zed_camera_info_publisher.py
  src/disparity_to_depth.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Uncomment to install C++ executables
# install(TARGETS ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Uncomment to install C++ libraries
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# Uncomment to install header files
# install(DIRECTORY include/
#   DESTINATION include
# )

ament_package()