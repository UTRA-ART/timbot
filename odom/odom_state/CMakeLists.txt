cmake_minimum_required(VERSION 3.8)
project(odom_state)

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
find_package(robot_localization REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

###########
## Build ##
###########

# Uncomment to build C++ libraries
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/odom_state.cpp
# )
# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   robot_localization
#   std_msgs
# )

# Uncomment to build C++ executables
# add_executable(${PROJECT_NAME}_node src/odom_state_node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp
#   robot_localization
#   std_msgs
# )

# Uncomment to link libraries
# target_link_libraries(${PROJECT_NAME}_node
#   ${PROJECT_NAME}
# )

#############
## Install ##
#############

# Install debugging scripts
install(PROGRAMS
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