cmake_minimum_required(VERSION 3.8)
project(nav_stack)

# Default to C++17
if(CMAKE_CXX_STANDARD LESS 17)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install debugging/testing scripts
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
