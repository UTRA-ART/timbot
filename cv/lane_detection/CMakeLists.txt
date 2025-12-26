cmake_minimum_required(VERSION 3.14.4)
project(lane_detection)

# Default warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Configure Python for Conda environment (if active)
if(DEFINED ENV{CONDA_PREFIX})
  set(Python3_ROOT_DIR "$ENV{CONDA_PREFIX}")
  set(Python3_FIND_VIRTUALENV FIRST)
  message(STATUS "Using Conda Python from: $ENV{CONDA_PREFIX}")
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

############################################################
#                    MESSAGE GENERATION                    #
############################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FloatList.msg"
  "msg/FloatArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# Allow other targets to use generated message code
ament_export_dependencies(rosidl_default_runtime)

############################################################
#                         EXECUTABLE                       #
############################################################

add_executable(lane_scan src/scan.cpp)

# Add generated include path for messages
target_include_directories(lane_scan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lane_scan
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  rosidl_default_runtime
)

# Link typesupport so messages are usable
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(lane_scan ${cpp_typesupport_target})

############################################################
#                        INSTALLATION                      #
############################################################

install(TARGETS
  lane_scan
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
