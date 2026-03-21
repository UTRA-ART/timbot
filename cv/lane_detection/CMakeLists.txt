cmake_minimum_required(VERSION 3.14.4)
project(lane_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Configure Python for Conda environment (if active)
if(DEFINED ENV{CONDA_PREFIX})
  set(Python3_ROOT_DIR "$ENV{CONDA_PREFIX}")
  set(Python3_FIND_VIRTUALENV FIRST)
  message(STATUS "Using Conda Python from: $ENV{CONDA_PREFIX}")
endif()

find_package(ament_cmake REQUIRED)

############################################################
#                        INSTALLATION                      #
############################################################

# Install Python executable scripts
install(PROGRAMS
  src/lane_detection_inference.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lane_detection_inference
)

# Install Python modules used by the inference node
install(FILES
  src/classical_lane_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
