cmake_minimum_required(VERSION 3.10)
project(rc110_object_detection VERSION 1.2.0)

find_package(CUDA REQUIRED)

find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  vision_msgs
  rc110_msgs
)

catkin_package()

add_executable(rc110_object_detection_node
  src/rc110_object_detection_node.cpp
  src/rc110_object_detection.cpp
  src/rc110_image_converter.cpp
)
add_dependencies(rc110_object_detection_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(rc110_object_detection_node
  PUBLIC
    cxx_std_17
)
target_include_directories(rc110_object_detection_node
  SYSTEM PUBLIC
    ${CUDA_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(rc110_object_detection_node
  jetson-inference
  ${catkin_LIBRARIES}
)

install(TARGETS rc110_object_detection_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# package
include(../../cmake/rc110_cpack.cmake)