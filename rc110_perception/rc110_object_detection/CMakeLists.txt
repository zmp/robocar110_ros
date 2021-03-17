cmake_minimum_required(VERSION 3.10)
project(rc110_object_detection VERSION 1.2.0)

find_package(CUDA REQUIRED)

find_package(jetson-utils QUIET)
find_package(jetson-inference QUIET)

if(NOT jetson-utils_FOUND OR NOT jetson-inference_FOUND)
  message(STATUS "This package needs jetson-inference")
  return()
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  vision_msgs
  std_msgs
)

catkin_package()

add_executable(rc110_object_detection
  src/rc110_object_detection_node.cpp
  src/rc110_object_detection.cpp
  src/rc110_image_converter.cpp
)
add_dependencies(rc110_object_detection
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(rc110_object_detection
  PUBLIC
    cxx_std_17
)
target_include_directories(rc110_object_detection
  SYSTEM PUBLIC
    ${CUDA_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(rc110_object_detection
  jetson-inference
  ${catkin_LIBRARIES}
)
