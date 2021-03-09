cmake_minimum_required(VERSION 3.10)
project(rc110_object_detection VERSION 1.2.0)

find_package(jetson-utils QUIET)
find_package(jetson-inference QUIET)

if(NOT jetson-utils_FOUND OR NOT jetson-inference_FOUND)
  message(STATUS "This package needs jetson-inference")
  return()
endif()

find_package(CUDA REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  vision_msgs
  std_msgs
)

catkin_package()
