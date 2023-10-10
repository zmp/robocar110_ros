cmake_minimum_required(VERSION 3.10)
project(rc110_msgs VERSION 2.1.0)

find_package(ament_cmake REQUIRED message_package)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB msg_files RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg srv/*.srv)
rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} DEPENDENCIES std_msgs rclcpp)
ament_export_dependencies(std_msgs rclcpp)
ament_package()

# package
include(../../cmake/rc110_cpack.cmake)
