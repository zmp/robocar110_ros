cmake_minimum_required(VERSION 3.10)
project(rc110_slam_cg VERSION 2.0.0)

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(pluginlib REQUIRED)
ament_package()
if(NOT DEFINED ENV{ROS_DISTRO})
    message(FATAL_ERROR "ROS_DISTRO is not defined. Please, source ros.")
endif()
set(ROS_DISTRO $ENV{ROS_DISTRO})
set(OGRE_PLUGIN_PATH "rviz_ogre_vendor")

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

# package
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${PROJECT_BINARY_DIR}/deb/conffiles;")
include(../../cmake/rc110_cpack.cmake)
