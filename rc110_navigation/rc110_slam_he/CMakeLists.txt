cmake_minimum_required(VERSION 3.10)
project(rc110_slam_he VERSION 1.7.1)

find_package(catkin REQUIRED)
catkin_package()

if(NOT DEFINED ENV{ROS_DISTRO})
    message(FATAL_ERROR "ROS_DISTRO is not defined. Please, source ros.")
endif()
set(ROS_DISTRO $ENV{ROS_DISTRO})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# package
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${PROJECT_SOURCE_DIR}/deb/postinst;")
set(CPACK_DEBIAN_PACKAGE_DEPENDS ros-$ENV{ROS_DISTRO}-rc110-laserscans-to-pointcloud)
include(../../cmake/rc110_cpack.cmake)
