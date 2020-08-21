cmake_minimum_required(VERSION 3.10)
project(rc110_service VERSION 1.0.0)

find_package(catkin REQUIRED)
catkin_package()

if(NOT DEFINED ENV{ROS_DISTRO})
    message(FATAL_ERROR "ROS_DISTRO is not defined. Please, source ros.")
endif()
set(ROS_DISTRO $ENV{ROS_DISTRO})

# copy configs to BIN and fill @variables@
file(REMOVE_RECURSE ${PROJECT_BINARY_DIR}/systemd)
file(GLOB systemd_files RELATIVE ${PROJECT_SOURCE_DIR} systemd/*)
foreach(file ${systemd_files})
    configure_file(${file} ${PROJECT_BINARY_DIR}/${file} @ONLY)
endforeach()

install(DIRECTORY ${PROJECT_BINARY_DIR}/systemd/ DESTINATION /usr/lib/systemd/user USE_SOURCE_PERMISSIONS)

set(CPACK_GENERATOR DEB CACHE STRING "")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT CACHE STRING "")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "ZMP Inc. <info@zmp.co.jp>" CACHE STRING "")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${CMAKE_PROJECT_NAME}" CACHE STRING "")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}" CACHE STRING "")
set(CPACK_DEBIAN_PACKAGE_RELEASE "" CACHE STRING "")
set(CPACK_PACKAGING_INSTALL_PREFIX /opt/ros/$ENV{ROS_DISTRO} CACHE STRING "")
include(CPack)
