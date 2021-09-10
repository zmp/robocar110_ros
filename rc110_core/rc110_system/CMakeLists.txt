cmake_minimum_required(VERSION 3.10)
project(rc110_system VERSION 1.6.0)

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

if (CATKIN_BUILD_BINARY_PACKAGE)
    install(DIRECTORY ${PROJECT_BINARY_DIR}/systemd/ DESTINATION /usr/lib/systemd/user USE_SOURCE_PERMISSIONS)
endif()

# package
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${PROJECT_SOURCE_DIR}/deb/postinst;")
set(CPACK_DEBIAN_PACKAGE_DEPENDS ros-$ENV{ROS_DISTRO}-rc110-launch)

set(CPACK_DEBIAN_PACKAGE_CONFLICTS ros-$ENV{ROS_DISTRO}-rc110-service)
set(CPACK_DEBIAN_PACKAGE_PROVIDES ros-$ENV{ROS_DISTRO}-rc110-service)
set(CPACK_DEBIAN_PACKAGE_REPLACES ros-$ENV{ROS_DISTRO}-rc110-service)
include(../../cmake/rc110_cpack.cmake)
