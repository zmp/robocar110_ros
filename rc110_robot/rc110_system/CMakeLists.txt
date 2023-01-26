cmake_minimum_required(VERSION 3.10)
project(rc110_system VERSION 1.12.1)

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

# prepare configuration files for roslaunch
set(RC110_SHARE_PATH /opt/ros/$ENV{ROS_DISTRO}/${CATKIN_PACKAGE_SHARE_DESTINATION})
file(REMOVE ${PROJECT_BINARY_DIR}/deb/conffiles)
file(GLOB_RECURSE config_files RELATIVE ${PROJECT_SOURCE_DIR} config/*.yaml)
foreach(file ${config_files})
    file(APPEND ${PROJECT_BINARY_DIR}/deb/conffiles "${RC110_SHARE_PATH}/${file}\n")
endforeach()

if (CATKIN_BUILD_BINARY_PACKAGE)
    install(DIRECTORY ${PROJECT_BINARY_DIR}/systemd/ DESTINATION /usr/lib/systemd/user USE_SOURCE_PERMISSIONS)
endif()

install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# tests
if (CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(launch)
endif()

# package
list(APPEND CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
        "${PROJECT_BINARY_DIR}/deb/conffiles"
)

set(CPACK_DEBIAN_PACKAGE_CONFLICTS ros-$ENV{ROS_DISTRO}-rc110-service)
set(CPACK_DEBIAN_PACKAGE_PROVIDES ros-$ENV{ROS_DISTRO}-rc110-service)
set(CPACK_DEBIAN_PACKAGE_REPLACES ros-$ENV{ROS_DISTRO}-rc110-service)
include(../../cmake/rc110_cpack.cmake)
