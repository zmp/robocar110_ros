cmake_minimum_required(VERSION 3.10)
project(rc110_common VERSION 1.7.0)

find_package(catkin)

catkin_package()

install(DIRECTORY launch models urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# prepare configuration files for roslaunch
set(RC110_SHARE_PATH /opt/ros/$ENV{ROS_DISTRO}/${CATKIN_PACKAGE_SHARE_DESTINATION})
file(REMOVE ${PROJECT_BINARY_DIR}/deb/conffiles)
file(GLOB_RECURSE config_files RELATIVE ${PROJECT_SOURCE_DIR} models/*.sdf urdf/*.xml)
foreach(file ${config_files})
    file(APPEND ${PROJECT_BINARY_DIR}/deb/conffiles "${RC110_SHARE_PATH}/${file}\n")
endforeach()

# tests
if (CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(launch)
endif()

# package
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${PROJECT_BINARY_DIR}/deb/conffiles;")
include(../../cmake/rc110_cpack.cmake)
