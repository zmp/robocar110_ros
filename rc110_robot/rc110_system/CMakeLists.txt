cmake_minimum_required(VERSION 3.10)
project(rc110_system VERSION 2.0.0)

find_package(ament_cmake REQUIRED)
ament_package()
find_package(ament_cmake REQUIRED)
ament_package()

# copy configs to BIN and fill @variables@
file(REMOVE_RECURSE ${PROJECT_BINARY_DIR}/systemd)
file(GLOB systemd_files RELATIVE ${PROJECT_SOURCE_DIR} systemd/*)
set(ROS_DISTRO $ENV{ROS_DISTRO})
set(ROS_DISTRO $ENV{ROS_DISTRO})
foreach(file ${systemd_files})
    configure_file(${file} ${PROJECT_BINARY_DIR}/${file} @ONLY)
endforeach()

# prepare configuration files for roslaunch
set(RC110_SHARE_PATH /opt/ros/$ENV{ROS_DISTRO}/share/${PROJECT_NAME})
set(RC110_SHARE_PATH /opt/ros/$ENV{ROS_DISTRO}/share/${PROJECT_NAME})
file(REMOVE ${PROJECT_BINARY_DIR}/deb/conffiles)
file(GLOB_RECURSE config_files RELATIVE ${PROJECT_SOURCE_DIR} config/*.yaml)
foreach(file ${config_files})
    file(APPEND ${PROJECT_BINARY_DIR}/deb/conffiles "${RC110_SHARE_PATH}/${file}\n")
endforeach()

install(DIRECTORY ${PROJECT_BINARY_DIR}/systemd/ DESTINATION systemd USE_SOURCE_PERMISSIONS)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${PROJECT_BINARY_DIR}/systemd/ DESTINATION systemd USE_SOURCE_PERMISSIONS)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# package
list(APPEND CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
        "${PROJECT_BINARY_DIR}/deb/conffiles"
)

set(CPACK_DEBIAN_PACKAGE_CONFLICTS ros-$ENV{ROS_DISTRO}-rc110-service)
set(CPACK_DEBIAN_PACKAGE_PROVIDES ros-$ENV{ROS_DISTRO}-rc110-service)
set(CPACK_DEBIAN_PACKAGE_REPLACES ros-$ENV{ROS_DISTRO}-rc110-service)
include(../../cmake/rc110_cpack.cmake)
