cmake_minimum_required(VERSION 3.10)
project(rc110 VERSION 2.0.0)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)

# prepare configuration files for roslaunch
set(RC110_SHARE_PATH /opt/ros/$ENV{ROS_DISTRO}/share/${PROJECT_NAME})
set(RC110_SHARE_PATH /opt/ros/$ENV{ROS_DISTRO}/share/${PROJECT_NAME})
file(REMOVE ${PROJECT_BINARY_DIR}/deb/conffiles)
file(GLOB_RECURSE config_files RELATIVE ${PROJECT_SOURCE_DIR} models/*.sdf urdf/*.xml)
foreach(file ${config_files})
    file(APPEND ${PROJECT_BINARY_DIR}/deb/conffiles "${RC110_SHARE_PATH}/${file}\n")
endforeach()

ament_package()
ament_package()

# install
install(DIRECTORY config launch models urdf
        DESTINATION share/${PROJECT_NAME}
        DESTINATION share/${PROJECT_NAME}
)

# package
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${PROJECT_BINARY_DIR}/deb/conffiles;")
include(../../cmake/rc110_cpack.cmake)
