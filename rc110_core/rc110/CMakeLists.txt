cmake_minimum_required(VERSION 3.10)
project(rc110 VERSION 1.9.0)

find_package(catkin)

catkin_package()

# prepare configuration files for roslaunch
set(RC110_SHARE_PATH /opt/ros/$ENV{ROS_DISTRO}/${CATKIN_PACKAGE_SHARE_DESTINATION})
file(REMOVE ${PROJECT_BINARY_DIR}/deb/conffiles)
file(GLOB_RECURSE config_files RELATIVE ${PROJECT_SOURCE_DIR} models/*.sdf urdf/*.xml)
foreach(file ${config_files})
    file(APPEND ${PROJECT_BINARY_DIR}/deb/conffiles "${RC110_SHARE_PATH}/${file}\n")
endforeach()

# devel files that also installed to /opt/ros/<>/
file(GLOB devel_files RELATIVE ${PROJECT_SOURCE_DIR}/devel devel/*.bash)
foreach(file ${devel_files})
    configure_file(${PROJECT_SOURCE_DIR}/devel/${file} ${CATKIN_DEVEL_PREFIX}/${file} COPYONLY)
endforeach()

# tests
if (CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(launch)
endif()

# install
install(DIRECTORY bin/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
)
install(DIRECTORY config launch models urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY devel/
        DESTINATION /opt/ros/$ENV{ROS_DISTRO}
)

# package
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${PROJECT_BINARY_DIR}/deb/conffiles;")
set(CPACK_DEBIAN_PACKAGE_DEPENDS python-gobject)  # zeroconf dependency on Ubuntu 18

include(../../cmake/rc110_cpack.cmake)
