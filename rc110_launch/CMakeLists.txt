cmake_minimum_required(VERSION 3.10)
project(rc110_launch VERSION 1.0.0)

set(catkin_exec_depends roslaunch)
find_package(catkin REQUIRED ${catkin_exec_depends})

catkin_package()

install(DIRECTORY config launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# prepare configuration files for roslaunch
set(RC110_CONFIG_PATH /opt/ros/$ENV{ROS_DISTRO}/${CATKIN_PACKAGE_SHARE_DESTINATION})
configure_file(${PROJECT_SOURCE_DIR}/deb/postinst ${PROJECT_BINARY_DIR}/deb/postinst @ONLY)

# package
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${PROJECT_BINARY_DIR}/deb/postinst;")
include(../cmake/rc110_cpack.cmake)
