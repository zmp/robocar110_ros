cmake_minimum_required(VERSION 3.10)
project(rc110_teleop VERSION 1.12.1)

find_package(catkin REQUIRED COMPONENTS
        ackermann_msgs
        angles
        rc110_msgs
        roscpp
        roslib
        sensor_msgs
        std_srvs
        topic_tools
)
find_package(yaml-cpp REQUIRED)

catkin_package()

add_executable(rc110_joy_teleop
        src/rc110_joy_teleop_node.cpp
        src/rc110_joy_teleop.cpp
)
add_dependencies(rc110_joy_teleop
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_compile_features(rc110_joy_teleop PUBLIC cxx_std_17)
target_include_directories(rc110_joy_teleop SYSTEM PUBLIC ${catkin_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS})
target_link_libraries(rc110_joy_teleop ${catkin_LIBRARIES} yaml-cpp)

install(TARGETS rc110_joy_teleop
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# prepare configuration files for roslaunch
set(RC110_SHARE_PATH /opt/ros/$ENV{ROS_DISTRO}/${CATKIN_PACKAGE_SHARE_DESTINATION})
file(REMOVE ${PROJECT_BINARY_DIR}/deb/conffiles)
file(GLOB_RECURSE config_files RELATIVE ${PROJECT_SOURCE_DIR} config/*.yaml)
foreach(file ${config_files})
    file(APPEND ${PROJECT_BINARY_DIR}/deb/conffiles "${RC110_SHARE_PATH}/${file}\n")
endforeach()

# package
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${PROJECT_BINARY_DIR}/deb/conffiles;")
include(../../cmake/rc110_cpack.cmake)
