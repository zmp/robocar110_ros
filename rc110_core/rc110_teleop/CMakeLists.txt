cmake_minimum_required(VERSION 3.10)
project(rc110_teleop VERSION 2.1.0)

find_package(Boost REQUIRED filesystem)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(ackermann_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(rc110_msgs REQUIRED)
find_package(rc110_topic_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(rc110_joy_teleop
        src/rc110_joy_teleop_node.cpp
        src/rc110_joy_teleop.cpp
)
target_compile_features(rc110_joy_teleop PUBLIC cxx_std_17)
target_link_libraries(rc110_joy_teleop ${YAML_CPP_LIBRARIES} Boost::filesystem)

ament_target_dependencies(rc110_joy_teleop
        ackermann_msgs
        angles
        rc110_msgs
        rc110_topic_tools
        rclcpp
        sensor_msgs
        std_msgs
        std_srvs
)

install(TARGETS rc110_joy_teleop
        RUNTIME DESTINATION lib/rc110_teleop
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# prepare configuration files for roslaunch
set(RC110_SHARE_PATH /opt/ros/$ENV{ROS_DISTRO}/share/${PROJECT_NAME})
file(REMOVE ${PROJECT_BINARY_DIR}/deb/conffiles)
file(GLOB_RECURSE config_files RELATIVE ${PROJECT_SOURCE_DIR} config/*.yaml)
foreach(file ${config_files})
    file(APPEND ${PROJECT_BINARY_DIR}/deb/conffiles "${RC110_SHARE_PATH}/${file}\n")
endforeach()

ament_package()

# package
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${PROJECT_BINARY_DIR}/deb/conffiles;")
include(../../cmake/rc110_cpack.cmake)
