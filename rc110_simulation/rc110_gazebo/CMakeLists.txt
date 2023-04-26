cmake_minimum_required(VERSION 3.10)
project(rc110_gazebo VERSION 2.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rc110_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rc110_topic_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(rc110_gazebo
               src/rc110_gazebo_node.cpp
               src/rc110_gazebo.cpp
               src/rc110_gazebo.hpp)

target_compile_features(rc110_gazebo PUBLIC cxx_std_17)
target_include_directories(rc110_gazebo
    SYSTEM PUBLIC
    ${rc110_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${ackermann_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${std_srvs_INCLUDE_DIRS}
)
target_link_libraries(rc110_gazebo
    rclcpp::rclcpp
    ${rc110_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${ackermann_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${std_srvs_LIBRARIES}
)
ament_target_dependencies(rc110_gazebo
        ackermann_msgs
        geometry_msgs
        rc110_msgs
        rclcpp
        sensor_msgs
        std_srvs
)
ament_package()

install(TARGETS rc110_gazebo
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/rc110_gazebo
)

install(DIRECTORY launch worlds DESTINATION share/rc110_gazebo)

# package
include(../../cmake/rc110_cpack.cmake)