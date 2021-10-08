cmake_minimum_required(VERSION 3.10)
project(rc110_msgs VERSION 1.7.0)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        roscpp
        std_msgs
)

add_message_files(DIRECTORY msg)
add_service_files(DIRECTORY srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)


# package
include(../../cmake/rc110_cpack.cmake)
