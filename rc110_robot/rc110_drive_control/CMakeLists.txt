cmake_minimum_required(VERSION 3.10)
project(rc110_drive_control VERSION 2.2.0)

set(RC110_COMPONENTS control CACHE STRING "")

find_package(Boost COMPONENTS system filesystem thread REQUIRED)
find_package(robocar110 REQUIRED ${RC110_COMPONENTS})
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rc110_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rc-control REQUIRED)

ament_package()

file(GLOB_RECURSE SRC "src/*.cpp")
add_executable(${PROJECT_NAME} ${SRC})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)
target_link_libraries(${PROJECT_NAME} rc-control)
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_srvs
        rc-control
        ackermann_msgs
        nav_msgs
        rc110_msgs
        sensor_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})

# package
set(CPACK_DEBIAN_PACKAGE_DEPENDS rc-system)
include(../../cmake/rc110_cpack.cmake)
