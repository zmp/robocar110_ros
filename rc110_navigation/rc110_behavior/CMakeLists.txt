cmake_minimum_required(VERSION 3.10)
project(rc110_behavior VERSION 2.1.0)

find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(laser_geometry REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

file(GLOB_RECURSE SRC "src/*.cpp")
add_executable(${PROJECT_NAME} ${SRC})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC include ${Boost_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
        ackermann_msgs
        sensor_msgs
        laser_geometry
        rclcpp
        tf2_ros
        tf2
        tf2_geometry_msgs
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

# package
include(../../cmake/rc110_cpack.cmake)