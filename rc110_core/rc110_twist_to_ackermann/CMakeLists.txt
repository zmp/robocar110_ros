cmake_minimum_required(VERSION 3.10)
project(rc110_twist_to_ackermann VERSION 2.0.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

file(GLOB_RECURSE SRC "src/*.cpp")
add_executable(${PROJECT_NAME} ${SRC})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    geometry_msgs
    ackermann_msgs
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    geometry_msgs
    ackermann_msgs
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()

# package
include(../../cmake/rc110_cpack.cmake)

