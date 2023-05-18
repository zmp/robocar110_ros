cmake_minimum_required(VERSION 3.10)
project(rc110_selector VERSION 2.0.0)

find_package(Qt5 5.9 REQUIRED Core Widgets Network)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 5.9 REQUIRED Core Widgets Network)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_AUTORCC ON)

add_executable(${PROJECT_NAME} src/main.cpp src/res.qrc)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Network)
ament_target_dependencies(${PROJECT_NAME} rclcpp)

ament_package()
target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Network)
ament_target_dependencies(${PROJECT_NAME} rclcpp)

ament_package()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# package
include(../../cmake/rc110_cpack.cmake)
