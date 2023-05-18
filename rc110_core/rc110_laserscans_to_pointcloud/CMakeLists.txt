cmake_minimum_required(VERSION 3.10)
project(rc110_laserscans_to_pointcloud VERSION 2.0.0)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

file(GLOB_RECURSE SRC "src/*.cpp")
add_executable(${PROJECT_NAME} ${SRC})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC include ${Boost_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  laser_geometry
  tf2_ros)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC include ${Boost_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  laser_geometry
  tf2_ros)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

# package
include(../../cmake/rc110_cpack.cmake)