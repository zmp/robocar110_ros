cmake_minimum_required(VERSION 3.10)
project(rc110_topic_tools VERSION 2.0.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        srv/MuxSelect.srv
    DEPENDENCIES
        builtin_interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        srv/MuxSelect.srv
    DEPENDENCIES
        builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(mux_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(mux_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(mux src/mux.cpp)
target_compile_features(mux PUBLIC cxx_std_17)
target_link_libraries(mux ${mux_typesupport})
ament_target_dependencies(mux rclcpp std_msgs)
install(TARGETS mux RUNTIME DESTINATION lib/${PROJECT_NAME})
add_executable(mux src/mux.cpp)
target_compile_features(mux PUBLIC cxx_std_17)
target_link_libraries(mux ${mux_typesupport})
ament_target_dependencies(mux rclcpp std_msgs)
install(TARGETS mux RUNTIME DESTINATION lib/${PROJECT_NAME})

add_executable(multi_demux src/multi_demux.cpp)
target_compile_features(multi_demux PUBLIC cxx_std_17)
ament_target_dependencies(multi_demux rclcpp std_msgs)
install(TARGETS multi_demux RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_target_dependencies(multi_demux rclcpp std_msgs)
install(TARGETS multi_demux RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
ament_package()

# package
include(../../cmake/rc110_cpack.cmake)
