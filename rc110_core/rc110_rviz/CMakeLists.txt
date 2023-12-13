cmake_minimum_required(VERSION 3.10)
project(rc110_rviz VERSION 2.1.0)

find_package(Qt5 5.9 REQUIRED Core Widgets)
find_package(ament_cmake REQUIRED)

find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rc110_msgs REQUIRED)
find_package(rc110_topic_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

# marker
add_executable(rc110_marker src/rc110_marker.cpp)
target_compile_features(rc110_marker PUBLIC cxx_std_17)
ament_target_dependencies(rc110_marker
        rclcpp
        visualization_msgs
)

# panel
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

file(GLOB_RECURSE SRC src/*.cpp res/*.qrc)
add_library(${PROJECT_NAME} SHARED ${SRC})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PRIVATE RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

ament_target_dependencies(${PROJECT_NAME}
        ackermann_msgs
        nav_msgs
        rc110_msgs
        rc110_topic_tools
        rclcpp
        rviz2
        rviz_common
        sensor_msgs
        std_srvs
        visualization_msgs
)

# install
set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME ros)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()

install(DIRECTORY bin/ DESTINATION bin/ USE_SOURCE_PERMISSIONS)

install(DIRECTORY config images launch rviz DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} rc110_marker
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

set(ROS_DISTRO $ENV{ROS_DISTRO})
configure_file(install/rc110_rviz.desktop ${PROJECT_BINARY_DIR}/rc110_rviz.desktop @ONLY)
install(FILES ${PROJECT_BINARY_DIR}/rc110_rviz.desktop
        DESTINATION /usr/share/applications/
        COMPONENT rviz_main EXCLUDE_FROM_ALL
)
install(FILES install/rc110_ros.png
        DESTINATION /usr/share/pixmaps/
        COMPONENT rviz_main EXCLUDE_FROM_ALL
)

# package
set(CPACK_DEB_COMPONENT_INSTALL ON)  # a way to add system files to package installation
set(CPACK_DEBIAN_RVIZ_MAIN_PACKAGE_NAME ros-$ENV{ROS_DISTRO}-rc110-rviz)
set(CPACK_DEBIAN_RVIZ_MAIN_PACKAGE_DEPENDS ros-$ENV{ROS_DISTRO}-rc110-rviz-ros)
set(CPACK_DEBIAN_RVIZ_MAIN_DESCRIPTION "Desktop files for robot which depend on package with ros stuff")
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
set(RC110_DEP.rviz_video_panel ros-$ENV{ROS_DISTRO}-rviz-video-panel)
include(../../cmake/rc110_cpack.cmake)
