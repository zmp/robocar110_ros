cmake_minimum_required(VERSION 3.10)
project(rc110_rviz VERSION 1.12.0)

find_package(Qt5 5.9 REQUIRED Core Widgets)

find_package(catkin REQUIRED
        rviz
        std_srvs
        topic_tools
        ackermann_msgs
        nav_msgs
        rc110_msgs
        sensor_msgs
        visualization_msgs
)

catkin_package()

# marker
add_executable(rc110_marker src/rc110_marker.cpp)
target_compile_features(rc110_marker PUBLIC cxx_std_17)
target_include_directories(rc110_marker SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(rc110_marker ${catkin_LIBRARIES})

# panel
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

file(GLOB_RECURSE SRC src/*.cpp res/*.qrc)
add_library(${PROJECT_NAME} ${SRC})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        Qt5::Widgets
)

# install
install(DIRECTORY images launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS ${PROJECT_NAME} rc110_marker
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_BUILD_BINARY_PACKAGE)
    configure_file(install/rc110_rviz.desktop ${PROJECT_BINARY_DIR}/rc110_rviz.desktop @ONLY)

    install(FILES ${PROJECT_BINARY_DIR}/rc110_rviz.desktop DESTINATION /usr/share/applications/)
    install(FILES install/rc110_ros.png DESTINATION /usr/share/pixmaps/)
endif()

# package
set(RC110_DEP.rviz_video_panel ros-$ENV{ROS_DISTRO}-rviz-video-panel)
include(../../cmake/rc110_cpack.cmake)
