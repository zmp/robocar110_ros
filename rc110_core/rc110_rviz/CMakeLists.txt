cmake_minimum_required(VERSION 3.10)
project(rc110_rviz VERSION 1.10.2)

find_package(Qt5 5.9 REQUIRED Core Widgets)

find_package(catkin REQUIRED
        rviz
        std_srvs
        topic_tools
        ackermann_msgs
        nav_msgs
        param_tools
        rc110_msgs
        sensor_msgs
)

catkin_package()

if(NOT DEFINED ENV{ROS_DISTRO})
    message(FATAL_ERROR "ROS_DISTRO is not defined. Please, source ros.")
endif()
set(ROS_DISTRO $ENV{ROS_DISTRO})

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

install(DIRECTORY images launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS ${PROJECT_NAME}
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


add_executable(${PROJECT_NAME}_app app/main.cpp)
target_link_libraries(${PROJECT_NAME}_app ${PROJECT_NAME})

# package
set(RC110_DEP.multirelay ros-$ENV{ROS_DISTRO}-multirelay)
set(RC110_DEP.param_tools ros-$ENV{ROS_DISTRO}-param-tools)
include(../../cmake/rc110_cpack.cmake)
