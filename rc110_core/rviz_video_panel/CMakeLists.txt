cmake_minimum_required(VERSION 3.10)
project(rviz_video_panel VERSION 2.1.0)

find_package(Qt5 5.9 REQUIRED Core Widgets Multimedia MultimediaWidgets)
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)


file(GLOB_RECURSE SRC src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${SRC})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PRIVATE RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY)
target_link_libraries(${PROJECT_NAME}
        Qt5::Widgets
        Qt5::Multimedia
        Qt5::MultimediaWidgets
)
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        rviz2
        rviz_common
)

# install
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# package
include(../../cmake/rc110_cpack.cmake)
