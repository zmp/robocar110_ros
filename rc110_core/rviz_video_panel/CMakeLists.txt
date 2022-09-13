cmake_minimum_required(VERSION 3.10)
project(rviz_video_panel VERSION 1.12.0)

find_package(Qt5 5.9 REQUIRED Core Widgets Multimedia MultimediaWidgets)
find_package(catkin REQUIRED rviz)

catkin_package()

file(GLOB_RECURSE SRC src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${SRC})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        Qt5::Widgets
        Qt5::Multimedia
        Qt5::MultimediaWidgets
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# package
include(../../cmake/rc110_cpack.cmake)
