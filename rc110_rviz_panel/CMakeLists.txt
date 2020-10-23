cmake_minimum_required(VERSION 3.10)
project(rc110_rviz_panel VERSION 1.0.0)

set(catkin_exec_depends
        rviz
        ackermann_msgs
        sensor_msgs
)
find_package(catkin REQUIRED ${catkin_exec_depends})

catkin_package()

find_package(Qt5 5.9 REQUIRED Core Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

file(GLOB_RECURSE SRC src/*.cpp)
add_library(${PROJECT_NAME} ${SRC})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        Qt5::Widgets
        rc-base
)

target_compile_definitions(${PROJECT_NAME} PUBLIC nssv_CONFIG_SELECT_STRING_VIEW=1) # disable string view for behaviortree_cpp_v3

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


add_executable(${PROJECT_NAME}_app app/main.cpp)
target_link_libraries(${PROJECT_NAME}_app ${PROJECT_NAME})

# package
include(../cmake/rc110_cpack.cmake)
