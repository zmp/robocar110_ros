cmake_minimum_required(VERSION 3.10)
project(rc110_selector VERSION 1.10.0)

find_package(Qt5 5.9 REQUIRED Core Widgets)

find_package(catkin REQUIRED roscpp)

catkin_package()

set(CMAKE_AUTORCC ON)

add_executable(${PROJECT_NAME} src/main.cpp src/res.qrc)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        Qt5::Widgets
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# package
include(../../cmake/rc110_cpack.cmake)
