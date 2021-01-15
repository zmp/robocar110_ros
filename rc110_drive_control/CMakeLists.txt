cmake_minimum_required(VERSION 3.10)
project(rc110_drive_control VERSION 1.1.0)

set(RC110_COMPONENTS control CACHE STRING "")
find_package(robocar110 REQUIRED ${RC110_COMPONENTS})

find_package(catkin REQUIRED
        roscpp
        std_srvs
        ackermann_msgs
        nav_msgs
        sensor_msgs
        rc110_msgs
)

catkin_package()

file(GLOB_RECURSE SRC "src/*.cpp")
add_executable(${PROJECT_NAME} ${SRC})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_14)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        rc-control
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# package
set(CPACK_DEBIAN_PACKAGE_DEPENDS rc-system)
include(../cmake/rc110_cpack.cmake)
