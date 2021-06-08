cmake_minimum_required(VERSION 3.10)
project(rc110_simulation VERSION 1.4.0)

find_package(catkin REQUIRED
        roscpp
        geometry_msgs
        ackermann_msgs
        rc110_msgs
)

catkin_package()

set(INTERFACE_NODE ${PROJECT_NAME}_interface)
file(GLOB_RECURSE SRC "src/*.cpp")
add_executable(${INTERFACE_NODE} ${SRC})

add_dependencies(${INTERFACE_NODE} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_features(${INTERFACE_NODE} PUBLIC cxx_std_17)
target_include_directories(${INTERFACE_NODE} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${INTERFACE_NODE}
        ${catkin_LIBRARIES}
)

install(TARGETS ${INTERFACE_NODE}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch meshes rviz urdf worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# package
include(../cmake/rc110_cpack.cmake)
