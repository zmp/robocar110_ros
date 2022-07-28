cmake_minimum_required(VERSION 3.10)
project(multirelay VERSION 1.10.2)

find_package(catkin REQUIRED
        param_tools
        topic_tools
)

catkin_package()

add_executable(${PROJECT_NAME} src/multirelay.cpp src/multirelay_node.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# package
set(RC110_DEP.param_tools ros-$ENV{ROS_DISTRO}-param-tools)
include(../../cmake/rc110_cpack.cmake)
