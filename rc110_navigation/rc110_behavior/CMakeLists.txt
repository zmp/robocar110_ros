cmake_minimum_required(VERSION 3.10)
project(rc110_behavior VERSION 1.4.0)

find_package(catkin REQUIRED
        ackermann_msgs
        sensor_msgs
        behaviortree_cpp_v3
        laser_geometry
        roscpp
)

catkin_package()

file(GLOB_RECURSE SRC "src/*.cpp")
add_executable(${PROJECT_NAME} ${SRC})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC nssv_CONFIG_SELECT_STRING_VIEW=1) # disable string view for behaviortree_cpp_v3

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# package
include(../../cmake/rc110_cpack.cmake)