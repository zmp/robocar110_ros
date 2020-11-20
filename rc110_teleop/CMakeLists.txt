cmake_minimum_required(VERSION 3.10)
project(rc110_teleop VERSION 1.0.0)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        ackermann_msgs
        rc110_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    ackermann_msgs
)

add_executable(rc110_joy_teleop
  src/rc110_joy_teleop_node.cpp
  src/rc110_joy_teleop.cpp
)
add_dependencies(rc110_joy_teleop
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(rc110_joy_teleop PUBLIC cxx_std_17)
target_include_directories(rc110_joy_teleop
  SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(rc110_joy_teleop
  ${catkin_LIBRARIES}
)

install(TARGETS rc110_joy_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# package
set(CPACK_DEBIAN_PACKAGE_DEPENDS rc-system)
include(../cmake/rc110_cpack.cmake)
