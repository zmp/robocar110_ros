cmake_minimum_required(VERSION 3.10)
project(rc110_teleop VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  ackermann_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    ackermann_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rc110_joy_teleop
  src/rc110_joy_teleop_node.cpp
  src/rc110_joy_teleop.cpp
  src/teleop_component.cpp
)
add_dependencies(rc110_joy_teleop
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rc110_joy_teleop
  ${catkin_LIBRARIES}
)
install(TARGETS rc110_joy_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
