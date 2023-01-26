cmake_minimum_required(VERSION 3.10)
project(rc110_topic_tools VERSION 1.12.1)

find_package(catkin REQUIRED
        topic_tools
        std_msgs
)

catkin_package()

add_executable(multi_demux src/multi_demux.cpp)
add_dependencies(multi_demux ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_compile_features(multi_demux PUBLIC cxx_std_17)
target_include_directories(multi_demux SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(multi_demux ${catkin_LIBRARIES})

install(TARGETS multi_demux
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# package
include(../../cmake/rc110_cpack.cmake)
