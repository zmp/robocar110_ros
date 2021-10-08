cmake_minimum_required(VERSION 3.10)
project(rc110_video_server VERSION 1.6.1)

find_package(PkgConfig REQUIRED)
foreach(dependency gstreamer-1.0 gstreamer-rtsp-1.0 gstreamer-rtsp-server-1.0)
    pkg_check_modules(${dependency} REQUIRED IMPORTED_TARGET ${dependency})
endforeach()

find_package(catkin REQUIRED
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
        PkgConfig::gstreamer-1.0
        PkgConfig::gstreamer-rtsp-1.0
        PkgConfig::gstreamer-rtsp-server-1.0
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# package
include(../../cmake/rc110_cpack.cmake)