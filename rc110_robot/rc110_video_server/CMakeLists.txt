cmake_minimum_required(VERSION 3.10)
project(rc110_video_server VERSION 2.0.0)

find_package(PkgConfig REQUIRED)
foreach(dependency gstreamer-1.0 gstreamer-rtsp-1.0 gstreamer-rtsp-server-1.0)
    pkg_check_modules(${dependency} REQUIRED IMPORTED_TARGET ${dependency})
endforeach()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

file(GLOB_RECURSE SRC "src/*.cpp")
add_executable(${PROJECT_NAME} ${SRC})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}
        PkgConfig::gstreamer-1.0
        PkgConfig::gstreamer-rtsp-1.0
        PkgConfig::gstreamer-rtsp-server-1.0
)
ament_target_dependencies(${PROJECT_NAME} rclcpp)

ament_package()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# package
include(../../cmake/rc110_cpack.cmake)