cmake_minimum_required(VERSION 3.10)
project(rc110_object_detection VERSION 1.12.0)

find_package(CUDA REQUIRED)  # Change CUDA to CUDAToolkit after upgrade to CMake >=3.17
find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)
find_package(OpenCV REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rc110_msgs REQUIRED)

link_directories(/usr/lib/aarch64-linux-gnu/tegra/)  # https://forums.developer.nvidia.com/t/cannot-link-usr-bin-ld-cannot-find-lnvbuf-utils/190973/13

add_executable(rc110_object_detection_node
  src/rc110_object_detection_node.cpp
  src/rc110_object_detection.cpp
  src/rc110_image_converter.cpp
  src/rc110_custom_detect_net.cpp
)

target_compile_features(rc110_object_detection_node
  PUBLIC
    cxx_std_17
)
target_include_directories(rc110_object_detection_node
    SYSTEM PUBLIC
    ${rc110_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${vision_msgs_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(rc110_object_detection_node
    jetson-inference
    jetson-utils
    rclcpp::rclcpp
    ${rc110_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${vision_msgs_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
ament_target_dependencies(rc110_object_detection_node
rc110_msgs
rclcpp
sensor_msgs
vision_msgs
)
ament_package()

install(TARGETS rc110_object_detection_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

# package
include(../../cmake/rc110_cpack.cmake)
