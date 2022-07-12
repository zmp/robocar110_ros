/*
 * Copyright (C) 2022 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "rc110_marker.hpp"

#include <visualization_msgs/Marker.h>

namespace zmp
{
RC110Marker::RC110Marker()
{
    meshFile = ros::param::param("~mesh_file", std::string()),
    publisher = handle.advertise<visualization_msgs::Marker>("rviz_marker", 1, bool("latch"));

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.mesh_resource = meshFile;
    marker.mesh_use_embedded_materials = true;
    publisher.publish(marker);
}
}  // namespace zmp

int main(int argc, char* argv[])
try {
    ros::init(argc, argv, "rc110_marker");

    zmp::RC110Marker node;
    ros::spin();

    return EXIT_SUCCESS;
}  //
catch (std::exception& ex) {
    std::cerr << "Exception in main(): " << ex.what() << std::endl;
    return EXIT_FAILURE;
} catch (...) {
    std::cerr << "Unknown exception in main()" << std::endl;
    return EXIT_FAILURE;
}