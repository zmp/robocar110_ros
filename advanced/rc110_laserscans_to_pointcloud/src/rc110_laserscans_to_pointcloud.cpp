/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "rc110_laserscans_to_pointcloud.hpp"

namespace zmp
{
constexpr char BASE_LINK[] = "base_link";
constexpr char LIDAR_LINK_1[] = "front_lidar";
constexpr char LIDAR_LINK_2[] = "rear_lidar";

Rc110LaserScansToPointCloud::Rc110LaserScansToPointCloud(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate)
{
    handlePrivate.param<std::string>("base_frame", baseFrame, BASE_LINK);
    handlePrivate.param<std::string>("lidar_frame_1", lidarFrame1, LIDAR_LINK_1);
    handlePrivate.param<std::string>("lidar_frame_2", lidarFrame2, LIDAR_LINK_2);

    subscribers.push_back(handle.subscribe("scan_1", 2, &Rc110LaserScansToPointCloud::onScan, this));
    subscribers.push_back(handle.subscribe("scan_2", 2, &Rc110LaserScansToPointCloud::onScan, this));

    transformer1.waitForTransform(baseFrame, lidarFrame1, ros::Time(0), ros::Duration(5));
    transformer2.waitForTransform(baseFrame, lidarFrame2, ros::Time(0), ros::Duration(5));

    cloudPublisher = handle.advertise<sensor_msgs::PointCloud2>("lidar_cloud", 1);
}

void Rc110LaserScansToPointCloud::onScan(const sensor_msgs::LaserScan::ConstPtr& message)
{
    std::string frameId = message->header.frame_id;
    if (frameId == lidarFrame1) {
        projection.transformLaserScanToPointCloud(baseFrame, *message, cloud1, transformer1);
    } else if (frameId == lidarFrame2) {
        projection.transformLaserScanToPointCloud(baseFrame, *message, cloud2, transformer2);
        mergeAndPublishCloud();
    }
}

void Rc110LaserScansToPointCloud::mergeAndPublishCloud()
{
    sensor_msgs::PointCloud2 cloud = cloud1;
    cloud.width += cloud2.width;
    cloud.row_step += cloud2.row_step;
    cloud.data.insert(cloud.data.end(), cloud2.data.begin(), cloud2.data.end());
    cloudPublisher.publish(cloud);
}

}  // namespace zmp
