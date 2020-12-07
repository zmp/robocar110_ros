/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Andrei Pak
 */
#pragma once

#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>

namespace zmp
{
/**
 */
class Rc110LaserScansToPointCloud
{
public:
    Rc110LaserScansToPointCloud(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate);

private:
    void onScan(const sensor_msgs::LaserScan::ConstPtr& message);
    void mergeAndPublishCloud();

private:
    std::string baseFrame, lidarFrame1, lidarFrame2;
    laser_geometry::LaserProjection projection;
    std::vector<ros::Subscriber> subscribers;
    sensor_msgs::PointCloud2 cloud1, cloud2;
    tf::TransformListener transformer1, transformer2;
    ros::Publisher cloudPublisher;
};
}  // namespace zmp
