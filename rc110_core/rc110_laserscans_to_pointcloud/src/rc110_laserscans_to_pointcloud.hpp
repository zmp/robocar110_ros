/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
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
    using ApproxSyncPolicy =
            message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan>;
    using ApproxSync = message_filters::Synchronizer<ApproxSyncPolicy>;

public:
    Rc110LaserScansToPointCloud();

private:
    void onConnect();
    void onScans(const sensor_msgs::LaserScan::ConstPtr& front, const sensor_msgs::LaserScan::ConstPtr& rear);

private:
    ros::NodeHandle handle;
    std::string outputFrameId, frontLidarFrameId, rearLidarFrameId;
    float angleThreshold;
    laser_geometry::LaserProjection projection;
    std::unique_ptr<ApproxSync> sync;
    message_filters::Subscriber<sensor_msgs::LaserScan> frontSubscriber, rearSubscriber;
    bool subscribed = false;
    tf::TransformListener frontTransformer, rearTransformer;
    ros::Publisher cloudPublisher;
};
}  // namespace zmp
