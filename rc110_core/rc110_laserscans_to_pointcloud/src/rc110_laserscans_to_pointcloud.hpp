/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <laser_geometry/laser_geometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>

namespace zmp
{
/**
 */
class Rc110LaserScansToPointCloud : public rclcpp::Node
{
    using ApproxSyncPolicy =
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;
    using ApproxSync = message_filters::Synchronizer<ApproxSyncPolicy>;

public:
    Rc110LaserScansToPointCloud();

private:
    void onTimer();
    void onScans(const sensor_msgs::msg::LaserScan::ConstSharedPtr& front, const sensor_msgs::msg::LaserScan::ConstSharedPtr& rear);

private:
    std::string outputFrameId, frontLidarFrameId, rearLidarFrameId;
    double angleThreshold;
    laser_geometry::LaserProjection projection;
    std::unique_ptr<ApproxSync> sync;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> frontSubscriber, rearSubscriber;
    bool subscribed = false;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPublisher;
    rclcpp::TimerBase::SharedPtr timer;
};
}  // namespace zmp
