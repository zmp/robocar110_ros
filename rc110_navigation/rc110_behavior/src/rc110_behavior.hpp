/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace zmp
{
/**
 * Simple behavior example. It's reading sensors and moving drives accordingly.
 * ┌───────────┬───────────┐
 * │           │           │
 * │   left    │   right   │
 * │           │           │
 * └─────┬─────┴─────┬─────┘
 *       │   near    │
 *       └───────────┘
 *             ■ ← robot origin
 *
 * if there's a point in near box, stop.
 * if in left box, move right.
 * if in right box, move left.
 * if neither, move forward.
 */
class Rc110Behavior : public rclcpp::Node
{
public:
    Rc110Behavior();

private:
    void onCloud(const sensor_msgs::msg::PointCloud2& cloud);
    void publishCommand(float speed, float steering);

private:
    rclcpp::SubscriptionBase::SharedPtr cloudSubscriber;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drivePublisher;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    geometry_msgs::msg::TransformStamped baseTransform;

    std::vector<double> stopCommand = {0, 0};
    std::vector<double> forwardCommand = {0.4, 0};
    std::vector<double> leftCommand = {0.25, 28};
    std::vector<double> rightCommand = {0.25, -28};
};
}  // namespace zmp
