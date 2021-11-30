/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

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
class Rc110Behavior
{
public:
    Rc110Behavior();

private:
    void onCloud(const sensor_msgs::PointCloud2& cloud);
    void publishCommand(float speed, float steering);

private:
    ros::NodeHandle handle;
    ros::Subscriber cloudSubscriber;
    ros::Publisher drivePublisher;

    std::vector<float> stopCommand = {0, 0};
    std::vector<float> forwardCommand = {0.4f, 0};
    std::vector<float> leftCommand = {0.25f, 28};
    std::vector<float> rightCommand = {0.25f, -28};
};
}  // namespace zmp
