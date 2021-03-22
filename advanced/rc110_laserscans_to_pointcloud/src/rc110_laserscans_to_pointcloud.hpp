/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
