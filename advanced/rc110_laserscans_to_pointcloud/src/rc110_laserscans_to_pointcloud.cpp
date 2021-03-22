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
