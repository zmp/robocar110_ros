/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "rc110_laserscans_to_pointcloud.hpp"

#include <sensor_msgs/point_cloud2_iterator.h>

namespace zmp
{
constexpr char FRONT_LIDAR_FRAME_ID[] = "front_lidar";
constexpr char REAR_LIDAR_FRAME_ID[] = "rear_lidar";

namespace
{
/*
 * Add points to map sorted by distance.
 */
void addPointsByDistance(const sensor_msgs::PointCloud2& cloud, std::map<float, std::vector<float>>& pointsByDistance)
{
    int pointSize = cloud.point_step / sizeof(float);
    using Iter = sensor_msgs::PointCloud2ConstIterator<float>;

    for (Iter it = {cloud, "x"}; it != it.end(); ++it) {
        float x = it[0], y = it[1];
        float distance = hypot(y, x);
        const float* pointBegin = &it[0];
        pointsByDistance.emplace(distance, std::vector<float>(pointBegin, pointBegin + pointSize));
    }
}

/*
 * Add points skipping ones that are angularly too close to each other.
 */
void addPointsByAngle(const std::map<float, std::vector<float>>& pointsByDistance,
                      std::map<float, std::vector<float>>& points,
                      float angleAllowance)
{
    for (auto& distancePoint : pointsByDistance) {
        float x = distancePoint.second[0], y = distancePoint.second[1];
        float angle = atan2(y, x);

        auto nextIt = points.lower_bound(angle);
        if (nextIt != points.end() && nextIt->first - angle < angleAllowance) {
            continue;
        }
        if (nextIt != points.begin() && angle - (--nextIt)->first < angleAllowance) {
            continue;
        }
        points.emplace(angle, distancePoint.second);
    }
}
}  // namespace

Rc110LaserScansToPointCloud::Rc110LaserScansToPointCloud() :
        outputFrameId(ros::param::param<std::string>("~output_frame_id", REAR_LIDAR_FRAME_ID)),
        frontLidarFrameId(ros::param::param<std::string>("~front_lidar_frame_id", FRONT_LIDAR_FRAME_ID)),
        rearLidarFrameId(ros::param::param<std::string>("~rear_lidar_frame_id", REAR_LIDAR_FRAME_ID)),
        angleThreshold(ros::param::param("~angle_threshold", 0.9f))  // +90% / -90% of angle_increment
{
    frontTransformer.waitForTransform(outputFrameId, frontLidarFrameId, ros::Time(0), ros::Duration(5));
    rearTransformer.waitForTransform(outputFrameId, rearLidarFrameId, ros::Time(0), ros::Duration(5));

    sync = std::make_unique<ApproxSync>(ApproxSyncPolicy(3), frontSubscriber, rearSubscriber);
    sync->registerCallback(&Rc110LaserScansToPointCloud::onScans, this);

    auto onConnectCallback = [this](const ros::SingleSubscriberPublisher&) { onConnect(); };
    cloudPublisher = handle.advertise<sensor_msgs::PointCloud2>("points2", 1, onConnectCallback, onConnectCallback);
}

void Rc110LaserScansToPointCloud::onConnect()
{
    if (cloudPublisher.getNumSubscribers()) {
        if (!subscribed) {
            subscribed = true;
            frontSubscriber.subscribe(handle, "front_scan", 1);
            rearSubscriber.subscribe(handle, "rear_scan", 1);
        }
    } else {
        subscribed = false;
        frontSubscriber.unsubscribe();
        rearSubscriber.unsubscribe();
    }
}

void Rc110LaserScansToPointCloud::onScans(const sensor_msgs::LaserScan::ConstPtr& front,
                                          const sensor_msgs::LaserScan::ConstPtr& rear)
{
    if (front->header.frame_id != frontLidarFrameId || rear->header.frame_id != rearLidarFrameId) {
        ROS_ERROR_STREAM("Wrong scan frame id: " << front->header.frame_id << ", " << rear->header.frame_id);
        return;
    }

    sensor_msgs::PointCloud2 frontCloud, rearCloud;
    projection.transformLaserScanToPointCloud(outputFrameId, *front, frontCloud, frontTransformer);
    projection.transformLaserScanToPointCloud(outputFrameId, *rear, rearCloud, rearTransformer);

    // points sorted by distance from rear lidar
    // rear lidar as main gives a bit better result as coordinates origin is near it
    std::map<float, std::vector<float>> pointsByDistance;
    addPointsByDistance(frontCloud, pointsByDistance);
    addPointsByDistance(rearCloud, pointsByDistance);

    // unique and sorted by angle points (-pi, pi]
    // all points behind other points relative to rear lidar are discarded
    std::map<float, std::vector<float>> points;
    float angleAllowance = rear->angle_increment * angleThreshold;
    addPointsByAngle(pointsByDistance, points, angleAllowance);

    std::vector<uint8_t> data;
    for (auto& point : points) {
        auto begin = reinterpret_cast<uint8_t*>(point.second.data());
        auto end = begin + point.second.size() * sizeof(float);
        data.insert(data.end(), begin, end);
    }

    rearCloud.width = points.size();
    rearCloud.row_step = points.size() * rearCloud.point_step;
    rearCloud.data = data;
    cloudPublisher.publish(rearCloud);
}
}  // namespace zmp
