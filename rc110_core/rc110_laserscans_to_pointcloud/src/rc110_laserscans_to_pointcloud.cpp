/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "rc110_laserscans_to_pointcloud.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace zmp
{
constexpr char FRONT_LIDAR_FRAME_ID[] = "front_lidar";
constexpr char REAR_LIDAR_FRAME_ID[] = "rear_lidar";

using namespace std::chrono_literals;

namespace
{
/*
 * Add points to map sorted by distance.
 */
void addPointsByDistance(const sensor_msgs::msg::PointCloud2& cloud, std::map<float, std::vector<float>>& pointsByDistance)
{
    int pointSize = cloud.point_step / sizeof(float);
    int cloudSize = cloud.row_step * cloud.height / sizeof(float);
    auto data = reinterpret_cast<const float*>(cloud.data.data());

    for (auto it = data; it < data + cloudSize; it += pointSize) {
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
        Node("laserscans_to_pointcloud"),
        outputFrameId(declare_parameter("output_frame_id", REAR_LIDAR_FRAME_ID)),
        frontLidarFrameId(declare_parameter("front_lidar_frame_id", FRONT_LIDAR_FRAME_ID)),
        rearLidarFrameId(declare_parameter("rear_lidar_frame_id", REAR_LIDAR_FRAME_ID)),
        angleThreshold(declare_parameter("angle_threshold", 0.9)),  // +90% / -90% of angle_increment
        tfBuffer(get_clock()),
        tfListener(tfBuffer)
{
    sync = std::make_unique<ApproxSync>(ApproxSyncPolicy(3), frontSubscriber, rearSubscriber);
    sync->registerCallback(&Rc110LaserScansToPointCloud::onScans, this);

    cloudPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("points2", 1);
    timer = create_wall_timer(500ms, [this] { onTimer(); });
}

void Rc110LaserScansToPointCloud::onTimer()
{
    if (cloudPublisher->get_subscription_count()) {
        if (!subscribed) {
            subscribed = true;
            frontSubscriber.subscribe(this, "front_scan");
            rearSubscriber.subscribe(this, "rear_scan");
        }
    } else {
        subscribed = false;
        frontSubscriber.unsubscribe();
        rearSubscriber.unsubscribe();
    }
}

void Rc110LaserScansToPointCloud::onScans(const sensor_msgs::msg::LaserScan::ConstSharedPtr& front,
                                          const sensor_msgs::msg::LaserScan::ConstSharedPtr& rear)
{
    if (front->header.frame_id != frontLidarFrameId || rear->header.frame_id != rearLidarFrameId) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "Wrong scan frame id: " << front->header.frame_id << ", " << rear->header.frame_id);
        return;
    }

    sensor_msgs::msg::PointCloud2 frontCloud, rearCloud;
    projection.transformLaserScanToPointCloud(outputFrameId, *front, frontCloud, tfBuffer);
    projection.transformLaserScanToPointCloud(outputFrameId, *rear, rearCloud, tfBuffer);

    // points sorted by distance from rear lidar
    // rear lidar as main gives a bit better result as coordinates origin is near it
    std::map<float, std::vector<float>> pointsByDistance;
    addPointsByDistance(frontCloud, pointsByDistance);
    addPointsByDistance(rearCloud, pointsByDistance);

    // unique and sorted by angle points (-pi, pi]
    // all points behind other points relative to rear lidar are discarded
    std::map<float, std::vector<float>> points;
    auto angleAllowance = float(rear->angle_increment * angleThreshold);
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
    cloudPublisher->publish(rearCloud);
}
}  // namespace zmp
