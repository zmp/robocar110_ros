/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "rc110_behavior.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Geometry>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace zmp
{
using namespace std::chrono_literals;

namespace
{
constexpr float DEG_TO_RAD = M_PI / 180;
constexpr float INF = std::numeric_limits<float>::infinity();

// coordinates relative to robot origin, i.e. center of rear wheels axis
const Eigen::AlignedBox2f nearBox = {Eigen::Vector2f(0.2, -0.15f), Eigen::Vector2f(0.5f, 0.15f)};
const Eigen::AlignedBox2f leftBox = {Eigen::Vector2f(0.5f, 0), Eigen::Vector2f(1.0f, 0.35f)};
const Eigen::AlignedBox2f rightBox = {Eigen::Vector2f(0.5f, -0.35f), Eigen::Vector2f(1.0f, 0)};
}  // namespace

Rc110Behavior::Rc110Behavior() :
        Node("rc110_behavior"),
        cloudSubscriber(create_subscription<sensor_msgs::msg::PointCloud2>(
                "points2",
                1,
                std::bind(&Rc110Behavior::onCloud, this, std::placeholders::_1))),
        drivePublisher(create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_ad", 1)),
        tfBuffer(get_clock()),
        tfListener(tfBuffer),
        forwardCommand(declare_parameter("forward_command", forwardCommand)),
        leftCommand(declare_parameter("left_command", leftCommand)),
        rightCommand(declare_parameter("right_command", rightCommand))
{
}

void Rc110Behavior::onCloud(const sensor_msgs::msg::PointCloud2& cloud)
{
    if (baseTransform.child_frame_id.empty()) {
        auto frameId = cloud.header.frame_id;
        auto prefix = frameId.substr(0, frameId.find('/'));
        try {
            baseTransform = tfBuffer.lookupTransform(frameId, prefix + "/base_link", now(), 5s);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "%s", ex.what());
            return;
        }
    }
    sensor_msgs::msg::PointCloud2 baseCloud;
    tf2::doTransform(cloud, baseCloud, baseTransform);

    float closestLeftX = INF;
    float closestRightX = INF;
    auto result = forwardCommand;

    using Iter = sensor_msgs::PointCloud2ConstIterator<float>;
    for (Iter it = {baseCloud, "x"}; it != it.end(); ++it) {
        float x = it[0], y = it[1];
        Eigen::Vector2f point = {x, y};

        if (nearBox.contains(point)) {
            result = stopCommand;
            break;
        } else if (leftBox.contains(point)) {
            closestLeftX = std::min(x, closestLeftX);
        } else if (rightBox.contains(point)) {
            closestRightX = std::min(x, closestRightX);
        }
    }

    if (result == forwardCommand) {
        if (closestLeftX != INF || closestRightX != INF) {
            result = closestLeftX < closestRightX ? rightCommand : leftCommand;
        }
    }
    if (result.size() != 2) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "Command should have 2 values: speed and steering! Current number of values: "
                                    << result.size());
        return;
    }
    publishCommand(float(result[0]), float(result[1]));
}

void Rc110Behavior::publishCommand(float speed, float steering)
{
    ackermann_msgs::msg::AckermannDriveStamped message;
    message.drive.speed = speed;
    message.drive.steering_angle = steering * DEG_TO_RAD;

    drivePublisher->publish(message);
}

}  // namespace zmp
