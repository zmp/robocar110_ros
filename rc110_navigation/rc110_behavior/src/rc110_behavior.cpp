/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "rc110_behavior.hpp"

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <Eigen/Geometry>

namespace zmp
{
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
        handle(),
        cloudSubscriber(handle.subscribe("points2", 1, &Rc110Behavior::onCloud, this)),
        drivePublisher(handle.advertise<ackermann_msgs::AckermannDriveStamped>("drive_ad", 1))
{
    ros::NodeHandle privateHandle("~");
    privateHandle.param("forward_command", forwardCommand, forwardCommand);
    privateHandle.param("left_command", leftCommand, leftCommand);
    privateHandle.param("right_command", rightCommand, rightCommand);
}

void Rc110Behavior::onCloud(const sensor_msgs::PointCloud2& cloud)
{
    float closestLeftX = INF;
    float closestRightX = INF;
    auto result = forwardCommand;

    using Iter = sensor_msgs::PointCloud2ConstIterator<float>;
    for (Iter it = {cloud, "x"}; it != it.end(); ++it) {
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
        ROS_ERROR_STREAM("Command should have 2 values: speed and steering! Current number of values: " << result.size());
    }
    publishCommand(result[0], result[1]);
}

void Rc110Behavior::publishCommand(float speed, float steering)
{
    ackermann_msgs::AckermannDriveStamped message;
    message.drive.speed = speed;
    message.drive.steering_angle = steering * DEG_TO_RAD;

    drivePublisher.publish(message);
}

}  // namespace zmp
