/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Andrei Pak
 */
#include "check_obstacle.hpp"

#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <Eigen/Geometry>

namespace zmp
{
BT::NodeStatus CheckObstacle::tick()
{
    static Eigen::AlignedBox2f nearBox = {Eigen::Vector2f(0, -0.2f), Eigen::Vector2f(0.2f, 0.2f)};
    static Eigen::AlignedBox2f farLeftBox = {Eigen::Vector2f(0.2f, 0), Eigen::Vector2f(1.0f, 0.3f)};
    static Eigen::AlignedBox2f farRightBox = {Eigen::Vector2f(0.2f, -0.3f), Eigen::Vector2f(1.0f, 0)};

    if (cloud.data.empty()) {
        return BT::NodeStatus::FAILURE;
    }

    std::string result = "none";
    float closestLeftX = std::numeric_limits<float>::infinity();
    float closestRightX = std::numeric_limits<float>::infinity();

    using Iter = sensor_msgs::PointCloud2ConstIterator<float>;
    for (Iter it = {cloud, "x"}; it != it.end(); ++it) {
        float x = it[0], y = it[1];
        Eigen::Vector2f point = {x, -y};  // flip along x axis (temporary solution)

        if (nearBox.contains(point)) {
            result = "near";
            break;
        } else if (farLeftBox.contains(point)) {
            result = "far";
            closestLeftX = x;
        } else if (farRightBox.contains(point)) {
            result = "far";
            closestRightX = x;
        }
    }

    if (result == "far") {
        result = closestLeftX < closestRightX ? "left" : "right";
    }

    setOutput("switch", result);
    return BT::NodeStatus::SUCCESS;
}
}  // namespace zmp