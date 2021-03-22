/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
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
            closestLeftX = std::min(x, closestLeftX);
        } else if (farRightBox.contains(point)) {
            result = "far";
            closestRightX = std::min(x, closestRightX);
        }
    }

    if (result == "far") {
        result = closestLeftX < closestRightX ? "left" : "right";
    }

    setOutput("switch", result);
    return BT::NodeStatus::SUCCESS;
}
}  // namespace zmp