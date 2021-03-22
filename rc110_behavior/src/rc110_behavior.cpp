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
#include "rc110_behavior.hpp"

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <laser_geometry/laser_geometry.h>

#include "nodes/check_obstacle.hpp"
#include "nodes/drive_action.hpp"

namespace zmp
{
Rc110Behavior::Rc110Behavior(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate) :
        parameters({.treeFile = handlePrivate.param<std::string>("tree_file", "behavior.xml")})
{
    laserSubscriber = handle.subscribe("scan", 1, &Rc110Behavior::onLaser, this);
    drivePublisher = handle.advertise<ackermann_msgs::AckermannDriveStamped>("drive_ad", 1);

    registerNodeBuilder<CheckObstacle>(std::cref(cloud));
    registerNodeBuilder<DriveAction>(std::ref(drivePublisher));

    try {
        behaviorTree = behaviorTreeFactory.createTreeFromFile(parameters.treeFile);
    } catch (std::runtime_error& e) {
        throw std::runtime_error("Unable to find tree file in: " + parameters.treeFile);
    }
}

void Rc110Behavior::update()
{
    if (behaviorTree.tickRoot() == BT::NodeStatus::FAILURE) {
        ROS_INFO_THROTTLE(10, "Behavior tree failed");
    }
}

void Rc110Behavior::onLaser(const sensor_msgs::LaserScan& scan)
{
    laser_geometry::LaserProjection projection;
    projection.projectLaser(scan, cloud);
}

}  // namespace zmp
