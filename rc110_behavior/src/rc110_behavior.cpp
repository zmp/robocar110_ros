/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Andrei Pak
 */
#include "rc110_behavior.hpp"

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <laser_geometry/laser_geometry.h>

#include "nodes/drive_action.hpp"
#include "nodes/get_obstacle.hpp"

namespace zmp
{
Rc110Behavior::Rc110Behavior(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate) :
        parameters({.treeFile = handlePrivate.param<std::string>("tree_file", "behavior.xml"),
                    .frameId = handlePrivate.param<std::string>("frame_id", "rc110_base")})
{
    laserSubscriber = handle.subscribe("scan", 1, &Rc110Behavior::onLaser, this);
    drivePublisher = handle.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 1);

    registerNodeBuilder<GetObstacle>(std::cref(cloud));
    registerNodeBuilder<DriveAction>(std::ref(drivePublisher));

    behaviorTree = behaviorTreeFactory.createTreeFromFile(parameters.treeFile);
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
