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

#include <ackermann_msgs/AckermannDrive.h>
#include <laser_geometry/laser_geometry.h>

#include "nodes/drive_action.hpp"
#include "nodes/get_obstacle.hpp"

namespace zmp
{
Rc110Behavior::Rc110Behavior(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate) :
        parameters({.treeFile = handlePrivate.param<std::string>("tree_file", "behavior.xml")})
{
    laserSubscriber = handle.subscribe("scan", 1, &Rc110Behavior::onLaser, this);
    drivePublisher = handle.advertise<ackermann_msgs::AckermannDrive>("drive", 1);

    registerNodeBuilder<GetObstacle>(std::cref(cloud));
    registerNodeBuilder<DriveAction>(std::ref(drivePublisher));

    behaviorTree = behaviorTreeFactory.createTreeFromFile(parameters.treeFile);
}

void Rc110Behavior::update()
{
    behaviorTree.tickRoot();
}

void Rc110Behavior::onLaser(const sensor_msgs::LaserScan& scan)
{
    laser_geometry::LaserProjection projection;
    projection.projectLaser(scan, cloud);
}

}  // namespace zmp