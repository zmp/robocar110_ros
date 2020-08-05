/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Andrei Pak
 */
#include "drive_action.hpp"

#include <ackermann_msgs/AckermannDrive.h>

namespace zmp
{
BT::NodeStatus DriveAction::tick()
{
    auto speed = getInput<std::string>("speed");
    auto steering = getInput<std::string>("steering");
    if (!speed || !steering) {
        return BT::NodeStatus::FAILURE;
    }

	ackermann_msgs::AckermannDrive drive;
    drive.speed = std::stof(speed.value());
    drive.steering_angle = std::stof(steering.value());

    drivePublisher.publish(drive);
    return BT::NodeStatus::SUCCESS;
}
}  // namespace zmp