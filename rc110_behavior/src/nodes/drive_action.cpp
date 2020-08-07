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
#include <boost/math/constants/constants.hpp>

namespace zmp
{
BT::NodeStatus DriveAction::tick()
{
	using namespace boost::math::float_constants;

    auto speed = getInput<double>("speed");
    auto steering = getInput<double>("steering");
    if (!speed || !steering) {
        return BT::NodeStatus::FAILURE;
    }

	ackermann_msgs::AckermannDrive drive;
    drive.speed = speed.value();
    drive.steering_angle = static_cast<float>(steering.value()) * degree;

    drivePublisher.publish(drive);
    return BT::NodeStatus::SUCCESS;
}
}  // namespace zmp