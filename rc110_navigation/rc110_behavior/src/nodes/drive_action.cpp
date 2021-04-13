/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "drive_action.hpp"

#include <ackermann_msgs/AckermannDriveStamped.h>

#include <boost/math/constants/constants.hpp>

namespace zmp
{
constexpr float DEG_TO_RAD = boost::math::float_constants::degree;

BT::NodeStatus DriveAction::tick()
{
    auto speed = getInput<double>("speed");
    auto steering = getInput<double>("steering");
    if (!speed || !steering) {
        ROS_INFO_THROTTLE(10, "Check you set values for speed and steering.");
        return BT::NodeStatus::FAILURE;
    }

    ackermann_msgs::AckermannDriveStamped cmdMsg;
    cmdMsg.drive.speed = speed.value();
    cmdMsg.drive.steering_angle = static_cast<float>(steering.value()) * DEG_TO_RAD;

    drivePublisher.publish(cmdMsg);
    return BT::NodeStatus::SUCCESS;
}
}  // namespace zmp
