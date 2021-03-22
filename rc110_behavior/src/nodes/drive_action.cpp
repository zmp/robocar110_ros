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
        return BT::NodeStatus::FAILURE;
    }

    ackermann_msgs::AckermannDriveStamped cmdMsg;
    cmdMsg.drive.speed = speed.value();
    cmdMsg.drive.steering_angle = static_cast<float>(steering.value()) * DEG_TO_RAD;

    drivePublisher.publish(cmdMsg);
    return BT::NodeStatus::SUCCESS;
}
}  // namespace zmp
