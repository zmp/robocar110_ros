/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "rc110_twist_to_ackermann.hpp"

namespace zmp
{
Rc110TwistToAckermann::Rc110TwistToAckermann()
{
    twistSubscriber = handle.subscribe("cmd_vel", 2, &Rc110TwistToAckermann::onTwist, this);
    drivePublisher = handle.advertise<ackermann_msgs::AckermannDriveStamped>("drive_ad", 1);
}

void Rc110TwistToAckermann::onTwist(const geometry_msgs::Twist::ConstPtr& twist)
{
    ackermann_msgs::AckermannDriveStamped drive;
    drive.header.stamp = ros::Time::now();
    drive.drive.speed = float(twist->linear.x);
    drive.drive.steering_angle = float(twist->angular.z);
    drivePublisher.publish(drive);
}

}  // namespace zmp
