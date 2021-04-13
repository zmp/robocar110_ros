/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace zmp
{
/**
 * Twist to AckermannDriveStamped.
 * The Twist message is just a representation of Ackermann.
 * See: teb_local_planner : cmd_angle_instead_rotvel for details.
 */
class Rc110TwistToAckermann
{
public:
    Rc110TwistToAckermann();

private:
    void onTwist(const geometry_msgs::Twist::ConstPtr& twist);

private:
    ros::NodeHandle handle;
    ros::Subscriber twistSubscriber;
    ros::Publisher drivePublisher;
};
}  // namespace zmp
