/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */

#include "rc110_twist_to_ackermann.hpp"

namespace zmp
{
Rc110TwistToAckermann::Rc110TwistToAckermann() : rclcpp::Node("rc110_twist_to_ackermann")
{
    using std::placeholders::_1;
    twistSubscriber =
            create_subscription<geometry_msgs::msg::Twist>("cmd_vel",
                                                           2,
                                                           std::bind(&Rc110TwistToAckermann::onTwist, this, _1));
    drivePublisher = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive_ad", rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal));
}

void Rc110TwistToAckermann::onTwist(const geometry_msgs::msg::Twist::ConstSharedPtr& twist)
{
    ackermann_msgs::msg::AckermannDriveStamped drive;
    drive.header.stamp = now();
    drive.drive.speed = float(twist->linear.x);
    drive.drive.steering_angle = float(twist->angular.z);
    drivePublisher->publish(drive);
}

}  // namespace zmp
