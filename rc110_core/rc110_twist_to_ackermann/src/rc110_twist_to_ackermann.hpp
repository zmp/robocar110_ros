/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace zmp
{
/**
 * Twist to AckermannDriveStamped.
 * The Twist message is just a representation of Ackermann.
 * See: teb_local_planner : cmd_angle_instead_rotvel for details.
 */
class Rc110TwistToAckermann : public rclcpp::Node
{
public:
    Rc110TwistToAckermann();

private:
    void onTwist(const geometry_msgs::msg::Twist::ConstSharedPtr& twist);

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSubscriber;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drivePublisher;
};
}  // namespace zmp
