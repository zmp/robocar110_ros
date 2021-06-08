/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Paulo Morales
 */
#include "rc110_simulation_interface.hpp"

#include <geometry_msgs/Twist.h>

#include <cmath>

namespace
{
constexpr double UPDATE_RATE = 30.0;
constexpr double WHEEL_BASE = 0.26;
}  // namespace

namespace zmp
{
Rc110SimulationInterface::Rc110SimulationInterface()
{
    driveSubscriber = handle.subscribe("drive", 2, &Rc110SimulationInterface::onAckermannDrive, this);
    twistPublisher = handle.advertise<geometry_msgs::Twist>("drive_twist", 1);

    statusUpdateTimer = handle.createTimer(ros::Duration(1 / UPDATE_RATE),
                                           [this](const ros::TimerEvent&) { onStatusUpdateTimer(); });
}

void Rc110SimulationInterface::onStatusUpdateTimer()
{
    // TODO:
    // motor_speed_goal
    // steering_angle_goal
    // baseboard_error
    // robot_status
    // drive_status
    // offsets_status
    // servo_temperature
    // baseboard_temperature
    // servo_battery
    // motor_battery
    // motor_rate
    // wheel_speeds
}

void Rc110SimulationInterface::onAckermannDrive(const ackermann_msgs::AckermannDriveStamped::ConstPtr& drive)
{
    auto speed = drive->drive.speed;
    auto steeringAngle = drive->drive.steering_angle;

    geometry_msgs::Twist twist;
    twist.linear.x = speed;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    // https://answers.ros.org/question/93531/cmd_velgeometry_msgstwist-to-ackermann_msgs/
    twist.angular.z = (speed / WHEEL_BASE) * std::tan(steeringAngle);

    twistPublisher.publish(twist);
}

}  // namespace zmp
