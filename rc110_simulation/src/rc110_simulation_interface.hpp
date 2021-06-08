/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Paulo Morales
 */
#pragma once

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/ros.h>

namespace zmp
{
class Rc110SimulationInterface
{
public:
    Rc110SimulationInterface();

private:
    void onStatusUpdateTimer();
    void onAckermannDrive(const ackermann_msgs::AckermannDriveStamped::ConstPtr& drive);

private:
    ros::NodeHandle handle;
    ros::Timer statusUpdateTimer;
    ros::Subscriber driveSubscriber;
    ros::Publisher twistPublisher;
};
}  // namespace zmp
