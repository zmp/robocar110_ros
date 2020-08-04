/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Andrei Pak
 */
#pragma once

#include <ackermann_msgs/AckermannDrive.h>
#include <ros/ros.h>

#include "RcControl.h"

namespace zmp
{
class Rc110DriveControl
{
public:
    struct Parameters {
        std::string rs232_device;
        std::string rs485_device;
    };

public:
    Rc110DriveControl(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate);
    ~Rc110DriveControl();

private:
    void onDrive(const ackermann_msgs::AckermannDrive& message);
    void onTwistTimer(const ros::TimerEvent&);

private:
    zrc::RcControl control;
    Parameters parameters;
    ros::Subscriber driveSubscriber;
    ros::Publisher twistPublisher;
    ros::Timer twistTimer;
};
}  // namespace zmp