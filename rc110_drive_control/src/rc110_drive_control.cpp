/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Andrei Pak
 */
#include "rc110_drive_control.hpp"

#include <geometry_msgs/Twist.h>

namespace zmp
{
Rc110DriveControl::Rc110DriveControl(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate) :
        parameters({.rs232_device = handlePrivate.param<std::string>("rs232_device", "/dev/ttyUSB1"),
                    .rs485_device = handlePrivate.param<std::string>("rs485_device", "/dev/ttyUSB0")})
{
    control.init();
    control.Start();

    control.SetReportFlagReq(0b1111);  // report: sensor, obstacle, power, ?
    control.SetServoEnable(1);
    control.SetMotorEnableReq(1);
    control.SetDriveSpeed(0);
    control.SetSteerAngle(0);

    driveSubscriber = handle.subscribe("drive", 10, &Rc110DriveControl::onDrive, this);
    twistPublisher = handle.advertise<geometry_msgs::Twist>("twist", 10);

    twistTimer = handle.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent& event) { onTwistTimer(event); });

    ROS_INFO("RC 1/10 drive control started");
}

Rc110DriveControl::~Rc110DriveControl()
{
    // to fix: endless loop
    //    control.Stop();
    //    control.Close();
}

void Rc110DriveControl::onDrive(const ackermann_msgs::AckermannDrive& message)
{
    control.SetDriveSpeed(message.speed);
    control.SetSteerAngle(message.steering_angle);
}

void Rc110DriveControl::onTwistTimer(const ros::TimerEvent&)
{
    int speed = 0;
    if (!control.GetPresentSpeed(&speed)) {
        return;
    }
    float angle = 0;
    if (!control.GetPresentAngle(&angle)) {
        return;
    }

    geometry_msgs::Twist twist;
    twist.linear.x = speed;
    twist.angular.z = angle;
    twistPublisher.publish(twist);
}
}  // namespace zmp