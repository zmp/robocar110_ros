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
/**
 * Node for interfacing with vehicle baseboard to control speed & steering,
 * and to get various vehicle status & sensor data for publishing.
 */
class Rc110DriveControl
{
public:
    struct Parameters {
        std::string rs232_device;
        std::string rs485_device;
        float steeringOffset;
    };

public:
    Rc110DriveControl(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate);
    ~Rc110DriveControl();

private:
    void onDrive(const ackermann_msgs::AckermannDrive& message);
    void onStatusUpdateTimer(const ros::TimerEvent&);

    void getAndPublishDriveStatus();
    void getAndPublishImu();
    void getAndPublishServoTemperature();
    void getAndPublishBaseboardTemperature();
    void publishTemperature(float temperature, ros::Publisher& publisher);
    void getAndPublishBattery();

private:
    zrc::RcControl control;
    Parameters parameters;
    ros::Subscriber driveSubscriber;
    ros::Publisher driveStatusPublisher;
    ros::Publisher imuPublisher;
    ros::Publisher servoTemperaturePublisher;
    ros::Publisher baseboardTemperaturePublisher;
    ros::Publisher motorBatteryPublisher;
    ros::Timer statusUpdateTimer;
};
}  // namespace zmp
