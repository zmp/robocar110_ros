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

#include <zmp/RcControl.hpp>

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
    };

public:
    Rc110DriveControl(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate);
    ~Rc110DriveControl();

private:
    void onDrive(const ackermann_msgs::AckermannDrive& message);
    void onStatusUpdateTimer(const ros::TimerEvent&);

    void publishDriveStatus(const DriveInfo& drive);
    void publishOdometry(const DriveInfo& drive);
    void getAndPublishImu();
    void getAndPublishServoTemperature();
    void getAndPublishBaseboardTemperature();
    void publishTemperature(float temperature, ros::Publisher& publisher);
    void getAndPublishBattery();

private:
    Parameters parameters;
    RcControl control;
    ros::Subscriber driveSubscriber;
    ros::Publisher driveStatusPublisher;
    ros::Publisher imuPublisher;
    ros::Publisher servoTemperaturePublisher;
    ros::Publisher baseboardTemperaturePublisher;
    ros::Publisher motorBatteryPublisher;
    ros::Publisher odometryPublisher;
    ros::Timer statusUpdateTimer;
};
}  // namespace zmp
