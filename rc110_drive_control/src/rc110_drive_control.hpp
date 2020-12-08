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

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <rc110_msgs/Offsets.h>

#include <string>
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
        std::string baseFrameId = "base_link";
        std::string imuFrameId = "imu_link";
        double rate = 30.0;  // Hz
    };

public:
    Rc110DriveControl(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate);
    ~Rc110DriveControl();

private:
    bool onEnableBoard(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
    bool onMotorState(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
    bool onServoState(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
    void onDrive(const ackermann_msgs::AckermannDriveStamped::ConstPtr& message);
    void onOffsets(const rc110_msgs::Offsets::ConstPtr& message);
    void onStatusUpdateTimer(const ros::TimerEvent&);

    void publishErrors();
    void getAndPublishRobotStatus();
    void getAndPublishDriveInfo();
    void getAndPublishServoInfo();
    void getAndPublishImu();
    void getAndPublishBaseboardTemperature();
    void getAndPublishMotorBattery();
    void getAndPublishOtherSensors();

    void publishDriveStatus(const DriveInfo& drive);
    void publishOdometry(const DriveInfo& drive);
    static void publishTemperature(ros::Publisher& publisher, float temperature);
    static void publishBattery(ros::Publisher& publisher, float voltage, float current);

private:
    Parameters parameters;
    RcControl control;
    std::vector<ros::ServiceServer> services;
    std::vector<ros::Subscriber> subscribers;
    std::map<std::string, ros::Publisher> publishers;
    ros::Timer statusUpdateTimer;
    std::atomic<BaseboardError> baseboardError = BaseboardError::NONE;
    BaseboardError lastBaseboardError = BaseboardError::NONE;
};
}  // namespace zmp
