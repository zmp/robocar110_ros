/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
