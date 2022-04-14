/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <rc110_msgs/Offsets.h>
#include <rc110_msgs/SetInteger.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/transform_broadcaster.h>

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
        std::string baseFrameId;
        std::string odomFrameId;
        std::string imuFrameId;
        double rate;                     // Hz
        double odometryOnlyAngleOffset;  // Deg
        double steeringVelocity;         // Deg/s
    };

public:
    Rc110DriveControl();
    ~Rc110DriveControl();

private:
    bool onEnableBoard(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
    bool onMotorState(rc110_msgs::SetInteger::Request& request, rc110_msgs::SetInteger::Response& response);
    bool onServoState(rc110_msgs::SetInteger::Request& request, rc110_msgs::SetInteger::Response& response);
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
    ros::NodeHandle handle;
    Parameters parameters;
    RcControl control;
    float wheelBase;
    std::vector<ros::ServiceServer> services;
    std::vector<ros::Subscriber> subscribers;
    std::map<std::string, ros::Publisher> publishers;
    tf2_ros::TransformBroadcaster odometryBroadcaster;
    ros::Timer statusUpdateTimer;
    std::atomic<BaseboardError> baseboardError = BaseboardError::NONE;
    BaseboardError lastBaseboardError = BaseboardError::NONE;
    nav_msgs::Odometry odometry;
    double estimatedYaw = 0;
};
}  // namespace zmp
