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
class Rc110Gazebo
{
public:
    struct Parameters {
        std::string baseFrameId;
        double rate;  // Hz
    };

public:
    Rc110Gazebo();

private:
    void onDrive(const ackermann_msgs::AckermannDriveStamped::ConstPtr& drive);
    void onStatusUpdateTimer();

    void getAndPublishRobotStatus();
    void getAndPublishDriveInfo();
    void getAndPublishServoInfo();
    void getAndPublishBaseboardTemperature();
    void getAndPublishMotorBattery();
    void getAndPublishOtherSensors();

    static void publishTemperature(ros::Publisher& publisher, float temperature);
    static void publishBattery(ros::Publisher& publisher, float voltage, float current);

private:
    ros::NodeHandle handle;
    Parameters parameters;
    std::vector<ros::Subscriber> subscribers;
    std::map<std::string, ros::Publisher> publishers;
    ros::Timer statusUpdateTimer;
    float speed = 0;
    float steeringAngle = 0;
    float steeringAngleVelocity = 0;
    double lastDriveTime = 0;
};
}  // namespace zmp
