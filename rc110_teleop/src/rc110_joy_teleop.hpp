/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by btran
 */

#pragma once

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <angles/angles.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <mutex>
#include <string>
#include <vector>

namespace zmp
{
class Rc110JoyTeleop
{
public:
    struct Param {
        int deadManButton = 4;
        int steeringAxis = 2;
        int speedAxis = 1;
        double maxSteeringAngleRad = angles::from_degrees(30);

        // steering angle of the car = 0.7 * steering angle of servo due to the hardware
        // from the hardware document, the operation speed is 0.11 / 60 [sec/degree]
        double maxSteeringAngleVelRad = angles::from_degrees(381.81);  // 60 * 0.7 / 0.11

        double maxSpeed = 2.7;  // [m/s]
        std::string frameId = "rc110_base";
        double rate = 30.0;  // Hz
    };

public:
    Rc110JoyTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~Rc110JoyTeleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joyMsg);
    void publish();

private:
    Param m_param;

    ros::Subscriber m_joySub;
    ros::Publisher m_drivePub;
    bool m_deadmanPressed;
    bool m_stopMessagePublished;
    ackermann_msgs::AckermannDriveStamped m_lastMessage;
    std::mutex m_publishMutex;
    ros::Timer m_timer;
};
}  // namespace zmp
