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

#include <map>
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
        int gearUpButton = 6;
        int gearDownButton = 7;
        int steeringAxis = 3;
        int speedAxis = 1;
        double maxSteeringAngleRad = angles::from_degrees(30);

        // steering angle of the car = 0.7 * steering angle of servo due to the hardware
        // from the hardware document, the operation speed is 0.11 / 60 [sec/degree]
        double maxSteeringAngleVelRad = angles::from_degrees(381.81);  // 60 * 0.7 / 0.11

        double maxSpeed = 2.8;  // [m/s]
        std::string frameId = "rc110_base";
        double rate = 30.0;  // Hz
    };

public:
    Rc110JoyTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joyMsg);
    void publish();

private:
    Param m_param;

    ros::Subscriber m_joySub;
    ros::Publisher m_drivePub;
    bool m_deadmanPressed = false;
    bool m_gearUpPressed = false;
    bool m_gearDownPressed = false;
    bool m_stopMessagePublished = false;
    ackermann_msgs::AckermannDriveStamped m_lastMessage;
    ros::Timer m_timer;
    std::map<int, int> m_axisDirection;  /// +1 or -1 for inverted axis
    int m_gear = 0;
};
}  // namespace zmp
