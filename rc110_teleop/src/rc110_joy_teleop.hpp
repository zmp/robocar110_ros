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
#include <rc110_msgs/Status.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace zmp
{
/**
 * A node that handles joystick message and provides drive message based on it.
 * Also it can enable/disable baseboard and AD mode.
 */
class Rc110JoyTeleop
{
public:
    struct Param {
        int deadManButton = 4;
        int gearUpButton = 6;
        int gearDownButton = 7;
        int boardButton = 10;
        int adButton = 11;
        int steeringAxis = 3;
        int speedAxis = 1;
        double maxSteeringAngleRad = angles::from_degrees(30);

        // steering angle of the car = 0.7 * steering angle of servo due to the hardware
        // from the hardware document, the operation speed is 0.11 / 60 [sec/degree]
        double maxSteeringAngleVelRad = angles::from_degrees(381.81);  // 60 * 0.7 / 0.11

        std::vector<double> gears = { 0.3, 0.6, 1.0 };
        std::string frameId = "base_link";
        double rate = 30.0;  // Hz
    };

public:
    Rc110JoyTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    void publishDrive();
    void updateToggles(const sensor_msgs::Joy::ConstPtr& message);
    bool checkButtonClicked(const sensor_msgs::Joy::ConstPtr& message, int button);

    void onJoy(const sensor_msgs::Joy::ConstPtr& message);
    void onRobotStatus(const rc110_msgs::Status& message);
    void onAdModeChanged(const std_msgs::String& message);

private:
    Param m_param;

    std::vector<ros::Subscriber> m_subscribers;
    ros::Publisher m_drivePub;
    sensor_msgs::Joy::ConstPtr m_joyMessage;
    ackermann_msgs::AckermannDriveStamped m_driveMessage;
    ros::Timer m_timer;
    std::map<int, int> m_axisDirection;  /// +1 or -1 for inverted axis
    int m_gear = 0;
    bool m_stopMessagePublished = false;
    bool m_deadManPressed = false;
    bool m_boardEnabled = false;
    bool m_adEnabled = false;
};
}  // namespace zmp
