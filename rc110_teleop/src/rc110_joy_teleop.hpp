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
