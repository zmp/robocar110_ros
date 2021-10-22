/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
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
        std::vector<double> steering = {3};
        int steeringAuxiliary = -1;  // auxiliary axis for better precision
        std::vector<double> accel = {1};

        std::string frameId = "base_link";
        std::vector<double> gears = {0.3, 0.6, 1.0};
        double rate = 30.0;  // Hz
    };

public:
    Rc110JoyTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    void updateAxis(std::vector<double>& axis, double defaultMax);
    void publishDrive();
    void updateToggles(const sensor_msgs::Joy::ConstPtr& message);
    bool checkButtonClicked(const sensor_msgs::Joy::ConstPtr& message, int button);
    bool checkAxisChanged(const sensor_msgs::Joy::ConstPtr& message, int axis);
    float getAxisValue(const sensor_msgs::Joy::ConstPtr& message, int axis);
    float mapAxisValue(const std::vector<double>& axis, float value);

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
    ros::Time m_lastTime;
    std::map<int, int> m_axisDirection;   /// +1 or -1 for inverted axis
    std::map<int, bool> m_axisActivated;  /// joy_node workaround
    int m_gear = 0;
    bool m_stopMessagePublished = false;
    bool m_publishingEnabled = false;
    bool m_boardEnabled = false;
    bool m_adEnabled = false;
};
}  // namespace zmp
