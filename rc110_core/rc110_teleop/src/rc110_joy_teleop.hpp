/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by btran, Andrei Pak
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
        std::string rc = "zmp";
        std::string frameId = "base_link";
        std::vector<double> gears = {0.3, 0.6, 1.0};
        double rate = 30.0;  // Hz
        std::string joyType;
        std::map<std::string, std::string> joyTypes;

        // dynamic
        int deadManButton = 4;
        int nextRobotButton = 5;
        int gearUpButton = 6;
        int gearDownButton = 7;
        int boardButton = 10;
        int adButton = 11;
        std::vector<double> steering = {3};
        int steeringAuxiliary = 2;  // auxiliary axis for better precision
        std::vector<double> accel = {1};
    };

public:
    Rc110JoyTeleop();

private:
    void setupJoystick(const std::string& device);
    std::string getJoyType(const std::string& device);
    std::string joyNameToType(const std::string& joyName);
    void setupRosConnections();
    void updateAxis(std::vector<double>& axis, double defaultMax);
    void publishDrive();
    void updateToggles(const sensor_msgs::Joy::ConstPtr& message);
    void onRobotNameTimer();
    void incrementRobotName();
    void setupRobotName(const std::string& name);
    void publishRobotName();
    bool checkButtonClicked(const sensor_msgs::Joy::ConstPtr& message, int button);
    bool checkAxisChanged(const sensor_msgs::Joy::ConstPtr& message, int axis);
    float getAxisValue(const sensor_msgs::Joy::ConstPtr& message, int axis);
    float mapAxisValue(const std::vector<double>& axis, float value);

    void onJoy(const sensor_msgs::Joy::ConstPtr& message);
    void onRobotStatus(const rc110_msgs::Status& message);
    void onAdModeChanged(const std_msgs::String& message);

private:
    ros::NodeHandle handle;
    Param param;

    std::map<std::string, ros::Subscriber> subscribers;
    std::map<std::string, ros::Publisher> publishers;
    sensor_msgs::Joy::ConstPtr joyMessage;
    ackermann_msgs::AckermannDriveStamped driveMessage;
    ros::Timer driveTimer, nextRobotTimer;
    ros::Time lastTime;
    std::map<int, int> axisDirection;   /// +1 or -1 for inverted axis
    std::map<int, bool> axisActivated;  /// joy_node workaround
    int gear = 0;
    bool stopMessagePublished = false;
    bool boardEnabled = false;
    bool adEnabled = false;
    std::string joyDevice = "/dev/<unset>";
    std::string selectedRobot;
    ros::V_string robotNames;
};
}  // namespace zmp
