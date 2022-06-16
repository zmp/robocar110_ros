/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by btran, Andrei Pak
 */

#pragma once

#include <angles/angles.h>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <map>
#include <mutex>
#include <rc110_msgs/msg/status.hpp>
#include <rc110_topic_tools/srv/mux_select.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <vector>

namespace zmp
{
/**
 * A node that handles joystick message and provides drive message based on it.
 * Also it can enable/disable baseboard and AD mode.
 */
class Rc110JoyTeleop : public rclcpp::Node
{
public:
    struct Parameters {
        std::string rc;
        std::string frameId;
        std::vector<double> gears;
        double rate;
        std::string joyPath;
        std::string joyType;

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
    void setupJoystick(const std::string& joyPath);
    std::string getJoyType(const std::string& joyPath) const;
    std::string getJoyDescription(const std::string& joyPath) const;
    std::map<std::string, std::string> getTypeConfigs() const;
    int matchDescription(const std::string& description, const std::string& config) const;
    void setupRosConnections();
    void updateAxis(std::vector<double>& axis, double defaultMax);
    void publishDrive();
    void updateToggles(const sensor_msgs::msg::Joy::ConstSharedPtr& message);
    void onRobotNameTimer();
    void incrementRobotName();
    void setupRobotName(const std::string& name);
    void connectRobot(const std::vector<std::string>& names);
    void pingRobot();
    bool checkButtonClicked(const sensor_msgs::msg::Joy::ConstSharedPtr& message, int button);
    bool checkAxisChanged(const sensor_msgs::msg::Joy::ConstSharedPtr& message, int axis);
    float getAxisValue(const sensor_msgs::msg::Joy::ConstSharedPtr& message, int axis);
    float mapAxisValue(const std::vector<double>& axis, float value);

    void onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr& message);
    void onRobotStatus(const rc110_msgs::msg::Status& message);
    void onAdModeChanged(const std_msgs::msg::String& message);

    template <typename T>
    void publish(const std::string& topic, const T& value)
    {
        std::static_pointer_cast<rclcpp::Publisher<T>>(publishers[topic])->publish(value);
    }

private:
    Parameters param;
    const std::string configPath;

    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscribers;
    std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> enableBoardService;
    std::shared_ptr<rclcpp::Client<rc110_topic_tools::srv::MuxSelect>> muxDriveService;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> teleopPingService;
    sensor_msgs::msg::Joy::ConstSharedPtr joyMessage;
    ackermann_msgs::msg::AckermannDriveStamped driveMessage;
    rclcpp::TimerBase::SharedPtr driveTimer, nextRobotTimer;
    rclcpp::Time lastTime = {0, 0, RCL_ROS_TIME};
    std::map<int, int> axisDirection;   /// +1 or -1 for inverted axis
    std::map<int, bool> axisActivated;  /// joy_node workaround
    int gear = 0;
    bool stopMessagePublished = false;
    bool boardEnabled = false;
    bool adEnabled = false;
    std::string selectedRobot;
    std::vector<std::string> robotNames;
    std::map<std::string, std::string> joyTypes;
};
}  // namespace zmp
