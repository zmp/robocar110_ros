/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Paulo Morales
 */
#pragma once

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rc110_msgs/msg/baseboard_error.hpp>
#include <rc110_msgs/msg/motor_rate.hpp>
#include <rc110_msgs/msg/status.hpp>
#include <rc110_msgs/msg/wheel_speeds.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/float32.hpp>
namespace zmp
{
class Rc110Gazebo : public rclcpp::Node
{
public:
    struct Parameters 
    {
        std::string baseFrameId = "";
        double rate_hz = 0.0;
    };

public:
    Rc110Gazebo();

private:
    bool onTeleopPing(std_srvs::srv::SetBool::Request& request, std_srvs::srv::SetBool::Response& response);
    bool onTeleopStatus(std_srvs::srv::Trigger::Request& request, std_srvs::srv::Trigger::Response& response);
    void onDrive(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr& drive);
    void onStatusUpdateTimer();

    void getAndPublishRobotStatus();
    void getAndPublishDriveInfo();
    void getAndPublishServoInfo();
    void getAndPublishBaseboardTemperature();
    void getAndPublishMotorBattery();
    void getAndPublishOtherSensors();

    sensor_msgs::msg::Temperature createTemperatureMsg(const float& temperature);
    sensor_msgs::msg::BatteryState createBatteryMsg(const float& voltage, const float& current);
    template <typename T>
    inline void publish(const std::string& topic, const T& value)
    {
        std::static_pointer_cast<rclcpp::Publisher<T>>(publishers[topic])->publish(value);
    }

private:
    Parameters parameters;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> onTeleopPingSrv;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> onTeleopStatusSrv;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr onDriveSub;
    std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers;
    rclcpp::TimerBase::SharedPtr statusUpdateTimer;
    float speed = 0;
    float steeringAngle = 0;
    float steeringAngleVelocity = 0;
    rclcpp::Time lastDriveTime = {0, 0, RCL_ROS_TIME};
    rclcpp::Time lastTeleopPing = {0, 0, RCL_ROS_TIME};
};
}  // namespace zmp