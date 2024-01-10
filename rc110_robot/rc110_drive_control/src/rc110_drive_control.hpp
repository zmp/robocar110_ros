/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#pragma once

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rc110_msgs/msg/offsets.hpp>
#include <rc110_msgs/srv/set_integer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <string>
#include <zmp/RcControl.hpp>

namespace zmp
{
/**
 * Node for interfacing with vehicle baseboard to control speed & steering,
 * and to get various vehicle status & sensor data for publishing.
 */
class Rc110DriveControl : public rclcpp::Node
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
    bool onEnableBoard(std_srvs::srv::SetBool::Request& request, std_srvs::srv::SetBool::Response& response);
    bool onTeleopPing(std_srvs::srv::SetBool::Request& request, std_srvs::srv::SetBool::Response& response);
    bool onTeleopStatus(std_srvs::srv::Trigger::Request& request, std_srvs::srv::Trigger::Response& response);
    bool onMotorState(rc110_msgs::srv::SetInteger::Request& request, rc110_msgs::srv::SetInteger::Response& response);
    bool onServoState(rc110_msgs::srv::SetInteger::Request& request, rc110_msgs::srv::SetInteger::Response& response);
    void onDrive(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr& message);
    void onServoTorque(const std_msgs::msg::Float32::ConstSharedPtr& message);
    void onOffsets(const rc110_msgs::msg::Offsets::ConstSharedPtr& message);
    void onStatusUpdateTimer();

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
    void publishTemperature(const std::string& topic, float temperature);
    void publishBattery(const std::string& topic, float voltage, float current);
    void publishFloat32(const std::string& topic, float value);

    template <typename T>
    void publish(const std::string& topic, const T& value)
    {
        std::static_pointer_cast<rclcpp::Publisher<T>>(publishers[topic])->publish(value);
    }

private:
    Parameters parameters;
    RcControl control;
    float wheelBase;
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers;
    std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers;
    tf2_ros::TransformBroadcaster odometryBroadcaster;
    rclcpp::TimerBase::SharedPtr statusUpdateTimer;
    std::atomic<BaseboardError> baseboardError = BaseboardError::NONE;
    BaseboardError lastBaseboardError = BaseboardError::NONE;
    nav_msgs::msg::Odometry odometry;
    double estimatedYaw = 0;
    rclcpp::Time lastTeleopPing = {0, 0, RCL_ROS_TIME};
};

// short version of create_subscription, similar to ROS1
template <typename T, typename Class>
auto subscribe(const std::string& topic, int depth, void (Class::*callback)(const T*), Class* object)
{
    return object->template create_subscription<T>(topic, depth, [object, callback](typename T::ConstSharedPtr in) {
        (object->*callback)(in.get());
    });
}

}  // namespace zmp
