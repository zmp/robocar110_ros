/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Paulo Morales
 */
#include "rc110_gazebo.hpp"

#include <geometry_msgs/Twist.h>
#include <rc110_msgs/BaseboardError.h>
#include <rc110_msgs/MotorRate.h>
#include <rc110_msgs/Status.h>
#include <rc110_msgs/WheelSpeeds.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Float32.h>

namespace
{
constexpr double WHEEL_BASE = 0.26;
constexpr double TELEOP_PING_TIMEOUT = 0.4;  // sec
}  // namespace

namespace zmp
{
Rc110Gazebo::Rc110Gazebo() :
        parameters({
                .baseFrameId = ros::param::param<std::string>("~base_frame_id", "base_link"),
                .rate = ros::param::param<double>("~rate", 30),
        })
{
    services.push_back(handle.advertiseService("teleop_ping", &Rc110Gazebo::onTeleopPing, this));
    services.push_back(handle.advertiseService("teleop_status", &Rc110Gazebo::onTeleopStatus, this));

    subscribers.push_back(handle.subscribe("drive", 2, &Rc110Gazebo::onDrive, this));

    publishers["drive_twist"] = handle.advertise<geometry_msgs::Twist>("drive_twist", 1);
    publishers["motor_speed_goal"] = handle.advertise<std_msgs::Float32>("motor_speed_goal", 1, true);
    publishers["steering_angle_goal"] = handle.advertise<std_msgs::Float32>("steering_angle_goal", 1, true);

    publishers["baseboard_error"] = handle.advertise<rc110_msgs::BaseboardError>("baseboard_error", 10, true);
    publishers["robot_status"] = handle.advertise<rc110_msgs::Status>("robot_status", 1, true);
    publishers["drive_status"] = handle.advertise<ackermann_msgs::AckermannDriveStamped>("drive_status", 1);

    publishers["servo_temperature"] = handle.advertise<sensor_msgs::Temperature>("servo_temperature", 1);
    publishers["baseboard_temperature"] = handle.advertise<sensor_msgs::Temperature>("baseboard_temperature", 1);
    publishers["servo_battery"] = handle.advertise<sensor_msgs::BatteryState>("servo_battery", 1);
    publishers["motor_battery"] = handle.advertise<sensor_msgs::BatteryState>("motor_battery", 1);
    publishers["motor_rate"] = handle.advertise<rc110_msgs::MotorRate>("motor_rate", 1);
    publishers["wheel_speeds"] = handle.advertise<rc110_msgs::WheelSpeeds>("wheel_speeds", 1);

    statusUpdateTimer = handle.createTimer(ros::Duration(1 / parameters.rate),
                                           [this](const ros::TimerEvent&) { onStatusUpdateTimer(); });
    // publish no error
    publishers["baseboard_error"].publish(rc110_msgs::BaseboardError());
}

bool Rc110Gazebo::onTeleopPing(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
    if (request.data) {
        // allow new connection, if time from the last ping is long enough
        response.success = ros::Time::now() - lastTeleopPing > ros::Duration(TELEOP_PING_TIMEOUT);
        if (!response.success) {
            return false;
        }
    }
    lastTeleopPing = ros::Time::now();
    return true;
}

bool Rc110Gazebo::onTeleopStatus(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
    response.success = ros::Time::now() - lastTeleopPing <= ros::Duration(TELEOP_PING_TIMEOUT);
    return true;
}

void Rc110Gazebo::onDrive(const ackermann_msgs::AckermannDriveStamped::ConstPtr& drive)
{
    double timeDiff = (drive->header.stamp - lastDriveTime).toSec();
    lastDriveTime = drive->header.stamp;

    speed = drive->drive.speed;
    steeringAngleVelocity = timeDiff == 0 ? 0.f : float((drive->drive.steering_angle - steeringAngle) / timeDiff);
    steeringAngle = drive->drive.steering_angle;

    geometry_msgs::Twist twist;
    twist.linear.x = speed;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    // https://answers.ros.org/question/93531/cmd_velgeometry_msgstwist-to-ackermann_msgs/
    twist.angular.z = (speed / WHEEL_BASE) * std::tan(steeringAngle);

    publishers["drive_twist"].publish(twist);
}

void Rc110Gazebo::onStatusUpdateTimer()
{
    getAndPublishRobotStatus();
    getAndPublishDriveInfo();
    getAndPublishServoInfo();

    getAndPublishBaseboardTemperature();
    getAndPublishMotorBattery();
    getAndPublishOtherSensors();
}

void Rc110Gazebo::getAndPublishRobotStatus()
{
    rc110_msgs::Status message;
    message.board_enabled = true;
    message.motor_state = rc110_msgs::Status::MOTOR_ON;
    message.servo_state = rc110_msgs::Status::MOTOR_ON;

    publishers["robot_status"].publish(message);
}

void Rc110Gazebo::getAndPublishDriveInfo()
{
    ackermann_msgs::AckermannDriveStamped message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = parameters.baseFrameId;

    message.drive.speed = speed;
    message.drive.steering_angle = steeringAngle;
    message.drive.steering_angle_velocity = steeringAngleVelocity;

    publishers["drive_status"].publish(message);
}

void Rc110Gazebo::getAndPublishServoInfo()
{
    publishTemperature(publishers["servo_temperature"], 40);
    publishBattery(publishers["servo_battery"], 8, std::abs(0.1f * steeringAngleVelocity));
}

void Rc110Gazebo::getAndPublishBaseboardTemperature()
{
    publishTemperature(publishers["baseboard_temperature"], 50);
}

void Rc110Gazebo::getAndPublishMotorBattery()
{
    publishBattery(publishers["motor_battery"], 8, -0.1f * speed);
}

void Rc110Gazebo::getAndPublishOtherSensors()
{
    {
        rc110_msgs::MotorRate message;
        message.header.stamp = ros::Time::now();
        message.header.frame_id = parameters.baseFrameId;

        message.motor_rate = speed * 10;  // not exact value
        message.estimated_speed = speed;

        publishers["motor_rate"].publish(message);
    }
    {
        rc110_msgs::WheelSpeeds message;
        message.header.stamp = ros::Time::now();
        message.header.frame_id = parameters.baseFrameId;

        message.speed_fl = speed;
        message.speed_fr = speed;
        message.speed_rl = speed;
        message.speed_rr = speed;

        publishers["wheel_speeds"].publish(message);
    }
}

void Rc110Gazebo::publishTemperature(ros::Publisher& publisher, float temperature)
{
    sensor_msgs::Temperature message;
    message.header.stamp = ros::Time::now();

    message.variance = 0;
    message.temperature = temperature;

    publisher.publish(message);
}

void Rc110Gazebo::publishBattery(ros::Publisher& publisher, float voltage, float current)
{
    sensor_msgs::BatteryState message;
    message.header.stamp = ros::Time::now();

    message.voltage = voltage;
    message.current = current;
    message.present = true;

    publisher.publish(message);
}

}  // namespace zmp
