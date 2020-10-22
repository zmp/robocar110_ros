/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by Andrei Pak
 */
#include "rc110_drive_control.hpp"

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

namespace zmp
{
enum EnableType : uint8_t {
    ENABLE_OFF = 0,  // turn off
    ENALBE_ON = 1,   // turn on
    ENABLE_ASK = 2,  // do nothing, just ask current status
};

Rc110DriveControl::Rc110DriveControl(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate) :
        parameters({.baseFrameId = handlePrivate.param<std::string>("base_frame_id", "rc110_base"),
                    .imuFrameId = handlePrivate.param<std::string>("imu_frame_id", "rc110_imu"),
                    .rate = handlePrivate.param<double>("rate", 30)})
{
    control.Start();
    control.EnableMotor();
    control.ChangeDriveSpeed(0);

    enableBoardService = handle.advertiseService("enable_board", &Rc110DriveControl::onEnableBoard, this);

    driveSubscriber = handle.subscribe("drive", 10, &Rc110DriveControl::onDrive, this);
    driveStatusPublisher = handle.advertise<ackermann_msgs::AckermannDriveStamped>("drive_status", 10);
    imuPublisher = handle.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
    servoTemperaturePublisher = handle.advertise<sensor_msgs::Temperature>("servo_temperature", 10);
    baseboardTemperaturePublisher = handle.advertise<sensor_msgs::Temperature>("baseboard_temperature", 10);
    servoBatteryPublisher = handle.advertise<sensor_msgs::BatteryState>("servo_battery", 10);
    motorBatteryPublisher = handle.advertise<sensor_msgs::BatteryState>("motor_battery", 10);
    odometryPublisher = handle.advertise<nav_msgs::Odometry>("odometry", 10);

    statusUpdateTimer = handle.createTimer(ros::Duration(1 / parameters.rate),
                                           [this](const ros::TimerEvent& event) { onStatusUpdateTimer(event); });

    ROS_INFO("RC 1/10 drive control started");
}

Rc110DriveControl::~Rc110DriveControl()
{
    control.Stop();
}

bool Rc110DriveControl::onEnableBoard(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
    if (request.data == ENABLE_ASK) {
        response.success = true;
        response.message = control.IsBaseboardEnabled() ? "enabled" : "disabled";
    } else {
        if ((response.success = control.EnableBaseboard(request.data))) {
            response.message = request.data ? "enabled" : "disabled";
        } else {
            response.message = "disabled";
        }
    }
    return true;
}

void Rc110DriveControl::onDrive(const ackermann_msgs::AckermannDriveStamped::ConstPtr& message)
{
    control.ChangeDriveSpeed(message->drive.speed);
    control.ChangeSteeringAngle(message->drive.steering_angle);
}

void Rc110DriveControl::onStatusUpdateTimer(const ros::TimerEvent&)
{
    getAndPublishDriveInfo();
    getAndPublishServoInfo();
    getAndPublishImu();
    getAndPublishBaseboardTemperature();
    getAndPublishMotorBattery();
}

void Rc110DriveControl::getAndPublishDriveInfo()
{
    auto driveInfo = control.GetDriveInfo();
    publishDriveStatus(driveInfo);
    publishOdometry(driveInfo);
}

void Rc110DriveControl::getAndPublishServoInfo()
{
    auto servoInfo = control.GetServoInfo();
    publishTemperature(servoTemperaturePublisher, servoInfo.temperature);
    publishBattery(servoBatteryPublisher, servoInfo.voltage, servoInfo.current);
}

void Rc110DriveControl::getAndPublishImu()
{
    SensorInfo sensor = control.GetSensorInfo();

    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = parameters.imuFrameId;
    msg.orientation_covariance[0] = -1;  // data not available

    msg.angular_velocity.x = 0;
    msg.angular_velocity.y = 0;
    msg.angular_velocity.z = sensor.gyro;

    msg.linear_acceleration.x = sensor.acc_x;
    msg.linear_acceleration.y = sensor.acc_y;
    msg.linear_acceleration.z = sensor.acc_z;

    imuPublisher.publish(msg);
}

void Rc110DriveControl::getAndPublishBaseboardTemperature()
{
    ThermoInfo thermo = control.GetThermoInfo();
    float baseboardTemperature = std::max(thermo.board_1, thermo.board_2);
    publishTemperature(baseboardTemperaturePublisher, baseboardTemperature);
}

void Rc110DriveControl::getAndPublishMotorBattery()
{
    PowerInfo power = control.GetPowerInfo();
    publishBattery(motorBatteryPublisher, power.motorVoltage, power.motorCurrent);
}

void Rc110DriveControl::publishDriveStatus(const DriveInfo& drive)
{
    ackermann_msgs::AckermannDriveStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = parameters.baseFrameId;

    msg.drive.speed = drive.speed;
    msg.drive.steering_angle = drive.steeringAngle;
    msg.drive.steering_angle_velocity = drive.steeringAngularSpeed;

    driveStatusPublisher.publish(msg);
}

void Rc110DriveControl::publishOdometry(const DriveInfo& drive)
{
    nav_msgs::Odometry message;
    message.header.stamp = ros::Time::now();
    message.twist.twist.linear.x = drive.speed;
    message.twist.twist.angular.z = drive.angularSpeed;

    odometryPublisher.publish(message);
}

void Rc110DriveControl::publishTemperature(ros::Publisher& publisher, float temperature)
{
    sensor_msgs::Temperature msg;
    msg.header.stamp = ros::Time::now();

    msg.variance = 0;
    msg.temperature = temperature;

    publisher.publish(msg);
}

void Rc110DriveControl::publishBattery(ros::Publisher& publisher, float voltage, float current)
{
    sensor_msgs::BatteryState msg;
    msg.header.stamp = ros::Time::now();

    msg.voltage = voltage;
    msg.current = current;
    msg.present = true;

    publisher.publish(msg);
}

}  // namespace zmp
