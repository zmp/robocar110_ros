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
Rc110DriveControl::Rc110DriveControl(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate) : parameters({})
{
    control.Start();

    control.ChangeReportFlags(0b1111);  // report: sensor, obstacle, power, ?
    control.EnableMotor();
    control.ChangeDriveSpeed(0);

    driveSubscriber = handle.subscribe("drive", 10, &Rc110DriveControl::onDrive, this);
    driveStatusPublisher = handle.advertise<ackermann_msgs::AckermannDriveStamped>("drive_status", 10);
    imuPublisher = handle.advertise<sensor_msgs::Imu>("imu", 10);
    servoTemperaturePublisher = handle.advertise<sensor_msgs::Temperature>("servo_temperature", 10);
    baseboardTemperaturePublisher = handle.advertise<sensor_msgs::Temperature>("baseboard_temperature", 10);
    motorBatteryPublisher = handle.advertise<sensor_msgs::BatteryState>("motor_battery", 10);
    odometryPublisher = handle.advertise<nav_msgs::Odometry>("odometry", 10);

    statusUpdateTimer =
            handle.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent& event) { onStatusUpdateTimer(event); });

    ROS_INFO("RC 1/10 drive control started");
}

Rc110DriveControl::~Rc110DriveControl()
{
    // to fix: endless loop
    //    control.Stop();
}

void Rc110DriveControl::onDrive(const ackermann_msgs::AckermannDrive& message)
{
    control.ChangeDriveSpeed(message.speed);
    control.ChangeSteeringAngle(message.steering_angle);
}

void Rc110DriveControl::onStatusUpdateTimer(const ros::TimerEvent&)
{
    auto driveStatus = control.GetVehicleDriveStatus();
    publishDriveStatus(driveStatus);
    publishOdometry(driveStatus);

    getAndPublishImu();
    getAndPublishServoTemperature();
    getAndPublishBaseboardTemperature();
    getAndPublishBattery();
}

void Rc110DriveControl::publishDriveStatus(const DriveStatus& drive)
{
    ackermann_msgs::AckermannDriveStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "rc110_base";

    msg.drive.speed = drive.speed;
    msg.drive.steering_angle = drive.steeringAngle;
    msg.drive.steering_angle_velocity = drive.steeringAngularSpeed;

    driveStatusPublisher.publish(msg);
}

void Rc110DriveControl::publishOdometry(const DriveStatus& drive)
{
    nav_msgs::Odometry message;
    message.header.stamp = ros::Time::now();
    message.twist.twist.linear.x = drive.speed;
    message.twist.twist.angular.z = drive.angularSpeed;

    odometryPublisher.publish(message);
}

void Rc110DriveControl::getAndPublishImu()
{
    SENSOR_VALUE sensor = control.GetSensorInfo();

    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "rc110_imu";
    msg.orientation_covariance[0] = -1;  // data not available

    msg.angular_velocity.x = 0;
    msg.angular_velocity.y = 0;
    msg.angular_velocity.z = sensor.gyro;

    msg.linear_acceleration.x = sensor.acc_x;
    msg.linear_acceleration.y = sensor.acc_y;
    msg.linear_acceleration.z = sensor.acc_z;

    imuPublisher.publish(msg);
}

void Rc110DriveControl::getAndPublishServoTemperature()
{
    auto servoTemperature = static_cast<float>(control.GetServoTemperature());
    publishTemperature(servoTemperature, servoTemperaturePublisher);
}

void Rc110DriveControl::getAndPublishBaseboardTemperature()
{
    THERMO_VALUE thermo = control.GetThermoInfo();
    float baseboardTemperature = std::max(thermo.fet1, thermo.fet2);
    publishTemperature(baseboardTemperature, baseboardTemperaturePublisher);
}

void Rc110DriveControl::publishTemperature(float temperature, ros::Publisher& publisher)
{
    sensor_msgs::Temperature msg;
    msg.header.stamp = ros::Time::now();

    msg.variance = 0;
    msg.temperature = temperature;

    publisher.publish(msg);
}

void Rc110DriveControl::getAndPublishBattery()
{
    POWER_VALUE power = control.GetPowerInfo();

    sensor_msgs::BatteryState msg;
    msg.header.stamp = ros::Time::now();

    msg.voltage = power.battery_level;
    msg.current = power.motor_current;
    msg.present = true;

    motorBatteryPublisher.publish(msg);
}

}  // namespace zmp