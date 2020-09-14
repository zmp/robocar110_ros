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
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>

#include <boost/math/constants/constants.hpp>

namespace constants = boost::math::float_constants;

namespace
{
constexpr float MM_TO_M = 1 / 1e3;   // millimeter to meter conversion factor
constexpr float G_TO_MS2 = 9.80665;  // G to m/s2 conversion factor
constexpr float MA_TO_A = 1 / 1e3;   // milliampere to ampere conversion factor

float calculate4WheelSpeed(float frontLeftWheelSpeed,
                           float frontRightWheelSpeed,
                           float rearLeftWheelSpeed,
                           float rearRightWheelSpeed)
{
    return (frontLeftWheelSpeed + frontRightWheelSpeed + rearLeftWheelSpeed + rearRightWheelSpeed) / 4;
}
}  // namespace

namespace zmp
{
Rc110DriveControl::Rc110DriveControl(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate) :
        parameters({})
{
    control.init();
    control.Start();

    control.SetReportFlagReq(0b1111);  // report: sensor, obstacle, power, ?
    control.SetServoEnable(1);
    control.SetMotorEnableReq(1);
    control.SetDriveSpeed(0);
    control.SetSteerAngle(0);

    driveSubscriber = handle.subscribe("drive", 10, &Rc110DriveControl::onDrive, this);
    driveStatusPublisher = handle.advertise<ackermann_msgs::AckermannDriveStamped>("drive_status", 10);
    imuPublisher = handle.advertise<sensor_msgs::Imu>("imu", 10);
    servoTemperaturePublisher = handle.advertise<sensor_msgs::Temperature>("servo_temperature", 10);
    baseboardTemperaturePublisher = handle.advertise<sensor_msgs::Temperature>("baseboard_temperature", 10);
    motorBatteryPublisher = handle.advertise<sensor_msgs::BatteryState>("motor_battery", 10);

    statusUpdateTimer =
            handle.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent& event) { onStatusUpdateTimer(event); });

    ROS_INFO("RC 1/10 drive control started");
}

Rc110DriveControl::~Rc110DriveControl()
{
    // to fix: endless loop
    //    control.Stop();
    //    control.Close();
}

void Rc110DriveControl::onDrive(const ackermann_msgs::AckermannDrive& message)
{
    control.SetDriveSpeed(message.speed);
    control.SetSteerAngle(message.steering_angle * constants::radian);
}

void Rc110DriveControl::onStatusUpdateTimer(const ros::TimerEvent&)
{
    getAndPublishDriveStatus();
    getAndPublishImu();
    getAndPublishServoTemperature();
    getAndPublishBaseboardTemperature();
    getAndPublishBattery();
}

void Rc110DriveControl::getAndPublishDriveStatus()
{
    SENSOR_VALUE sensor;
    if (!control.GetSensorInfoReq(&sensor)) {
        ROS_ERROR("Failed to get sensor info.");
        return;
    }
    float speed = ::calculate4WheelSpeed(sensor.enc_1, sensor.enc_2, sensor.enc_3, sensor.enc_4) * MM_TO_M;

    DRIVE_VALUE drive;
    if (!control.GetServoInfoReq(1, PRESENT_POSITION_L, PRESENT_VOLTS_H - PRESENT_POSITION_L, &drive)) {
        ROS_ERROR("Failed to get servo info angle.");
        return;
    }
    float angle = drive.present_position * constants::degree;

    ackermann_msgs::AckermannDriveStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "rc110_base";

    msg.drive.speed = speed;
    msg.drive.steering_angle = angle;

    driveStatusPublisher.publish(msg);
}

void Rc110DriveControl::getAndPublishImu()
{
    SENSOR_VALUE sensor;
    if (!control.GetSensorInfoReq(&sensor)) {
        ROS_ERROR("Failed to get sensor info.");
        return;
    }

    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "rc110_imu";
    msg.orientation_covariance[0] = -1;  // data not available

    msg.angular_velocity.x = 0;
    msg.angular_velocity.y = 0;
    msg.angular_velocity.z = sensor.gyro * constants::degree;

    msg.linear_acceleration.x = sensor.acc_x * G_TO_MS2;
    msg.linear_acceleration.y = sensor.acc_y * G_TO_MS2;
    msg.linear_acceleration.z = sensor.acc_z * G_TO_MS2;

    imuPublisher.publish(msg);
}

void Rc110DriveControl::getAndPublishServoTemperature()
{
    DRIVE_VALUE drive;
    if (!control.GetServoInfoReq(1, PRESENT_POSITION_L, PRESENT_VOLTS_H - PRESENT_POSITION_L, &drive)) {
        ROS_ERROR("Failed to get servo info angle.");
        return;
    }
    float servoTemperature = static_cast<float>(drive.present_temperature);
    publishTemperature(servoTemperature, servoTemperaturePublisher);
}

void Rc110DriveControl::getAndPublishBaseboardTemperature()
{
    THERMO_VALUE thermo;
    if (!control.GetThermoInfoReq(&thermo)) {
        ROS_ERROR("Failed to get thermo info.");
        return;
    }
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
    POWER_VALUE power;
    if (!control.GetPowerInfoReq(&power)) {
        ROS_ERROR("Failed to get motor power info.");
        return;
    }

    sensor_msgs::BatteryState msg;
    msg.header.stamp = ros::Time::now();

    msg.voltage = power.battery_level;
    msg.current = power.motor_current * MA_TO_A;
    msg.present = true;

    motorBatteryPublisher.publish(msg);
}

}  // namespace zmp