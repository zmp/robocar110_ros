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

#include <geometry_msgs/TwistStamped.h>
#include <boost/math/constants/constants.hpp>

namespace
{
constexpr float DEG_TO_RAD = M_PI / 180.0;  // degree to radians conversion factor
constexpr float MM_TO_M = 1 / 1e3;          // millimeter to meter conversion factor
constexpr float G_TO_MS2 = 9.80665;         // G to m/s2 conversion factor
constexpr float MA_TO_A = 1 / 1e3;          // milliampere to ampere conversion factor

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
        parameters({.rs232_device = handlePrivate.param<std::string>("rs232_device", "/dev/ttyUSB1"),
                    .rs485_device = handlePrivate.param<std::string>("rs485_device", "/dev/ttyUSB0")})
{
    control.init();
    control.Start();

    control.SetReportFlagReq(0b1111);  // report: sensor, obstacle, power, ?
    control.SetServoEnable(1);
    control.SetMotorEnableReq(1);
    control.SetDriveSpeed(0);
    control.SetSteerAngle(0);

    driveSubscriber = handle.subscribe("drive", 10, &Rc110DriveControl::onDrive, this);
    driveStatusPublisher = handle.advertise<geometry_msgs::TwistStamped>("drive_status", 10);
    imuPublisher = handle.advertise<sensor_msgs::Imu>("imu", 10);
    servoTemperaturePublisher = handle.advertise<sensor_msgs::Temperature>("servo_temperature", 10);
    motorTemperaturePublisher = handle.advertise<sensor_msgs::Temperature>("motor_temperature", 10);
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
    using namespace boost::math::float_constants;

    control.SetDriveSpeed(message.speed);
    control.SetSteerAngle(message.steering_angle * radian);
}

void Rc110DriveControl::onStatusUpdateTimer(const ros::TimerEvent&)
{
    // driving information
    zrc::SENSOR_VALUE wheelAndImuData;
    if (!control.GetSensorInfoReq(&wheelAndImuData)) {
        ROS_WARN("Failed to get sensor info.");
        return;
    }
    float speed = ::calculate4WheelSpeed(wheelAndImuData.enc_1,
                                         wheelAndImuData.enc_2,
                                         wheelAndImuData.enc_3,
                                         wheelAndImuData.enc_4);
    float angle = 0;
    if (!control.GetPresentAngle(&angle)) {
        ROS_WARN("Failed to get current angle.");
        return;
    }

    // thermals
    int servoTemperature = 0;
    if (!control.GetPresentTemp(&servoTemperature)) {
        ROS_WARN("Failed to get servo temperature.");
        return;
    }
    zrc::THERMO_VALUE thermo;
    if (!control.GetThermoInfoReq(&thermo)) {
        ROS_WARN("Failed to get thermo info.");
        return;
    }

    // power
    zrc::POWER_VALUE power;
    if (!control.GetPowerInfoReq(&power)) {
        ROS_WARN("Failed to get power info.");
        return;
    }

    auto time = ros::Time::now();
    publishDriveStatus(time, speed * MM_TO_M, angle * DEG_TO_RAD);
    publishImu(time, wheelAndImuData);
    publishTemperature(time, static_cast<float>(servoTemperature), servoTemperaturePublisher);
    publishTemperature(time, thermo.motor, motorTemperaturePublisher);
    publishBattery(time, power);
}

void Rc110DriveControl::publishDriveStatus(const ros::Time& time, float speed, float angle)
{
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = time;

    twist.twist.linear.x = speed;
    twist.twist.angular.z = angle;
    driveStatusPublisher.publish(twist);
}

void Rc110DriveControl::publishImu(const ros::Time& time, const zrc::SENSOR_VALUE& wheelAndImuData)
{
    sensor_msgs::Imu imu;
    imu.header.stamp = time;
    imu.orientation_covariance[0] = -1;  // data not available

    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = wheelAndImuData.gyro * DEG_TO_RAD;

    imu.linear_acceleration.x = wheelAndImuData.acc_x * G_TO_MS2;
    imu.linear_acceleration.y = wheelAndImuData.acc_y * G_TO_MS2;
    imu.linear_acceleration.z = wheelAndImuData.acc_z * G_TO_MS2;

    driveStatusPublisher.publish(imu);
}

void Rc110DriveControl::publishTemperature(const ros::Time& time, float temperature, ros::Publisher& publisher)
{
    sensor_msgs::Temperature msg;
    msg.header.stamp = time;

    msg.variance = 0;
    msg.temperature = temperature;

    publisher.publish(msg);
}

void Rc110DriveControl::publishBattery(const ros::Time& time, const zrc::POWER_VALUE& powerInfo)
{
    sensor_msgs::BatteryState msg;
    msg.header.stamp = time;

    msg.voltage = powerInfo.battery_level;
    msg.current = powerInfo.motor_current * MA_TO_A;
    msg.present = true;

    motorBatteryPublisher.publish(msg);
}

}  // namespace zmp