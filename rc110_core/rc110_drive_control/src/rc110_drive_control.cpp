/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "rc110_drive_control.hpp"

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <rc110_msgs/BaseboardError.h>
#include <rc110_msgs/MotorRate.h>
#include <rc110_msgs/Status.h>
#include <rc110_msgs/WheelSpeeds.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/math/constants/constants.hpp>

namespace zmp
{
constexpr float DEG_TO_RAD = boost::math::float_constants::degree;

Rc110DriveControl::Rc110DriveControl(ros::NodeHandle& handle, ros::NodeHandle& handlePrivate) :
        parameters({
                .baseFrameId = handlePrivate.param<std::string>("base_frame_id", "base_link"),
                .odomFrameId = handlePrivate.param<std::string>("odom_frame_id", "odom"),
                .imuFrameId = handlePrivate.param<std::string>("imu_frame_id", "imu_link"),
                .rate = handlePrivate.param<double>("rate", 30),
                .odometryOnlyAngleOffset = handlePrivate.param<double>("odometry_only_angle_offset", 1),
        }),
        control([this](BaseboardError error) { baseboardError = error; }),
        wheelBase(control.GetWheelBase())
{
    control.Start();

    services.push_back(handle.advertiseService("enable_board", &Rc110DriveControl::onEnableBoard, this));
    services.push_back(handle.advertiseService("motor_state", &Rc110DriveControl::onMotorState, this));
    services.push_back(handle.advertiseService("servo_state", &Rc110DriveControl::onServoState, this));

    subscribers.push_back(handle.subscribe("drive", 10, &Rc110DriveControl::onDrive, this));
    subscribers.push_back(handle.subscribe("offsets", 10, &Rc110DriveControl::onOffsets, this));

    publishers["motor_speed_goal"] = handle.advertise<std_msgs::Float32>("motor_speed_goal", 10, true);
    publishers["steering_angle_goal"] = handle.advertise<std_msgs::Float32>("steering_angle_goal", 10, true);

    publishers["baseboard_error"] = handle.advertise<rc110_msgs::BaseboardError>("baseboard_error", 10, true);
    publishers["robot_status"] = handle.advertise<rc110_msgs::Status>("robot_status", 10, true);
    publishers["drive_status"] = handle.advertise<ackermann_msgs::AckermannDriveStamped>("drive_status", 10);
    publishers["offsets_status"] = handle.advertise<rc110_msgs::Offsets>("offsets_status", 10, true);

    publishers["imu"] = handle.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
    publishers["servo_temperature"] = handle.advertise<sensor_msgs::Temperature>("servo_temperature", 10);
    publishers["baseboard_temperature"] = handle.advertise<sensor_msgs::Temperature>("baseboard_temperature", 10);
    publishers["servo_battery"] = handle.advertise<sensor_msgs::BatteryState>("servo_battery", 10);
    publishers["motor_battery"] = handle.advertise<sensor_msgs::BatteryState>("motor_battery", 10);
    publishers["odometry"] = handle.advertise<nav_msgs::Odometry>("odometry", 10);
    publishers["motor_rate"] = handle.advertise<rc110_msgs::MotorRate>("motor_rate", 10);
    publishers["wheel_speeds"] = handle.advertise<rc110_msgs::WheelSpeeds>("wheel_speeds", 10);

    statusUpdateTimer = handle.createTimer(ros::Duration(1 / parameters.rate),
                                           [this](const ros::TimerEvent& event) { onStatusUpdateTimer(event); });

    // publish no error
    publishers["baseboard_error"].publish(rc110_msgs::BaseboardError());

    // publish offsets
    rc110_msgs::Offsets offsetsMessage;
    auto offsets = control.GetOffsets();
    offsetsMessage.gyro = offsets.gyro;
    offsetsMessage.accel_x = offsets.accelX;
    offsetsMessage.accel_y = offsets.accelY;
    offsetsMessage.accel_z = offsets.accelZ;
    offsetsMessage.motor_current = offsets.motorCurrent;
    offsetsMessage.steering = offsets.steeringAngle;
    publishers["offsets_status"].publish(offsetsMessage);

    ROS_INFO("RC 1/10 drive control started");
}

Rc110DriveControl::~Rc110DriveControl()
{
    control.Stop();
}

bool Rc110DriveControl::onEnableBoard(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
    response.success = control.EnableBaseboard(request.data);
    return true;
}

bool Rc110DriveControl::onMotorState(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
    auto state = MotorState(request.data);
    response.success = control.EnableMotor(state);
    return true;
}

bool Rc110DriveControl::onServoState(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
    auto state = MotorState(request.data);
    response.success = control.EnableServo(state);
    return true;
}

void Rc110DriveControl::onDrive(const ackermann_msgs::AckermannDriveStamped::ConstPtr& message)
{
    if (control.ChangeDriveSpeed(message->drive.speed)) {
        std_msgs::Float32 speedMessage;
        speedMessage.data = message->drive.speed;
        publishers["motor_speed_goal"].publish(speedMessage);
    }
    if (control.ChangeSteeringAngle(message->drive.steering_angle)) {
        std_msgs::Float32 angleMessage;
        angleMessage.data = message->drive.steering_angle;
        publishers["steering_angle_goal"].publish(angleMessage);
    }
}

void Rc110DriveControl::onOffsets(const rc110_msgs::Offsets::ConstPtr& message)
{
    control.SetOffsets({
            .gyro = message->gyro,
            .accelX = message->accel_x,
            .accelY = message->accel_y,
            .accelZ = message->accel_z,
            .motorCurrent = message->motor_current,
            .steeringAngle = message->steering,
    });
    publishers["offsets_status"].publish(message);
}

void Rc110DriveControl::onStatusUpdateTimer(const ros::TimerEvent&)
{
    publishErrors();
    getAndPublishRobotStatus();
    getAndPublishDriveInfo();
    getAndPublishServoInfo();
    getAndPublishImu();
    getAndPublishBaseboardTemperature();
    getAndPublishMotorBattery();
    getAndPublishOtherSensors();
}

void Rc110DriveControl::publishErrors()
{
    if (lastBaseboardError == baseboardError) {
        return;
    }
    lastBaseboardError = baseboardError;

    rc110_msgs::BaseboardError message;
    message.data = uint8_t(baseboardError.load());

    publishers["baseboard_error"].publish(message);
}

void Rc110DriveControl::getAndPublishRobotStatus()
{
    rc110_msgs::Status message;
    message.board_enabled = control.IsBaseboardEnabled();
    message.motor_state = uint8_t(control.GetMotorState());
    message.servo_state = uint8_t(control.GetServoState());

    publishers["robot_status"].publish(message);
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
    publishTemperature(publishers["servo_temperature"], servoInfo.temperature);
    publishBattery(publishers["servo_battery"], servoInfo.voltage, servoInfo.current);
}

void Rc110DriveControl::getAndPublishImu()
{
    SensorInfo sensor = control.GetSensorInfo();

    sensor_msgs::Imu message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = parameters.imuFrameId;

    message.angular_velocity.x = 0;
    message.angular_velocity.y = 0;
    message.angular_velocity.z = sensor.gyroYaw;

    message.linear_acceleration.x = sensor.accelX;
    message.linear_acceleration.y = sensor.accelY;
    message.linear_acceleration.z = sensor.accelZ;

    // data not available
    message.orientation = {};
    message.orientation_covariance.fill(-1);
    message.angular_velocity_covariance.fill(-1);
    message.linear_acceleration_covariance.fill(-1);

    publishers["imu"].publish(message);
}

void Rc110DriveControl::getAndPublishBaseboardTemperature()
{
    ThermoInfo thermo = control.GetThermoInfo();
    float baseboardTemperature = std::max(thermo.board_1, thermo.board_2);
    publishTemperature(publishers["baseboard_temperature"], baseboardTemperature);
}

void Rc110DriveControl::getAndPublishMotorBattery()
{
    PowerInfo power = control.GetPowerInfo();
    publishBattery(publishers["motor_battery"], power.motorVoltage, power.motorCurrent);
}

void Rc110DriveControl::getAndPublishOtherSensors()
{
    SensorInfo sensor = control.GetSensorInfo();
    {
        rc110_msgs::MotorRate message;
        message.header.stamp = ros::Time::now();
        message.header.frame_id = parameters.baseFrameId;

        message.motor_rate = sensor.motorRate;
        message.estimated_speed = sensor.estimatedSpeed;

        publishers["motor_rate"].publish(message);
    }
    {
        rc110_msgs::WheelSpeeds message;
        message.header.stamp = ros::Time::now();
        message.header.frame_id = parameters.baseFrameId;

        message.speed_fl = sensor.speedFL;
        message.speed_fr = sensor.speedFR;
        message.speed_rl = sensor.speedRL;
        message.speed_rr = sensor.speedRR;

        publishers["wheel_speeds"].publish(message);
    }
}

void Rc110DriveControl::publishDriveStatus(const DriveInfo& drive)
{
    ackermann_msgs::AckermannDriveStamped message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = parameters.baseFrameId;

    message.drive.speed = drive.speed;
    message.drive.steering_angle = drive.steeringAngle;
    message.drive.steering_angle_velocity = drive.steeringAngularSpeed;

    publishers["drive_status"].publish(message);
}

void Rc110DriveControl::publishOdometry(const DriveInfo& drive)
{
    double lastTimestamp = odometry.header.stamp.toSec();
    double t = lastTimestamp == 0 ? 0 : drive.timestamp - lastTimestamp;

    float adjustment = float(parameters.odometryOnlyAngleOffset) * DEG_TO_RAD;
    float adjustedAngle = drive.steeringAngle + (drive.steeringAngle < 0 ? adjustment : -adjustment);
    float circleRadius = wheelBase / std::tan(adjustedAngle);
    float angularSpeed = drive.speed / circleRadius;
    tf2::Quaternion tfYaw;
    tfYaw.setRPY(0, 0, estimatedYaw);

    // odometry
    odometry.header.stamp = ros::Time().fromSec(drive.timestamp);
    odometry.header.frame_id = parameters.odomFrameId;
    odometry.child_frame_id = parameters.baseFrameId;
    odometry.twist.twist.linear.x = drive.speed;
    odometry.twist.twist.angular.z = angularSpeed;

    // https://en.wikipedia.org/wiki/Dead_reckoning
    estimatedYaw += angularSpeed * t;
    odometry.pose.pose.position.x += drive.speed * std::cos(estimatedYaw) * t;
    odometry.pose.pose.position.y += drive.speed * std::sin(estimatedYaw) * t;
    odometry.pose.pose.position.z = 0;
    odometry.pose.pose.orientation = tf2::toMsg(tfYaw);

    publishers["odometry"].publish(odometry);

    // transform
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = parameters.odomFrameId;
    tf.child_frame_id = parameters.baseFrameId;
    tf.transform.translation.x = odometry.pose.pose.position.x;
    tf.transform.translation.y = odometry.pose.pose.position.y;
    tf.transform.translation.z = odometry.pose.pose.position.z;
    tf.transform.rotation = odometry.pose.pose.orientation;
    odometryBroadcaster.sendTransform(tf);
}

void Rc110DriveControl::publishTemperature(ros::Publisher& publisher, float temperature)
{
    sensor_msgs::Temperature message;
    message.header.stamp = ros::Time::now();

    message.variance = 0;
    message.temperature = temperature;

    publisher.publish(message);
}

void Rc110DriveControl::publishBattery(ros::Publisher& publisher, float voltage, float current)
{
    sensor_msgs::BatteryState message;
    message.header.stamp = ros::Time::now();

    message.voltage = voltage;
    message.current = current;
    message.present = true;

    publisher.publish(message);
}

}  // namespace zmp
