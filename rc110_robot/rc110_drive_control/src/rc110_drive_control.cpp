/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Andrei Pak
 */
#include "rc110_drive_control.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rc110_msgs/msg/baseboard_error.hpp>
#include <rc110_msgs/msg/motor_rate.hpp>
#include <rc110_msgs/msg/status.hpp>
#include <rc110_msgs/msg/wheel_speeds.hpp>
#include <rc110_msgs/msg/steering_motor_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/time.hpp>

namespace zmp
{
namespace
{
constexpr float G_TO_MS2 = 9.8f;            // G to m/s2 conversion factor (Tokyo)
constexpr float DEG_TO_RAD = M_PI / 180;    // degrees to radians
constexpr float YAW_RANGE = 600.f;          // deg
constexpr float YAW_NONLINEARITY = 0.001f;  // 0.1 %
constexpr float YAW_SIGMA = YAW_RANGE * YAW_NONLINEARITY * DEG_TO_RAD;
constexpr float YAW_VARIANCE = YAW_SIGMA * YAW_SIGMA;
constexpr float ACC_RANGE = 3;              // g
constexpr float ACC_NONLINEARITY = 0.003f;  // 0.3%
constexpr float ACC_SIGMA = ACC_RANGE * ACC_NONLINEARITY * G_TO_MS2;
constexpr float ACC_VARIANCE = ACC_SIGMA * ACC_SIGMA;
constexpr float ACC_CROSS_AXIS = 0.01;  // 1%

constexpr double TELEOP_PING_TIMEOUT = 0.4;  // sec
}  // namespace

Rc110DriveControl::Rc110DriveControl() :
        Node("rc110_drive_control"),
        parameters({
                declare_parameter("base_frame_id", "base_link"),
                declare_parameter("odom_frame_id", "odom"),
                declare_parameter("imu_frame_id", "imu_link"),
                declare_parameter("rate", 30.0),
                declare_parameter("odometry_only_angle_offset", 1.0),
                declare_parameter("steering_velocity", 90.0),
        }),
        control([this](BaseboardError error) { baseboardError = error; }),
        wheelBase(control.GetWheelBase()),
        odometryBroadcaster(this)
{
    control.Start();
    control.SetSteeringVelocity(float(parameters.steeringVelocity));

    using namespace std_srvs::srv;
    services.push_back(
            create_service<std_srvs::srv::SetBool>("enable_board",
                                    [this](std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                                        onEnableBoard(*request, *response);
                                    }));
    services.push_back(create_service<std_srvs::srv::SetBool>("teleop_ping", 
                                    [this](std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                                        onTeleopPing(*request, *response);
                                    }));
    services.push_back(create_service<std_srvs::srv::Trigger>("teleop_status", 
                                    [this](std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                                        onTeleopStatus(*request, *response);
                                    }));
    services.push_back(
            create_service<rc110_msgs::srv::SetInteger>("motor_state",
                                    [this](std::shared_ptr<rc110_msgs::srv::SetInteger::Request> request, std::shared_ptr<rc110_msgs::srv::SetInteger::Response> response) {
                                        onMotorState(*request, *response);
                                    }));
    services.push_back(
            create_service<rc110_msgs::srv::SetInteger>("servo_state",
                                    [this](std::shared_ptr<rc110_msgs::srv::SetInteger::Request> request, std::shared_ptr<rc110_msgs::srv::SetInteger::Response> response) {
                                        onServoState(*request, *response);
                                    }));

    subscribers.push_back(create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10,[this](const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr& message){
                                        onDrive(message);
                                    }));
    subscribers.push_back(create_subscription<std_msgs::msg::Float32>("set_servo_torque", 10,[this](const std_msgs::msg::Float32::ConstSharedPtr& message){onServoTorque(message);}));
    subscribers.push_back(create_subscription<rc110_msgs::msg::Offsets>("offsets", 10,  [this](const rc110_msgs::msg::Offsets::ConstSharedPtr& message) {
                                        onOffsets(message);
                                    }));

    publishers["motor_speed_goal"] =
            create_publisher<std_msgs::msg::Float32>("motor_speed_goal", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));
    publishers["steering_angle_goal"] =
            create_publisher<std_msgs::msg::Float32>("steering_angle_goal", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));

    publishers["baseboard_error"] =
            create_publisher<rc110_msgs::msg::BaseboardError>("baseboard_error", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));
    publishers["robot_status"] = create_publisher<rc110_msgs::msg::Status>("robot_status", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));
    publishers["drive_status"] = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_status", 10);
    publishers["offsets_status"] =
            create_publisher<rc110_msgs::msg::Offsets>("offsets_status", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));

    publishers["imu"] = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    publishers["servo_temperature"] = create_publisher<sensor_msgs::msg::Temperature>("servo_temperature", 10);
    publishers["baseboard_temperature"] = create_publisher<sensor_msgs::msg::Temperature>("baseboard_temperature", 10);
    publishers["servo_battery"] = create_publisher<sensor_msgs::msg::BatteryState>("servo_battery", 10);
    publishers["servo_torque"] = create_publisher<std_msgs::msg::Float32>("servo_torque", 10);
    publishers["steering_motor_info"] = create_publisher<rc110_msgs::msg::SteeringMotorInfo>("steering_motor_info", 10);
    publishers["motor_battery"] = create_publisher<sensor_msgs::msg::BatteryState>("motor_battery", 10);
    publishers["odometry"] = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    publishers["motor_rate"] = create_publisher<rc110_msgs::msg::MotorRate>("motor_rate", 10);
    publishers["wheel_speeds"] = create_publisher<rc110_msgs::msg::WheelSpeeds>("wheel_speeds", 10);

    statusUpdateTimer = create_wall_timer(std::chrono::milliseconds(int(1000.0 / parameters.rate)),
                                          [this]() { onStatusUpdateTimer(); });

    // publish no error
    publish("baseboard_error", rc110_msgs::msg::BaseboardError());

    // publish offsets
    rc110_msgs::msg::Offsets offsetsMessage;
    auto offsets = control.GetOffsets();
    offsetsMessage.gyro = offsets.gyro;
    offsetsMessage.accel_x = offsets.accelX;
    offsetsMessage.accel_y = offsets.accelY;
    offsetsMessage.accel_z = offsets.accelZ;
    offsetsMessage.motor_current = offsets.motorCurrent;
    offsetsMessage.steering = offsets.steeringAngle;
    publish("offsets_status", offsetsMessage);

    RCLCPP_INFO(get_logger(), "RC 1/10 drive control started");
}

Rc110DriveControl::~Rc110DriveControl()
{
    control.Stop();
}

bool Rc110DriveControl::onEnableBoard(std_srvs::srv::SetBool::Request& request, std_srvs::srv::SetBool::Response& response)
{
    response.success = control.EnableBaseboard(request.data);
    return true;
}

bool Rc110DriveControl::onTeleopPing(std_srvs::srv::SetBool::Request& request, std_srvs::srv::SetBool::Response& response)
{
    if (request.data) {
        // allow new connection, if time from the last ping is long enough
        response.success = now() - lastTeleopPing > rclcpp::Duration::from_seconds(TELEOP_PING_TIMEOUT);
        if (!response.success) {
            return false;
        }
    }
    lastTeleopPing = now();
    return true;
}

bool Rc110DriveControl::onTeleopStatus(std_srvs::srv::Trigger::Request& request, std_srvs::srv::Trigger::Response& response)
{
    (void)request;
    response.success = now() - lastTeleopPing <= rclcpp::Duration::from_seconds(TELEOP_PING_TIMEOUT);
    return true;
}

bool Rc110DriveControl::onMotorState(rc110_msgs::srv::SetInteger::Request& request, rc110_msgs::srv::SetInteger::Response& response)
{
    auto state = MotorState(request.data);
    response.success = control.EnableMotor(state);
    return true;
}

bool Rc110DriveControl::onServoState(rc110_msgs::srv::SetInteger::Request& request, rc110_msgs::srv::SetInteger::Response& response)
{
    auto state = MotorState(request.data);
    response.success = control.EnableServo(state);
    return true;
}

void Rc110DriveControl::onDrive(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr& message)
{
    float limitedSpeed;
    if (control.ChangeDriveSpeed(message->drive.speed, &limitedSpeed)) {
        std_msgs::msg::Float32 speedMessage;
        speedMessage.data = limitedSpeed;
        publish("motor_speed_goal", speedMessage);
    }
    float limitedAngle;
    if (control.ChangeSteeringAngle(message->drive.steering_angle, &limitedAngle)) {
        std_msgs::msg::Float32 angleMessage;
        angleMessage.data = limitedAngle;
        publish("steering_angle_goal", angleMessage);
    }
}

void Rc110DriveControl::onServoTorque(const std_msgs::msg::Float32::ConstSharedPtr& message)
{
    float torque = message->data;
    control.ChangeSteeringTorque(torque);
}

void Rc110DriveControl::onOffsets(const rc110_msgs::msg::Offsets::ConstSharedPtr& message)
{
    control.SetOffsets({
            message->gyro,
            message->accel_x,
            message->accel_y,
            message->accel_z,
            message->motor_current,
            message->steering,
    });
    publish("offsets_status", *message);
}

void Rc110DriveControl::onStatusUpdateTimer()
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

    rc110_msgs::msg::BaseboardError message;
    message.data = uint8_t(baseboardError.load());

    publish("baseboard_error", message);
}

void Rc110DriveControl::getAndPublishRobotStatus()
{
    rc110_msgs::msg::Status message;
    message.board_enabled = control.IsBaseboardEnabled();
    message.motor_state = uint8_t(control.GetMotorState());
    message.servo_state = uint8_t(control.GetServoState());

    publish("robot_status", message);
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
    publishTemperature("servo_temperature", servoInfo.temperature);
    publishBattery("servo_battery", servoInfo.voltage, servoInfo.current);
    publishFloat32("servo_torque", servoInfo.maxTorque);
    publishSteeringMotorInfo("steering_motor_info", (uint16_t)servoInfo.modelNumberL | ((uint16_t)servoInfo.modelNumberH << 8), servoInfo.firmwareVersion);
}

void Rc110DriveControl::getAndPublishImu()
{
    SensorInfo sensor = control.GetSensorInfo();

    sensor_msgs::msg::Imu message;
    message.header.stamp = now();
    message.header.frame_id = parameters.imuFrameId;

    message.angular_velocity.x = 0;
    message.angular_velocity.y = 0;
    message.angular_velocity.z = sensor.gyroYaw;
    message.angular_velocity_covariance = {
            // clang-format off
            0, 0, 0,
            0, 0, 0,
            0, 0, YAW_VARIANCE,
    };  // clang-format on

    float aX = sensor.accelX, aY = sensor.accelY, aZ = sensor.accelZ;
    if (!(aX > 0.1f || aY > 0.1f || aZ > 0.1f)) {
        // workaround for: https://github.com/ccny-ros-pkg/imu_tools/issues/140
        aX = aY = 0;
        aZ = G_TO_MS2;
    }
    message.linear_acceleration.x = aX;
    message.linear_acceleration.y = aY;
    message.linear_acceleration.z = aZ;
    message.linear_acceleration_covariance = {
            // clang-format off
            ACC_VARIANCE, std::abs(aX * aY * ACC_CROSS_AXIS), std::abs(aX * aZ * ACC_CROSS_AXIS),
            std::abs(aY * aX * ACC_CROSS_AXIS), ACC_VARIANCE, std::abs(aY * aZ * ACC_CROSS_AXIS),
            std::abs(aZ * aX * ACC_CROSS_AXIS), std::abs(aZ * aY * ACC_CROSS_AXIS), ACC_VARIANCE,
    };  // clang-format on

    // data not available
    message.orientation = geometry_msgs::msg::Quaternion();
    message.orientation_covariance.fill(-1);

    publish("imu", message);
}

void Rc110DriveControl::getAndPublishBaseboardTemperature()
{
    ThermoInfo thermo = control.GetThermoInfo();
    float baseboardTemperature = std::max(thermo.board_1, thermo.board_2);
    publishTemperature("baseboard_temperature", baseboardTemperature);
}

void Rc110DriveControl::getAndPublishMotorBattery()
{
    PowerInfo power = control.GetPowerInfo();
    publishBattery("motor_battery", power.motorVoltage, power.motorCurrent);
}

void Rc110DriveControl::getAndPublishOtherSensors()
{
    SensorInfo sensor = control.GetSensorInfo();
    {
        rc110_msgs::msg::MotorRate message;
        message.header.stamp = now();
        message.header.frame_id = parameters.baseFrameId;

        message.motor_rate = sensor.motorRate;
        message.estimated_speed = sensor.estimatedSpeed;

        publish("motor_rate", message);
    }
    {
        rc110_msgs::msg::WheelSpeeds message;
        message.header.stamp = now();
        message.header.frame_id = parameters.baseFrameId;

        message.speed_fl = sensor.speedFL;
        message.speed_fr = sensor.speedFR;
        message.speed_rl = sensor.speedRL;
        message.speed_rr = sensor.speedRR;

        publish("wheel_speeds", message);
    }
}

void Rc110DriveControl::publishDriveStatus(const DriveInfo& drive)
{
    ackermann_msgs::msg::AckermannDriveStamped message;
    message.header.stamp = now();
    message.header.frame_id = parameters.baseFrameId;

    message.drive.speed = drive.speed;
    message.drive.steering_angle = drive.steeringAngle;
    message.drive.steering_angle_velocity = drive.steeringAngularSpeed;

    publish("drive_status", message);
}

void Rc110DriveControl::publishOdometry(const DriveInfo& drive)
{
    double lastTimestamp = rclcpp::Time(odometry.header.stamp).seconds();
    double t = lastTimestamp == 0 ? 0 : drive.timestamp - lastTimestamp;
    auto currentTime = rclcpp::Time(int64_t(drive.timestamp * 1e9), RCL_ROS_TIME);

    float adjustment = float(parameters.odometryOnlyAngleOffset) * DEG_TO_RAD;
    float adjustedAngle = drive.steeringAngle + (drive.steeringAngle < 0 ? adjustment : -adjustment);
    float circleRadius = wheelBase / std::tan(adjustedAngle);
    float angularSpeed = drive.speed / circleRadius;
    tf2::Quaternion tfYaw;
    tfYaw.setRPY(0, 0, estimatedYaw);

    // odometry
    odometry.header.stamp = currentTime;
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

    publish("odometry", odometry);

    // transform
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = t == 0 ? now() : currentTime;  // avoid TF_OLD_DATA flood
    tf.header.frame_id = parameters.odomFrameId;
    tf.child_frame_id = parameters.baseFrameId;
    tf.transform.translation.x = odometry.pose.pose.position.x;
    tf.transform.translation.y = odometry.pose.pose.position.y;
    tf.transform.translation.z = odometry.pose.pose.position.z;
    tf.transform.rotation = odometry.pose.pose.orientation;
    odometryBroadcaster.sendTransform(tf);
}

void Rc110DriveControl::publishTemperature(const std::string& topic, float temperature)
{
    sensor_msgs::msg::Temperature message;
    message.header.stamp = now();

    message.variance = 0;
    message.temperature = temperature;

    publish(topic, message);
}

void Rc110DriveControl::publishBattery(const std::string& topic, float voltage, float current)
{
    sensor_msgs::msg::BatteryState message;
    message.header.stamp = now();

    message.voltage = voltage;
    message.current = current;
    message.present = true;

    publish(topic, message);
}

void Rc110DriveControl::publishFloat32(const std::string& topic, float value)
{
    std_msgs::msg::Float32 message;

    message.data = value;

    publish(topic, message);
}

void Rc110DriveControl::publishSteeringMotorInfo(const std::string& topic, uint16_t modelNumber, uint8_t firmwareVersion)
{
    rc110_msgs::msg::SteeringMotorInfo message;

    message.model_number = modelNumber;
    message.firmware_version = firmwareVersion;

    publish(topic, message);
}

}  // namespace zmp
