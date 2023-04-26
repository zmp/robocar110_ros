/*
 * Copyright (C) 2021 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by Paulo Morales
 */
#include "rc110_gazebo.hpp"

namespace
{
constexpr double WHEEL_BASE = 0.26;
constexpr double TELEOP_PING_TIMEOUT_SEC = 0.4;
}  // namespace

namespace zmp
{
Rc110Gazebo::Rc110Gazebo() : Node("rc110_gazebo") 
{
    parameters.baseFrameId = declare_parameter("~base_frame_id", parameters.baseFrameId);
    parameters.rate_hz = declare_parameter("~rate", parameters.rate_hz);
    onTeleopPingSrv = create_service<std_srvs::srv::SetBool>("teleop_ping",
                                     [this](std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                            std::shared_ptr<std_srvs::srv::SetBool::Response> response)
                                           {
                                                onTeleopPing(*request, *response);
                                           });
    onTeleopStatusSrv = create_service<std_srvs::srv::Trigger>("teleop_status",
                                       [this](std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
                                              {
                                                  onTeleopStatus(*request, *response);
                                              });
    onDriveSub = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 2, std::bind(&Rc110Gazebo::onDrive, this, std::placeholders::_1));
    publishers["drive_twist"] = create_publisher<geometry_msgs::msg::Twist>("/drive_twist",
                                                                            rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal)); 
    publishers["motor_speed_goal"] = create_publisher<std_msgs::msg::Float32>("/motor_speed_goal",
                                                                              rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal));
    publishers["steering_angle_goal"] = create_publisher<std_msgs::msg::Float32>("/steering_angle_goal",
                                                                                 rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal));
    publishers["baseboard_error"] = create_publisher<rc110_msgs::msg::BaseboardError>("/baseboard_error",
                                                                                      rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));
    publishers["robot_status"] = create_publisher<rc110_msgs::msg::Status>("/robot_status",
                                                                           rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal)); 
    publishers["drive_status"] = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_status",
                                                                                              rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal));
    publishers["servo_temperature"] = create_publisher<sensor_msgs::msg::Temperature>("/servo_temperature",
                                                                                      rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal)); 
    publishers["baseboard_temperature"] = create_publisher<sensor_msgs::msg::Temperature>("/baseboard_temperature",
                                                                                          rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal)); 
    publishers["servo_battery"] = create_publisher<sensor_msgs::msg::Temperature>("/servo_battery",
                                                                                  rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal));
    publishers["motor_battery"] = create_publisher<sensor_msgs::msg::Temperature>("/motor_battery",
                                                                                  rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal));
    publishers["motor_rate"] = create_publisher<rc110_msgs::msg::MotorRate>("/motor_rate",
                                                                            rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal));
    publishers["wheel_speeds"] = create_publisher<rc110_msgs::msg::WheelSpeeds>("/wheel_speeds",
                                                                                rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::TransientLocal));
    statusUpdateTimer = create_wall_timer(std::chrono::milliseconds(int(1 / parameters.rate_hz)), [this] {onStatusUpdateTimer();});
    publish<rc110_msgs::msg::BaseboardError>("baseboard_error", rc110_msgs::msg::BaseboardError());
}

bool Rc110Gazebo::onTeleopPing(std_srvs::srv::SetBool::Request& request, std_srvs::srv::SetBool::Response& response)
{
    if (request.data)
    {
        // allow new connection, if time from the last ping is long enough
        response.success = now() - lastTeleopPing > rclcpp::Duration::from_seconds(TELEOP_PING_TIMEOUT_SEC);
        if (!response.success)
        {
            return false;
        }
    }
    lastTeleopPing = now();
    return true;
}

bool Rc110Gazebo::onTeleopStatus(std_srvs::srv::Trigger::Request& request, std_srvs::srv::Trigger::Response& response)
{
    response.success = now() - lastTeleopPing <= rclcpp::Duration::from_seconds(TELEOP_PING_TIMEOUT_SEC);
    return true;
}

void Rc110Gazebo::onDrive(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr& drive)
{
    double timeDiff = (static_cast<rclcpp::Time>(drive->header.stamp) - lastDriveTime).to_chrono<std::chrono::seconds>().count();
    lastDriveTime = drive->header.stamp;

    speed = drive->drive.speed;
    steeringAngleVelocity = timeDiff == 0 ? 0.f : float((drive->drive.steering_angle - steeringAngle) / timeDiff);
    steeringAngle = drive->drive.steering_angle;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = speed;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    // https://answers.ros.org/question/93531/cmd_velgeometry_msgstwist-to-ackermann_msgs/
    twist.angular.z = (speed / WHEEL_BASE) * std::tan(steeringAngle);

    publish<geometry_msgs::msg::Twist>("drive_twist", twist);
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
    rc110_msgs::msg::Status message;
    message.board_enabled = true;
    message.motor_state = rc110_msgs::msg::Status::MOTOR_ON;
    message.servo_state = rc110_msgs::msg::Status::MOTOR_ON;

    publish<rc110_msgs::msg::Status>("robot_status", message);
}

void Rc110Gazebo::getAndPublishDriveInfo()
{
    ackermann_msgs::msg::AckermannDriveStamped message;
    message.header.stamp = now();
    message.header.frame_id = parameters.baseFrameId;

    message.drive.speed = speed;
    message.drive.steering_angle = steeringAngle;
    message.drive.steering_angle_velocity = steeringAngleVelocity;

    publish<ackermann_msgs::msg::AckermannDriveStamped>("drive_status", message);
}

void Rc110Gazebo::getAndPublishServoInfo()
{
    publish<sensor_msgs::msg::Temperature>("servo_temperature", createTemperatureMsg(40));
    publish<sensor_msgs::msg::BatteryState>("servo_battery", createBatteryMsg(8.f, std::abs(0.1f * steeringAngleVelocity)));
}

void Rc110Gazebo::getAndPublishBaseboardTemperature()
{
    publish<sensor_msgs::msg::Temperature>("baseboard_temperature", createTemperatureMsg(50));
}

void Rc110Gazebo::getAndPublishMotorBattery()
{
    publish<sensor_msgs::msg::BatteryState>("motor_battery", createBatteryMsg(8.f, -0.1f * speed));
}

void Rc110Gazebo::getAndPublishOtherSensors()
{
    {
        rc110_msgs::msg::MotorRate message;
        message.header.stamp = now();
        message.header.frame_id = parameters.baseFrameId;

        message.motor_rate = speed * 10;  // not exact value
        message.estimated_speed = speed;

        publish<rc110_msgs::msg::MotorRate>("motor_rate", message);
    }
    {
        rc110_msgs::msg::WheelSpeeds message;
        message.header.stamp = now();
        message.header.frame_id = parameters.baseFrameId;

        message.speed_fl = speed;
        message.speed_fr = speed;
        message.speed_rl = speed;
        message.speed_rr = speed;

        publish<rc110_msgs::msg::WheelSpeeds>("wheel_speeds", message);
    }
}

sensor_msgs::msg::Temperature Rc110Gazebo::createTemperatureMsg(const float& temperature)
{
    sensor_msgs::msg::Temperature message;
    message.header.stamp = now();

    message.variance = 0;
    message.temperature = temperature;

    return message;
}

sensor_msgs::msg::BatteryState Rc110Gazebo::createBatteryMsg(const float& voltage, const float& current)
{
    sensor_msgs::msg::BatteryState message;
    message.header.stamp = now();
    message.voltage = voltage;
    message.current = current;
    message.present = true;
    return message;
}

}  // namespace zmp