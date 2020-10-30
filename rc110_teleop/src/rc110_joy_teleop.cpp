/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 * All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *
 * Written by btran
 */

#include "rc110_joy_teleop.hpp"

namespace zmp
{
namespace
{
constexpr double GEAR_1 = 0.83;    // 3 km/h
constexpr double GEAR_2 = 1.67;    // 6 km/h
constexpr float CURVE_POWER = 10;  // similar to x^2

float correctSteeringAngle(float angle)  // angle = [-1.0 .. 1.0]
{
    float value = (std::exp(std::log(CURVE_POWER + 1) * std::abs(angle)) - 1) / CURVE_POWER;
    return angle < 0 ? -value : value;
}
}  // namespace

Rc110JoyTeleop::Rc110JoyTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        m_joySub(pnh.subscribe<sensor_msgs::Joy>("input_joy", 10, &zmp::Rc110JoyTeleop::joyCallback, this)),
        m_drivePub(pnh.advertise<ackermann_msgs::AckermannDriveStamped>("output_cmd", 1))
{
    pnh.param<int>("base_dead_man_button", m_param.deadManButton, m_param.deadManButton);
    pnh.param<int>("gear_up_button", m_param.gearUpButton, m_param.gearUpButton);
    pnh.param<int>("gear_down_button", m_param.gearDownButton, m_param.gearDownButton);
    pnh.param<int>("base_steering_axis", m_param.steeringAxis, m_param.steeringAxis);
    pnh.param<int>("base_speed_axis", m_param.speedAxis, m_param.speedAxis);

    int direction = m_param.steeringAxis < 0 ? -1 : 1;
    m_param.steeringAxis *= direction;
    m_axisDirection[m_param.steeringAxis] = direction;

    direction = m_param.speedAxis < 0 ? -1 : 1;
    m_param.speedAxis *= direction;
    m_axisDirection[m_param.speedAxis] = direction;

    double maxSteeringAngleDeg;
    if (pnh.getParam("max_steering_angle_deg", maxSteeringAngleDeg)) {
        m_param.maxSteeringAngleRad = angles::from_degrees(maxSteeringAngleDeg);
    }

    pnh.param<double>("max_speed", m_param.maxSpeed, m_param.maxSpeed);
    pnh.param<std::string>("base_frame_id", m_param.frameId, m_param.frameId);
    m_timer = pnh.createTimer(ros::Duration(1 / m_param.rate), [this](const ros::TimerEvent&) { publish(); });
}

void Rc110JoyTeleop::publish()
{
    if (m_deadmanPressed) {
        m_drivePub.publish(m_lastMessage);
        m_stopMessagePublished = false;
    } else if (!m_stopMessagePublished) {
        m_drivePub.publish(ackermann_msgs::AckermannDriveStamped());
        m_stopMessagePublished = true;
    }
}

void Rc110JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joyMsg)
{
    m_deadmanPressed = joyMsg->buttons[m_param.deadManButton];
    bool gearUp = joyMsg->buttons[m_param.gearUpButton];
    bool gearDown = joyMsg->buttons[m_param.gearDownButton];

    if (gearUp && !m_gearUpPressed) ++m_gear;
    if (gearDown && !m_gearDownPressed) --m_gear;

    m_gearUpPressed = gearUp;
    m_gearDownPressed = gearDown;
    m_gear = m_deadmanPressed ? std::max(1, m_gear) : 0;

    double speedFactor = (m_gear < 2) ? GEAR_1 : (m_gear < 3) ? GEAR_2 : m_param.maxSpeed;
    float correctedAngle = correctSteeringAngle(joyMsg->axes[m_param.steeringAxis]) * float(m_param.maxSteeringAngleRad);

    m_lastMessage.drive.steering_angle = m_axisDirection[m_param.steeringAxis] * correctedAngle;
    m_lastMessage.drive.speed = m_axisDirection[m_param.speedAxis] * joyMsg->axes[m_param.speedAxis] * speedFactor;
    m_lastMessage.header.stamp = ros::Time::now();
    m_lastMessage.header.frame_id = m_param.frameId;
}
}  // namespace zmp
