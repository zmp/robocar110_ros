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
Rc110JoyTeleop::Rc110JoyTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        m_joySub(pnh.subscribe<sensor_msgs::Joy>("input_joy", 10, &zmp::Rc110JoyTeleop::joyCallback, this)),
        m_drivePub(pnh.advertise<ackermann_msgs::AckermannDriveStamped>("output_cmd", 1)),
        m_deadmanPressed(false),
        m_stopMessagePublished(false)
{
    pnh.param<int>("base_dead_man_button", m_param.deadManButton, m_param.deadManButton);
    pnh.param<int>("base_steering_axis", m_param.steeringAxis, m_param.steeringAxis);
    pnh.param<int>("base_speed_axis", m_param.speedAxis, m_param.speedAxis);

    double maxSteeringAngleDeg;
    if (pnh.getParam("max_steering_angle_deg", maxSteeringAngleDeg)) {
        m_param.maxSteeringAngleRad = angles::from_degrees(maxSteeringAngleDeg);
    }

    pnh.param<double>("max_speed", m_param.maxSpeed, m_param.maxSpeed);
    pnh.param<std::string>("base_frame_id", m_param.frameId, m_param.frameId);
    m_timer = pnh.createTimer(ros::Duration(1 / m_param.rate), std::bind(&Rc110JoyTeleop::publish, this));
}

Rc110JoyTeleop::~Rc110JoyTeleop() {}

void Rc110JoyTeleop::publish()
{
    std::lock_guard<std::mutex> lock(m_publishMutex);
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
    m_lastMessage.drive.steering_angle = joyMsg->axes[m_param.steeringAxis] * m_param.maxSteeringAngleRad;
    m_lastMessage.drive.speed = joyMsg->axes[m_param.speedAxis] * m_param.maxSpeed;
    m_lastMessage.header.stamp = ros::Time::now();
    m_lastMessage.header.frame_id = m_param.frameId;

    m_deadmanPressed = joyMsg->buttons[m_param.deadManButton];
}
}  // namespace zmp
