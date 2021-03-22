/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Written by btran
 */

#include "rc110_joy_teleop.hpp"

#include <std_srvs/SetBool.h>
#include <topic_tools/MuxSelect.h>

namespace zmp
{
namespace
{
constexpr float CURVE_POWER = 10;  // similar to x^2

float correctSteeringAngle(float angle)  // angle = [-1.0 .. 1.0]
{
    float value = (std::exp(std::log(CURVE_POWER + 1) * std::abs(angle)) - 1) / CURVE_POWER;
    return angle < 0 ? -value : value;
}
}  // namespace

Rc110JoyTeleop::Rc110JoyTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        m_drivePub(nh.advertise<ackermann_msgs::AckermannDriveStamped>("drive_manual", 1))
{
    pnh.param<int>("base_dead_man_button", m_param.deadManButton, m_param.deadManButton);
    pnh.param<int>("gear_up_button", m_param.gearUpButton, m_param.gearUpButton);
    pnh.param<int>("gear_down_button", m_param.gearDownButton, m_param.gearDownButton);
    pnh.param<int>("board_button", m_param.boardButton, m_param.boardButton);
    pnh.param<int>("ad_button", m_param.adButton, m_param.adButton);
    pnh.param<int>("base_steering_axis", m_param.steeringAxis, m_param.steeringAxis);
    pnh.param<int>("base_speed_axis", m_param.speedAxis, m_param.speedAxis);
    pnh.param<std::string>("base_frame_id", m_param.frameId, m_param.frameId);

    m_subscribers.push_back(nh.subscribe("joy", 1, &zmp::Rc110JoyTeleop::onJoy, this));
    m_subscribers.push_back(nh.subscribe("robot_status", 1, &Rc110JoyTeleop::onRobotStatus, this));
    m_subscribers.push_back(nh.subscribe("mux_drive/selected", 1, &Rc110JoyTeleop::onAdModeChanged, this));

    int direction = m_param.steeringAxis < 0 ? -1 : 1;
    m_param.steeringAxis *= direction;
    m_axisDirection[m_param.steeringAxis] = direction;

    direction = m_param.speedAxis < 0 ? -1 : 1;
    m_param.speedAxis *= direction;
    m_axisDirection[m_param.speedAxis] = direction;

    double maxSteeringAngleDeg = pnh.param("max_steering_angle_deg", 0.0);
    if (maxSteeringAngleDeg != 0.0) {
        m_param.maxSteeringAngleRad = angles::from_degrees(maxSteeringAngleDeg);
    }
    std::vector<double> gears = pnh.param("gears", m_param.gears);
    if (!gears.empty()) {
        m_param.gears = gears;
    }

    m_timer = pnh.createTimer(ros::Duration(1 / m_param.rate), [this](const ros::TimerEvent&) { publishDrive(); });
}

void Rc110JoyTeleop::publishDrive()
{
    if (m_deadManPressed) {
        m_drivePub.publish(m_driveMessage);
        m_stopMessagePublished = false;
    } else if (!m_stopMessagePublished) {
        m_drivePub.publish(ackermann_msgs::AckermannDriveStamped());
        m_stopMessagePublished = true;
    }
}

void Rc110JoyTeleop::updateToggles(const sensor_msgs::Joy::ConstPtr& message)
{
    if (checkButtonClicked(message, m_param.boardButton)) {
        std_srvs::SetBool service;
        service.request.data = uint8_t(!m_boardEnabled);  // toggle board
        ros::service::call("enable_board", service);
    }

    if (checkButtonClicked(message, m_param.adButton)) {
        topic_tools::MuxSelect service;
        service.request.topic = m_adEnabled ? "drive_manual" : "drive_ad";  // toggle AD
        ros::service::call("mux_drive/select", service);
    }
}

bool Rc110JoyTeleop::checkButtonClicked(const sensor_msgs::Joy::ConstPtr& message, int button)
{
    if (!m_joyMessage) {
        return false;
    }
    return message->buttons[button] && !m_joyMessage->buttons[button];
}

void Rc110JoyTeleop::onJoy(const sensor_msgs::Joy::ConstPtr& message)
{
    if (checkButtonClicked(message, m_param.gearUpButton)) ++m_gear;
    if (checkButtonClicked(message, m_param.gearDownButton)) --m_gear;

    auto maxGear = int(m_param.gears.size()) - 1;

    m_deadManPressed = message->buttons[m_param.deadManButton];
    m_gear = m_deadManPressed ? std::clamp(m_gear, 0, maxGear) : 0;

    auto speedFactor = float(m_param.gears[m_gear]);
    auto correctedAngle = float(m_param.maxSteeringAngleRad) * correctSteeringAngle(message->axes[m_param.steeringAxis]);

    m_driveMessage.drive.steering_angle = m_axisDirection[m_param.steeringAxis] * correctedAngle;
    m_driveMessage.drive.speed = m_axisDirection[m_param.speedAxis] * message->axes[m_param.speedAxis] * speedFactor;
    m_driveMessage.header.stamp = ros::Time::now();
    m_driveMessage.header.frame_id = m_param.frameId;

    updateToggles(message);

    m_joyMessage = message;
}

void Rc110JoyTeleop::onRobotStatus(const rc110_msgs::Status& message)
{
    m_boardEnabled = message.board_enabled;
}

void Rc110JoyTeleop::onAdModeChanged(const std_msgs::String& message)
{
    m_adEnabled = message.data == "drive_ad";
}

}  // namespace zmp
