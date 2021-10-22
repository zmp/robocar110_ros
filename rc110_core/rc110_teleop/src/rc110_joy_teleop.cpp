/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
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
enum AxisSetting { AXIS_ID, AXIS_MAX, AXIS_MIN };

float correctLeverAngle(float y, float x)  // x, y (- [-1.0 .. 1.0]
{
    return y * (1 - std::fabs(x) / 2);  // [0 .. 1] is distributed on both y axis and [0 .. pi/2] angle of lever
}

bool isLeverAxis(const std::vector<double>& axis)
{
    return axis[AXIS_MIN] < 0;
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
    pnh.param("steering", m_param.steering, m_param.steering);
    pnh.param("steering_aux", m_param.steeringAuxiliary, m_param.steeringAuxiliary);
    pnh.param("accel", m_param.accel, m_param.accel);
    pnh.param<std::string>("base_frame_id", m_param.frameId, m_param.frameId);

    updateAxis(m_param.steering, 28.0 * 1.3);  // by default, max value + 30% to being able to reach max easily
    updateAxis(m_param.accel, 1.0);

    if (m_param.steeringAuxiliary == -1) {
        m_param.steeringAuxiliary = m_param.steering[AXIS_ID] - 1;
    }

    std::vector<double> gears = pnh.param("gears", m_param.gears);
    if (!gears.empty()) {
        m_param.gears = gears;
    }

    pnh.param("rate", m_param.rate, m_param.rate);
    m_timer = pnh.createTimer(ros::Duration(1 / m_param.rate), [this](const ros::TimerEvent&) { publishDrive(); });

    m_subscribers.push_back(nh.subscribe("joy", 1, &zmp::Rc110JoyTeleop::onJoy, this));
    m_subscribers.push_back(nh.subscribe("robot_status", 1, &Rc110JoyTeleop::onRobotStatus, this));
    m_subscribers.push_back(nh.subscribe("mux_drive/selected", 1, &Rc110JoyTeleop::onAdModeChanged, this));
}

void Rc110JoyTeleop::updateAxis(std::vector<double>& axis, double defaultMax)
{
    switch (axis.size()) {
        case 0:
            throw std::runtime_error("Please, specify all axes!");
        case 1:
            axis = {axis[AXIS_ID], defaultMax, -defaultMax};
            break;
        case 2:
            axis = {axis[AXIS_ID], axis[AXIS_MAX], -axis[AXIS_MAX]};
            break;
    }
    if (axis[AXIS_MIN] < axis[AXIS_MAX]) {
        m_axisDirection[axis[AXIS_ID]] = 1;
    } else {
        m_axisDirection[axis[AXIS_ID]] = -1;
        std::swap(axis[AXIS_MIN], axis[AXIS_MAX]);
    }
}

void Rc110JoyTeleop::publishDrive()
{
    if (m_param.deadManButton == -1) {
        // if dead man button deactivated, publish when there are motion commands in last 500 ms
        if ((ros::Time::now() - m_lastTime).toSec() < 0.5) {
            m_drivePub.publish(m_driveMessage);
        }
    } else if (m_joyMessage) {
        // if dead man button is activated, use it
        if (m_joyMessage->buttons[m_param.deadManButton]) {
            m_drivePub.publish(m_driveMessage);
            m_stopMessagePublished = false;
        } else if (!m_stopMessagePublished) {
            // and send additional stop message, immediately after button release
            m_drivePub.publish(ackermann_msgs::AckermannDriveStamped());
            m_stopMessagePublished = true;
        }
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
    return m_joyMessage && message->buttons[button] && !m_joyMessage->buttons[button];
}

bool Rc110JoyTeleop::checkAxisChanged(const sensor_msgs::Joy::ConstPtr& message, int axis)
{
    return m_joyMessage && message->axes[axis] != m_joyMessage->axes[axis];
}

float Rc110JoyTeleop::getAxisValue(const sensor_msgs::Joy::ConstPtr& message, int axis)
{
    return axis == -1 ? 0.f : message->axes[axis];
}

float Rc110JoyTeleop::mapAxisValue(const std::vector<double>& axis, float value)
{
    // workaround for joy_node bug which does not read initial axis value
    if (value != 0.f) {
        m_axisActivated[axis[AXIS_ID]] = true;
    }
    if (!m_axisActivated[axis[AXIS_ID]]) {
        return 0.f;
    }

    // value is [-1 .. 1], mapped to [min, max]
    float minValue = axis[AXIS_MIN];
    float maxValue = axis[AXIS_MAX];
    value = (value + 1) / 2;
    value = minValue + value * (maxValue - minValue);

    return m_axisDirection[axis[AXIS_ID]] * value;
}

void Rc110JoyTeleop::onJoy(const sensor_msgs::Joy::ConstPtr& message)
{
    auto minGear = -1;  // reverse gear, 1st gear, 2nd gear, and so on
    auto maxGear = int(m_param.gears.size()) - 1;
    auto steeringValue = getAxisValue(message, m_param.steering[AXIS_ID]);
    if (isLeverAxis(m_param.accel)) {
        float steeringAux = getAxisValue(message, m_param.steeringAuxiliary);
        steeringValue = correctLeverAngle(steeringValue, steeringAux);
    }
    steeringValue = mapAxisValue(m_param.steering, steeringValue);
    auto accelValue = mapAxisValue(m_param.accel, getAxisValue(message, m_param.accel[AXIS_ID]));

    if (checkButtonClicked(message, m_param.gearUpButton)) ++m_gear;
    if (checkButtonClicked(message, m_param.gearDownButton)) --m_gear;

    // for lever, reverse gear is activated by pulling the lever back
    if (isLeverAxis(m_param.accel)) {
        m_gear = accelValue < 0 ? -1 : std::max(0, m_gear);
    }
    // limit gear, and if no accelerator, initialize with lowest gear
    m_gear = std::fabs(accelValue) > 0.01f ? std::clamp(m_gear, minGear, maxGear) : std::min(0, m_gear);

    // reverse gear speed equal to negative first gear
    auto gearFactor = m_gear < 0 ? -1 * float(m_param.gears[0]) : float(m_param.gears[m_gear]);

    m_driveMessage.drive.steering_angle = angles::from_degrees(steeringValue);
    m_driveMessage.drive.speed = std::fabs(accelValue) * gearFactor;
    m_driveMessage.header.stamp = ros::Time::now();
    m_driveMessage.header.frame_id = m_param.frameId;

    updateToggles(message);
    m_joyMessage = message;
    m_lastTime = ros::Time::now();
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
