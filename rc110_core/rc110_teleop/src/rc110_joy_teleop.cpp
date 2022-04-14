/*
 * Copyright (C) 2020 ZMP Inc info@zmp.co.jp
 *
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 *
 * Written by btran, Andrei Pak
 */

#include "rc110_joy_teleop.hpp"

#ifdef __linux__
#include <fcntl.h>
#include <linux/joystick.h>
#endif

#include <ros/package.h>
#include <std_srvs/SetBool.h>
#include <topic_tools/MuxSelect.h>
#include <yaml-cpp/yaml.h>

#include <param_tools/param_tools.hpp>
#include <regex>

namespace zmp
{
namespace
{
constexpr char DEFAULT_JOY_TYPE[] = "elecom";

enum AxisSetting { AXIS_ID, AXIS_MAX, AXIS_MIN };

float correctLeverAngle(float y, float x)  // x, y (- [-1.0 .. 1.0]
{
    return y * (1 - std::fabs(x) / 2);  // [0 .. 1] is distributed on both y axis and [0 .. pi/2] angle of lever
}

bool isLeverAxis(const std::vector<double>& axis)
{
    return axis[AXIS_MIN] < 0;
}

template <typename T>
void updateParam(T& param, const YAML::Node& node)
{
    param = node ? node.as<T>() : param;
}

ros::V_string getRobotNames()
{
    ros::V_string nodeNames;
    ros::master::getNodes(nodeNames);

    static const std::regex expression("/(.*)/mux_drive$");

    ros::V_string robotNames;
    for (const auto& nodeName : nodeNames) {
        std::smatch match;
        if (std::regex_match(nodeName, match, expression); match.size()) {
            robotNames.push_back(match[1]);
        }
    }
    return robotNames;
}
}  // namespace

Rc110JoyTeleop::Rc110JoyTeleop() :
        m_param({.rc = ros::param::param("~rc", m_param.rc),
                 .frameId = ros::param::param("~frame_id", m_param.frameId),
                 .gears = ros::param::param("~gears", m_param.gears),
                 .rate = ros::param::param("~rate", m_param.rate),
                 .joyType = ros::param::param("~joy_type", m_param.joyType),
                 .joyTypes = ros::param::param("~joy_types", m_param.joyTypes)})
{
    if (m_param.gears.empty()) m_param.gears = {0.3};

    rcSubscriber = param_tools::instance().subscribe("/selected_rc", [this](const XmlRpc::XmlRpcValue& value) {
        setupRobotName(value);
    });

    driveTimer = handle.createTimer(ros::Duration(1 / m_param.rate), [this](const auto&) { publishDrive(); });
    nextRobotTimer = handle.createTimer(ros::Duration(1), [this](const auto&) { onRobotNameTimer(); });

    subscribers["joy"] = handle.subscribe("joy", 1, &Rc110JoyTeleop::onJoy, this);

    selectedRobot = m_param.rc;
    setupRosConnections();
}

void Rc110JoyTeleop::setupJoystick(const std::string& device)
{
    if (joyDevice == device) {
        return;
    }
    joyDevice = device;

    auto joyType = getJoyType(device);
    auto path = ros::package::getPath("rc110_teleop") + "/config/joy_" + joyType + ".yaml";
    YAML::Node config = YAML::LoadFile(path);

    updateParam(m_param.deadManButton, config["dead_man_button"]);
    updateParam(m_param.nextRobotButton, config["next_robot_button"]);
    updateParam(m_param.gearUpButton, config["gear_up_button"]);
    updateParam(m_param.gearDownButton, config["gear_down_button"]);
    updateParam(m_param.boardButton, config["board_button"]);
    updateParam(m_param.adButton, config["ad_button"]);
    updateParam(m_param.steering, config["steering"]);
    updateParam(m_param.steeringAuxiliary, config["steering_aux"]);
    updateParam(m_param.accel, config["accel"]);

    updateAxis(m_param.steering, 28.0 * 1.3);  // by default, max value + 30% to being able to reach max easily
    updateAxis(m_param.accel, 1.0);

    ROS_INFO_STREAM("Joystick type: " << joyType);
}

std::string Rc110JoyTeleop::getJoyType(const std::string& device)
{
    if (m_param.joyType.size()) {
        return m_param.joyType;
    }
#ifdef __linux__
    int fd = open(device.c_str(), O_RDONLY);
    if (fd != -1) {
        char joyNameBuffer[1024];
        ioctl(fd, JSIOCGNAME(sizeof joyNameBuffer), joyNameBuffer);
        close(fd);

        auto joyType = joyNameToType(joyNameBuffer);
        if (joyType.size()) {
            return joyType;
        }
    }
#endif
    return DEFAULT_JOY_TYPE;
}

std::string Rc110JoyTeleop::joyNameToType(const std::string& joyName)
{
    int maxMatch = 0;
    std::string joyType;
    for (const auto& [type, name] : m_param.joyTypes) {
        // check either whole name match
        if (int pos = name.find(joyName); pos != std::string::npos) {
            return type;
        }
        // or just partial match from the name start
        auto result = std::mismatch(name.begin(), name.end(), joyName.begin(), joyName.end());
        int iMis = result.first - name.begin();
        if (maxMatch < iMis) {
            maxMatch = iMis;
            joyType = type;
        }
    }
    return joyType;
}

void Rc110JoyTeleop::setupRosConnections()
{
    auto ns = selectedRobot.empty() ? "" : "/" + selectedRobot;

    subscribers["robot_status"] = handle.subscribe(ns + "/robot_status", 1, &Rc110JoyTeleop::onRobotStatus, this);
    subscribers["mux_drive"] = handle.subscribe(ns + "/mux_drive/selected", 1, &Rc110JoyTeleop::onAdModeChanged, this);
    publishers["drive_manual"] = handle.advertise<ackermann_msgs::AckermannDriveStamped>(ns + "/drive_manual", 1);
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
            publishers["drive_manual"].publish(m_driveMessage);
        }
        return;
    }

    if (m_joyMessage) {
        if (m_joyMessage->buttons[m_param.deadManButton]) {
            // if dead man button is pressed, send message
            publishers["drive_manual"].publish(m_driveMessage);
            m_stopMessagePublished = false;
        } else if (!m_stopMessagePublished) {
            // send additional stop message, immediately after button release
            publishers["drive_manual"].publish(ackermann_msgs::AckermannDriveStamped());
            m_stopMessagePublished = true;
        }
    }
}

void Rc110JoyTeleop::updateToggles(const sensor_msgs::Joy::ConstPtr& message)
{
    if (m_param.rc.empty() && checkButtonClicked(message, m_param.nextRobotButton)) {
        incrementRobotName();
    }
    auto ns = selectedRobot.empty() ? "" : "/" + selectedRobot;

    if (checkButtonClicked(message, m_param.boardButton)) {
        std_srvs::SetBool service;
        service.request.data = uint8_t(!m_boardEnabled);  // toggle board
        ros::service::call(ns + "/enable_board", service);
    }

    if (checkButtonClicked(message, m_param.adButton)) {
        topic_tools::MuxSelect service;
        service.request.topic = m_adEnabled ? "drive_manual" : "drive_ad";  // toggle AD
        ros::service::call(ns + "/mux_drive/select", service);
    }
}

void Rc110JoyTeleop::onRobotNameTimer()
{
    auto names = getRobotNames();
    if (robotNames != names) {
        robotNames = names;

        if (std::find(robotNames.begin(), robotNames.end(), selectedRobot) == robotNames.end()) {
            if (robotNames.size()) {
                setupRobotName(robotNames.front());
                publishRobotName();
            }
        }
    }
}

void Rc110JoyTeleop::incrementRobotName()
{
    if (robotNames.size() > 1) {
        auto it = std::find(robotNames.begin(), robotNames.end(), selectedRobot);
        int i = it - robotNames.begin();

        setupRobotName(robotNames[++i % robotNames.size()]);
        publishRobotName();
    }
}

void Rc110JoyTeleop::setupRobotName(const std::string& name)
{
    if (selectedRobot != name) {
        selectedRobot = name;

        // send stop message before switching robot
        if (publishers.count("drive_manual")) {
            publishers["drive_manual"].publish(ackermann_msgs::AckermannDriveStamped());
        }
        setupRosConnections();
    }
}

void Rc110JoyTeleop::publishRobotName()
{
    param_tools::instance().publish("/selected_rc", selectedRobot);
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
    setupJoystick(message->header.frame_id);

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
