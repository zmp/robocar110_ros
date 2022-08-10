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
        param({.rc = ros::param::param("~rc", param.rc),
               .frameId = ros::param::param("~frame_id", param.frameId),
               .gears = ros::param::param("~gears", param.gears),
               .rate = ros::param::param("~rate", param.rate),
               .joyType = ros::param::param("~joy_type", param.joyType),
               .joyTypes = ros::param::param("~joy_types", param.joyTypes)})
{
    if (param.gears.empty()) param.gears = {0.3};

    driveTimer = handle.createTimer(ros::Duration(1 / param.rate), [this](const auto&) { publishDrive(); });
    nextRobotTimer = handle.createTimer(ros::Duration(1), [this](const auto&) { onRobotNameTimer(); });

    subscribers["joy"] = handle.subscribe("joy", 1, &Rc110JoyTeleop::onJoy, this);
    publishers["teleop_rc"] = handle.advertise<std_msgs::String>("teleop_rc", 1, bool("latch"));

    selectedRobot = param.rc;
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

    updateParam(param.deadManButton, config["dead_man_button"]);
    updateParam(param.nextRobotButton, config["next_robot_button"]);
    updateParam(param.gearUpButton, config["gear_up_button"]);
    updateParam(param.gearDownButton, config["gear_down_button"]);
    updateParam(param.boardButton, config["board_button"]);
    updateParam(param.adButton, config["ad_button"]);
    updateParam(param.steering, config["steering"]);
    updateParam(param.steeringAuxiliary, config["steering_aux"]);
    updateParam(param.accel, config["accel"]);

    updateAxis(param.steering, 28.0 * 1.3);  // by default, max value + 30% to being able to reach max easily
    updateAxis(param.accel, 1.0);

    ROS_INFO_STREAM("Joystick type: " << joyType);
}

std::string Rc110JoyTeleop::getJoyType(const std::string& device)
{
    if (param.joyType.size()) {
        return param.joyType;
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
    for (const auto& [type, name] : param.joyTypes) {
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
        axisDirection[axis[AXIS_ID]] = 1;
    } else {
        axisDirection[axis[AXIS_ID]] = -1;
        std::swap(axis[AXIS_MIN], axis[AXIS_MAX]);
    }
}

void Rc110JoyTeleop::publishDrive()
{
    if (param.deadManButton == -1) {
        // if dead man button deactivated, publish when there are motion commands in last 500 ms
        if ((ros::Time::now() - lastTime).toSec() < 0.5) {
            publishers["drive_manual"].publish(driveMessage);
        }
        return;
    }

    if (joyMessage) {
        if (joyMessage->buttons[param.deadManButton]) {
            // if dead man button is pressed, send message
            publishers["drive_manual"].publish(driveMessage);
            stopMessagePublished = false;
        } else if (!stopMessagePublished) {
            // send additional stop message, immediately after button release
            publishers["drive_manual"].publish(ackermann_msgs::AckermannDriveStamped());
            stopMessagePublished = true;
        }
    }
}

void Rc110JoyTeleop::updateToggles(const sensor_msgs::Joy::ConstPtr& message)
{
    if (param.rc.empty() && checkButtonClicked(message, param.nextRobotButton)) {
        incrementRobotName();
    }
    auto ns = selectedRobot.empty() ? "" : "/" + selectedRobot;

    if (checkButtonClicked(message, param.boardButton)) {
        std_srvs::SetBool service;
        service.request.data = uint8_t(!boardEnabled);  // toggle board
        ros::service::call(ns + "/enable_board", service);
    }

    if (checkButtonClicked(message, param.adButton)) {
        topic_tools::MuxSelect service;
        service.request.topic = adEnabled ? "drive_manual" : "drive_ad";  // toggle AD
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
    std_msgs::String message;
    message.data = selectedRobot;
    publishers["teleop_rc"].publish(message);
}

bool Rc110JoyTeleop::checkButtonClicked(const sensor_msgs::Joy::ConstPtr& message, int button)
{
    return joyMessage && message->buttons[button] && !joyMessage->buttons[button];
}

bool Rc110JoyTeleop::checkAxisChanged(const sensor_msgs::Joy::ConstPtr& message, int axis)
{
    return joyMessage && message->axes[axis] != joyMessage->axes[axis];
}

float Rc110JoyTeleop::getAxisValue(const sensor_msgs::Joy::ConstPtr& message, int axis)
{
    return axis == -1 ? 0.f : message->axes[axis];
}

float Rc110JoyTeleop::mapAxisValue(const std::vector<double>& axis, float value)
{
    // workaround for joy_node bug which does not read initial axis value
    if (value != 0.f) {
        axisActivated[axis[AXIS_ID]] = true;
    }
    if (!axisActivated[axis[AXIS_ID]]) {
        return 0.f;
    }

    // value is [-1 .. 1], mapped to [min, max]
    float minValue = axis[AXIS_MIN];
    float maxValue = axis[AXIS_MAX];
    value = (value + 1) / 2;
    value = minValue + value * (maxValue - minValue);

    return axisDirection[axis[AXIS_ID]] * value;
}

void Rc110JoyTeleop::onJoy(const sensor_msgs::Joy::ConstPtr& message)
{
    setupJoystick(message->header.frame_id);

    auto minGear = -1;  // reverse gear, 1st gear, 2nd gear, and so on
    auto maxGear = int(param.gears.size()) - 1;
    auto steeringValue = getAxisValue(message, param.steering[AXIS_ID]);
    if (isLeverAxis(param.accel)) {
        float steeringAux = getAxisValue(message, param.steeringAuxiliary);
        steeringValue = correctLeverAngle(steeringValue, steeringAux);
    }
    steeringValue = mapAxisValue(param.steering, steeringValue);
    auto accelValue = mapAxisValue(param.accel, getAxisValue(message, param.accel[AXIS_ID]));

    if (checkButtonClicked(message, param.gearUpButton)) ++gear;
    if (checkButtonClicked(message, param.gearDownButton)) --gear;

    // for lever, reverse gear is activated by pulling the lever back
    if (isLeverAxis(param.accel)) {
        gear = accelValue < 0 ? -1 : std::max(0, gear);
    }
    // limit gear, and if no accelerator, initialize with lowest gear
    gear = std::fabs(accelValue) > 0.01f ? std::clamp(gear, minGear, maxGear) : std::min(0, gear);

    // reverse gear speed equal to negative first gear
    auto gearFactor = gear < 0 ? -1 * float(param.gears[0]) : float(param.gears[gear]);

    driveMessage.drive.steering_angle = angles::from_degrees(steeringValue);
    driveMessage.drive.speed = std::fabs(accelValue) * gearFactor;
    driveMessage.header.stamp = ros::Time::now();
    driveMessage.header.frame_id = param.frameId;

    updateToggles(message);
    joyMessage = message;
    lastTime = ros::Time::now();
}

void Rc110JoyTeleop::onRobotStatus(const rc110_msgs::Status& message)
{
    boardEnabled = message.board_enabled;
}

void Rc110JoyTeleop::onAdModeChanged(const std_msgs::String& message)
{
    adEnabled = message.data == "drive_ad";
}

}  // namespace zmp
