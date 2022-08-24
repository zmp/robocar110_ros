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
#include <unistd.h>
#endif

#include <ros/package.h>
#include <std_srvs/SetBool.h>
#include <topic_tools/MuxSelect.h>
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
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

    static const std::regex expression("/(.*)/drive_control$");

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
        param({
                ros::param::param("~rc", std::string("zmp")),
                ros::param::param("~frame_id", std::string("joy")),
                ros::param::param("~gears", std::vector<double>{0.3, 0.6, 1.0}),
                ros::param::param("~rate", 30.0),
                ros::param::param("~joy_path", std::string("")),
                ros::param::param("~joy_type", std::string("")),  // autodetect if empty
        }),
        configPath(ros::package::getPath("rc110_teleop") + "/config")
{
    if (param.gears.empty()) param.gears = {0.3};

    driveTimer = handle.createTimer(ros::Duration(1 / param.rate), [this](const auto&) { publishDrive(); });
    nextRobotTimer = handle.createTimer(ros::Duration(1), [this](const auto&) { onRobotNameTimer(); });

    subscribers["joy"] = handle.subscribe("joy", 1, &Rc110JoyTeleop::onJoy, this);

    selectedRobot = param.rc;

    setupJoystick(param.joyPath);
    setupRosConnections();
}

void Rc110JoyTeleop::setupJoystick(const std::string& joyPath)
{
    auto joyType = getJoyType(joyPath);
    if (joyType.empty()) {
        return;
    }
    YAML::Node configNode = YAML::LoadFile(configPath + "/joy_" + joyType + ".yaml");

    updateParam(param.deadManButton, configNode["dead_man_button"]);
    updateParam(param.nextRobotButton, configNode["next_robot_button"]);
    updateParam(param.gearUpButton, configNode["gear_up_button"]);
    updateParam(param.gearDownButton, configNode["gear_down_button"]);
    updateParam(param.boardButton, configNode["board_button"]);
    updateParam(param.adButton, configNode["ad_button"]);
    updateParam(param.steering, configNode["steering"]);
    updateParam(param.steeringAuxiliary, configNode["steering_aux"]);
    updateParam(param.accel, configNode["accel"]);

    updateAxis(param.steering, 28.0 * 1.3);  // by default, max value + 30% to being able to reach max easily
    updateAxis(param.accel, 1.0);
}

std::string Rc110JoyTeleop::getJoyType(const std::string& joyPath) const
{
    // Either get type directly from parameter,
    if (param.joyType.size()) {
        return param.joyType;
    }
    std::string joyType = DEFAULT_JOY_TYPE;

    // Or try to guess the type basing on device description.
    auto description = getJoyDescription(joyPath);
    if (!description.empty()) {
        int maxMatch = 0;
        for (auto& typeConfig : getTypeConfigs()) {
            int match = matchDescription(description, typeConfig.second);
            if (maxMatch < match) {
                maxMatch = match;
                joyType = typeConfig.first;
            }
        }
    }
    ROS_INFO_STREAM("Joystick type: " << joyType << ", basing on path: " << joyPath);
    return joyType;
}

std::string Rc110JoyTeleop::getJoyDescription(const std::string& joyPath) const
{
#ifdef __linux__
    int fd = open(joyPath.c_str(), O_RDONLY);
    if (fd != -1) {
        char joyNameBuffer[1024];
        ioctl(fd, JSIOCGNAME(sizeof joyNameBuffer), joyNameBuffer);
        close(fd);
        return joyNameBuffer;
    }
#endif
    return "";
}

std::map<std::string, std::string> Rc110JoyTeleop::getTypeConfigs() const
{
    std::regex expression(".*/joy_(.*)\\.yaml$");

    std::map<std::string, std::string> typeConfigs;
    for (auto& entry : boost::filesystem::directory_iterator(configPath)) {
        std::smatch match;
        if (std::regex_match(entry.path().string(), match, expression); match.size()) {
            typeConfigs.emplace(match[1].str(), match[0].str());
        }
    }
    return typeConfigs;
}

int Rc110JoyTeleop::matchDescription(const std::string& description, const std::string& config) const
{
    YAML::Node configNode = YAML::LoadFile(config);
    if (auto descriptionNode = configNode["description"]) {
        auto yamlDescription = descriptionNode.as<std::string>();

        // check either whole name match
        if (int pos = yamlDescription.find(description); pos != std::string::npos) {
            return description.size();
        }
        // or just partial match from the name start
        auto result =
                std::mismatch(yamlDescription.begin(), yamlDescription.end(), description.begin(), description.end());
        return result.first - yamlDescription.begin();
    }
    return 0;
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
    robotNames = getRobotNames();

    if (!std::count(robotNames.begin(), robotNames.end(), selectedRobot)) {
        for (int i = 0; i != robotNames.size(); ++i) {
            if (connectRobot(robotNames[i])) {
                setupRobotName(robotNames[i]);
                return;
            }
        }
    }
}

void Rc110JoyTeleop::incrementRobotName()
{
    if (robotNames.empty()) {
        return;
    }
    int size = int(robotNames.size());
    auto it = std::find(robotNames.begin(), robotNames.end(), selectedRobot);
    int iStart = (int(it - robotNames.begin())) % size;

    // check other robots and connect to the first robot that accept the connection
    for (int i = (iStart + 1) % size; i != iStart; i = (i + 1) % size) {
        if (connectRobot(robotNames[i])) {
            setupRobotName(robotNames[i]);
            return;
        }
    }
}

void Rc110JoyTeleop::setupRobotName(const std::string& name)
{
    ROS_INFO_STREAM(selectedRobot << " -> " << name);
    if (selectedRobot != name) {
        selectedRobot = name;

        // send stop message before switching robot
        if (publishers.count("drive_manual")) {
            publishers["drive_manual"].publish(ackermann_msgs::AckermannDriveStamped());
        }
        setupRosConnections();
    }
}

bool Rc110JoyTeleop::connectRobot(const std::string& name)
{
    std_srvs::SetBool service;
    service.request.data = true;
    if (ros::service::call("/" + name + "/teleop_ping", service)) {
        if (service.response.success) {
            return true;
        }
    }
    return false;
}

void Rc110JoyTeleop::pingRobot()
{
    if (!selectedRobot.empty()) {
        std_srvs::SetBool service;
        service.request.data = false;  // already connected
        ros::service::call("/" + selectedRobot + "/teleop_ping", service);
    }
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

    pingRobot();
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
