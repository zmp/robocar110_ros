/**
 * @file    rc110_joy_teleop.cpp
 *
 */

#include "rc110_joy_teleop.hpp"

namespace zmp
{
const std::vector<std::string> Rc110JoyTeleop::COMPONENT_NAMES = {"base"};

Rc110JoyTeleop::Rc110JoyTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        m_joySub(pnh.subscribe<sensor_msgs::Joy>("input_joy", 1, &zmp::Rc110JoyTeleop::joyCallback, this)),
        m_lastUpdateTime(ros::Time::now())
{
    TeleopComponentPtr teleopComponent;
    for (const std::string& componentName : COMPONENT_NAMES) {
        if (componentName == "base") {
            teleopComponent.reset(new RobotBaseTeleop(nh, pnh));
        } else {
            throw std::runtime_error("not supported component");
        }
        m_components.emplace(componentName, teleopComponent);
    }

    pnh.param<double>("max_interval_sec", m_param.maxIntervalSec, m_param.maxIntervalSec);
}

Rc110JoyTeleop::~Rc110JoyTeleop() {}

void Rc110JoyTeleop::publish(const ros::Duration& dt)
{
    if (ros::Time::now() - m_lastUpdateTime > ros::Duration(m_param.maxIntervalSec)) {
        for (auto& elem : m_components) {
            elem.second->stop();
        }
    } else {
        for (auto& elem : m_components) {
            elem.second->publish(dt);
        }
    }
}

void Rc110JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joyMsg)
{
    bool ok = true;
    for (auto& elem : m_components) {
        if (ok) {
            ok &= !elem.second->update(joyMsg);
        } else {
            // supressed by a higher priority component
            elem.second->stop();
        }
    }
    m_lastUpdateTime = ros::Time::now();
}
}  // namespace zmp
