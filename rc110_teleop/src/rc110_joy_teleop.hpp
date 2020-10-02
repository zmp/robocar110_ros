/**
 * @file    rc110_joy_teleop.hpp
 *
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "teleop_component.hpp"

namespace zmp
{
class Rc110JoyTeleop
{
public:
    using TeleopComponentPtr = std::shared_ptr<TeleopComponent>;

    struct Param {
        double maxIntervalSec = 0.25;
    };

public:
    Rc110JoyTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~Rc110JoyTeleop();

    void publish(const ros::Duration& dt);

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joyMsg);

private:
    // currently rc110 only has base component
    // in the far future if it ever has any more components; an arm for example;
    // stack more components here
    static const std::vector<std::string> COMPONENT_NAMES;
    std::unordered_map<std::string, TeleopComponentPtr> m_components;

    Param m_param;

    ros::Subscriber m_joySub;
    ros::Time m_lastUpdateTime;
};
}  // namespace zmp
