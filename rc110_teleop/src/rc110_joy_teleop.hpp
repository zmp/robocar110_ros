/**
 * @file    rc110_joy_teleop.hpp
 *
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <string>

namespace zmp
{
class Rc110JoyTeleop
{
public:
    struct Parameters {
        std::string frameId = "rc110_base";
        double maxSpeed = 0.5;          // [m/s]
        double maxSteeringAngle = 0.7;  // [rad]
    };

public:
    Rc110JoyTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~Rc110JoyTeleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joyMsg);

private:
    Parameters m_param;

    ros::Subscriber m_joySub;
    ros::Publisher m_drivePub;
};
}  // namespace zmp
