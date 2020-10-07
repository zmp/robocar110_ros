/**
 * @file    teleop_component.hpp
 *
 * @author  btran
 *
 */

#pragma once

// clang-format off

#include <string>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <angles/angles.h>
#include <sensor_msgs/Joy.h>

// clang-format on

namespace zmp
{
class TeleopComponent
{
public:
    TeleopComponent(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    virtual ~TeleopComponent();

    virtual bool update(const sensor_msgs::Joy::ConstPtr& joyMsg) = 0;

    virtual void publish(const ros::Duration& dt) = 0;

    virtual bool start();

    virtual bool stop();

protected:
    double integrateSpeed(const double desired, const double present, const double maxRate, const double dt);

protected:
    bool m_active;

    ackermann_msgs::AckermannDriveStamped m_desired;  // desired command message
    ackermann_msgs::AckermannDriveStamped m_last;     // last command message

    ros::Publisher m_drivePub;
};

class RobotBaseTeleop : public TeleopComponent
{
public:
    struct Param {
        int deadManButton = 4;
        int steeringAxis = 0;
        int speedAxis = 3;
        double minSteeringAngleRad = angles::from_degrees(-30);
        double maxSteeringAngleRad = angles::from_degrees(30);
        double maxSteeringAngleVelRad = angles::from_degrees(381.81);  // 60 * 0.7 / 0.11
        double minSpeed = -2.7;                                        // [m/s]
        double maxSpeed = 2.7;                                         // [m/s]
        double maxAcceleration = 1.0;                                  // [m/s2]
        std::string frameId = "rc110_base";
    };

public:
    RobotBaseTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    virtual ~RobotBaseTeleop();

    bool update(const sensor_msgs::Joy::ConstPtr& joyMsg) final;

    void publish(const ros::Duration& dt) final;

private:
    void initParam(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    Param m_param;
};
}  // namespace zmp
