/**
 * @file    teleop_component.cpp
 *
 * @author  btran
 *
 */

#include "teleop_component.hpp"

namespace zmp
{
// ----------------------------------------------------------------------------//
// TeleopComponent
// ----------------------------------------------------------------------------//

TeleopComponent::TeleopComponent(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        m_active(false),
        m_desired(ackermann_msgs::AckermannDriveStamped()),
        m_last(ackermann_msgs::AckermannDriveStamped()),
        m_drivePub(pnh.advertise<ackermann_msgs::AckermannDriveStamped>("output_cmd", 1))
{
}

TeleopComponent::~TeleopComponent() {}

double TeleopComponent::integrateSpeed(const double desired, const double present, const double maxRate, const double dt)
{
    return desired > present ? std::min(desired, present + maxRate * dt) : std::max(desired, present - maxRate * dt);
}

bool TeleopComponent::start()
{
    m_active = true;
    return m_active;
}

bool TeleopComponent::stop()
{
    // publish stop message
    m_desired = ackermann_msgs::AckermannDriveStamped();
    m_last = ackermann_msgs::AckermannDriveStamped();
    m_drivePub.publish(m_last);

    m_active = false;
    return m_active;
}

// ----------------------------------------------------------------------------//
// Robot Base Teleop
// ----------------------------------------------------------------------------//

RobotBaseTeleop::RobotBaseTeleop(ros::NodeHandle& nh, ros::NodeHandle& pnh) : TeleopComponent(nh, pnh)
{
    this->initParam(nh, pnh);
}

RobotBaseTeleop::~RobotBaseTeleop() {}

void RobotBaseTeleop::initParam(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    pnh.param<int>("base_dead_man_button", m_param.deadManButton, m_param.deadManButton);
    pnh.param<int>("base_steering_axis", m_param.steeringAxis, m_param.steeringAxis);
    pnh.param<int>("base_speed_axis", m_param.speedAxis, m_param.speedAxis);

    double maxSteeringAngleDeg;
    if (pnh.getParam("max_steering_angle_deg", maxSteeringAngleDeg)) {
        m_param.maxSteeringAngleRad = angles::from_degrees(maxSteeringAngleDeg);
    }

    pnh.param<double>("max_speed", m_param.maxSpeed, m_param.maxSpeed);
    pnh.param<double>("max_acceleration", m_param.maxAcceleration, m_param.maxAcceleration);
    pnh.param<std::string>("base_frame_id", m_param.frameId, m_param.frameId);
}

bool RobotBaseTeleop::update(const sensor_msgs::Joy::ConstPtr& joyMsg)
{
    if (!joyMsg->buttons[m_param.deadManButton]) {
        this->stop();
        return false;
    }

    this->start();

    m_desired.drive.steering_angle = joyMsg->axes[m_param.steeringAxis] * m_param.maxSteeringAngleRad;

    m_desired.drive.speed = joyMsg->axes[m_param.speedAxis] * m_param.maxSpeed;

    return true;
}

void RobotBaseTeleop::publish(const ros::Duration& dt)
{
    if (m_active) {
        m_last.drive.steering_angle = this->integrateSpeed(m_desired.drive.steering_angle,
                                                           m_last.drive.steering_angle,
                                                           m_param.maxSteeringAngleVelRad,
                                                           dt.toSec());
        m_last.drive.speed =
                this->integrateSpeed(m_desired.drive.speed, m_last.drive.speed, m_param.maxSpeed, dt.toSec());
        m_last.header.stamp = ros::Time::now();
        m_last.header.frame_id = m_param.frameId;

        m_drivePub.publish(m_last);
    }
}
}  // namespace zmp
